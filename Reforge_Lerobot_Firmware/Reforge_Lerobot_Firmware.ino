/*
 * LeRobot Rate Monotonic Data Acquisition - Phase 5 S2B
 *
 * Goal: Dual-mode architecture for calibration orchestration
 *
 * Architecture:
 *   - MODE_WAIT: Boot state, accepts USB commands only
 *   - MODE_MOVE: Blocking SYNCW/READJ for motion, no ISR
 *   - MODE_DATA_IDLE: Data mode idle, no ISR, no Serial4 frames
 *   - MODE_DATA_RUN: Full 500Hz data acquisition (ISR running)
 *
 * S2B Protocol:
 *   - USB: MODE,DATA -> DATA_IDLE (no ISR yet)
 *   - USB: RUN,START -> DATA_RUN (ISR starts)
 *   - USB: EVT,LOGGING_ON (after first Serial4 write)
 *   - USB: RUN,STOP -> DATA_IDLE (ISR stops)
 *   - USB: EVT,LOGGING_OFF
 *   - Serial4: Pure binary frames only (no text handshake)
 *
 * Key Changes from Phase 4:
 *   - Non-blocking boot (no Serial4 handshake required)
 *   - Mode state machine with explicit transitions
 *   - SYNCW command for multi-servo control
 *   - READJ command for 6-joint position readback
 *   - Preserved trajectory streaming (G command) in DATA mode
 *
 * Phase: 5 S2B (Mode Manager + Run Control)
 */

#include <Arduino.h>
#include <SPI.h>
#include "ISM330_Bare.h"
#include "ServoHardware.h"

// ============== SERVO HARDWARE MODULE ==============
// Phase 1: Blocking operations for MODE_MOVE (SYNCW, READJ, MOVE commands)
// Protocol details hidden inside ServoHardware.cpp
ServoHardware servo;

// ARM Cortex-M7 Data Memory Barrier (for Seqlock synchronization)
#if defined(__arm__) && defined(__IMXRT1062__)
  #define SEQLOCK_BARRIER() asm volatile("dmb" ::: "memory")
#else
  #define SEQLOCK_BARRIER() __sync_synchronize()
#endif

// ============== CONFIGURATION ==============
#define SAMPLE_RATE_HZ 500
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_RATE_HZ)  // 2000 us

#define SERIAL_CSV_BAUD 2000000  // 2 Mbaud
#define ENABLE_DEBUG_OUTPUT 0    // 0 = silent, 1 = startup + errors

// ============== HARDWARE - IMU ==============
#define IMU_CS 10      // Default SPI CS (requires 10k pull-down to GND)
#define IMU_SCK 13     // Default SPI SCK
#define IMU_MISO 12    // Default SPI MISO
#define IMU_MOSI 11    // Default SPI MOSI

// ISM330 bare-metal driver scaling constants
static const float ACCEL_SENS = 0.061f / 1000.0f * 9.80665f;  // +/-2 g mode
static const float GYRO_SENS = 4.375f / 1000.0f * (PI / 180.0f);  // +/-125 dps mode

// ============== HARDWARE - SERVO ==============
// NOTE: Servo hardware constants (baud rate, timeouts, protocol details) are now
// encapsulated in the ServoHardware module. See ServoHardware.h for configuration.

// ============== MODE STATE MACHINE ==============
enum ModeState : uint8_t {
    MODE_WAIT = 0,         // Boot state, waiting for USB command
    MODE_MOVE = 1,         // Motion mode, blocking RPC, no ISR
    MODE_DATA_IDLE = 2,    // Data mode idle, no ISR, no Serial4 frames
    MODE_DATA_RUN  = 3     // Data mode running, ISR active, Serial4 streaming
};

volatile ModeState g_mode = MODE_WAIT;
volatile ModeState g_mode_req = MODE_WAIT;
volatile bool mode_req_pending = false;  // Edge-triggered mode request flag

// Trajectory streaming target servo (configurable via SERVO,<id> command)
volatile uint8_t g_stream_servo_id = 1;  // Default to servo 1

// S2B run control flags
volatile bool run_req_start = false;
volatile bool run_req_stop  = false;

// S2B event emission flags
volatile bool evt_logging_on_pending  = false;
volatile bool evt_logging_off_pending = false;
volatile bool logging_has_written_any = false;  // Set true on first Serial4 write

static inline const char* mode_str(ModeState m) {
    switch(m) {
        case MODE_WAIT: return "WAIT";
        case MODE_MOVE: return "MOVE";
        case MODE_DATA_IDLE: return "DATA_IDLE";
        case MODE_DATA_RUN: return "DATA_RUN";
        default: return "?";
    }
}

// ============== DATA STRUCTURES ==============

// Enhanced sensor snapshot (64 bytes)
struct SensorData {
    // Frame timing
    uint32_t frame_ts_us;         // ISR entry time (= IMU sample time)
    uint32_t frame_index;         // Monotonic counter

    // IMU (read FIRST in ISR)
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;

    // Servo state
    int32_t servo_position;       // Encoder position (0-4095)
    int32_t servo_velocity;       // Reserved

    // Timing diagnostics
    uint16_t imu_read_us;         // IMU SPI transaction time
    uint16_t servo_age_us;        // frame_ts_us - servo.t_rx_us
    uint16_t isr_total_us;        // Total ISR execution time
    uint16_t servo_latency_us;    // t_rx_us - t_req_us (for diagnostics)

    // Jitter metrics
    int16_t period_error_us;      // Actual period - 2000us

    // Coherency validation
    uint32_t checksum;
    uint8_t coherency_flag;       // 0xAA = valid, 0xFF = torn, 0xEE = stale

    // Error tracking
    uint8_t servo_error_code;     // 0=OK, 1=timeout, 2=checksum, 3=framing

    // Command tracking (for host-driven goal streaming)
    uint16_t cmd_goal;            // Last commanded goal position (0xFFFF = none)
    uint32_t cmd_time_us;         // When command was sent (micros)
} __attribute__((aligned(32)));

static_assert(sizeof(SensorData) == 64, "SensorData must be exactly 64 bytes");

typedef SensorData CSVSample;

// ============== SERVO STATE ==============
// NOTE: Servo state management has been moved to ServoHardware module.
// Access via servo.snapshot() for ISR-safe reads.

// Command tracking (for logging commanded vs actual position)
volatile uint32_t g_last_cmd_time_us = 0;
volatile uint16_t g_last_cmd_goal = 0xFFFF;  // Sentinel: no command sent yet
volatile uint16_t g_last_cmd_time_ms = 0;    // Time parameter sent to servo

// NOTE: Servo RX FSM, read state, and bus manager moved to ServoHardware module.

// ============== COMMAND QUEUE ==============

#define CMD_QUEUE_SIZE 256
#define CMD_QUEUE_MASK (CMD_QUEUE_SIZE - 1)

struct ServoCommand {
    uint8_t servo_id;
    uint16_t goal_position;
    uint16_t goal_time_ms;   // Time for servo internal interpolation
    uint16_t goal_speed;
};

struct CommandQueue {
    ServoCommand buf[CMD_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};

CommandQueue cmd_queue = {{}, 0, 0};

// ============== SEQLOCK BUFFER ==============

struct SeqlockBuffer {
    volatile uint32_t seq __attribute__((aligned(4)));
    SensorData data;
} __attribute__((aligned(32)));

SeqlockBuffer shared __attribute__((aligned(32)));
SensorData last_valid_snapshot;

volatile uint32_t frame_index = 0;
volatile uint32_t last_isr_time_us = 0;

// ============== CSV RING BUFFER ==============
#define CSV_BUFFER_SIZE 4096

__attribute__((section(".bss.octram"))) __attribute__((aligned(32)))
static CSVSample csv_ring[CSV_BUFFER_SIZE];

volatile uint32_t csv_head = 0;
volatile uint32_t csv_tail = 0;
volatile uint32_t csv_drops = 0;

// ============== STATISTICS ==============

struct Statistics {
    uint32_t isr_count;
    uint32_t isr_overrun;
    uint32_t max_isr_duration_us;

    uint32_t torn_reads;
    uint32_t seqlock_retries;
    uint32_t seqlock_max_retries;
    uint32_t seqlock_reader_giveup;

    uint32_t servo_errors;
    uint32_t servo_timeouts;
    uint32_t servo_checksum_errors;
    uint32_t servo_framing_errors;
    uint32_t servo_resync_count;

    uint32_t cmd_queue_overflows;

    uint32_t csv_bytes_sent;
};

volatile Statistics stats = {0};
uint32_t last_stats_print_ms = 0;

// ============== USB GOAL PARSER ==============
// Non-blocking ASCII line parser for host goal commands
// Protocol: "G,<pos>,<speed>\n" where pos=[0..4095], speed=[0..1023]

struct UsbGoalParser {
    static const uint16_t BUF_SIZE = 256;  // Increased from 64
    char buf[BUF_SIZE];
    uint16_t idx;  // Increased from uint8_t
};

UsbGoalParser usb_goal = {{0}, 0};

// ============== ISR FLAGS ==============
volatile bool isr_active = false;
volatile bool logging_active = false;

// ============== INTERVAL TIMER ==============
IntervalTimer frameTimer;

// ============== FORWARD DECLARATIONS ==============
void frame_isr();
void servo_write_scheduler();
void usb_goal_service();
void process_usb_line(const char* line);
SensorData seqlock_read();
void csv_push_sample(const CSVSample& sample);
bool csv_buffer_full();
void csv_service();
void print_stats();

// Command queue functions
bool enqueue_goal_cmd(uint8_t id, uint16_t pos, uint16_t time_ms, uint16_t speed);
bool dequeue_goal_cmd(ServoCommand* cmd);
bool cmd_queue_empty();

// Mode management
void mode_service();
void run_service();
void stop_data_mode();
void start_data_mode();
void heartbeat_led_service();

// NOTE: MOVE mode servo operations now provided by ServoHardware module
// - servo.syncWriteFull() replaces sync_write_positions()
// - servo.readPosition() replaces read_servo_position_blocking()

// NOTE: Servo transport layer has been moved to ServoHardware module.
// The module owns Serial3 initialization and all bus operations.

// ============== COMMAND QUEUE FUNCTIONS ==============

bool enqueue_goal_cmd(uint8_t id, uint16_t pos, uint16_t time_ms, uint16_t speed) {
    uint8_t next = (cmd_queue.head + 1) & CMD_QUEUE_MASK;
    if (next == cmd_queue.tail) {
        stats.cmd_queue_overflows++;
        return false;  // Drop newest
    }
    cmd_queue.buf[cmd_queue.head] = {id, pos, time_ms, speed};
    cmd_queue.head = next;
    return true;
}

bool dequeue_goal_cmd(ServoCommand* cmd) {
    if (cmd_queue.head == cmd_queue.tail) return false;
    *cmd = cmd_queue.buf[cmd_queue.tail];
    cmd_queue.tail = (cmd_queue.tail + 1) & CMD_QUEUE_MASK;
    return true;
}

bool cmd_queue_empty() {
    return cmd_queue.head == cmd_queue.tail;
}

uint8_t cmd_queue_count() {
    return (cmd_queue.head - cmd_queue.tail) & CMD_QUEUE_MASK;
}

// NOTE: Feetech protocol functions (calculate_checksum, queue_servo_read_request,
// servo_read_service, servo_timeout_service, servo_protocol_fsm, process_servo_frame)
// have been moved to ServoHardware module. All bus operations now go through servo.poll().

// ============== MOVE MODE SERVO FUNCTIONS ==============
// NOTE: Blocking servo operations (sync_write_positions, read_servo_position_blocking)
// have been migrated to ServoHardware module. See ServoHardware.h for API.
// - servo.syncWriteFull() for multi-servo writes
// - servo.readPosition() for single servo reads

// ============== MODE TRANSITION FUNCTIONS ==============

void stop_data_mode() {
    // Disable ISR and logging
    logging_active = false;
    frameTimer.end();

    // Wait for any in-flight ISR to complete (PJRC edge case)
    uint32_t t0 = micros();
    while (isr_active && (micros() - t0 < 2000)) { /* spin */ }

    // Reset ServoHardware async state (flushes RX, clears FSM, resets flags)
    servo.resetAsyncState();
}

void start_data_mode() {
    // Reset timing baseline
    last_isr_time_us = micros();
    frame_index = 0;

    // Clear CSV ring buffer
    csv_head = 0;
    csv_tail = 0;
    csv_drops = 0;

    // Start ISR
    frameTimer.begin(frame_isr, SAMPLE_PERIOD_US);
    frameTimer.priority(32);  // Higher than UART ISR
    logging_active = true;
}

void mode_service() {
    // Edge-triggered: only process if new request pending
    if (!mode_req_pending) return;
    mode_req_pending = false;  // Clear flag immediately to prevent re-processing

    // Stop ISR when leaving DATA_RUN (only if transitioning away)
    if (g_mode == MODE_DATA_RUN && g_mode_req != MODE_DATA_RUN) {
        stop_data_mode();
    }

    #if ENABLE_DEBUG_OUTPUT
    if (g_mode_req != g_mode) {
        Serial.print("[DBG] Mode transition: ");
        Serial.print(mode_str(g_mode));
        Serial.print(" -> ");
        Serial.println(mode_str(g_mode_req));
    } else {
        Serial.print("[DBG] Mode request (already in mode): ");
        Serial.println(mode_str(g_mode));
    }
    #endif

    // Apply requested mode and send OK response (ALWAYS, even if already in that mode)
    // The pending flag ensures this only happens once per request
    switch (g_mode_req) {
        case MODE_WAIT:
            g_mode = MODE_WAIT;
            Serial.println("MODE,WAIT,OK");
            break;

        case MODE_MOVE:
            g_mode = MODE_MOVE;
            Serial.println("MODE,MOVE,OK");
            break;

        case MODE_DATA_IDLE:
            g_mode = MODE_DATA_IDLE;
            Serial.println("MODE,DATA,OK");
            break;

        // NOTE: MODE_DATA_RUN not handled here - run_service() controls it
        default:
            break;
    }
}

// ============== USB COMMAND PARSER ==============

void process_usb_line(const char* line) {
    if (!line || !line[0]) return;

    // ===== Universal Commands (any mode) =====

    // PING - heartbeat
    if (!strcmp(line, "PING")) {
        Serial.print("PONG,");
        Serial.println(mode_str(g_mode));
        return;
    }

    // MODE? - query
    if (!strcmp(line, "MODE?")) {
        Serial.print("MODE,");
        Serial.println(mode_str(g_mode));
        return;
    }

    // MODE,MOVE - enter motion mode
    if (!strcmp(line, "MODE,MOVE")) {
        g_mode_req = MODE_MOVE;
        mode_req_pending = true;
        return;  // mode_service() sends OK
    }

    // MODE,DATA - enter data idle mode (RUN,START activates ISR)
    if (!strcmp(line, "MODE,DATA")) {
        g_mode_req = MODE_DATA_IDLE;
        mode_req_pending = true;
        return;  // mode_service() sends OK
    }

    // RUN,START - start data acquisition (DATA_IDLE -> DATA_RUN)
    if (!strcmp(line, "RUN,START")) {
        if (g_mode != MODE_DATA_IDLE) {
            Serial.println("ERR,RUN_START_ONLY_IN_DATA");
            return;
        }
        run_req_start = true;
        return;  // run_service() sends OK
    }

    // RUN,STOP - stop data acquisition (DATA_RUN -> DATA_IDLE)
    if (!strcmp(line, "RUN,STOP")) {
        if (g_mode != MODE_DATA_RUN) {
            Serial.println("ERR,RUN_STOP_NOT_RUNNING");
            return;
        }
        run_req_stop = true;
        return;  // run_service() sends OK
    }

    // SERVO,<id> - configure stream target servo for G commands
    if (!strncmp(line, "SERVO,", 6)) {
        const char* p = line + 6;
        char* end;
        long id = strtol(p, &end, 10);
        if (end == p || id < 1 || id > 6) {
            Serial.println("ERR,SERVO_ID_INVALID");
            return;
        }
        g_stream_servo_id = (uint8_t)id;
        Serial.print("SERVO,");
        Serial.print(g_stream_servo_id);
        Serial.println(",OK");
        return;
    }

    // S/s - stats (single character only, not SYNCW)
    if ((line[0] == 'S' || line[0] == 's') && line[1] == '\0') {
        print_stats();
        return;
    }

    // ===== Mode-Required Commands =====

    if (g_mode == MODE_WAIT) {
        Serial.println("ERR,SET_MODE_FIRST");
        return;
    }

    // ===== MODE_MOVE Commands =====

    // SYNCW,<n>,<id1>,<pos1>,<time1>,<spd1>,<id2>,<pos2>,<time2>,<spd2>,...
    // Raw SYNC WRITE - Python builds the command
    if (!strncmp(line, "SYNCW,", 6)) {
        if (g_mode != MODE_MOVE) {
            Serial.println("ERR,SYNCW_ONLY_IN_MOVE");
            return;
        }

        const char* p = line + 6;
        char* end;

        // Parse count
        long count = strtol(p, &end, 10);
        if (end == p || count < 1 || count > 6) {
            Serial.println("ERR,PARSE,COUNT");
            return;
        }

        uint8_t ids[6];
        int16_t positions[6];   // int16_t for ServoHardware API
        uint16_t times[6];
        uint16_t speeds[6];
        p = end;

        for (int i = 0; i < count; i++) {
            if (*p != ',') { Serial.println("ERR,PARSE"); return; }
            p++;

            // ID
            long id = strtol(p, &end, 10);
            if (end == p) { Serial.println("ERR,PARSE,ID"); return; }
            ids[i] = (uint8_t)id;
            p = end;

            if (*p != ',') { Serial.println("ERR,PARSE"); return; }
            p++;

            // Position
            long pos = strtol(p, &end, 10);
            if (end == p) { Serial.println("ERR,PARSE,POS"); return; }
            if (pos < 0) pos = 0;
            if (pos > 4095) pos = 4095;
            positions[i] = (int16_t)pos;
            p = end;

            if (*p != ',') { Serial.println("ERR,PARSE"); return; }
            p++;

            // Time (milliseconds)
            long time_ms = strtol(p, &end, 10);
            if (end == p) { Serial.println("ERR,PARSE,TIME"); return; }
            if (time_ms < 0) time_ms = 0;
            if (time_ms > 65535) time_ms = 65535;
            times[i] = (uint16_t)time_ms;
            p = end;

            if (*p != ',') { Serial.println("ERR,PARSE"); return; }
            p++;

            // Speed
            long spd = strtol(p, &end, 10);
            if (end == p) { Serial.println("ERR,PARSE,SPD"); return; }
            if (spd < 0) spd = 0;
            if (spd > 1023) spd = 1023;
            speeds[i] = (uint16_t)spd;
            p = end;
        }

        // Execute SYNC WRITE via ServoHardware module
        bool ok = servo.syncWriteFull(ids, positions, times, speeds, (uint8_t)count);
        Serial.println(ok ? "OK" : "ERR,SYNCW_FAIL");
        return;
    }

    // READJ - read all 6 joint positions via ServoHardware
    if (!strcasecmp(line, "READJ")) {
        if (g_mode != MODE_MOVE) {
            Serial.println("ERR,READJ_ONLY_IN_MOVE");
            return;
        }

        Serial.print("POS");
        for (uint8_t id = 1; id <= 6; id++) {
            int16_t pos = servo.readPosition(id);
            // SERVO_ERROR (-9999) will be printed as-is, host can detect
            Serial.print(",");
            Serial.print(pos);
        }
        Serial.println();
        return;
    }

    // MOVE,<n>,<id1>,<pos1>,<time1>,<spd1>,...,<tol>,<timeout_ms>
    // Blocking move with settle verification
    if (!strncmp(line, "MOVE,", 5)) {
        if (g_mode != MODE_MOVE) {
            Serial.println("ERR,MOVE_ONLY_IN_MOVE");
            return;
        }

        const char* p = line + 5;
        char* end;

        // Parse count
        long count = strtol(p, &end, 10);
        if (end == p || count < 1 || count > 6) {
            Serial.println("ERR,PARSE,COUNT");
            return;
        }

        uint8_t ids[6];
        int16_t positions[6];   // int16_t for ServoHardware API
        uint16_t times[6];
        uint16_t speeds[6];
        p = end;

        // Parse servo parameters
        for (int i = 0; i < count; i++) {
            if (*p != ',') { Serial.println("ERR,PARSE"); return; }
            p++;

            // ID
            long id = strtol(p, &end, 10);
            if (end == p) { Serial.println("ERR,PARSE,ID"); return; }
            ids[i] = (uint8_t)id;
            p = end;

            if (*p != ',') { Serial.println("ERR,PARSE"); return; }
            p++;

            // Position
            long pos = strtol(p, &end, 10);
            if (end == p) { Serial.println("ERR,PARSE,POS"); return; }
            if (pos < 0) pos = 0;
            if (pos > 4095) pos = 4095;
            positions[i] = (int16_t)pos;
            p = end;

            if (*p != ',') { Serial.println("ERR,PARSE"); return; }
            p++;

            // Time (milliseconds)
            long time_ms = strtol(p, &end, 10);
            if (end == p) { Serial.println("ERR,PARSE,TIME"); return; }
            if (time_ms < 0) time_ms = 0;
            if (time_ms > 65535) time_ms = 65535;
            times[i] = (uint16_t)time_ms;
            p = end;

            if (*p != ',') { Serial.println("ERR,PARSE"); return; }
            p++;

            // Speed
            long spd = strtol(p, &end, 10);
            if (end == p) { Serial.println("ERR,PARSE,SPD"); return; }
            if (spd < 0) spd = 0;
            if (spd > 1023) spd = 1023;
            speeds[i] = (uint16_t)spd;
            p = end;
        }

        // Parse tolerance (encoder units)
        if (*p != ',') { Serial.println("ERR,PARSE"); return; }
        p++;
        long tol = strtol(p, &end, 10);
        if (end == p) { Serial.println("ERR,PARSE,TOL"); return; }
        if (tol < 1) tol = 1;
        if (tol > 200) tol = 200;
        uint16_t tolerance = (uint16_t)tol;
        p = end;

        // Parse timeout (milliseconds)
        if (*p != ',') { Serial.println("ERR,PARSE"); return; }
        p++;
        long timeout = strtol(p, &end, 10);
        if (end == p) { Serial.println("ERR,PARSE,TIMEOUT"); return; }
        if (timeout < 100) timeout = 100;
        if (timeout > 30000) timeout = 30000;
        uint32_t timeout_ms = (uint32_t)timeout;

        // Execute SYNC WRITE via ServoHardware module
        bool ok = servo.syncWriteFull(ids, positions, times, speeds, (uint8_t)count);
        if (!ok) {
            Serial.println("ERR,SYNCW_FAIL");
            return;
        }

        // Find max time to estimate settle duration
        uint16_t max_time = 0;
        for (int i = 0; i < count; i++) {
            if (times[i] > max_time) max_time = times[i];
        }

        // Wait for estimated move time (80% of max time)
        if (max_time > 0) {
            delay((max_time * 8) / 10);
        } else {
            delay(50);  // Small delay for immediate moves
        }

        // Poll until settled or timeout
        uint32_t start_ms = millis();
        uint32_t stable_start_ms = 0;
        const uint32_t STABLE_DURATION_MS = 50;  // Require 50ms stability
        bool ever_settled = false;

        while (millis() - start_ms < timeout_ms) {
            // Read all specified servos via ServoHardware
            bool all_settled = true;
            for (int i = 0; i < count; i++) {
                int16_t current_pos = servo.readPosition(ids[i]);
                if (current_pos == ServoHardware::SERVO_ERROR) {
                    // Read error (timeout or invalid response)
                    Serial.println("ERR,READ_FAIL");
                    return;
                }

                int16_t error = abs(positions[i] - current_pos);
                if (error > (int16_t)tolerance) {
                    all_settled = false;
                    break;
                }
            }

            if (all_settled) {
                if (stable_start_ms == 0) {
                    stable_start_ms = millis();
                    ever_settled = true;
                } else if (millis() - stable_start_ms >= STABLE_DURATION_MS) {
                    // Settled for required duration
                    Serial.println("OK");
                    return;
                }
            } else {
                stable_start_ms = 0;  // Reset stability timer
            }

            delay(10);  // Poll at ~100Hz
        }

        // Timeout
        Serial.println("ERR,TIMEOUT");
        return;
    }

    // ===== MODE_DATA_RUN Commands =====

    // G,<pos>,<time_ms>,<speed> - trajectory goal streaming with time-based interpolation
    // time_ms: servo internal interpolation time (0 = immediate mode)

    // TRAJ - Bulk load trajectory waypoints: TRAJ,<count>,<pos1>,<time1>,<speed1>,...
    if (strncmp(line, "TRAJ,", 5) == 0) {
        if (g_mode != MODE_DATA_IDLE && g_mode != MODE_DATA_RUN) {
            Serial.println("ERR,TRAJ_WRONG_MODE");
            return;
        }

        const char* p = line + 5;
        char* end;
        int count = strtol(p, &end, 10);
        if (end == p || count <= 0) {
            Serial.println("ERR,TRAJ_BAD_COUNT");
            return;
        }

        int loaded = 0;
        p = end;
        for (int i = 0; i < count; i++) {
            if (*p != ',') break;
            p++;

            long pos = strtol(p, &end, 10);
            if (end == p || *end != ',') break;
            p = end + 1;

            long time_ms = strtol(p, &end, 10);
            if (end == p || *end != ',') break;
            p = end + 1;

            long speed = strtol(p, &end, 10);
            if (end == p) break;
            p = end;

            if (!enqueue_goal_cmd(g_stream_servo_id, (uint16_t)pos, (uint16_t)time_ms, (uint16_t)speed)) {
                break;  // Queue full
            }
            loaded++;
        }

        Serial.print("TRAJ,OK,");
        Serial.println(loaded);
        return;
    }

    if (line[0] == 'G' || line[0] == 'g') {
        if (g_mode != MODE_DATA_RUN) {
            Serial.println("ERR,G_ONLY_IN_DATA");
            return;
        }

        const char* p = line + 1;
        if (*p == ',') ++p;

        char* end;
        long pos = strtol(p, &end, 10);
        if (end == p || *end != ',') return;

        p = end + 1;
        long time_ms = strtol(p, &end, 10);
        if (end == p || *end != ',') return;

        long speed = strtol(end + 1, &end, 10);

        if (pos < 0) pos = 0;
        if (pos > 4095) pos = 4095;
        if (time_ms < 0) time_ms = 0;
        if (time_ms > 65535) time_ms = 65535;
        if (speed < 0) speed = 0;
        if (speed > 1023) speed = 1023;

        enqueue_goal_cmd(g_stream_servo_id, (uint16_t)pos, (uint16_t)time_ms, (uint16_t)speed);
        return;
    }

    // Unknown command
    Serial.println("ERR,UNKNOWN_CMD");
}

void usb_goal_service() {
    const int MAX_BYTES_PER_LOOP = 128;  // Increased from 32
    int processed = 0;

    while (Serial.available() > 0 && processed < MAX_BYTES_PER_LOOP) {
        char c = (char)Serial.read();
        processed++;

        if (c == '\r') continue;

        if (c == '\n') {
            usb_goal.buf[usb_goal.idx] = '\0';
            usb_goal.idx = 0;
            process_usb_line(usb_goal.buf);
        } else {
            if (usb_goal.idx < UsbGoalParser::BUF_SIZE - 1) {
                usb_goal.buf[usb_goal.idx++] = c;
            } else {
                // Overflow -> drop line
                usb_goal.idx = 0;
            }
        }
    }
}

// ============== SERVO WRITE SCHEDULER ==============
// Dequeues commands and sends via ServoHardware

void servo_write_scheduler() {
    // Check command queue
    ServoCommand cmd;
    if (!dequeue_goal_cmd(&cmd)) return;

    // Send via ServoHardware (handles rate limiting and bus state)
    servo.writeGoal(cmd.servo_id, (int16_t)cmd.goal_position,
                    cmd.goal_time_ms, cmd.goal_speed);

    // Capture command for logging
    g_last_cmd_time_us = micros();
    g_last_cmd_goal = cmd.goal_position;
    g_last_cmd_time_ms = cmd.goal_time_ms;
}

// ============== SEQLOCK READER ==============

SensorData seqlock_read() {
    SensorData snapshot;
    uint32_t s1, s2 = 0;
    uint8_t retries = 0;
    const uint8_t MAX_RETRIES = 5;

    do {
        s1 = shared.seq;
        SEQLOCK_BARRIER();

        if (s1 & 1) {
            retries++;
            if (retries >= MAX_RETRIES) {
                stats.seqlock_reader_giveup++;
                snapshot = last_valid_snapshot;
                snapshot.coherency_flag = 0xEE;
                return snapshot;
            }
            delayMicroseconds(1);
            continue;
        }

        snapshot = shared.data;
        SEQLOCK_BARRIER();
        s2 = shared.seq;
        retries++;

    } while (s1 != s2 && retries < MAX_RETRIES);

    // Verify checksum
    uint32_t accel_bits;
    memcpy(&accel_bits, &snapshot.accel_x_mps2, sizeof(uint32_t));
    uint32_t expected = snapshot.frame_ts_us ^ snapshot.frame_index ^ accel_bits;

    if (snapshot.checksum != expected || snapshot.coherency_flag != 0xAA) {
        stats.torn_reads++;
        snapshot.coherency_flag = 0xFF;
    } else {
        last_valid_snapshot = snapshot;
    }

    stats.seqlock_retries += retries;
    if (retries > stats.seqlock_max_retries) {
        stats.seqlock_max_retries = retries;
    }

    return snapshot;
}

// ============== FRAME ISR @ 500Hz (IMU FIRST) ==============

void frame_isr() {
    if (!logging_active) return;

    if (isr_active) {
        stats.isr_overrun++;
        return;
    }
    isr_active = true;

    uint32_t isr_start = micros();
    uint32_t expected_time = last_isr_time_us + SAMPLE_PERIOD_US;

    // ===== SEQLOCK WRITE BEGIN =====
    shared.seq++;
    SEQLOCK_BARRIER();

    SensorData& d = shared.data;
    d.frame_ts_us = isr_start;
    d.frame_index = frame_index++;

    // ---- 1) IMU READ FIRST (bare-metal SPI, ~33-130us) ----
    uint32_t imu_start = micros();
    ISM330::RawSample raw;
    ISM330::readRaw(raw);
    uint32_t imu_end = micros();

    d.accel_x_mps2 = raw.ax * ACCEL_SENS;
    d.accel_y_mps2 = raw.ay * ACCEL_SENS;
    d.accel_z_mps2 = raw.az * ACCEL_SENS;
    d.gyro_x_rads = raw.gx * GYRO_SENS;
    d.gyro_y_rads = raw.gy * GYRO_SENS;
    d.gyro_z_rads = raw.gz * GYRO_SENS;
    d.imu_read_us = imu_end - imu_start;

    // ---- 2) QUEUE SERVO TX (via ServoHardware) ----
    servo.queueRead(g_stream_servo_id);

    // ---- 3) SNAPSHOT SERVO STATE (via ServoHardware) ----
    {
        ServoHardware::Snapshot snap = servo.snapshot();

        d.servo_position = snap.position;

        // Pack queue depth (upper 4 bits) + servo error (lower 4 bits)
        // Queue depth 0-15 (capped), error code 0-3
        uint8_t q_depth = cmd_queue_count();
        if (q_depth > 15) q_depth = 15;
        d.servo_error_code = (q_depth << 4) | (snap.error_code & 0x0F);

        if (snap.t_rx_us != 0 && snap.t_rx_us <= d.frame_ts_us && snap.error_code == 0) {
            d.servo_age_us = (uint16_t)(d.frame_ts_us - snap.t_rx_us);
            d.servo_latency_us = (uint16_t)(snap.t_rx_us - snap.t_req_us);
        } else {
            d.servo_age_us = 0xFFFF;
            d.servo_latency_us = 0;
        }
    }

    // ---- 4) TIMING DIAGNOSTICS ----
    uint32_t isr_end = micros();
    d.isr_total_us = isr_end - isr_start;
    d.period_error_us = (int16_t)(isr_start - expected_time);

    // ---- 4.5) COMMAND TRACKING ----
    d.cmd_goal = g_last_cmd_goal;
    d.cmd_time_us = g_last_cmd_time_us;

    // ---- 5) COHERENCY CHECKSUM ----
    uint32_t accel_bits;
    memcpy(&accel_bits, &d.accel_x_mps2, sizeof(uint32_t));
    d.checksum = d.frame_ts_us ^ d.frame_index ^ accel_bits;
    d.coherency_flag = 0xAA;

    // ===== SEQLOCK WRITE END =====
    SEQLOCK_BARRIER();
    shared.seq++;

    last_isr_time_us = isr_start;

    uint32_t isr_duration = micros() - isr_start;
    if (isr_duration > stats.max_isr_duration_us) {
        stats.max_isr_duration_us = isr_duration;
    }

    stats.isr_count++;
    isr_active = false;
}

// ============== CSV RING BUFFER ==============

bool csv_buffer_full() {
    return ((csv_head + 1) % CSV_BUFFER_SIZE) == csv_tail;
}

void csv_push_sample(const CSVSample& sample) {
    if (csv_buffer_full()) {
        csv_drops++;
        return;
    }
    csv_ring[csv_head] = sample;
    csv_head = (csv_head + 1) % CSV_BUFFER_SIZE;
}

void csv_service() {
    const uint32_t MAX_DRAIN = 4;
    const uint32_t MAX_BUDGET_US = 500;
    uint32_t start_us = micros();
    uint32_t drained = 0;

    while (csv_tail != csv_head &&
           drained < MAX_DRAIN &&
           (micros() - start_us) < MAX_BUDGET_US) {

        CSVSample& sample = csv_ring[csv_tail];
        size_t written = Serial4.write((uint8_t*)&sample, sizeof(CSVSample));
        stats.csv_bytes_sent += written;

        // S2B: Signal first successful write for EVT,LOGGING_ON
        if (written > 0) {
            logging_has_written_any = true;
        }

        csv_tail = (csv_tail + 1) % CSV_BUFFER_SIZE;
        drained++;
    }
}

// ============== HEARTBEAT LED ==============

void heartbeat_led_service() {
    static uint32_t last_blink = 0;
    uint32_t interval;

    // Different blink rates per mode for visual feedback
    switch (g_mode) {
        case MODE_WAIT:        interval = 1000; break;  // Slow (1 Hz)
        case MODE_MOVE:        interval = 250;  break;  // Fast (4 Hz)
        case MODE_DATA_IDLE:  interval = 500;  break;  // Medium (2 Hz)
        case MODE_DATA_RUN: interval = 100;  break;  // Very fast (10 Hz)
        default:               interval = 500;
    }

    if (millis() - last_blink >= interval) {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

// ============== STATISTICS ==============

void print_stats() {
    uint32_t now = millis();
    if (now - last_stats_print_ms < 5000) return;
    last_stats_print_ms = now;

    uint32_t buffer_used = (csv_head >= csv_tail)
        ? (csv_head - csv_tail)
        : (CSV_BUFFER_SIZE - csv_tail + csv_head);
    float buffer_pct = (buffer_used * 100.0) / CSV_BUFFER_SIZE;

    Serial.println("========== PHASE 5 STATS ==========");
    Serial.print("Mode: "); Serial.println(mode_str(g_mode));
    Serial.print("Frames: "); Serial.println(stats.isr_count);
    Serial.print("Drops: "); Serial.println(csv_drops);
    Serial.print("Overruns: "); Serial.println(stats.isr_overrun);
    Serial.print("Max ISR: "); Serial.print(stats.max_isr_duration_us); Serial.println(" us");
    Serial.print("Buffer: "); Serial.print(buffer_pct, 1); Serial.println(" %");

    Serial.println("--- Seqlock ---");
    Serial.print("Torn reads: "); Serial.println(stats.torn_reads);
    Serial.print("Max retries: "); Serial.println(stats.seqlock_max_retries);

    Serial.println("--- Servo ---");
    Serial.print("Errors: "); Serial.println(stats.servo_errors);
    Serial.print("Timeouts: "); Serial.println(stats.servo_timeouts);
    Serial.print("Checksum: "); Serial.println(stats.servo_checksum_errors);
    Serial.print("Framing: "); Serial.println(stats.servo_framing_errors);
    Serial.print("Resyncs: "); Serial.println(stats.servo_resync_count);

    Serial.println("--- Command Queue ---");
    Serial.print("Overflows: "); Serial.println(stats.cmd_queue_overflows);

    Serial.println("====================================");
}

// ============== SETUP ==============

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // Debug serial (USB)
    Serial.begin(115200);
    delay(100);

    #if ENABLE_DEBUG_OUTPUT
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("LeRobot RM Data Acquisition - Phase 5");
    Serial.println("Mode Manager Architecture");
    Serial.println("========================================");
    #endif

    // CSV output serial (data plane)
    Serial4.begin(SERIAL_CSV_BAUD);
    delay(100);

    // Servo bus (ServoHardware owns Serial3 initialization)
    servo.init(Serial3);
    #if ENABLE_DEBUG_OUTPUT
    Serial.println("ServoHardware initialized @ 1 Mbaud");
    #endif

    // Initialize IMU
    ISM330::Config imu_cfg;
    imu_cfg.spi = &SPI;
    imu_cfg.cs_pin = IMU_CS;
    imu_cfg.spi_hz = 4000000;

    if (!ISM330::init(imu_cfg)) {
        #if ENABLE_DEBUG_OUTPUT
        Serial.println("[FATAL] IMU init failed");
        #endif
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(500);
        }
    }
    delay(5);

    #if ENABLE_DEBUG_OUTPUT
    Serial.println("ISM330DHCX initialized");
    #endif

    // ========== MODE MANAGER INIT (no blocking handshake) ==========
    logging_active = false;
    g_mode = MODE_WAIT;
    g_mode_req = MODE_WAIT;

    // Initialize timing (for when DATA mode starts later)
    last_isr_time_us = micros();

    // NOTE: frameTimer NOT started here - mode_service() handles it

    #if ENABLE_DEBUG_OUTPUT
    Serial.println("");
    Serial.println("HELLO,PHASE5_S2B");
    Serial.println("Commands:");
    Serial.println("  PING              - heartbeat");
    Serial.println("  MODE?             - query mode");
    Serial.println("  MODE,MOVE         - motion mode (ISR off)");
    Serial.println("  MODE,DATA         - data idle (no frames until RUN,START)");
    Serial.println("  RUN,START         - start ISR + Serial4 streaming");
    Serial.println("  RUN,STOP          - stop ISR + streaming");
    Serial.println("  SYNCW,n,id,p,t,s,.. - sync write (MOVE)");
    Serial.println("  READJ             - read 6 joints (MOVE)");
    Serial.println("  MOVE,n,id,p,t,s,..,tol,timeout - blocking move (MOVE)");
    Serial.println("  G,pos,spd         - goal stream (DATA_RUN)");
    Serial.println("Events: EVT,LOGGING_ON / EVT,LOGGING_OFF");
    Serial.println("");
    Serial.println("Waiting for MODE command...");
    #endif
}

// ============== RUN SERVICE (S2B ISR control) ==============

void run_service() {
    // Handle RUN,START request
    if (run_req_start) {
        run_req_start = false;

        if (g_mode != MODE_DATA_IDLE) {
            Serial.println("ERR,RUN_START_ONLY_IN_DATA");
            return;
        }

        start_data_mode();
        g_mode = MODE_DATA_RUN;

        logging_has_written_any = false;
        evt_logging_on_pending = true;
        Serial.println("RUN,START,OK");
        return;
    }

    // Handle RUN,STOP request
    if (run_req_stop) {
        run_req_stop = false;

        if (g_mode != MODE_DATA_RUN) {
            Serial.println("ERR,RUN_STOP_NOT_RUNNING");
            return;
        }

        stop_data_mode();
        g_mode = MODE_DATA_IDLE;

        evt_logging_off_pending = true;
        Serial.println("RUN,STOP,OK");
        return;
    }
}

// ============== MAIN LOOP ==============

void loop() {
    // ===== ALWAYS: USB Command Service =====
    usb_goal_service();

    // ===== ALWAYS: Mode Transition Service =====
    // +++ Sets flag to be for function to be called in run_service ++++
    mode_service();

    // ===== ALWAYS: Run Service ( ISR control) =====
    run_service();

    // ===== MODE-SPECIFIC SERVICES =====
    switch (g_mode) {
        case MODE_WAIT:
            // Idle - only USB commands processed
            break;

        case MODE_MOVE:
            // Motion mode: blocking RPC operations via ServoHardware
            // No async servicing needed - blocking calls handle everything
            break;

        case MODE_DATA_IDLE:
            // Data mode idle - no ISR, no Serial4 frames
            break;

        case MODE_DATA_RUN:
            // Full 500Hz data acquisition pipeline
            // ServoHardware handles: RX parsing, timeouts, queued TX
            servo.poll();
            servo_write_scheduler();

            // Snapshot and log
            {
                SensorData snapshot = seqlock_read();
                static uint32_t last_frame = UINT32_MAX;
                if (snapshot.frame_index != last_frame) {
                    csv_push_sample(snapshot);
                    last_frame = snapshot.frame_index;
                }
            }
            csv_service();

            // S2B: emit EVT after first Serial4 write
            if (evt_logging_on_pending && logging_has_written_any) {
                Serial.println("EVT,LOGGING_ON");
                evt_logging_on_pending = false;
            }
            break;
    }

    // S2B: emit OFF event outside switch
    if (evt_logging_off_pending) {
        Serial.println("EVT,LOGGING_OFF");
        evt_logging_off_pending = false;
    }

    // ===== ALWAYS: Heartbeat LED =====
    heartbeat_led_service();

    // ===== PERIODIC: Stats (debug mode, DATA_RUN only) =====
    #if ENABLE_DEBUG_OUTPUT
    if (g_mode == MODE_DATA_RUN) {
        print_stats();
    }
    #endif
}
