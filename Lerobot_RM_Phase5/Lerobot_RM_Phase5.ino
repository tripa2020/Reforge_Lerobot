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
#define SERVO_BAUD 1000000        // 1 Mbps (Feetech default)
#define SERVO_ID 6                // Default servo for G command (gripper)
#define SERVO_TIMEOUT_US 800      // Response timeout

// Feetech Protocol
#define FEETECH_HEADER_1 0xFF
#define FEETECH_HEADER_2 0xFF
#define INSTR_READ_DATA 0x02
#define INSTR_WRITE_DATA 0x03
#define INSTR_SYNC_WRITE 0x83
#define REG_PRESENT_POSITION_L 0x38
#define REG_GOAL_POSITION_L 0x2A

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

// ============== SERVO STATE (FSM -> Frame ISR) ==============

struct ServoState {
    volatile int32_t  position;       // Last encoder position (0-4095, or -1 on error)
    volatile uint32_t t_req_us;       // When READ request was sent
    volatile uint32_t t_rx_us;        // When full response was parsed
    volatile uint8_t  error_code;     // 0=OK, 1=timeout, 2=checksum, 3=framing
    volatile uint32_t generation;     // Coherency counter (increment LAST)
};

// Global shared servo state
volatile ServoState latest_servo = {
    -1,       // position: invalid
    0,        // t_req_us
    0,        // t_rx_us
    0xFF,     // error_code: not initialized
    0         // generation
};

// Command tracking (for logging commanded vs actual position)
volatile uint32_t g_last_cmd_time_us = 0;
volatile uint16_t g_last_cmd_goal = 0xFFFF;  // Sentinel: no command sent yet

// ============== SERVO RX FSM ==============

enum ServoRxState {
    RX_WAIT_HEADER1,
    RX_WAIT_HEADER2,
    RX_WAIT_ID,
    RX_WAIT_LEN,
    RX_WAIT_BODY
};

struct ServoRxFSM {
    ServoRxState state;
    uint8_t buf[16];
    uint8_t idx;
    uint8_t expected_len;
};

ServoRxFSM servo_rx = {RX_WAIT_HEADER1, {0}, 0, 0};

// ============== SERVO READ STATE (Frame ISR sets flag, main loop sends) ==============

struct ServoReadState {
    volatile bool     request_in_flight;   // True if waiting for response
    volatile bool     request_queued;      // Frame ISR asked for a READ
    volatile uint32_t current_t_req_us;    // When we *actually* sent READ
};

ServoReadState servo_read = {false, false, 0};

// ============== SERVO BUS MANAGER (for writes + timeout) ==============

enum ServoBusState {
    BUS_IDLE,
    BUS_READ_WAIT,
    BUS_WRITE_WAIT
};

struct ServoBusManager {
    ServoBusState state;
    uint32_t last_write_us;
    uint32_t write_request_sent_us;  // For write timeout tracking
};

ServoBusManager servo_bus = {BUS_IDLE, 0, 0};

const uint32_t SERVO_WRITE_INTERVAL_US = 10000; // 100 Hz max

// ============== COMMAND QUEUE ==============

#define CMD_QUEUE_SIZE 16
#define CMD_QUEUE_MASK (CMD_QUEUE_SIZE - 1)

struct ServoCommand {
    uint8_t servo_id;
    uint16_t goal_position;
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
void servo_timeout_service();
void servo_protocol_fsm();
void servo_read_service();
void servo_write_scheduler();
void usb_goal_service();
void process_usb_line(const char* line);
void process_servo_frame(const uint8_t* buf, uint8_t len, uint32_t t_rx);
uint8_t calculate_checksum(const uint8_t* data, uint8_t len);
bool queue_servo_read_request();
SensorData seqlock_read();
void csv_push_sample(const CSVSample& sample);
bool csv_buffer_full();
void csv_service();
void print_stats();

// Command queue functions
bool enqueue_goal_cmd(uint8_t id, uint16_t pos, uint16_t speed);
bool dequeue_goal_cmd(ServoCommand* cmd);
bool cmd_queue_empty();

// Mode management
void mode_service();
void run_service();
void stop_data_mode();
void start_data_mode();
void heartbeat_led_service();

// MOVE mode servo operations (blocking)
bool sync_write_positions(const uint8_t* ids, const uint16_t* positions,
                          const uint16_t* speeds, uint8_t count);
int16_t read_servo_position_blocking(uint8_t id);

// ============== SERVO TRANSPORT (Serial3-backed) ==============

static inline uint8_t servo_transport_write(const uint8_t* data, uint8_t len) {
    size_t written = Serial3.write(data, len);
    Serial3.flush();  // Block until TX complete (required for half-duplex timing)
    return (uint8_t)written;
}

static inline int servo_transport_read_byte() {
    if (Serial3.available() > 0) {
        return Serial3.read();
    }
    return -1;
}

static inline void servo_transport_flush_rx() {
    while (Serial3.available() > 0) {
        Serial3.read();
    }
}

// ============== COMMAND QUEUE FUNCTIONS ==============

bool enqueue_goal_cmd(uint8_t id, uint16_t pos, uint16_t speed) {
    uint8_t next = (cmd_queue.head + 1) & CMD_QUEUE_MASK;
    if (next == cmd_queue.tail) {
        stats.cmd_queue_overflows++;
        return false;  // Drop newest
    }
    cmd_queue.buf[cmd_queue.head] = {id, pos, speed};
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

// ============== FEETECH PROTOCOL ==============

uint8_t calculate_checksum(const uint8_t* data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {
        sum += data[i];
    }
    return ~sum;
}

// ============== SERVO READ REQUEST (Called from Frame ISR) ==============

bool queue_servo_read_request() {
    if (servo_read.request_in_flight) {
        return false;
    }

    if (servo_bus.state == BUS_WRITE_WAIT) {
        return false;
    }

    servo_read.request_queued = true;
    return true;
}

// ============== SERVO READ SERVICE (Called from Main Loop) ==============

void servo_read_service() {
    if (!servo_read.request_queued) return;
    if (servo_read.request_in_flight) return;
    if (servo_bus.state == BUS_WRITE_WAIT) return;

    // Build READ_POSITION request (8 bytes)
    uint8_t req[8];
    req[0] = FEETECH_HEADER_1;
    req[1] = FEETECH_HEADER_2;
    req[2] = SERVO_ID;
    req[3] = 0x04;  // Length
    req[4] = INSTR_READ_DATA;
    req[5] = REG_PRESENT_POSITION_L;
    req[6] = 0x02;  // Read 2 bytes
    req[7] = calculate_checksum(req, 8);

    uint8_t sent = servo_transport_write(req, 8);
    if (sent < 8) {
        return;
    }

    servo_read.current_t_req_us = micros();
    servo_read.request_in_flight = true;
    servo_read.request_queued = false;
    servo_bus.state = BUS_READ_WAIT;
}

// ============== SERVO TIMEOUT SERVICE ==============

void servo_timeout_service() {
    uint32_t now_us = micros();

    // Check read timeout
    if (servo_read.request_in_flight) {
        uint32_t elapsed = now_us - servo_read.current_t_req_us;

        if (elapsed > SERVO_TIMEOUT_US) {
            latest_servo.error_code = 1;  // Timeout
            SEQLOCK_BARRIER();
            latest_servo.generation++;

            stats.servo_timeouts++;
            stats.servo_errors++;

            servo_read.request_in_flight = false;
            servo_bus.state = BUS_IDLE;
            servo_rx.state = RX_WAIT_HEADER1;
        }
    }

    // Check write timeout
    if (servo_bus.state == BUS_WRITE_WAIT) {
        uint32_t elapsed = now_us - servo_bus.write_request_sent_us;

        if (elapsed > SERVO_TIMEOUT_US) {
            servo_bus.state = BUS_IDLE;
            servo_rx.state = RX_WAIT_HEADER1;
        }
    }
}

// ============== SERVO PROTOCOL FSM ==============

void servo_protocol_fsm() {
    int byte;

    while ((byte = servo_transport_read_byte()) >= 0) {
        switch (servo_rx.state) {

        case RX_WAIT_HEADER1:
            if (byte == FEETECH_HEADER_1) {
                servo_rx.buf[0] = byte;
                servo_rx.state = RX_WAIT_HEADER2;
            }
            break;

        case RX_WAIT_HEADER2:
            if (byte == FEETECH_HEADER_2) {
                servo_rx.buf[1] = byte;
                servo_rx.state = RX_WAIT_ID;
            } else {
                servo_rx.state = RX_WAIT_HEADER1;
                stats.servo_resync_count++;
            }
            break;

        case RX_WAIT_ID:
            servo_rx.buf[2] = byte;
            servo_rx.state = RX_WAIT_LEN;
            break;

        case RX_WAIT_LEN:
            servo_rx.buf[3] = byte;
            servo_rx.expected_len = byte;
            servo_rx.idx = 4;
            servo_rx.state = RX_WAIT_BODY;
            break;

        case RX_WAIT_BODY:
            servo_rx.buf[servo_rx.idx++] = byte;
            if (servo_rx.idx >= (uint8_t)(servo_rx.expected_len + 4)) {
                uint32_t t_rx = micros();
                process_servo_frame(servo_rx.buf, servo_rx.idx, t_rx);
                servo_rx.state = RX_WAIT_HEADER1;
            }
            break;
        }
    }
}

// ============== PROCESS SERVO FRAME ==============

void process_servo_frame(const uint8_t* buf, uint8_t len, uint32_t t_rx) {
    // Check if this is a WRITE ACK (6 bytes, len field = 0x02)
    if (len == 6 && buf[3] == 0x02) {
        servo_bus.state = BUS_IDLE;
        return;
    }

    bool valid = true;
    uint8_t error_code = 0;

    if (len < 6) {
        valid = false;
        error_code = 3;  // Framing
    }

    if (valid && buf[2] != SERVO_ID) {
        valid = false;
        error_code = 3;  // Wrong servo
    }

    if (valid) {
        uint8_t calc = calculate_checksum(buf, len);
        if (buf[len - 1] != calc) {
            valid = false;
            error_code = 2;  // Checksum
            stats.servo_checksum_errors++;
        }
    }

    if (valid && buf[4] != 0x00) {
        valid = false;
        error_code = 3;  // Servo reported error
    }

    if (valid && len >= 8) {
        int32_t pos = buf[5] | (buf[6] << 8);

        latest_servo.position = pos;
        latest_servo.t_req_us = servo_read.current_t_req_us;
        latest_servo.t_rx_us = t_rx;
        latest_servo.error_code = 0;
        SEQLOCK_BARRIER();
        latest_servo.generation++;
    } else {
        if (error_code == 0) error_code = 3;
        latest_servo.error_code = error_code;
        SEQLOCK_BARRIER();
        latest_servo.generation++;

        stats.servo_errors++;
        if (error_code == 3) stats.servo_framing_errors++;
    }

    servo_read.request_in_flight = false;
    servo_bus.state = BUS_IDLE;
}

// ============== MOVE MODE SERVO FUNCTIONS ==============

// SYNC WRITE to multiple servos
// Packet: FF FF FE LEN INSTR ADDR DATA_LEN [ID1 D1 D2 D3 D4] [ID2...] CHECKSUM
bool sync_write_positions(const uint8_t* ids, const uint16_t* positions,
                          const uint16_t* speeds, uint8_t count) {
    if (count == 0 || count > 6) return false;

    // Feetech SYNC_WRITE to 0x2A requires 6 bytes per servo:
    // 0x2A-0x2B: Goal Position (2 bytes)
    // 0x2C-0x2D: Goal Time (2 bytes, 0=immediate)
    // 0x2E-0x2F: Goal Speed (2 bytes)
    const uint8_t DATA_LEN = 6;
    uint8_t pkt_len = (DATA_LEN + 1) * count + 4;
    uint8_t pkt[64];

    pkt[0] = FEETECH_HEADER_1;
    pkt[1] = FEETECH_HEADER_2;
    pkt[2] = 0xFE;  // Broadcast ID
    pkt[3] = pkt_len;
    pkt[4] = INSTR_SYNC_WRITE;
    pkt[5] = REG_GOAL_POSITION_L;
    pkt[6] = DATA_LEN;

    uint8_t idx = 7;
    for (uint8_t i = 0; i < count; i++) {
        pkt[idx++] = ids[i];
        pkt[idx++] = positions[i] & 0xFF;         // 0x2A: pos_lo
        pkt[idx++] = (positions[i] >> 8) & 0xFF;  // 0x2B: pos_hi
        pkt[idx++] = 0x00;                        // 0x2C: time_lo (0=immediate)
        pkt[idx++] = 0x00;                        // 0x2D: time_hi
        pkt[idx++] = speeds[i] & 0xFF;            // 0x2E: speed_lo
        pkt[idx++] = (speeds[i] >> 8) & 0xFF;     // 0x2F: speed_hi
    }

    // Checksum (sum bytes 2..idx-1, invert)
    uint8_t sum = 0;
    for (uint8_t i = 2; i < idx; i++) sum += pkt[i];
    pkt[idx++] = ~sum;

    servo_transport_flush_rx();
    uint8_t sent = servo_transport_write(pkt, idx);

    // SYNC WRITE broadcast has no response
    return (sent == idx);
}

// Read single servo position (blocking, called sequentially for READJ)
int16_t read_servo_position_blocking(uint8_t id) {
    uint8_t req[8];
    req[0] = FEETECH_HEADER_1;
    req[1] = FEETECH_HEADER_2;
    req[2] = id;
    req[3] = 0x04;
    req[4] = INSTR_READ_DATA;
    req[5] = REG_PRESENT_POSITION_L;
    req[6] = 0x02;
    req[7] = calculate_checksum(req, 8);

    servo_transport_flush_rx();
    servo_transport_write(req, 8);

    // Wait for response (blocking OK in MOVE mode)
    uint32_t start = micros();
    uint8_t buf[16];
    uint8_t idx = 0;

    while ((micros() - start) < 2000 && idx < 8) {
        if (Serial3.available()) {
            buf[idx++] = Serial3.read();
        }
    }

    // Parse: FF FF ID LEN ERR POS_L POS_H CHECKSUM
    if (idx >= 8 && buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == id) {
        uint8_t calc = calculate_checksum(buf, 8);
        if (buf[7] == calc && buf[4] == 0x00) {  // No error
            return (int16_t)(buf[5] | (buf[6] << 8));
        }
    }
    return -1;  // Error or timeout
}

// ============== MODE TRANSITION FUNCTIONS ==============

void stop_data_mode() {
    // Disable ISR and logging
    logging_active = false;
    frameTimer.end();

    // Wait for any in-flight ISR to complete (PJRC edge case)
    uint32_t t0 = micros();
    while (isr_active && (micros() - t0 < 2000)) { /* spin */ }

    // Purge servo bus state
    servo_transport_flush_rx();
    delayMicroseconds(50);  // Allow in-flight bytes to arrive
    servo_transport_flush_rx();

    // Reset servo FSM state
    servo_read.request_in_flight = false;
    servo_read.request_queued = false;
    servo_bus.state = BUS_IDLE;
    servo_rx.state = RX_WAIT_HEADER1;
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

    // SYNCW,<n>,<id1>,<pos1>,<spd1>,<id2>,<pos2>,<spd2>,...
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
        uint16_t positions[6];
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
            positions[i] = (uint16_t)pos;
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

        // Execute SYNC WRITE
        bool ok = sync_write_positions(ids, positions, speeds, (uint8_t)count);
        Serial.println(ok ? "OK" : "ERR,SYNCW_FAIL");
        return;
    }

    // READJ - read all 6 joint positions
    if (!strcasecmp(line, "READJ")) {
        if (g_mode != MODE_MOVE) {
            Serial.println("ERR,READJ_ONLY_IN_MOVE");
            return;
        }

        Serial.print("POS");
        for (uint8_t id = 1; id <= 6; id++) {
            int16_t pos = read_servo_position_blocking(id);
            Serial.print(",");
            Serial.print(pos);
        }
        Serial.println();
        return;
    }

    // ===== MODE_DATA_RUN Commands =====

    // G,<pos>,<speed> - trajectory goal streaming (existing behavior)
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

        long speed = strtol(end + 1, &end, 10);

        if (pos < 0) pos = 0;
        if (pos > 4095) pos = 4095;
        if (speed < 0) speed = 0;
        if (speed > 1023) speed = 1023;

        enqueue_goal_cmd(g_stream_servo_id, (uint16_t)pos, (uint16_t)speed);
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

void servo_write_scheduler() {
    uint32_t now_us = micros();

    // Rate limit writes
    if (now_us - servo_bus.last_write_us < SERVO_WRITE_INTERVAL_US) return;

    // Don't write if read is pending
    if (servo_bus.state != BUS_IDLE) return;

    // Check command queue
    ServoCommand cmd;
    if (!dequeue_goal_cmd(&cmd)) return;

    // Build WRITE_GOAL packet
    uint8_t pkt[11];
    pkt[0] = FEETECH_HEADER_1;
    pkt[1] = FEETECH_HEADER_2;
    pkt[2] = cmd.servo_id;
    pkt[3] = 7;  // Length: instr(1) + addr(1) + data(4) + checksum(1)
    pkt[4] = INSTR_WRITE_DATA;
    pkt[5] = REG_GOAL_POSITION_L;
    pkt[6] = cmd.goal_position & 0xFF;
    pkt[7] = (cmd.goal_position >> 8) & 0xFF;
    pkt[8] = cmd.goal_speed & 0xFF;
    pkt[9] = (cmd.goal_speed >> 8) & 0xFF;
    pkt[10] = calculate_checksum(pkt, 11);

    servo_transport_flush_rx();
    servo_transport_write(pkt, 11);

    // Capture command for logging
    g_last_cmd_time_us = now_us;
    g_last_cmd_goal = cmd.goal_position;

    servo_bus.write_request_sent_us = now_us;
    servo_bus.last_write_us = now_us;
    servo_bus.state = BUS_WRITE_WAIT;

    servo_rx.state = RX_WAIT_HEADER1;
    servo_rx.idx = 0;
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

    // ---- 2) QUEUE SERVO TX ----
    queue_servo_read_request();

    // ---- 3) SNAPSHOT SERVO STATE ----
    {
        ServoState s;
        uint32_t g1, g2;

        do {
            g1 = latest_servo.generation;
            s.position = latest_servo.position;
            s.t_req_us = latest_servo.t_req_us;
            s.t_rx_us = latest_servo.t_rx_us;
            s.error_code = latest_servo.error_code;
            g2 = latest_servo.generation;
        } while (g1 != g2);

        d.servo_position = s.position;
        d.servo_error_code = s.error_code;

        if (s.t_rx_us != 0 && s.t_rx_us <= d.frame_ts_us && s.error_code == 0) {
            d.servo_age_us = (uint16_t)(d.frame_ts_us - s.t_rx_us);
            d.servo_latency_us = (uint16_t)(s.t_rx_us - s.t_req_us);
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

    // Servo UART
    Serial3.begin(SERVO_BAUD);
    #if ENABLE_DEBUG_OUTPUT
    Serial.println("Serial3 initialized @ 1 Mbaud");
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
    Serial.println("  SYNCW,n,id,p,s,.. - sync write (MOVE)");
    Serial.println("  READJ             - read 6 joints (MOVE)");
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
            // Motion mode: blocking RPC operations, no ISR
            servo_protocol_fsm();
            servo_timeout_service();
            break;

        case MODE_DATA_IDLE:
            // Data mode idle - no ISR, no Serial4 frames
            break;

        case MODE_DATA_RUN:
            // Full 500Hz data acquisition pipeline
            servo_protocol_fsm();
            servo_timeout_service();
            servo_read_service();
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
