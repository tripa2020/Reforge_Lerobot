/*
 * LeRobot Rate Monotonic Data Acquisition - Phase 3 (Option B')
 *
 * Goal: Tight encoder-IMU synchronization (<800µs) with non-blocking UART
 *
 * Architecture (Option B'):
 *   - IntervalTimer @ 500Hz triggers Frame ISR:
 *       1) Read IMU (this IS the sample time)
 *       2) Queue servo TX request (non-blocking, fills TX ring)
 *       3) Snapshot latest_servo (from previous cycle)
 *   - Custom UART3 driver with ISR-driven ring buffers
 *   - UART3 RX ISR fires when servo response bytes arrive (no waiting)
 *   - Main loop: servo_protocol_fsm() parses RX bytes, updates latest_servo
 *   - One-cycle pipeline: "ask" in frame N, "know" in frame N+1
 *
 * Timing Analysis (why <800µs is achievable):
 *   - t_frame = ISR entry (IMU sample time)
 *   - t_req = t_frame + ~50µs (after IMU read, in same ISR)
 *   - Servo encoder sample ≈ t_req + 200-400µs (internal delay)
 *   - Total misalignment ≈ 250-500µs ✓ (well under 800µs)
 *
 * Core Principle: "ISR = move bytes to/from ring buffers. Protocol/logic = non-ISR code."
 *
 * Phase 3 Features:
 *   ✅ Custom UART3 driver with 256-byte ring buffers
 *   ✅ FIFO depth = 1 for predictable ISR timing
 *   ✅ IMU read FIRST in Frame ISR (frame_ts_us = IMU sample time)
 *   ✅ Servo TX queued in Frame ISR (tight sync with IMU)
 *   ✅ Servo RX handled by UART ISR + main loop FSM (no blocking)
 *   ✅ Command queue for goal positions (FIFO, drop-newest on overflow)
 *   ✅ Explicit timestamps: t_req_us, t_rx_us, frame_ts_us
 *
 * Phase: 3 (Option B' - Synchronized TX Architecture)
 */

#include <Arduino.h>
#include <SPI.h>
#include "ISM330_Bare.h"
#include "uart3_driver.h"

// ARM Cortex-M7 Data Memory Barrier (for Seqlock synchronization)
#if defined(__arm__) && defined(__IMXRT1062__)
  #define SEQLOCK_BARRIER() asm volatile("dmb" ::: "memory")
#else
  #define SEQLOCK_BARRIER() __sync_synchronize()
#endif

// ============== CONFIGURATION ==============
#define SAMPLE_RATE_HZ 500
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_RATE_HZ)  // 2000 µs

#define SERIAL_CSV_BAUD 2000000  // 2 Mbaud
#define ENABLE_DEBUG_OUTPUT 1    // 0 = silent, 1 = startup + errors

// ============== HARDWARE - IMU ==============
#define IMU_CS 10      // Default SPI CS (requires 10kΩ pull-down to GND)
#define IMU_SCK 13     // Default SPI SCK
#define IMU_MISO 12    // Default SPI MISO
#define IMU_MOSI 11    // Default SPI MOSI

// ISM330 bare-metal driver scaling constants
static const float ACCEL_SENS = 0.061f / 1000.0f * 9.80665f;  // ±2 g mode
static const float GYRO_SENS = 4.375f / 1000.0f * (PI / 180.0f);  // ±125 dps mode

// ============== HARDWARE - SERVO ==============
#define SERVO_BAUD 1000000        // 1 Mbps (Feetech default)
#define SERVO_ID 6                // Gripper motor
#define SERVO_TIMEOUT_US 800      // Response timeout

// Feetech Protocol
#define FEETECH_HEADER_1 0xFF
#define FEETECH_HEADER_2 0xFF
#define INSTR_READ_DATA 0x02
#define INSTR_WRITE_DATA 0x03
#define REG_PRESENT_POSITION_L 0x38
#define REG_GOAL_POSITION_L 0x2A

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
    int16_t period_error_us;      // Actual period - 2000µs

    // Coherency validation
    uint32_t checksum;
    uint8_t coherency_flag;       // 0xAA = valid, 0xFF = torn, 0xEE = stale

    // Error tracking
    uint8_t servo_error_code;     // 0=OK, 1=timeout, 2=checksum, 3=framing

    uint8_t reserved[6];
} __attribute__((aligned(32)));

static_assert(sizeof(SensorData) == 64, "SensorData must be exactly 64 bytes");

typedef SensorData CSVSample;

// ============== SERVO STATE (FSM → Frame ISR) ==============

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

// ============== SERVO READ STATE (Frame ISR triggers TX) ==============
// Option B': Frame ISR fires servo TX immediately after IMU read
// This achieves <800µs encoder-IMU alignment (instead of up to 2000µs decoupled)

struct ServoReadState {
    volatile bool     request_in_flight;   // True if waiting for response
    volatile uint32_t current_t_req_us;    // When frame ISR queued the last READ
};

ServoReadState servo_read = {false, 0};

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

// ============== ISR FLAGS ==============
volatile bool isr_active = false;
volatile bool logging_active = false;

// ============== INTERVAL TIMER ==============
IntervalTimer frameTimer;

// ============== FORWARD DECLARATIONS ==============
void frame_isr();
void servo_timeout_service();  // Just handles timeout (TX is in frame_isr)
void servo_protocol_fsm();
void servo_write_scheduler();
void process_servo_frame(const uint8_t* buf, uint8_t len, uint32_t t_rx);
uint8_t calculate_checksum(const uint8_t* data, uint8_t len);
bool queue_servo_read_request();  // Called from frame_isr
SensorData seqlock_read();
void csv_push_sample(const CSVSample& sample);
bool csv_buffer_full();
void csv_service();
void print_stats();

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
// Option B': Queue servo TX immediately after IMU read for tight synchronization

bool queue_servo_read_request() {
    // Don't send if already waiting for response
    if (servo_read.request_in_flight) {
        return false;
    }

    // Don't send if write is in progress
    if (servo_bus.state == BUS_WRITE_WAIT) {
        return false;
    }

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

    // Queue to TX ring buffer (non-blocking, just fills ring)
    uint8_t sent = uart3_send_bytes(req, 8);
    if (sent < 8) {
        // TX buffer full (shouldn't happen)
        return false;
    }

    // Record timestamp and mark request in flight
    servo_read.current_t_req_us = micros();
    servo_read.request_in_flight = true;
    servo_bus.state = BUS_READ_WAIT;

    return true;
}

// ============== SERVO TIMEOUT SERVICE ==============
// Handles timeout detection for both reads and writes (TX scheduling now in frame_isr)

void servo_timeout_service() {
    uint32_t now_us = micros();

    // Check read timeout
    if (servo_read.request_in_flight) {
        uint32_t elapsed = now_us - servo_read.current_t_req_us;

        if (elapsed > SERVO_TIMEOUT_US) {
            // Timeout - update servo state
            latest_servo.error_code = 1;  // Timeout
            SEQLOCK_BARRIER();
            latest_servo.generation++;

            stats.servo_timeouts++;
            stats.servo_errors++;

            // Clear in-flight flag so next frame ISR can send new request
            servo_read.request_in_flight = false;
            servo_bus.state = BUS_IDLE;

            // Reset RX FSM for resync
            servo_rx.state = RX_WAIT_HEADER1;
        }
    }

    // Check write timeout (just return to IDLE, no error logging for writes)
    if (servo_bus.state == BUS_WRITE_WAIT) {
        uint32_t elapsed = now_us - servo_bus.write_request_sent_us;

        if (elapsed > SERVO_TIMEOUT_US) {
            // Write ack timeout - just return to IDLE
            servo_bus.state = BUS_IDLE;
            servo_rx.state = RX_WAIT_HEADER1;
        }
    }
}

// ============== SERVO PROTOCOL FSM ==============

void servo_protocol_fsm() {
    int byte;

    while ((byte = uart3_get_byte()) >= 0) {
        switch (servo_rx.state) {

        case RX_WAIT_HEADER1:
            if (byte == FEETECH_HEADER_1) {
                servo_rx.buf[0] = byte;
                servo_rx.state = RX_WAIT_HEADER2;
            }
            // else: discard, keep waiting
            break;

        case RX_WAIT_HEADER2:
            if (byte == FEETECH_HEADER_2) {
                servo_rx.buf[1] = byte;
                servo_rx.state = RX_WAIT_ID;
            } else {
                // Not a valid header - resync
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
            // Total frame = header(2) + id(1) + len(1) + body(len)
            // Body includes: error(1) + data(len-2) + checksum(1)
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
    bool valid = true;
    uint8_t error_code = 0;

    // Minimum valid frame: header(2) + id(1) + len(1) + error(1) + data + checksum(1)
    if (len < 6) {
        valid = false;
        error_code = 3;  // Framing
    }

    // Verify ID matches
    if (valid && buf[2] != SERVO_ID) {
        valid = false;
        error_code = 3;  // Wrong servo
    }

    // Verify checksum
    if (valid) {
        uint8_t calc = calculate_checksum(buf, len);
        if (buf[len - 1] != calc) {
            valid = false;
            error_code = 2;  // Checksum
            stats.servo_checksum_errors++;
        }
    }

    // Check servo error byte (buf[4])
    if (valid && buf[4] != 0x00) {
        valid = false;
        error_code = 3;  // Servo reported error
    }

    // Update ServoState
    if (valid && len >= 8) {
        // Parse position (little-endian, bytes 5 and 6)
        int32_t pos = buf[5] | (buf[6] << 8);

        latest_servo.position = pos;
        latest_servo.t_req_us = servo_read.current_t_req_us;  // From frame ISR
        latest_servo.t_rx_us = t_rx;
        latest_servo.error_code = 0;
        SEQLOCK_BARRIER();
        latest_servo.generation++;
    } else {
        // Error path
        if (error_code == 0) error_code = 3;  // Default to framing
        latest_servo.error_code = error_code;
        SEQLOCK_BARRIER();
        latest_servo.generation++;

        stats.servo_errors++;
        if (error_code == 3) stats.servo_framing_errors++;
    }

    // Clear in-flight flag so next frame ISR can send new request
    servo_read.request_in_flight = false;
    servo_bus.state = BUS_IDLE;
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

    // Build WRITE_GOAL packet (11 bytes: header + id + len + instr + addr + pos(2) + speed(2) + checksum)
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

    uart3_flush_rx();
    uart3_send_bytes(pkt, 11);

    servo_bus.write_request_sent_us = now_us;
    servo_bus.last_write_us = now_us;
    servo_bus.state = BUS_WRITE_WAIT;

    // Reset RX FSM for ack
    servo_rx.state = RX_WAIT_HEADER1;
    servo_rx.idx = 0;

    // Note: Write ack is 6 bytes, handled by process_servo_frame()
    // For simplicity, we'll timeout and return to IDLE if no ack
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
    // NOTE: frame_ts_us is ISR entry time, not actual IMU internal sample time.
    // True IMU sample occurs inside ISM330's ODR pipeline, ~0-1ms before we read.
    // For sub-ms accuracy, would need ISM330 FIFO + hardware timestamping.
    // For <1ms alignment goal, this approximation is acceptable.
    d.frame_ts_us = isr_start;
    d.frame_index = frame_index++;

    // ---- 1) IMU READ FIRST (bare-metal SPI, ~33-130µs) ----
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

    // ---- 2) QUEUE SERVO TX (Option B': non-blocking, tight sync with IMU) ----
    // This achieves <800µs encoder-IMU alignment because:
    //   t_req ≈ t_frame + imu_read_us (~33-130µs)
    //   servo encoder sample ≈ t_req + 200-400µs internal delay
    //   total misalignment ≈ 200-500µs (well under 800µs target)
    queue_servo_read_request();  // Just fills TX ring, returns fast

    // ---- 3) SNAPSHOT SERVO STATE (generation counter, no I/O) ----
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

        // Compute servo age: how old is the servo sample?
        // NOTE: micros() wraps every ~70 minutes. Unsigned subtraction handles
        // wrap correctly (modulo 2^32 arithmetic), so (frame_ts - t_rx) gives
        // correct age even across wrap boundary. The <= check below may give
        // false negative at wrap, but for typical <30 min sessions this is fine.
        // For longer sessions, remove the <= check and rely on error_code only.
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
        csv_tail = (csv_tail + 1) % CSV_BUFFER_SIZE;
        drained++;
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

    Serial.println("========== PHASE 3 STATS ==========");
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

    Serial.println("--- UART3 ---");
    Serial.print("RX bytes: "); Serial.println(uart3_stats.rx_bytes);
    Serial.print("TX bytes: "); Serial.println(uart3_stats.tx_bytes);
    Serial.print("RX overflow: "); Serial.println(uart3_stats.rx_overflow);
    Serial.print("ISR count: "); Serial.println(uart3_stats.isr_count);

    Serial.println("--- Command Queue ---");
    Serial.print("Overflows: "); Serial.println(stats.cmd_queue_overflows);

    Serial.println("====================================");
}

// ============== SETUP ==============

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // Debug serial
    Serial.begin(115200);
    delay(100);  // Allow USB serial to initialize

    #if ENABLE_DEBUG_OUTPUT
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("LeRobot RM Data Acquisition - Phase 3");
    Serial.println("Decoupled UART Architecture");
    Serial.println("========================================");
    #endif

    // CSV output serial
    Serial4.begin(SERIAL_CSV_BAUD);
    delay(100);  // Allow Serial4 hardware UART to initialize before handshake

    // Custom UART3 driver (replaces Serial3)
    uart3_init(SERVO_BAUD);
    #if ENABLE_DEBUG_OUTPUT
    Serial.println("UART3 driver initialized @ 1 Mbaud");
    Serial.println("  - 256-byte ring buffers");
    Serial.println("  - FIFO depth = 1");
    Serial.println("  - ISR priority = 64");
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

    // Handshake protocol
    bool handshake_complete = false;
    uint32_t last_ready_ms = 0;
    String rx_buffer = "";

    while (!handshake_complete) {
        if (millis() - last_ready_ms > 500) {
            Serial4.println("READY");
            last_ready_ms = millis();
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }

        while (Serial4.available()) {
            char c = Serial4.read();
            if (c == '\n') {
                rx_buffer.trim();
                if (rx_buffer == "START") {
                    handshake_complete = true;
                    Serial4.println("ACK");
                    #if ENABLE_DEBUG_OUTPUT
                    Serial.println("Handshake complete");
                    #endif
                    break;
                }
                rx_buffer = "";
            } else {
                rx_buffer += c;
            }
        }
    }

    delay(100);

    // Initialize timing
    last_isr_time_us = micros();

    // Start frame timer
    frameTimer.begin(frame_isr, SAMPLE_PERIOD_US);
    frameTimer.priority(32);  // Higher priority than UART3 ISR (64)

    logging_active = true;

    #if ENABLE_DEBUG_OUTPUT
    Serial.println("*** LOGGING ACTIVE ***");
    Serial.println("Frame ISR @ 500Hz (priority 32)");
    Serial.println("UART3 ISR (priority 64)");
    Serial.println("Servo FSM in main loop");
    #endif
}

// ============== MAIN LOOP ==============

void loop() {
    // ===== PRIORITY 0: Servo Protocol FSM (decode RX bytes) =====
    // UART RX ISR pushed bytes into ring; FSM parses and updates latest_servo
    servo_protocol_fsm();

    // ===== PRIORITY 1: Servo Timeout Service =====
    // TX scheduling is now in frame_isr; this just handles timeouts
    servo_timeout_service();

    // ===== PRIORITY 2: Servo Write Scheduler (goal commands @ 40-100Hz) =====
    servo_write_scheduler();

    // ===== PRIORITY 3: Telemetry =====
    SensorData snapshot = seqlock_read();

    static uint32_t last_frame = UINT32_MAX;
    if (snapshot.frame_index != last_frame) {
        csv_push_sample(snapshot);
        last_frame = snapshot.frame_index;
    }

    csv_service();

    // ===== PRIORITY 4: User Commands =====
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 's' || cmd == 'S') {
            print_stats();
        }
    }

    // ===== PRIORITY 5: Periodic Stats =====
    #if ENABLE_DEBUG_OUTPUT
    print_stats();
    #endif

    // ===== PRIORITY 6: Heartbeat LED =====
    static uint32_t last_blink = 0;
    uint32_t blink_interval = logging_active ? 100 : 500;

    if (millis() - last_blink > blink_interval) {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
