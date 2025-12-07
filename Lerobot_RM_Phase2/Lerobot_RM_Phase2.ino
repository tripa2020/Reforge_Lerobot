/*
 * LeRobot Rate Monotonic Data Acquisition - Phase 2.1.2
 *
 * Goal: Background servo polling + IMU @ 500Hz with offline interpolation
 *
 * Architecture:
 *   - IntervalTimer @ 500Hz triggers ISR (IMU read + servo snapshot)
 *   - Servo bus manager FSM in main loop polls servo @ ~526Hz
 *   - ISR snapshots latest_servo via generation counter (no UART I/O in ISR!)
 *   - Seqlock (Writer-Preferred) for lock-free ISR→main loop synchronization
 *   - CSV logger streams to Serial4 @ 2 Mbaud
 *
 * Phase 2.1.2 Features:
 *   ✅ Seqlock (replaces Phase 1 double-buffer)
 *   ✅ IMU reading @ 500Hz in ISR
 *   ✅ Background servo polling @ ~526Hz (servo_bus_service FSM)
 *   ✅ Generation counter coherency (ServoState shared bus_manager→ISR)
 *   ✅ Midpoint timestamp: sample_ts_us = (t_req + t_resp) / 2
 *   ✅ Servo age calculation for offline interpolation
 *   ✅ Expected ISR WCET: ~135µs (IMU + servo snapshot)
 *
 * Offline Interpolation Data (per CSV row):
 *   - timestamp_us: IMU sample time (primary timeline)
 *   - servo_position: Zero-order hold of latest servo reading
 *   - servo_read_us: Age of servo sample (timestamp_us - sample_ts_us)
 *   - servo_error_code: 0=OK, 1=timeout, 2=checksum, 3=framing
 *
 * Phase 2.1.3 (Deferred):
 *   ⏸ servo_set_goal_position() for writes @ 100Hz
 *   ⏸ DMA for Serial4 TX (optional, based on validation)
 *
 * Phase: 2.1.2 (Background Servo Polling)
 */

#include <Arduino.h>
#include <SPI.h>
#include "ISM330_Bare.h"
// ARM Cortex-M7 Data Memory Barrier (for Seqlock synchronization)
#if defined(__arm__) && defined(__IMXRT1062__)
  // Teensy 4.1 (ARM Cortex-M7) - use inline assembly
  #define SEQLOCK_BARRIER() asm volatile("dmb" ::: "memory")
#else
  // Fallback for other platforms
  #define SEQLOCK_BARRIER() __sync_synchronize()
#endif

// ============== CONFIGURATION ==============
#define SAMPLE_RATE_HZ 500
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_RATE_HZ)  // 2000 µs

#define SERIAL_CSV_BAUD 2000000  // 2 Mbaud (Phase 1 validated)
#define ENABLE_DEBUG_OUTPUT 0    // 0 = silent, 1 = startup + errors

// ============== TIMING LIMITATIONS ==============
// KNOWN ISSUE: micros() wraparound every ~71 minutes (2^32 µs)
// Impact: Time interval calculations (now - start) will fail across wraparound.
// Affected: servo timeout checks, jitter calculation, debug rate limiting.
// Mitigation: None implemented. Acceptable for short test runs (<1 hour).
// TODO (Phase 3): Use wraparound-safe arithmetic if long-duration needed.

// ============== HARDWARE - IMU ==============
// Default SPI pins (validated with ISM330DHCX MODE0 @ 4MHz)
#define IMU_CS 10      // Default SPI CS (requires 10kΩ pull-down to GND)
#define IMU_SCK 13     // Default SPI SCK
#define IMU_MISO 12    // Default SPI MISO
#define IMU_MOSI 11    // Default SPI MOSI

// ISM330 bare-metal driver scaling constants
// ±2 g mode: 0.061 mg/LSB (datasheet) -> m/s²
static const float ACCEL_SENS = 0.061f / 1000.0f * 9.80665f;
// ±125 dps mode: 4.375 mdps/LSB (datasheet) -> rad/s
static const float GYRO_SENS = 4.375f / 1000.0f * (PI / 180.0f);

// ============== HARDWARE - SERVO ==============
// Serial3 @ 1 Mbaud (tested in feetech_servo_timing_test)
#define SERVO_SERIAL Serial3
#define SERVO_BAUD 1000000          // 1 Mbps (Feetech default)
#define SERVO_ID 6                  // Gripper motor
#define SERVO_TIMEOUT_MICROS 800    // Must be well under 2000us (ISR period)

// Feetech Protocol (STS3215 Communication Manual)
#define FEETECH_HEADER_1 0xFF
#define FEETECH_HEADER_2 0xFF

// Instructions
#define INSTR_READ_DATA 0x02
#define INSTR_WRITE_DATA 0x03
#define INSTR_SYNC_WRITE 0x83

// Registers (STS3215 specific)
#define REG_PRESENT_POSITION_L 0x38  // Current position low byte
#define REG_PRESENT_POSITION_H 0x39  // Current position high byte

// ============== DATA STRUCTURES ==============

// Enhanced sensor snapshot (64 bytes, cache-aligned) with diagnostics
struct SensorData {
    // ===== TIMESTAMPS =====
    uint32_t timestamp_us;        // ISR entry time (shared by both sensors)
    uint32_t generation;          // Monotonic counter (detect drops)

    // ===== SENSOR DATA =====
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;

    int32_t servo_position;       // Encoder position (0-4095)
    int32_t servo_velocity;       // Reserved for Phase 2.1+

    // ===== TIMING DIAGNOSTICS =====
    uint16_t imu_read_us;         // IMU SPI transaction time
    uint16_t servo_read_us;       // Servo UART transaction time
    uint16_t isr_total_us;        // Total ISR execution time (WCET)
    uint16_t imu_servo_skew_us;   // Time between servo start and IMU start

    // ===== JITTER METRICS =====
    int16_t period_error_us;      // Actual period - 2000µs (signed jitter)

    // ===== COHERENCY VALIDATION =====
    uint32_t checksum;            // XOR checksum for torn read detection
    uint8_t coherency_flag;       // 0xAA = valid, 0xFF = torn read, 0xEE = stale

    // ===== ERROR TRACKING =====
    uint8_t servo_error_code;     // 0=OK, 1=timeout, 2=checksum, 3=framing

    uint8_t reserved[6];          // Padding to 64 bytes
} __attribute__((aligned(32)));

static_assert(sizeof(SensorData) == 64, "SensorData must be exactly 64 bytes");

typedef SensorData CSVSample;  // Alias for consistency

// ============== SHARED SERVO STATE (BUS MANAGER → ISR) ==============
// Phase 2.1.2: Background servo polling writes here, ISR reads via generation counter
//
// Coherency Protocol:
//   Writer (servo_bus_service): Update fields, SEQLOCK_BARRIER(), increment generation LAST
//   Reader (ISR): Read generation, copy fields, read generation again, retry if changed
//
// This avoids noInterrupts() - Cortex-M7 32-bit loads are atomic, generation catches torn reads

struct ServoState {
    volatile int32_t position;       // Last encoder position (0-4095, or -1 on error)
    volatile uint32_t timestamp_us;  // When response was fully received (t_resp)
    volatile uint32_t sample_ts_us;  // Midpoint (t_req + t_resp) / 2 = best physical sample time
    volatile uint8_t error_code;     // 0=OK, 1=timeout, 2=checksum, 3=framing
    volatile uint32_t generation;    // Coherency counter (incremented LAST by writer)
};

// Global shared servo state (written by bus manager, read by ISR)
volatile ServoState latest_servo = {
    -1,       // position: invalid until first successful read
    0,        // timestamp_us: no valid timestamp yet (t_resp)
    0,        // sample_ts_us: no valid sample time yet (midpoint)
    0xFF,     // error_code: 0xFF = not yet initialized
    0         // generation: starts at 0
};

// ============== SERVO BUS MANAGER (Serial3 FSM) ==============
// Phase 2.1.2: Non-blocking state machine for servo communication
//
// States:
//   SERVO_IDLE         - Waiting for next poll interval
//   SERVO_WAIT_RESPONSE - Request sent, accumulating response bytes
//
// Timing:
//   Poll interval: 1900µs (~526 Hz, just under 2000µs ISR period)
//   Timeout: 800µs (validated in Phase 2.1.1)
//   Round-trip: ~460-760µs (TX 80µs + servo 300-600µs + RX 80µs)

enum ServoBusState {
    SERVO_IDLE,
    SERVO_WAIT_RESPONSE
};

struct ServoBusManager {
    ServoBusState state;

    uint32_t last_poll_us;       // When we last completed a poll cycle
    uint32_t request_sent_us;    // When we sent the current request

    uint8_t rx_buf[16];          // Response buffer (8 bytes expected + safety margin)
    uint8_t rx_count;            // Bytes received so far

    // Rate control for goal writes (Phase 2.1.3)
    uint32_t last_write_us;      // When we last sent a goal command
    int32_t pending_goal_pos;    // -1 = no pending goal, else target position
};

// Global servo bus manager state
ServoBusManager servo_bus = {
    SERVO_IDLE,     // state: start idle
    0,              // last_poll_us
    0,              // request_sent_us
    {0},            // rx_buf
    0,              // rx_count
    0,              // last_write_us
    -1              // pending_goal_pos: no pending goal
};

// Timing constants (microseconds)
const uint32_t SERVO_POLL_INTERVAL_US  = 1900;   // ~526 Hz (just under ISR period)
const uint32_t SERVO_TIMEOUT_US        = 800;    // Must be < SAMPLE_PERIOD_US
const uint32_t SERVO_WRITE_INTERVAL_US = 10000;  // 100 Hz max goal write rate

// ============== SEQLOCK BUFFER ==============

struct SeqlockBuffer {
    volatile uint32_t seq __attribute__((aligned(4)));  // Sequence counter
    SensorData data;                                     // 64-byte sensor snapshot
} __attribute__((aligned(32)));

SeqlockBuffer shared __attribute__((aligned(32)));
SensorData last_valid_snapshot;  // Fallback for Writer-Preferred mode

volatile uint32_t sample_generation = 0;
volatile uint32_t last_isr_time_us = 0;

// ============== CSV RING BUFFER ==============
#define CSV_BUFFER_SIZE 4096  // 4096 samples × 64 bytes = 256KB

__attribute__((section(".bss.octram"))) __attribute__((aligned(32)))
static CSVSample csv_ring[CSV_BUFFER_SIZE];

volatile uint32_t csv_head = 0;  // Write index
volatile uint32_t csv_tail = 0;  // Read index
volatile uint32_t csv_drops = 0; // Overflow counter

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

    uint32_t csv_bytes_sent;
};

volatile Statistics stats = {0};
uint32_t last_stats_print_ms = 0;

// ============== NON-BLOCKING DEBUG RING BUFFER ==============
// Sustainable design: rate-limited drain, drop-oldest on overflow
#define DEBUG_BUFFER_SIZE 256
char debug_ring[DEBUG_BUFFER_SIZE][64];  // 256 messages × 64 bytes = 16KB
volatile uint16_t debug_head = 0;        // Write index (fast, < 1µs)
volatile uint16_t debug_tail = 0;        // Read index (slow drain in loop)
volatile uint32_t debug_drops = 0;       // Overflow counter

// ============== ISR FLAGS ==============
volatile bool isr_active = false;
volatile bool logging_active = false;

// ============== INTERVAL TIMER ==============
IntervalTimer sensorTimer;

// ============== FORWARD DECLARATIONS ==============
void sensorReadTimer_ISR();
int32_t read_servo_position(uint8_t servo_id, uint32_t* duration_us, uint8_t* error_code);
uint8_t calculate_checksum(uint8_t* data, uint8_t len);
void servo_bus_service();  // Phase 2.1.2: Non-blocking servo polling FSM
SensorData seqlock_read();
void csv_push_sample(const CSVSample& sample);
bool csv_buffer_full();
void csv_service();
void print_stats();
void debug_log(const char* msg);
void debug_service();

// ============== NON-BLOCKING DEBUG FUNCTIONS ==============

// Fast, non-blocking debug log (< 1µs write time)
// Drops oldest message if buffer full (graceful degradation)
void debug_log(const char* msg) {
    if (!ENABLE_DEBUG_OUTPUT) return;

    uint16_t next_head = (debug_head + 1) % DEBUG_BUFFER_SIZE;

    // If buffer full, drop oldest message (advance tail)
    if (next_head == debug_tail) {
        debug_drops++;
        debug_tail = (debug_tail + 1) % DEBUG_BUFFER_SIZE;
    }

    // Fast copy to ring buffer (< 1µs)
    snprintf(debug_ring[debug_head], 63, "%s", msg);
    debug_head = next_head;
}

// Rate-limited debug drain (lowest priority in main loop)
// Drains 1 message every 10ms minimum (max 2ms blocking per call)
// Sustainable: 100 messages/second max
void debug_service() {
    if (!ENABLE_DEBUG_OUTPUT) return;

    const uint32_t MIN_INTERVAL_US = 10000;  // 10ms minimum between drains
    static uint32_t last_drain_us = 0;

    // Rate limit: don't drain too often
    uint32_t now = micros();
    if (now - last_drain_us < MIN_INTERVAL_US) {
        return;  // Skip this call
    }

    // Drain 1 message if available
    if (debug_tail != debug_head) {
        Serial.println(debug_ring[debug_tail]);  // Blocking (~2ms)
        debug_tail = (debug_tail + 1) % DEBUG_BUFFER_SIZE;
        last_drain_us = now;
    }
}

// ============== FEETECH PROTOCOL FUNCTIONS ==============

uint8_t calculate_checksum(uint8_t* data, uint8_t len) {
    // Checksum = ~(ID + LEN + INSTR + PARAMS)
    // Skip header (0xFF 0xFF) and checksum byte
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {
        sum += data[i];
    }
    return ~sum;  // Bitwise NOT
}

// Read current servo position (encoder value 0-4095)
// Returns position on success, -1 on error
// Sets duration_us to transaction time, error_code to failure reason
int32_t read_servo_position(uint8_t servo_id, uint32_t* duration_us, uint8_t* error_code) {
    *error_code = 0;  // Reset error

    // Prepare read request
    // Request:  [0xFF 0xFF ID LEN INSTR REG NUM_BYTES CKS]
    // Response: [0xFF 0xFF ID LEN ERR POS_L POS_H CKS] (8 bytes)
    uint8_t request[8];
    request[0] = FEETECH_HEADER_1;
    request[1] = FEETECH_HEADER_2;
    request[2] = servo_id;
    request[3] = 0x04;  // Length
    request[4] = INSTR_READ_DATA;
    request[5] = REG_PRESENT_POSITION_L;
    request[6] = 0x02;  // Read 2 bytes (position)
    request[7] = calculate_checksum(request, 8);

    // Clear RX buffer
    while (SERVO_SERIAL.available()) SERVO_SERIAL.read();

    // Send request and measure time
    uint32_t start_us = micros();
    SERVO_SERIAL.write(request, 8);
    // NOTE: No flush() - it's blocking (~200µs). TX buffer drains in background.
    // At 1Mbaud, 8 bytes = 80µs transmission time (acceptable latency).

    // Wait for 8-byte response with microsecond precision timeout
    // ASSUMPTION: This function ONLY supports the fixed 8-byte STS/SCS READ_DATA
    // position frame (read 2 bytes from REG_PRESENT_POSITION_L). The Feetech
    // protocol allows variable-length responses, but this code hardcodes rx_count < 8.
    // If you add other servo commands later, this assumption will break.
    uint32_t timeout_start = micros();
    while (SERVO_SERIAL.available() < 8) {
        if (micros() - timeout_start > SERVO_TIMEOUT_MICROS) {
            *duration_us = micros() - start_us;
            *error_code = 1;  // Timeout
            stats.servo_timeouts++;
            return -1;
        }
    }

    uint32_t end_us = micros();
    *duration_us = end_us - start_us;

    // Read 8-byte response
    uint8_t response[8];
    SERVO_SERIAL.readBytes(response, 8);

    // Verify header
    if (response[0] != FEETECH_HEADER_1 || response[1] != FEETECH_HEADER_2) {
        *error_code = 3;  // Framing error
        return -1;
    }

    // Verify ID
    if (response[2] != servo_id) {
        *error_code = 3;  // Framing error (wrong servo)
        return -1;
    }

    // Verify checksum (CRITICAL for data integrity)
    uint8_t calc_checksum = calculate_checksum(response, 8);
    if (response[7] != calc_checksum) {
        *error_code = 2;  // Checksum error
        stats.servo_checksum_errors++;
        return -1;
    }

    // Check error byte (servo-reported error)
    if (response[4] != 0x00) {
        *error_code = 3;  // Framing/protocol error
        return -1;
    }

    // Extract position (little-endian)
    int32_t position = response[5] | (response[6] << 8);
    return position;
}

// ============== SERVO BUS SERVICE (Phase 2.1.2) ==============
// Non-blocking state machine for servo polling
// Runs in main loop, updates latest_servo via generation counter protocol
// ISR reads latest_servo snapshot (no UART I/O in ISR!)

void servo_bus_service() {
    uint32_t now_us = micros();

    switch (servo_bus.state) {

    case SERVO_IDLE:
        // Wait for poll interval before sending next request
        if ((now_us - servo_bus.last_poll_us) >= SERVO_POLL_INTERVAL_US) {

            // Build read position request (8 bytes)
            uint8_t request[8];
            request[0] = FEETECH_HEADER_1;
            request[1] = FEETECH_HEADER_2;
            request[2] = SERVO_ID;
            request[3] = 0x04;  // Length
            request[4] = INSTR_READ_DATA;
            request[5] = REG_PRESENT_POSITION_L;
            request[6] = 0x02;  // Read 2 bytes (position)
            request[7] = calculate_checksum(request, 8);

            // Clear RX buffer (discard any stale data)
            while (SERVO_SERIAL.available()) {
                SERVO_SERIAL.read();
            }

            // Record request time BEFORE sending (t_req)
            servo_bus.request_sent_us = micros();

            // Send request (non-blocking, TX buffer drains in background)
            SERVO_SERIAL.write(request, 8);
            // NOTE: No flush()! flush() blocks for ~80µs at 1Mbaud

            servo_bus.rx_count = 0;
            servo_bus.state = SERVO_WAIT_RESPONSE;
        }
        break;

    case SERVO_WAIT_RESPONSE:
        // Accumulate bytes (non-blocking)
        while (SERVO_SERIAL.available() && servo_bus.rx_count < 8) {
            servo_bus.rx_buf[servo_bus.rx_count++] = SERVO_SERIAL.read();
        }

        if (servo_bus.rx_count >= 8) {
            // Got full response - record response time (t_resp)
            uint32_t t_resp = micros();
            uint32_t t_req = servo_bus.request_sent_us;

            // t_sample based on t_req + half servo latency
            // (midpoint is unreliable when loop() is starved by csv_service)
            const uint32_t SERVO_LATENCY_US = 400;  // STS3032 typical response latency
            uint32_t t_sample = t_req + SERVO_LATENCY_US / 2;

            // Validate response
            bool valid = true;
            uint8_t error_code = 0;

            // Check header
            if (servo_bus.rx_buf[0] != FEETECH_HEADER_1 ||
                servo_bus.rx_buf[1] != FEETECH_HEADER_2) {
                valid = false;
                error_code = 3;  // Framing error
            }

            // Check servo ID
            if (valid && servo_bus.rx_buf[2] != SERVO_ID) {
                valid = false;
                error_code = 3;  // Framing error (wrong servo)
            }

            // Check checksum
            if (valid) {
                uint8_t calc_cks = calculate_checksum(servo_bus.rx_buf, 8);
                if (servo_bus.rx_buf[7] != calc_cks) {
                    valid = false;
                    error_code = 2;  // Checksum error
                    stats.servo_checksum_errors++;
                }
            }

            // Check servo error byte
            if (valid && servo_bus.rx_buf[4] != 0x00) {
                valid = false;
                error_code = 3;  // Servo reported error
            }

            // Update latest_servo (generation counter protocol)
            if (valid) {
                int32_t position = servo_bus.rx_buf[5] | (servo_bus.rx_buf[6] << 8);

                // Write all fields FIRST
                latest_servo.position = position;
                latest_servo.timestamp_us = t_resp;
                latest_servo.sample_ts_us = t_sample;
                latest_servo.error_code = 0;

                // Memory barrier then increment generation LAST
                SEQLOCK_BARRIER();
                latest_servo.generation++;
            } else {
                // Error path: update error but keep last valid position
                latest_servo.error_code = error_code;
                SEQLOCK_BARRIER();
                latest_servo.generation++;

                stats.servo_errors++;
            }

            servo_bus.last_poll_us = t_req;  // Use request time, not "whenever we noticed"
            servo_bus.state = SERVO_IDLE;
        }
        else if ((now_us - servo_bus.request_sent_us) >= SERVO_TIMEOUT_US) {
            // Timeout - no response from servo
            latest_servo.error_code = 1;  // Timeout
            SEQLOCK_BARRIER();
            latest_servo.generation++;

            stats.servo_timeouts++;
            stats.servo_errors++;

            servo_bus.last_poll_us = servo_bus.request_sent_us;  // Use request time, not "whenever we noticed"
            servo_bus.state = SERVO_IDLE;
        }
        // else: still waiting, stay in SERVO_WAIT_RESPONSE
        break;
    }
}

// ============== SEQLOCK READER (WRITER-PREFERRED) ==============

SensorData seqlock_read() {
    SensorData snapshot;
    uint32_t s1, s2 = 0;  // Initialize s2 to suppress compiler warning
    uint8_t retries = 0;
    const uint8_t MAX_RETRIES = 5;  // Writer-preferred: give up after 5 retries

    do {
        s1 = shared.seq;
        SEQLOCK_BARRIER();  // ARM Data Memory Barrier (ensure seq read completes before data reads)

        // Check if writer is active (seq is odd)
        if (s1 & 1) {
            retries++;
            if (retries >= MAX_RETRIES) {
                // Writer-preferred: Reader gives up, use stale data
                stats.seqlock_reader_giveup++;
                snapshot = last_valid_snapshot;
                snapshot.coherency_flag = 0xEE;  // Mark as stale
                return snapshot;
            }
            delayMicroseconds(1);  // Brief backoff
            continue;
        }

        // Copy data (fast memcpy, ~50ns for 64 bytes on Cortex-M7)
        snapshot = shared.data;

        SEQLOCK_BARRIER();  // Ensure data reads complete before seq read
        s2 = shared.seq;

        retries++;

    } while (s1 != s2 && retries < MAX_RETRIES);

    // Verify coherency with checksum (Phase 2.1.1: no servo in checksum)
    uint32_t accel_bits;
    memcpy(&accel_bits, &snapshot.accel_x_mps2, sizeof(uint32_t));
    uint32_t expected_checksum = snapshot.timestamp_us ^ snapshot.generation ^ accel_bits;

    if (snapshot.checksum != expected_checksum || snapshot.coherency_flag != 0xAA) {
        stats.torn_reads++;
        snapshot.coherency_flag = 0xFF;  // Mark as corrupted (torn read detected)
    } else {
        last_valid_snapshot = snapshot;  // Cache for Writer-preferred fallback
    }

    stats.seqlock_retries += retries;
    if (retries > stats.seqlock_max_retries) {
        stats.seqlock_max_retries = retries;
    }

    return snapshot;
}

// ============== ISR: SENSOR SAMPLING @ 500Hz ==============

void sensorReadTimer_ISR() {
    // Don't log until handshake complete
    if (!logging_active) {
        return;
    }

    // CRITICAL: Check for overrun (previous ISR didn't finish)
    if (isr_active) {
        stats.isr_overrun++;
        return;  // Skip this sample to prevent corruption
    }
    isr_active = true;

    uint32_t isr_start = micros();
    uint32_t expected_time = last_isr_time_us + SAMPLE_PERIOD_US;  // 2000 µsT

    // ===== SEQLOCK WRITE SEQUENCE =====

    // 1. Lock (increment seq to odd)
    shared.seq++;
    SEQLOCK_BARRIER();  // Ensure seq write completes before data writes

    // 2. Capture timestamp FIRST
    uint32_t timestamp = micros();
    shared.data.timestamp_us = timestamp;
    shared.data.generation = sample_generation++;

    // 3. **PHASE 2.1.2: SNAPSHOT SERVO FROM BUS MANAGER**
    // Servo is polled in background by servo_bus_service(), we just read latest_servo
    // using generation counter coherency (no UART I/O in ISR!)
    {
        ServoState servo_snap;
        uint32_t g1, g2;

        // Generation counter read protocol (retry if bus manager updated mid-read)
        do {
            g1 = latest_servo.generation;
            servo_snap.position = latest_servo.position;
            servo_snap.timestamp_us = latest_servo.timestamp_us;
            servo_snap.sample_ts_us = latest_servo.sample_ts_us;
            servo_snap.error_code = latest_servo.error_code;
            g2 = latest_servo.generation;
        } while (g1 != g2);  // Retry if generation changed (torn read)

        shared.data.servo_position = servo_snap.position;
        shared.data.servo_error_code = servo_snap.error_code;

        // Compute servo age: how old is the servo sample at this IMU tick?
        // servo_read_us repurposed as servo_age_us for offline interpolation
        uint16_t servo_age = 0xFFFF;  // Default: invalid/unknown

        // Only compute age if servo data is valid (no errors)
        if (servo_snap.error_code == 0 &&
            servo_snap.sample_ts_us != 0 &&
            servo_snap.sample_ts_us <= timestamp) {
            servo_age = (uint16_t)(timestamp - servo_snap.sample_ts_us);
        }
        // else: leave as 0xFFFF (invalid) if error or bad timestamp

        shared.data.servo_read_us = servo_age;  // Age in µs (0xFFFF = invalid/error)
    }

    // 4. Read IMU (SPI @ 4MHz) - Bare-metal driver (ISR-safe, no allocations)
    uint32_t imu_start = micros();
    ISM330::RawSample raw;
    ISM330::readRaw(raw);
    uint32_t imu_end = micros();

    // Scale raw int16_t values to physical units (float math OK in ISR on Cortex-M7 with FPU)
    shared.data.accel_x_mps2 = raw.ax * ACCEL_SENS;
    shared.data.accel_y_mps2 = raw.ay * ACCEL_SENS;
    shared.data.accel_z_mps2 = raw.az * ACCEL_SENS;
    shared.data.gyro_x_rads = raw.gx * GYRO_SENS;
    shared.data.gyro_y_rads = raw.gy * GYRO_SENS;
    shared.data.gyro_z_rads = raw.gz * GYRO_SENS;
    shared.data.imu_read_us = imu_end - imu_start;

    // 5. Timing diagnostics
    uint32_t isr_end = micros();
    shared.data.isr_total_us = isr_end - isr_start;
    // NOTE: Jitter calculation vulnerable to micros() wraparound (see TIMING LIMITATIONS).
    // Will produce incorrect result if wraparound occurs between samples (~71 min uptime).
    shared.data.period_error_us = (int16_t)(isr_start - expected_time);  // Signed jitter
    // imu_servo_skew_us already set via servo_read_us (repurposed as servo_age_us)

    // 6. Coherency checksum (simple XOR of key fields)
    // BUGFIX: Use shared.data.generation (value N), not sample_generation (N+1 after post-increment)
    uint32_t accel_x_bits;
    memcpy(&accel_x_bits, &shared.data.accel_x_mps2, sizeof(uint32_t));
    shared.data.checksum = timestamp ^ shared.data.generation ^ accel_x_bits;
    shared.data.coherency_flag = 0xAA;  // Valid marker

    // 7. Unlock (increment seq to even)
    SEQLOCK_BARRIER();  // Ensure all data writes complete before seq write
    shared.seq++;

    // ===== END SEQLOCK WRITE =====

    last_isr_time_us = isr_start;

    // Track ISR duration
    uint32_t isr_duration = micros() - isr_start;
    if (isr_duration > stats.max_isr_duration_us) {
        stats.max_isr_duration_us = isr_duration;
    }

    stats.isr_count++;
    isr_active = false;
}

// ============== CSV RING BUFFER FUNCTIONS ==============

bool csv_buffer_full() {
    uint32_t next_head = (csv_head + 1) % CSV_BUFFER_SIZE;
    return (next_head == csv_tail);
}

void csv_push_sample(const CSVSample& sample) {
    if (csv_buffer_full()) {
        csv_drops++;
        Serial.print('1');  // Minimal overflow indicator
        return;
    }

    csv_ring[csv_head] = sample;
    csv_head = (csv_head + 1) % CSV_BUFFER_SIZE;
}

void csv_service() {
    // Phase 1 polling architecture (DMA deferred to Phase 2.3)
    // Limited drain + time budget to avoid starving servo_bus_service()
    const uint32_t MAX_DRAIN = 4;        // was 64 - yield more often
    const uint32_t MAX_BUDGET_US = 500;  // 500µs time budget per call
    uint32_t start_us = micros();
    uint32_t drained = 0;

    while (csv_tail != csv_head &&
           drained < MAX_DRAIN &&
           (micros() - start_us) < MAX_BUDGET_US) {
        CSVSample& sample = csv_ring[csv_tail];

        // Write binary sample to Serial4 (BLOCKING - acceptable at 500Hz with 34.7% utilization)
        size_t written = Serial4.write((uint8_t*)&sample, sizeof(CSVSample));

        if (written != sizeof(CSVSample)) {
            #if ENABLE_DEBUG_OUTPUT
            Serial.println("[WARN] Serial4 write incomplete");
            #endif
        }

        stats.csv_bytes_sent += written;
        csv_tail = (csv_tail + 1) % CSV_BUFFER_SIZE;
        drained++;
    }
}

// ============== STATISTICS ==============

void print_stats() {
    uint32_t now = millis();
    if (now - last_stats_print_ms < 5000) return;  // Every 5 seconds
    last_stats_print_ms = now;

    uint32_t buffer_used = (csv_head >= csv_tail)
        ? (csv_head - csv_tail)
        : (CSV_BUFFER_SIZE - csv_tail + csv_head);

    float buffer_pct = (buffer_used * 100.0) / CSV_BUFFER_SIZE;

    Serial.println("========== PHASE 2.1.1 STATS ==========");
    Serial.print("Samples: "); Serial.println(stats.isr_count);
    Serial.print("Drops: "); Serial.println(csv_drops);
    Serial.print("Overruns: "); Serial.println(stats.isr_overrun);
    Serial.print("Max ISR: "); Serial.print(stats.max_isr_duration_us); Serial.println(" us");
    Serial.print("Buffer: "); Serial.print(buffer_pct, 1); Serial.println(" %");
    Serial.print("CSV Sent: "); Serial.print(stats.csv_bytes_sent / 1024); Serial.println(" KB");

    Serial.println("--- Seqlock ---");
    Serial.print("Torn reads: "); Serial.println(stats.torn_reads);
    Serial.print("Retries (total): "); Serial.println(stats.seqlock_retries);
    Serial.print("Max retries: "); Serial.println(stats.seqlock_max_retries);
    Serial.print("Reader giveups: "); Serial.println(stats.seqlock_reader_giveup);

    Serial.println("--- Servo ---");
    Serial.print("Errors: "); Serial.println(stats.servo_errors);
    Serial.print("Timeouts: "); Serial.println(stats.servo_timeouts);
    Serial.print("Checksum fails: "); Serial.println(stats.servo_checksum_errors);

    Serial.println("--- Debug ---");
    uint16_t debug_pending = (debug_head >= debug_tail)
        ? (debug_head - debug_tail)
        : (DEBUG_BUFFER_SIZE - debug_tail + debug_head);
    Serial.print("Pending: "); Serial.println(debug_pending);
    Serial.print("Drops: "); Serial.println(debug_drops);

    Serial.println("=====================================");
}

// ============== SETUP ==============

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // Debug serial
    Serial.begin(115200);
    #if ENABLE_DEBUG_OUTPUT
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("LeRobot RM Data Acquisition - Phase 2.1.1");
    Serial.println("========================================");
    Serial.println("Features: Seqlock + IMU-ONLY @ 500Hz");
    Serial.println("Goal: Validate seqlock foundation");
    #endif

    // CSV output serial
    Serial4.begin(SERIAL_CSV_BAUD);
    #if ENABLE_DEBUG_OUTPUT
    Serial.print("Serial4: "); Serial.print(SERIAL_CSV_BAUD); Serial.println(" baud");
    #endif

    // Servo serial
    SERVO_SERIAL.begin(SERVO_BAUD);
    #if ENABLE_DEBUG_OUTPUT
    Serial.print("Serial3 (Servo): "); Serial.print(SERVO_BAUD); Serial.println(" baud");
    Serial.print("Servo ID: "); Serial.println(SERVO_ID);
    #endif

    // ============== INITIALIZE IMU ==============
    #if ENABLE_DEBUG_OUTPUT
    Serial.println("\n=== IMU INITIALIZATION ===");
    #endif

    // Initialize bare-metal ISM330 driver
    ISM330::Config imu_cfg;
    imu_cfg.spi    = &SPI;
    imu_cfg.cs_pin = IMU_CS;
    imu_cfg.spi_hz = 4000000;  // 4 MHz (validated in baremetal test)

    if (!ISM330::init(imu_cfg)) {
        #if ENABLE_DEBUG_OUTPUT
        Serial.println("[FATAL] Failed to find ISM330DHCX chip");
        Serial.println("Check: Breadboard connections, CS (pin 10), SPI wiring");
        Serial.println("WHO_AM_I register check failed (expected 0x6B)");
        #endif
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(500);
        }
    }

    // Give SW_RESET time to complete (datasheet: ~50µs, safe margin: 5ms)
    delay(5);

    #if ENABLE_DEBUG_OUTPUT
    Serial.println("ISM330DHCX Found!");
    Serial.println("Accelerometer range: ±2 g");
    Serial.println("Gyro range: ±125 dps");
    Serial.println("Accelerometer ODR: 208 Hz");
    Serial.println("Gyro ODR: 208 Hz");
    Serial.println("SPI Mode: MODE0 @ 4 MHz");
    Serial.println("Register config: BDU=1, IF_INC=1, I2C_disable=1, DEVICE_CONF=1");
    Serial.println("=== IMU READY ===");
    #endif

    // ============== HANDSHAKE PROTOCOL ==============
    #if ENABLE_DEBUG_OUTPUT
    Serial.println("\n=== WAITING FOR PYTHON SCRIPT ===");
    Serial.println("Sending READY signal on Serial4...");
    Serial.println("Start Python script: parse_phase2_csv.py");
    #endif

    // Wait for START command from Python
    bool handshake_complete = false;
    uint32_t last_ready_ms = 0;
    String rx_buffer = "";

    while (!handshake_complete) {
        // Send READY every 500ms
        if (millis() - last_ready_ms > 500) {
            Serial4.println("READY");
            last_ready_ms = millis();

            #if ENABLE_DEBUG_OUTPUT
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            #endif
        }

        // Check for START command
        while (Serial4.available()) {
            char c = Serial4.read();
            if (c == '\n') {
                rx_buffer.trim();
                if (rx_buffer == "START") {
                    handshake_complete = true;
                    Serial4.println("ACK");  // Acknowledge
                    #if ENABLE_DEBUG_OUTPUT
                    Serial.println("=== HANDSHAKE COMPLETE ===");
                    Serial.println("Python script connected!");
                    #endif
                    break;
                }
                rx_buffer = "";
            } else {
                rx_buffer += c;
            }
        }
    }

    delay(100);  // Brief pause for Python to process ACK

    // ============== TEST SERVO COMMUNICATION ==============
    #if ENABLE_DEBUG_OUTPUT
    Serial.println("\n=== TESTING SERVO ===");
    #endif

    // Attempt to read servo position
    uint32_t test_duration;
    uint8_t test_error;
    int32_t test_pos = read_servo_position(SERVO_ID, &test_duration, &test_error);

    if (test_pos >= 0) {
        #if ENABLE_DEBUG_OUTPUT
        Serial.print("Servo OK! Position: "); Serial.print(test_pos);
        Serial.print(", Duration: "); Serial.print(test_duration); Serial.println(" us");
        #endif
    } else {
        #if ENABLE_DEBUG_OUTPUT
        Serial.print("[WARN] Servo read failed (error code: "); Serial.print(test_error);
        Serial.println("). Continuing anyway (will log errors).");
        #endif
    }

    // ============== START SENSOR TIMER ==============

    // Initialize last_isr_time for jitter calculation
    last_isr_time_us = micros();

    sensorTimer.begin(sensorReadTimer_ISR, SAMPLE_PERIOD_US);
    sensorTimer.priority(64);  // Lower priority than hardware interrupts

    #if ENABLE_DEBUG_OUTPUT
    Serial.print("Sampling @ "); Serial.print(SAMPLE_RATE_HZ); Serial.println(" Hz");
    Serial.println("========================================");
    #endif

    // Enable logging after everything is initialized
    logging_active = true;

    #if ENABLE_DEBUG_OUTPUT
    Serial.println("*** LOGGING ACTIVE ***");
    Serial.println("Press 's' for stats.");
    Serial.println("CSV output on Serial4 (binary format)");
    Serial.println("\nPhase 2.1.1 Validation Criteria (IMU-ONLY):");
    Serial.println("  [ ] Data rate: 500 Hz stable (not 0.1 Hz!)");
    Serial.println("  [ ] ISR WCET < 100 us (expect ~49us)");
    Serial.println("  [ ] Zero torn reads (coherency_flag == 0xAA)");
    Serial.println("  [ ] Period jitter < 100 us");
    Serial.println("  [ ] Generation counter gaps = 0");
    #endif
}

// ============== MAIN LOOP ==============

void loop() {
    // ===== PRIORITY 0: Servo Bus Service (Phase 2.1.2) =====
    // Must run frequently to maintain ~526 Hz polling rate
    // Non-blocking FSM, updates latest_servo for ISR to read
    servo_bus_service();

    // ===== PRIORITY 1: Telemetry (Real Data) =====
    // Read sensor data from Seqlock (Writer-Preferred)
    SensorData snapshot = seqlock_read();

    // CRITICAL: Only push NEW samples (deduplicate by generation counter)
    // Main loop runs much faster than 500Hz ISR, so we'd get duplicates otherwise
    // This mirrors Phase 1's newData flag approach using generation counter
    static uint32_t last_generation = UINT32_MAX;  // Will never match first sample (gen=0)
    if (snapshot.generation != last_generation) {
        csv_push_sample(snapshot);
        last_generation = snapshot.generation;
    }

    // Drain CSV buffer to Serial4 (Phase 1 polling, DMA deferred to Phase 2.3)
    csv_service();

    // ===== PRIORITY 2: User Commands =====
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 's' || cmd == 'S') {
            print_stats();
        }
    }

    // ===== PRIORITY 3: Periodic Stats (Every 5 seconds) =====
    #if ENABLE_DEBUG_OUTPUT
    print_stats();
    #endif

    // ===== PRIORITY 4: Debug Drain (LOWEST PRIORITY) =====
    // Rate-limited: 1 message per 10ms max (sustainable)
    #if ENABLE_DEBUG_OUTPUT
    debug_service();
    #endif

    // ===== PRIORITY 5: Heartbeat LED =====
    static uint32_t last_blink = 0;
    uint32_t blink_interval = logging_active ? 100 : 500;  // 5 Hz : 1 Hz

    if (millis() - last_blink > blink_interval) {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
