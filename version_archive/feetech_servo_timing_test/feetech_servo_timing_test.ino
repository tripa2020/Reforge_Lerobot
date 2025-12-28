/*
 * Feetech Servo Timing Characterization Test
 *
 * Purpose: Measure actual WCET for Feetech STS/SCS servo communication
 * to validate theoretical calculations before Phase 2 integration.
 *
 * Theoretical predictions (1 servo @ 1 Mbps):
 *   - Sync write (38 bytes): 0.38ms = 380µs
 *   - Read request (8 bytes): 0.08ms = 80µs
 *   - Response (12 bytes):   0.12ms = 120µs
 *   - Total round-trip:      0.58ms = 580µs

 
 */

#include <Arduino.h>

// ============== CONFIGURATION ==============
#define SERVO_ID 6          // Test servo ID (gripper motor)
#define SERVO_BAUD 1000000  // 1 Mbps (default Feetech baud rate)
#define SERVO_SPEED 200     // Movement speed (0=max speed, 100-300=slow, 1000=very slow)
#define TEST_ITERATIONS 1000
#define STREAM_FREQUENCY 100 // Hz for streaming test (100 Hz = 10ms period)
#define STREAM_DURATION 10  // seconds for streaming test

// ============== FEETECH PROTOCOL ==============

// Packet structure (Feetech Communication Protocol Manual)
// Header: 0xFF 0xFF
// ID: Servo ID (0-253)
// Length: Packet length (ID + LEN + INSTR + PARAMS + CHECKSUM)
// Instruction: Command byte
// Parameters: Payload
// Checksum: ~(ID + LEN + INSTR + PARAMS)

#define FEETECH_HEADER_1 0xFF
#define FEETECH_HEADER_2 0xFF

// Instructions
#define INSTR_PING 0x01
#define INSTR_READ_DATA 0x02
#define INSTR_WRITE_DATA 0x03
#define INSTR_REG_WRITE 0x04
#define INSTR_ACTION 0x05
#define INSTR_SYNC_WRITE 0x83

// Registers (STS3215 specific - check your servo manual)
#define REG_MODE 0x21              // Mode register (servo/motor/step mode)
#define REG_ALARM_SHUTDOWN 0x24    // Alarm shutdown conditions
#define REG_TORQUE_ENABLE 0x28     // Torque enable (0=off, 1=on)
#define REG_GOAL_POSITION_L 0x2A  // Goal position low byte
#define REG_GOAL_POSITION_H 0x2B  // Goal position high byte
#define REG_LOCK 0x37              // EEPROM lock (1=locked, prevents writes)
#define REG_PRESENT_POSITION_L 0x38  // Current position low byte
#define REG_PRESENT_POSITION_H 0x39  // Current position high byte

// ============== TIMING METRICS ==============

struct TimingMetrics {
    uint32_t sync_write_us;    // Sync write duration
    uint32_t read_request_us;  // Read request duration
    uint32_t read_response_us; // Time from request to response
    uint32_t total_us;         // Total command + read cycle
};

// Statistics tracking
TimingMetrics min_time = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
TimingMetrics max_time = {0, 0, 0, 0};
TimingMetrics sum_time = {0, 0, 0, 0};
uint32_t error_count = 0;

// ============== DWT CYCLE COUNTER ==============

// Teensy 4.x ARM Cortex-M7 @ 600 MHz has built-in DWT cycle counter
// ARM_DWT_CYCCNT is provided by Teensy core (imxrt.h)

void dwt_init() {
    // Enable DWT cycle counter for precision timing
    ARM_DEMCR |= ARM_DEMCR_TRCENA;  // Enable trace
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;  // Enable cycle counter
    ARM_DWT_CYCCNT = 0;  // Reset counter
}

uint32_t dwt_get_cycles() {
    return ARM_DWT_CYCCNT;
}

// Convert cycles to microseconds (600 MHz = 600 cycles/µs)
uint32_t cycles_to_us(uint32_t cycles) {
    return cycles / 600;
}

// ============== FEETECH PROTOCOL FUNCTIONS ==============

uint8_t calculate_checksum(uint8_t* data, uint8_t len) {
    // Checksum = ~(ID + LEN + INSTR + PARAMS)
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {  // Skip header (0xFF 0xFF) and checksum byte
        sum += data[i];
    }
    return ~sum;  // Bitwise NOT
}

// Send sync write command with speed control (Protocol Manual Page 6, Example 4)
// Packet: [0xFF 0xFF ID LEN INSTR REG POS_L POS_H TIME_L TIME_H SPEED_L SPEED_H CKS]
void send_sync_write(uint8_t servo_id, uint16_t position) {
    uint8_t packet[13];  // Expanded from 9 to 13 bytes (added time + speed)
    packet[0] = FEETECH_HEADER_1;
    packet[1] = FEETECH_HEADER_2;
    packet[2] = servo_id;
    packet[3] = 0x09;  // Length: 9 bytes (INSTR + REG + POS_L + POS_H + TIME_L + TIME_H + SPEED_L + SPEED_H + CKS)
    packet[4] = INSTR_WRITE_DATA;
    packet[5] = REG_GOAL_POSITION_L;  // 0x2A - Start address
    packet[6] = position & 0xFF;        // Position low byte
    packet[7] = (position >> 8) & 0xFF; // Position high byte
    packet[8] = 0x00;                   // Time low byte (0 = move immediately)
    packet[9] = 0x00;                   // Time high byte
    packet[10] = SERVO_SPEED & 0xFF;    // Speed low byte (from config)
    packet[11] = (SERVO_SPEED >> 8) & 0xFF; // Speed high byte
    packet[12] = calculate_checksum(packet, 13);

    // Clear RX buffer before sending
    while (Serial3.available()) Serial3.read();

    // Send command
    Serial3.write(packet, 13);
    Serial3.flush();  // Wait for TX complete
}

// Send read request and wait for response
// Request:  [0xFF 0xFF ID LEN INSTR REG NUM_BYTES CKS]
// Response: [0xFF 0xFF ID LEN ERR POS_L POS_H CKS] (8 bytes per manual Section 1.3.2)
int32_t read_position(uint8_t servo_id, uint32_t* response_time_us) {
    // Prepare read request
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
    while (Serial3.available()) Serial3.read();

    // Send request and start timer
    uint32_t start_cycles = dwt_get_cycles();
    Serial3.write(request, 8);
    Serial3.flush();

    // Wait for 8-byte response (Header(2) + ID(1) + Len(1) + Err(1) + Data(2) + Checksum(1))
    uint32_t timeout_start = millis();
    while (Serial3.available() < 8) {
        if (millis() - timeout_start > 10) {
            *response_time_us = 0xFFFFFFFF;  // Timeout marker
            return -1;
        }
    }

    uint32_t end_cycles = dwt_get_cycles();
    *response_time_us = cycles_to_us(end_cycles - start_cycles);

    // Read 8-byte response
    uint8_t response[8];
    Serial3.readBytes(response, 8);

    // Verify header
    if (response[0] != FEETECH_HEADER_1 || response[1] != FEETECH_HEADER_2) {
        return -2;  // Bad header
    }

    // Verify ID
    if (response[2] != servo_id) {
        return -3;  // Wrong servo
    }

    // Verify checksum (CRITICAL: validates data integrity)
    uint8_t calc_checksum = calculate_checksum(response, 8);
    if (response[7] != calc_checksum) {
        return -5;  // Checksum mismatch
    }

    // Check error byte
    if (response[4] != 0x00) {
        return -4;  // Servo reported error
    }

    // Extract position (little-endian)
    int32_t position = response[5] | (response[6] << 8);
    return position;
}

// ============== TEST FUNCTIONS ==============

void run_timing_test() {
    Serial.println("\n========================================");
    Serial.println("Feetech Servo Timing Characterization");
    Serial.println("========================================");
    Serial.print("Servo ID: "); Serial.println(SERVO_ID);
    Serial.print("Baud rate: "); Serial.print(SERVO_BAUD); Serial.println(" bps");
    Serial.print("Test iterations: "); Serial.println(TEST_ITERATIONS);
    Serial.println();

    // Reset statistics
    min_time = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
    max_time = {0, 0, 0, 0};
    sum_time = {0, 0, 0, 0};
    error_count = 0;

    // Test position values (oscillate between two positions)
    uint16_t pos_a = 2048;  // Center position
    uint16_t pos_b = 4000;  // +500 units from center

    Serial.println("Running test...");
    uint32_t test_start = millis();

    for (uint32_t i = 0; i < TEST_ITERATIONS; i++) {
        // Alternate between two positions
        uint16_t target_pos = (i % 2 == 0) ? pos_a : pos_b;

        // ===== 1. SYNC WRITE (Position Command) =====
        uint32_t sync_start = dwt_get_cycles();
        send_sync_write(SERVO_ID, target_pos);
        uint32_t sync_end = dwt_get_cycles();
        uint32_t sync_us = cycles_to_us(sync_end - sync_start);

        // Small delay to let servo process command
        delayMicroseconds(100);

        // ===== 2. READ POSITION (Request + Response) =====
        uint32_t read_start = dwt_get_cycles();
        uint32_t response_time_us;
        int32_t current_pos = read_position(SERVO_ID, &response_time_us);
        uint32_t read_end = dwt_get_cycles();
        uint32_t read_total_us = cycles_to_us(read_end - read_start);

        // Check for errors
        if (current_pos < 0) {
            error_count++;
            continue;  // Skip this iteration
        }

        // Calculate total cycle time
        uint32_t total_us = sync_us + read_total_us;

        // Update statistics
        if (sync_us < min_time.sync_write_us) min_time.sync_write_us = sync_us;
        if (sync_us > max_time.sync_write_us) max_time.sync_write_us = sync_us;
        sum_time.sync_write_us += sync_us;

        if (response_time_us < min_time.read_response_us) min_time.read_response_us = response_time_us;
        if (response_time_us > max_time.read_response_us) max_time.read_response_us = response_time_us;
        sum_time.read_response_us += response_time_us;

        if (read_total_us < min_time.read_request_us) min_time.read_request_us = read_total_us;
        if (read_total_us > max_time.read_request_us) max_time.read_request_us = read_total_us;
        sum_time.read_request_us += read_total_us;

        if (total_us < min_time.total_us) min_time.total_us = total_us;
        if (total_us > max_time.total_us) max_time.total_us = total_us;
        sum_time.total_us += total_us;

        // Progress indicator (every 100 iterations)
        if ((i + 1) % 100 == 0) {
            Serial.print(".");
            if ((i + 1) % 1000 == 0) {
                Serial.print(" "); Serial.println(i + 1);
            }
        }
    }

    uint32_t test_duration = millis() - test_start;

    // ===== PRINT RESULTS =====
    Serial.println("\n\n========================================");
    Serial.println("TEST RESULTS");
    Serial.println("========================================");

    uint32_t valid_iterations = TEST_ITERATIONS - error_count;

    Serial.print("Valid iterations: "); Serial.print(valid_iterations);
    Serial.print(" / "); Serial.println(TEST_ITERATIONS);
    Serial.print("Errors: "); Serial.print(error_count);
    Serial.print(" ("); Serial.print((error_count * 100.0) / TEST_ITERATIONS, 1);
    Serial.println("%)");
    Serial.print("Test duration: "); Serial.print(test_duration); Serial.println(" ms");
    Serial.println();

    // Calculate averages
    float avg_sync = (float)sum_time.sync_write_us / valid_iterations;
    float avg_read = (float)sum_time.read_request_us / valid_iterations;
    float avg_response = (float)sum_time.read_response_us / valid_iterations;
    float avg_total = (float)sum_time.total_us / valid_iterations;

    // Print timing table
    Serial.println("--- Timing Breakdown (µs) ---");
    Serial.println("Operation         | Min    | Max    | Avg    | Theory");
    Serial.println("------------------|--------|--------|--------|--------");

    Serial.print("Sync Write        | ");
    Serial.print(min_time.sync_write_us); Serial.print("   | ");
    Serial.print(max_time.sync_write_us); Serial.print("   | ");
    Serial.print(avg_sync, 1); Serial.print("  | ");
    Serial.println("380");

    Serial.print("Read (Total)      | ");
    Serial.print(min_time.read_request_us); Serial.print("   | ");
    Serial.print(max_time.read_request_us); Serial.print("   | ");
    Serial.print(avg_read, 1); Serial.print("  | ");
    Serial.println("200");

    Serial.print("  Response Time   | ");
    Serial.print(min_time.read_response_us); Serial.print("   | ");
    Serial.print(max_time.read_response_us); Serial.print("   | ");
    Serial.print(avg_response, 1); Serial.print("  | ");
    Serial.println("120");

    Serial.print("TOTAL Cycle       | ");
    Serial.print(min_time.total_us); Serial.print("   | ");
    Serial.print(max_time.total_us); Serial.print("   | ");
    Serial.print(avg_total, 1); Serial.print("  | ");
    Serial.println("580");

    Serial.println();

    // ===== PHASE 2 BUDGET ANALYSIS =====
    Serial.println("--- Phase 2 Budget Analysis @ 1kHz ---");
    Serial.print("Period (Ti): 1000 µs\n");
    Serial.print("IMU read (Ci_imu): 35 µs (measured in Phase 1)\n");
    Serial.print("Servo read (Ci_servo): "); Serial.print(avg_total, 1); Serial.println(" µs (measured)");

    float total_isr = 35 + avg_total;
    float utilization = (total_isr / 1000.0) * 100.0;

    Serial.print("Total ISR (Ci_total): "); Serial.print(total_isr, 1); Serial.println(" µs");
    Serial.print("Utilization (U): "); Serial.print(utilization, 1); Serial.println("%");

    if (total_isr < 1000) {
        Serial.print("Margin: "); Serial.print(1000 - total_isr, 1); Serial.println(" µs");
        Serial.println("✅ SCHEDULABLE for 1kHz Phase 2");
    } else {
        Serial.println("❌ WARNING: Exceeds 1kHz budget!");
    }

    Serial.println();

    // ===== THEORETICAL vs ACTUAL =====
    Serial.println("--- Theoretical vs Actual ---");
    Serial.print("Sync write: Theory=380µs, Actual="); Serial.print(avg_sync, 1);
    Serial.print("µs ("); Serial.print((avg_sync / 380.0) * 100.0, 1); Serial.println("%)");

    Serial.print("Read cycle: Theory=200µs, Actual="); Serial.print(avg_read, 1);
    Serial.print("µs ("); Serial.print((avg_read / 200.0) * 100.0, 1); Serial.println("%)");

    Serial.print("Total:      Theory=580µs, Actual="); Serial.print(avg_total, 1);
    Serial.print("µs ("); Serial.print((avg_total / 580.0) * 100.0, 1); Serial.println("%)");

    Serial.println("\n========================================");
}

// ============== STREAMING TEST DATA STRUCTURES ==============

struct CycleMetrics {
    uint32_t write_us;
    uint32_t read_us;
    uint32_t total_us;
    uint32_t cycle_us;
    uint8_t  echo_bytes;
    bool     read_success;
    int32_t  position;
};

struct StreamingStats {
    // Timing min/max/sum
    CycleMetrics min_metrics;
    CycleMetrics max_metrics;
    uint64_t sum_write_us;
    uint64_t sum_read_us;
    uint64_t sum_total_us;
    uint64_t sum_cycle_us;

    // Echo
    uint32_t echo_count;
    uint32_t total_echo_bytes;
    uint8_t  max_echo_bytes;

    // Errors
    uint32_t read_failures;
    uint32_t timing_violations;  // cycle_us > period
    uint32_t valid_cycles;
};

// ============== ECHO DIAGNOSTIC TEST ==============

void run_echo_diagnostic() {
    Serial.println("\n========================================");
    Serial.println("Echo Diagnostic & Mitigation Test");
    Serial.println("========================================");

    // Test multiple echo delay times
    uint16_t delays_us[] = {100, 200, 300, 500, 1000, 2000};
    const uint8_t num_delays = 6;

    Serial.println("\nTesting echo delay timings...");
    Serial.println("Delay(µs) | Captures | Avg Bytes | Max Bytes");
    Serial.println("----------|----------|-----------|----------");

    for (uint8_t d = 0; d < num_delays; d++) {
        uint16_t test_delay = delays_us[d];
        uint32_t captures = 0;
        uint32_t total_bytes = 0;
        uint8_t max_bytes = 0;
        const uint8_t test_count = 50;

        for (uint8_t i = 0; i < test_count; i++) {
            // Clear buffer
            while (Serial3.available()) Serial3.read();

            // Send write command
            send_sync_write(SERVO_ID, 2048);

            // Wait specified delay
            delayMicroseconds(test_delay);

            // Check echo
            uint8_t avail = Serial3.available();
            if (avail > 0) {
                captures++;
                total_bytes += avail;
                if (avail > max_bytes) max_bytes = avail;
                // Clear buffer
                while (Serial3.available()) Serial3.read();
            }

            delay(20);  // Wait between tests
        }

        float avg_bytes = captures > 0 ? (float)total_bytes / captures : 0;

        Serial.print(test_delay);
        Serial.print("      | ");
        Serial.print(captures);
        Serial.print("/");
        Serial.print(test_count);
        Serial.print("     | ");
        Serial.print(avg_bytes, 1);
        Serial.print("        | ");
        Serial.println(max_bytes);
    }

    Serial.println();

    // ===== ECHO BYTE ANALYSIS =====
    Serial.println("--- Echo Byte Dump (first 10 occurrences) ---");
    Serial.println("Collecting echo samples with 500µs delay...\n");

    uint8_t dump_count = 0;
    const uint8_t max_dumps = 10;

    while (dump_count < max_dumps) {
        // Clear buffer
        while (Serial3.available()) Serial3.read();

        // Send write command (store packet for comparison)
        uint8_t write_packet[13];
        write_packet[0] = FEETECH_HEADER_1;
        write_packet[1] = FEETECH_HEADER_2;
        write_packet[2] = SERVO_ID;
        write_packet[3] = 0x09;
        write_packet[4] = INSTR_WRITE_DATA;
        write_packet[5] = REG_GOAL_POSITION_L;
        write_packet[6] = (2048) & 0xFF;
        write_packet[7] = (2048 >> 8) & 0xFF;
        write_packet[8] = 0x00;
        write_packet[9] = 0x00;
        write_packet[10] = SERVO_SPEED & 0xFF;
        write_packet[11] = (SERVO_SPEED >> 8) & 0xFF;
        write_packet[12] = calculate_checksum(write_packet, 13);

        Serial3.write(write_packet, 13);
        Serial3.flush();

        // Wait for echo
        delayMicroseconds(500);

        uint8_t avail = Serial3.available();
        if (avail > 0) {
            uint8_t echo_buf[32];
            uint8_t read_count = 0;

            while (Serial3.available() && read_count < 32) {
                echo_buf[read_count++] = Serial3.read();
            }

            Serial.print("Echo #");
            Serial.print(dump_count + 1);
            Serial.print(" (");
            Serial.print(read_count);
            Serial.print(" bytes): ");

            for (uint8_t i = 0; i < read_count; i++) {
                if (echo_buf[i] < 0x10) Serial.print("0");
                Serial.print(echo_buf[i], HEX);
                Serial.print(" ");
            }

            Serial.println();

            // Compare to sent packet
            Serial.print("  Sent packet: ");
            for (uint8_t i = 0; i < 13; i++) {
                if (write_packet[i] < 0x10) Serial.print("0");
                Serial.print(write_packet[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            // Check if echo matches start of packet
            bool matches_start = true;
            for (uint8_t i = 0; i < read_count && i < 13; i++) {
                if (echo_buf[i] != write_packet[i]) {
                    matches_start = false;
                    break;
                }
            }

            if (matches_start) {
                Serial.println("  ✓ Matches start of TX packet");
            } else {
                Serial.println("  ✗ Does NOT match TX packet");
            }

            Serial.println();

            dump_count++;
        }

        delay(50);
    }

    // ===== MITIGATION STRATEGIES TEST =====
    Serial.println("--- Testing Mitigation Strategies ---");
    Serial.println();

    // Strategy 1: Aggressive clearing with timeout
    Serial.println("Strategy 1: Wait + Clear (500µs wait, clear all)");
    uint32_t s1_success = 0;
    uint32_t s1_total = 100;

    for (uint32_t i = 0; i < s1_total; i++) {
        // Clear before
        while (Serial3.available()) Serial3.read();

        // Write
        send_sync_write(SERVO_ID, 2048);

        // Wait for echo to fully arrive
        delayMicroseconds(500);

        // Aggressively clear everything
        while (Serial3.available()) Serial3.read();

        // Now read position
        uint32_t resp_time;
        int32_t pos = read_position(SERVO_ID, &resp_time);

        if (pos >= 0) s1_success++;

        delay(10);
    }

    Serial.print("  Success rate: ");
    Serial.print(s1_success);
    Serial.print("/");
    Serial.print(s1_total);
    Serial.print(" (");
    Serial.print((s1_success * 100.0) / s1_total, 1);
    Serial.println("%)");
    Serial.println();

    // Strategy 2: Timed delay between write and read
    Serial.println("Strategy 2: Fixed delay (1ms between write and read)");
    uint32_t s2_success = 0;
    uint32_t s2_total = 100;

    for (uint32_t i = 0; i < s2_total; i++) {
        // Clear before
        while (Serial3.available()) Serial3.read();

        // Write
        send_sync_write(SERVO_ID, 2048);

        // Wait 1ms for servo to process
        delayMicroseconds(1000);

        // Clear any residual echo
        while (Serial3.available()) Serial3.read();

        // Now read position
        uint32_t resp_time;
        int32_t pos = read_position(SERVO_ID, &resp_time);

        if (pos >= 0) s2_success++;

        delay(10);
    }

    Serial.print("  Success rate: ");
    Serial.print(s2_success);
    Serial.print("/");
    Serial.print(s2_total);
    Serial.print(" (");
    Serial.print((s2_success * 100.0) / s2_total, 1);
    Serial.println("%)");
    Serial.println();

    // Strategy 3: Header detection (skip echo, wait for servo response)
    Serial.println("Strategy 3: Header detection (skip until 0xFF 0xFF from servo)");
    uint32_t s3_success = 0;
    uint32_t s3_total = 100;

    for (uint32_t i = 0; i < s3_total; i++) {
        // Clear before
        while (Serial3.available()) Serial3.read();

        // Write
        send_sync_write(SERVO_ID, 2048);

        // Wait a bit
        delayMicroseconds(300);

        // Skip echo bytes (first 13 bytes or until timeout)
        uint8_t skipped = 0;
        uint32_t skip_start = millis();
        while (skipped < 13 && (millis() - skip_start) < 5) {
            if (Serial3.available()) {
                Serial3.read();
                skipped++;
            }
        }

        // Now read position
        uint32_t resp_time;
        int32_t pos = read_position(SERVO_ID, &resp_time);

        if (pos >= 0) s3_success++;

        delay(10);
    }

    Serial.print("  Success rate: ");
    Serial.print(s3_success);
    Serial.print("/");
    Serial.print(s3_total);
    Serial.print(" (");
    Serial.print((s3_success * 100.0) / s3_total, 1);
    Serial.println("%)");
    Serial.println();

    Serial.println("========================================");
    Serial.println("RECOMMENDATION:");

    uint32_t best_success = max(s1_success, max(s2_success, s3_success));

    if (best_success == s1_success) {
        Serial.println("Use Strategy 1: 500µs wait + aggressive clear");
        Serial.println("  delayMicroseconds(500);");
        Serial.println("  while(Serial3.available()) Serial3.read();");
    } else if (best_success == s2_success) {
        Serial.println("Use Strategy 2: 1ms delay between write and read");
        Serial.println("  delayMicroseconds(1000);");
    } else {
        Serial.println("Use Strategy 3: Skip first 13 bytes after write");
        Serial.println("  for(uint8_t i=0; i<13; i++) if(Serial3.available()) Serial3.read();");
    }

    Serial.println("========================================\n");
}

// ============== STREAMING TEST ==============

void run_streaming_test() {
    Serial.println("\n========================================");
    Serial.println("Streaming Write+Read Test (100 Hz)");
    Serial.println("========================================");

    // ===== STEP 1: Read current position =====
    Serial.print("Reading current servo position... ");
    uint32_t response_time;
    int32_t current_pos = read_position(SERVO_ID, &response_time);

    if (current_pos < 0) {
        Serial.println("ERROR: Cannot read servo position!");
        return;
    }
    Serial.print(current_pos);
    Serial.println();

    // ===== STEP 2: Configure trajectory =====
    int32_t center_pos = 2048;         // Center position (STS3215 range: 0-4095)
    int32_t amplitude = 500;           // ±500 units amplitude
    float sine_freq = 0.5;             // 0.5 Hz (2 second period for full cycle)

    Serial.println("\nTrajectory Configuration:");
    Serial.print("  Pattern: Sine wave\n");
    Serial.print("  Center: "); Serial.println(center_pos);
    Serial.print("  Amplitude: ±"); Serial.println(amplitude);
    Serial.print("  Range: "); Serial.print(center_pos - amplitude);
    Serial.print(" to "); Serial.println(center_pos + amplitude);
    Serial.print("  Sine frequency: "); Serial.print(sine_freq); Serial.println(" Hz");
    Serial.print("  Stream rate: "); Serial.print(STREAM_FREQUENCY); Serial.println(" Hz");
    Serial.print("  Duration: "); Serial.print(STREAM_DURATION); Serial.println(" sec");

    // ===== STEP 3: Move to center position =====
    Serial.print("\nMoving to center position ("); Serial.print(center_pos); Serial.println(")...");
    send_sync_write(SERVO_ID, center_pos);

    // Wait for servo to reach center (poll until within tolerance)
    const int32_t position_tolerance = 10;  // ±10 units
    const uint32_t move_timeout_ms = 5000;  // 5 second timeout
    uint32_t move_start = millis();
    int32_t check_pos = -1;
    bool reached_target = false;

    while (millis() - move_start < move_timeout_ms) {
        delay(50);  // Poll every 50ms
        check_pos = read_position(SERVO_ID, &response_time);

        if (check_pos >= 0) {
            int32_t error = abs(check_pos - center_pos);

            if (error <= position_tolerance) {
                reached_target = true;
                Serial.print("Reached center! Position: "); Serial.print(check_pos);
                Serial.print(" (error: "); Serial.print(error); Serial.println(" units)");
                break;
            }

            // Progress indicator
            if ((millis() - move_start) % 500 < 100) {
                Serial.print("  Position: "); Serial.print(check_pos);
                Serial.print(" (error: "); Serial.print(error); Serial.println(" units)");
            }
        }
    }

    if (!reached_target) {
        Serial.print("WARNING: Did not reach center within timeout! ");
        Serial.print("Final position: "); Serial.print(check_pos);
        Serial.print(" (error: "); Serial.print(abs(check_pos - center_pos));
        Serial.println(" units)");
        Serial.println("Proceeding anyway...");
    }

    // ===== STEP 4: Calculate test parameters =====
    uint32_t total_cycles = STREAM_FREQUENCY * STREAM_DURATION;  // 100 * 10 = 1000 cycles
    uint32_t period_us = 1000000UL / STREAM_FREQUENCY;  // 10000 µs = 10 ms

    // ===== STEP 5: Initialize statistics =====
    StreamingStats stats;
    memset(&stats, 0, sizeof(stats));

    // Initialize min values to max
    stats.min_metrics.write_us = 0xFFFFFFFF;
    stats.min_metrics.read_us = 0xFFFFFFFF;
    stats.min_metrics.total_us = 0xFFFFFFFF;
    stats.min_metrics.cycle_us = 0xFFFFFFFF;

    Serial.println("\nStarting sine wave stream in 2 seconds...");
    delay(2000);

    Serial.println("Streaming NOW!\n");

    // ===== STEP 6: Main streaming loop =====
    for (uint32_t i = 0; i < total_cycles; i++) {
        uint32_t cycle_start_us = micros();
        CycleMetrics m;
        memset(&m, 0, sizeof(m));

        // Calculate target position using sine wave
        // position(t) = center + amplitude * sin(2π * freq * t)
        float time_sec = (float)i / STREAM_FREQUENCY;
        int32_t target_pos = center_pos + (int32_t)(amplitude * sin(2 * PI * sine_freq * time_sec));

        // Clear RX buffer before write (to detect fresh echo)
        while (Serial3.available()) Serial3.read();

        // ===== WRITE PHASE (DWT timing) =====
        uint32_t write_start = dwt_get_cycles();
        send_sync_write(SERVO_ID, target_pos);
        uint32_t write_end = dwt_get_cycles();
        m.write_us = cycles_to_us(write_end - write_start);

        // ===== ECHO DETECTION =====
        delayMicroseconds(200);  // Wait for potential echo to arrive
        m.echo_bytes = Serial3.available();

        if (m.echo_bytes > 0) {
            stats.echo_count++;
            stats.total_echo_bytes += m.echo_bytes;
            if (m.echo_bytes > stats.max_echo_bytes) {
                stats.max_echo_bytes = m.echo_bytes;
            }
            // Clear echo from buffer
            while (Serial3.available()) Serial3.read();
        }

        // ===== READ PHASE (DWT timing) =====
        uint32_t read_response_us;
        uint32_t read_start = dwt_get_cycles();
        int32_t pos = read_position(SERVO_ID, &read_response_us);
        uint32_t read_end = dwt_get_cycles();
        m.read_us = cycles_to_us(read_end - read_start);

        if (pos >= 0) {
            m.read_success = true;
            m.position = pos;
            stats.valid_cycles++;
        } else {
            m.read_success = false;
            m.position = -1;
            stats.read_failures++;
        }

        // ===== CYCLE METRICS =====
        m.total_us = m.write_us + m.read_us;
        m.cycle_us = micros() - cycle_start_us;

        if (m.cycle_us > period_us) {
            stats.timing_violations++;
        }

        // ===== UPDATE STATISTICS =====
        // Min
        if (m.write_us < stats.min_metrics.write_us) stats.min_metrics.write_us = m.write_us;
        if (m.read_us < stats.min_metrics.read_us) stats.min_metrics.read_us = m.read_us;
        if (m.total_us < stats.min_metrics.total_us) stats.min_metrics.total_us = m.total_us;
        if (m.cycle_us < stats.min_metrics.cycle_us) stats.min_metrics.cycle_us = m.cycle_us;

        // Max
        if (m.write_us > stats.max_metrics.write_us) stats.max_metrics.write_us = m.write_us;
        if (m.read_us > stats.max_metrics.read_us) stats.max_metrics.read_us = m.read_us;
        if (m.total_us > stats.max_metrics.total_us) stats.max_metrics.total_us = m.total_us;
        if (m.cycle_us > stats.max_metrics.cycle_us) stats.max_metrics.cycle_us = m.cycle_us;

        // Sum
        stats.sum_write_us += m.write_us;
        stats.sum_read_us += m.read_us;
        stats.sum_total_us += m.total_us;
        stats.sum_cycle_us += m.cycle_us;

        // Progress indicator (every 100 writes = 1 second)
        if ((i + 1) % STREAM_FREQUENCY == 0) {
            Serial.print(".");
        }

        // Wait for next cycle (accurate timing)
        while ((micros() - cycle_start_us) < period_us) {
            // Busy-wait for precise 100 Hz timing
        }
    }

    // ===== STEP 7: Report results =====
    Serial.println("\n\n========================================");
    Serial.println("STREAMING TEST RESULTS (100 Hz)");
    Serial.println("========================================");
    Serial.print("Total cycles: "); Serial.println(total_cycles);
    Serial.print("Valid cycles: "); Serial.print(stats.valid_cycles);
    Serial.print(" ("); Serial.print((stats.valid_cycles * 100.0) / total_cycles, 1);
    Serial.println("%)");
    Serial.print("Read failures: "); Serial.print(stats.read_failures);
    Serial.print(" ("); Serial.print((stats.read_failures * 100.0) / total_cycles, 1);
    Serial.println("%)");
    Serial.print("Timing violations: "); Serial.print(stats.timing_violations);
    Serial.print(" (cycle > 10ms, ");
    Serial.print((stats.timing_violations * 100.0) / total_cycles, 1);
    Serial.println("%)");
    Serial.println();

    // Calculate averages
    float avg_write = (float)stats.sum_write_us / total_cycles;
    float avg_read = (float)stats.sum_read_us / total_cycles;
    float avg_total = (float)stats.sum_total_us / total_cycles;
    float avg_cycle = (float)stats.sum_cycle_us / total_cycles;

    // Timing breakdown
    Serial.println("--- Timing Breakdown (µs) ---");
    Serial.println("Operation         | Min    | Max    | Avg");
    Serial.println("------------------|--------|--------|--------");

    Serial.print("Write (13 bytes)  | ");
    Serial.print(stats.min_metrics.write_us); Serial.print("   | ");
    Serial.print(stats.max_metrics.write_us); Serial.print("   | ");
    Serial.println(avg_write, 1);

    Serial.print("Read (8+8 bytes)  | ");
    Serial.print(stats.min_metrics.read_us); Serial.print("   | ");
    Serial.print(stats.max_metrics.read_us); Serial.print("   | ");
    Serial.println(avg_read, 1);

    Serial.print("Total (W+R)       | ");
    Serial.print(stats.min_metrics.total_us); Serial.print("   | ");
    Serial.print(stats.max_metrics.total_us); Serial.print("   | ");
    Serial.println(avg_total, 1);

    Serial.print("Cycle Period      | ");
    Serial.print(stats.min_metrics.cycle_us); Serial.print("   | ");
    Serial.print(stats.max_metrics.cycle_us); Serial.print("   | ");
    Serial.println(avg_cycle, 1);

    Serial.println();

    // Echo statistics
    Serial.println("--- Echo Detection ---");
    Serial.print("Echo detected: "); Serial.print(stats.echo_count);
    Serial.print(" / "); Serial.print(total_cycles);
    Serial.print(" ("); Serial.print((stats.echo_count * 100.0) / total_cycles, 1);
    Serial.println("%)");

    if (stats.echo_count > 0) {
        Serial.print("Total echo bytes: "); Serial.println(stats.total_echo_bytes);
        Serial.print("Average echo bytes: ");
        Serial.println((float)stats.total_echo_bytes / stats.echo_count, 1);
        Serial.print("Max echo bytes: "); Serial.println(stats.max_echo_bytes);
        Serial.println("⚠️  TX ECHO DETECTED!");
        Serial.println("Expected: 13 bytes per echo (write packet size)");

        if (stats.read_failures > 0) {
            Serial.println("⚠️  Echo may be interfering with position reads!");
        }
    } else {
        Serial.println("✅ No TX echo detected!");
    }
    Serial.println();

    // Phase 3 compatibility analysis
    Serial.println("--- Phase 3 Compatibility @ 500 Hz ---");
    Serial.print("ISR period: 2000 µs\n");
    Serial.print("Write+Read (avg): "); Serial.print(avg_total, 1); Serial.println(" µs");
    Serial.print("IMU read (Phase 1): 35 µs\n");

    float total_isr = 35 + avg_total;
    float utilization = (total_isr / 2000.0) * 100.0;

    Serial.print("Total ISR budget: "); Serial.print(total_isr, 1); Serial.println(" µs");
    Serial.print("Utilization: "); Serial.print(utilization, 1); Serial.println("%");

    if (total_isr < 2000) {
        Serial.print("Margin: "); Serial.print(2000 - total_isr, 1); Serial.println(" µs");
        Serial.println("✅ SCHEDULABLE for 500 Hz Phase 3");
    } else {
        Serial.println("❌ WARNING: Exceeds 500 Hz budget!");
    }

    Serial.println("========================================\n");
}

// ============== SETUP ==============

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // Debug serial
    Serial.begin(115200);
    delay(2000);  // Wait for Serial Monitor

    Serial.println("\n========================================");
    Serial.println("Feetech Servo Timing Test");
    Serial.println("========================================");

    // Initialize Serial3 for servo communication
    Serial3.begin(SERVO_BAUD);
    Serial.println("Serial3 initialized @ 1 Mbps");

    // Initialize DWT cycle counter
    dwt_init();
    Serial.println("DWT cycle counter initialized");

    // Test servo ping
    Serial.print("\nTesting servo ID "); Serial.print(SERVO_ID); Serial.println("...");

    uint32_t response_time;
    int32_t pos = read_position(SERVO_ID, &response_time);

    if (pos >= 0) {
        Serial.print("✅ Servo found! Position: "); Serial.println(pos);
        Serial.print("   Response time: "); Serial.print(response_time); Serial.println(" µs");
    } else {
        Serial.print("❌ Servo not responding (error code: "); Serial.print(pos); Serial.println(")");
        Serial.println("\nCheck:");
        Serial.println("  - Servo power (5V)");
        Serial.println("  - Serial3 wiring (pins 14/15)");
        Serial.println("  - Servo ID matches SERVO_ID");
        Serial.println("  - Waveshare controller direction switching");
        Serial.println("\nHalting...");
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(500);
        }
    }

    Serial.println("\nCommands:");
    Serial.println("  't' - Run timing test (1000 iterations, read-only)");
    Serial.println("  's' - Run streaming test (100 Hz, 10 sec, write+read+echo)");
    Serial.println("  'e' - Run echo diagnostic & mitigation test");
}

// ============== MAIN LOOP ==============

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 't' || cmd == 'T') {
            run_timing_test();
        } else if (cmd == 's' || cmd == 'S') {
            run_streaming_test();
        } else if (cmd == 'e' || cmd == 'E') {
            run_echo_diagnostic();
        }
    }

    // Heartbeat LED
    static uint32_t last_blink = 0;
    if (millis() - last_blink > 1000) {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
