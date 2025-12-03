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
#define REG_GOAL_POSITION_L 0x2A  // Goal position low byte
#define REG_GOAL_POSITION_H 0x2B  // Goal position high byte
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
    uint16_t pos_b = 3500;  // +500 units from center

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

    Serial.println("\nPress 't' to run timing test...");
}

// ============== MAIN LOOP ==============

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 't' || cmd == 'T') {
            run_timing_test();
        }
    }

    // Heartbeat LED
    static uint32_t last_blink = 0;
    if (millis() - last_blink > 1000) {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
