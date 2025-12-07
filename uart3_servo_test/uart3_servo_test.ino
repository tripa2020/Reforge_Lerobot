/*
 * UART3 Driver Test - Servo Communication Verification
 *
 * Purpose: Verify uart3_driver.h/cpp works correctly for Feetech servo protocol
 *
 * Test Sequence:
 *   1. READ position test (10 requests @ 500ms intervals)
 *   2. WRITE goal position test (5 positions: 2000→500→2000→1000→2048)
 *   3. Rapid READ stress test (50 requests back-to-back)
 *   4. Print UART3 statistics
 *
 * Expected Results:
 *   - READ responses arrive within 800µs
 *   - WRITE acks received
 *   - No RX/TX overflows
 *   - Clean packet parsing (no checksum errors)
 *
 * Hardware:
 *   - Teensy 4.1
 *   - Serial3 (pins 14/15) → Feetech servo bus
 *   - Servo ID: 6 (configurable below)
 *
 * CRITICAL: After uart3_init(), NEVER use Serial3.write/read/print!
 */

#include <Arduino.h>
#include "uart3_driver.h"

// ============== CONFIGURATION ==============
#define SERVO_ID 6
#define SERVO_BAUD 1000000  // 1 Mbaud
#define SERVO_TIMEOUT_US 1000

// Feetech Protocol Constants
#define FEETECH_HEADER_1 0xFF
#define FEETECH_HEADER_2 0xFF
#define INSTR_READ_DATA 0x02
#define INSTR_WRITE_DATA 0x03
#define REG_PRESENT_POSITION_L 0x38
#define REG_GOAL_POSITION_L 0x2A

// ============== TEST STATE ==============
enum TestPhase {
    TEST_IDLE,
    TEST_READ_SERIES,
    TEST_WRITE_SERIES,
    TEST_STRESS,
    TEST_DONE
};

TestPhase current_phase = TEST_IDLE;
uint32_t test_start_ms = 0;
uint32_t last_request_ms = 0;
uint8_t test_counter = 0;

// Target positions for WRITE test
const uint16_t WRITE_GOALS[] = {2000, 500, 2000, 1000, 2048};
const uint8_t NUM_WRITE_GOALS = 5;

// ============== RX FSM ==============
enum RxState {
    RX_WAIT_HEADER1,
    RX_WAIT_HEADER2,
    RX_WAIT_ID,
    RX_WAIT_LEN,
    RX_WAIT_BODY
};

struct RxFSM {
    RxState state;
    uint8_t buf[16];
    uint8_t idx;
    uint8_t expected_len;
    uint32_t request_sent_us;
} rx_fsm = {RX_WAIT_HEADER1, {0}, 0, 0, 0};

// ============== STATISTICS ==============
struct TestStats {
    uint32_t read_success;
    uint32_t read_timeout;
    uint32_t read_checksum_error;
    uint32_t write_success;
    uint32_t write_timeout;
    uint32_t total_latency_us;  // Sum for averaging
    uint32_t max_latency_us;
    uint32_t min_latency_us;
} test_stats = {0, 0, 0, 0, 0, 0, 0, UINT32_MAX};

// ============== FEETECH PROTOCOL ==============

uint8_t calculate_checksum(const uint8_t* data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {
        sum += data[i];
    }
    return ~sum;
}

void send_read_position() {
    uint8_t req[8];
    req[0] = FEETECH_HEADER_1;
    req[1] = FEETECH_HEADER_2;
    req[2] = SERVO_ID;
    req[3] = 0x04;  // Length
    req[4] = INSTR_READ_DATA;
    req[5] = REG_PRESENT_POSITION_L;
    req[6] = 0x02;  // Read 2 bytes
    req[7] = calculate_checksum(req, 8);

    uint8_t sent = uart3_send_bytes(req, 8);
    rx_fsm.request_sent_us = micros();

    if (sent < 8) {
        Serial.println("[ERROR] TX ring buffer full!");
    } else {
        Serial.print("[TX] READ_POSITION request sent (");
        Serial.print(sent);
        Serial.println(" bytes)");
    }
}

void send_write_goal(uint16_t goal_position) {
    uint8_t pkt[11];
    pkt[0] = FEETECH_HEADER_1;
    pkt[1] = FEETECH_HEADER_2;
    pkt[2] = SERVO_ID;
    pkt[3] = 7;  // Length
    pkt[4] = INSTR_WRITE_DATA;
    pkt[5] = REG_GOAL_POSITION_L;
    pkt[6] = goal_position & 0xFF;
    pkt[7] = (goal_position >> 8) & 0xFF;
    pkt[8] = 0x00;  // Speed low byte (0 = max speed)
    pkt[9] = 0x00;  // Speed high byte
    pkt[10] = calculate_checksum(pkt, 11);

    uart3_flush_rx();  // Clear any stale responses
    uint8_t sent = uart3_send_bytes(pkt, 11);
    rx_fsm.request_sent_us = micros();

    if (sent < 11) {
        Serial.println("[ERROR] TX ring buffer full!");
    } else {
        Serial.print("[TX] WRITE_GOAL (position=");
        Serial.print(goal_position);
        Serial.print(") - ");
        Serial.print(sent);
        Serial.println(" bytes sent");
    }
}

// ============== RX HANDLER ==============

void process_response(const uint8_t* buf, uint8_t len, uint32_t rx_complete_us) {
    // Minimum frame: header(2) + id(1) + len(1) + error(1) + checksum(1) = 6 bytes
    if (len < 6) {
        Serial.println("[RX] ERROR: Frame too short");
        return;
    }

    // Verify ID
    if (buf[2] != SERVO_ID) {
        Serial.print("[RX] ERROR: Wrong servo ID (expected ");
        Serial.print(SERVO_ID);
        Serial.print(", got ");
        Serial.print(buf[2]);
        Serial.println(")");
        return;
    }

    // Verify checksum
    uint8_t calc = calculate_checksum(buf, len);
    if (buf[len - 1] != calc) {
        Serial.print("[RX] ERROR: Checksum mismatch (expected 0x");
        Serial.print(calc, HEX);
        Serial.print(", got 0x");
        Serial.print(buf[len - 1], HEX);
        Serial.println(")");
        test_stats.read_checksum_error++;
        return;
    }

    // Check servo error byte
    if (buf[4] != 0x00) {
        Serial.print("[RX] ERROR: Servo error byte = 0x");
        Serial.println(buf[4], HEX);
        return;
    }

    // Calculate latency
    uint32_t latency_us = rx_complete_us - rx_fsm.request_sent_us;
    test_stats.total_latency_us += latency_us;
    if (latency_us > test_stats.max_latency_us) {
        test_stats.max_latency_us = latency_us;
    }
    if (latency_us < test_stats.min_latency_us) {
        test_stats.min_latency_us = latency_us;
    }

    // Parse response type
    if (len >= 8) {
        // READ response (8 bytes: header + id + len + error + data[2] + checksum)
        int16_t position = buf[5] | (buf[6] << 8);
        Serial.print("[RX] READ response: position=");
        Serial.print(position);
        Serial.print(", latency=");
        Serial.print(latency_us);
        Serial.println(" µs");
        test_stats.read_success++;
    } else if (len == 6) {
        // WRITE ack (6 bytes: header + id + len + error + checksum)
        Serial.print("[RX] WRITE ack received, latency=");
        Serial.print(latency_us);
        Serial.println(" µs");
        test_stats.write_success++;
    } else {
        Serial.print("[RX] Unknown response length: ");
        Serial.println(len);
    }

    // Print hex dump for debugging
    Serial.print("    Hex: ");
    for (uint8_t i = 0; i < len; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void service_rx() {
    int byte;

    while ((byte = uart3_get_byte()) >= 0) {

        switch (rx_fsm.state) {
        case RX_WAIT_HEADER1:
            if (byte == FEETECH_HEADER_1) {
                rx_fsm.buf[0] = byte;
                rx_fsm.state = RX_WAIT_HEADER2;
            }
            break;

        case RX_WAIT_HEADER2:
            if (byte == FEETECH_HEADER_2) {
                rx_fsm.buf[1] = byte;
                rx_fsm.state = RX_WAIT_ID;
            } else {
                Serial.println("[RX] Resync: bad header2");
                rx_fsm.state = RX_WAIT_HEADER1;
            }
            break;

        case RX_WAIT_ID:
            rx_fsm.buf[2] = byte;
            rx_fsm.state = RX_WAIT_LEN;
            break;

        case RX_WAIT_LEN:
            rx_fsm.buf[3] = byte;
            rx_fsm.expected_len = byte;
            rx_fsm.idx = 4;
            rx_fsm.state = RX_WAIT_BODY;
            break;

        case RX_WAIT_BODY:
            rx_fsm.buf[rx_fsm.idx++] = byte;
            if (rx_fsm.idx >= (rx_fsm.expected_len + 4)) {
                uint32_t rx_complete = micros();
                process_response(rx_fsm.buf, rx_fsm.idx, rx_complete);
                rx_fsm.state = RX_WAIT_HEADER1;
            }
            break;
        }
    }

    // Check timeout
    if (rx_fsm.request_sent_us > 0) {
        uint32_t now = micros();
        if ((now - rx_fsm.request_sent_us) > SERVO_TIMEOUT_US) {
            if (rx_fsm.state != RX_WAIT_HEADER1) {
                Serial.print("[RX] TIMEOUT (");
                Serial.print(now - rx_fsm.request_sent_us);
                Serial.println(" µs)");
                test_stats.read_timeout++;
                rx_fsm.state = RX_WAIT_HEADER1;
                rx_fsm.request_sent_us = 0;
            }
        }
    }
}

// ============== TEST ORCHESTRATION ==============

void run_test_sequence() {
    uint32_t now_ms = millis();

    switch (current_phase) {
    case TEST_IDLE:
        Serial.println("\n========================================");
        Serial.println("UART3 Servo Communication Test");
        Serial.println("========================================");
        Serial.print("Servo ID: ");
        Serial.println(SERVO_ID);
        Serial.print("Baud rate: ");
        Serial.println(SERVO_BAUD);
        Serial.println();
        Serial.println("Phase 1: READ Position Test (10 requests @ 500ms)");
        Serial.println("----------------------------------------");
        current_phase = TEST_READ_SERIES;
        test_counter = 0;
        test_start_ms = now_ms;
        last_request_ms = 0;
        break;

    case TEST_READ_SERIES:
        if (test_counter >= 10) {
            delay(1000);
            Serial.println("\n----------------------------------------");
            Serial.println("Phase 2: WRITE Goal Position Test (5 positions)");
            Serial.println("----------------------------------------");
            current_phase = TEST_WRITE_SERIES;
            test_counter = 0;
            last_request_ms = 0;
        } else if (now_ms - last_request_ms >= 500) {
            Serial.print("\n[Test ");
            Serial.print(test_counter + 1);
            Serial.println("/10]");
            send_read_position();
            test_counter++;
            last_request_ms = now_ms;
        }
        break;

    case TEST_WRITE_SERIES:
        if (test_counter >= NUM_WRITE_GOALS) {
            delay(1000);
            Serial.println("\n----------------------------------------");
            Serial.println("Phase 3: Stress Test (50 rapid READs)");
            Serial.println("----------------------------------------");
            current_phase = TEST_STRESS;
            test_counter = 0;
            last_request_ms = 0;
        } else if (now_ms - last_request_ms >= 1000) {
            Serial.print("\n[Write ");
            Serial.print(test_counter + 1);
            Serial.print("/");
            Serial.print(NUM_WRITE_GOALS);
            Serial.println("]");
            send_write_goal(WRITE_GOALS[test_counter]);
            test_counter++;
            last_request_ms = now_ms;
        }
        break;

    case TEST_STRESS:
        if (test_counter >= 50) {
            delay(2000);  // Let all responses arrive
            current_phase = TEST_DONE;
        } else if (now_ms - last_request_ms >= 20) {  // 50 Hz
            if (test_counter == 0) {
                Serial.println("Sending 50 READ requests...");
            }
            send_read_position();
            test_counter++;
            last_request_ms = now_ms;
            if (test_counter % 10 == 0) {
                Serial.print("  Sent ");
                Serial.print(test_counter);
                Serial.println(" requests");
            }
        }
        break;

    case TEST_DONE:
        print_final_stats();
        Serial.println("\n*** Test complete. Send 'r' to restart. ***");
        current_phase = TEST_IDLE;
        test_counter = 0xFF;  // Prevent restarting automatically
        break;
    }
}

void print_final_stats() {
    Serial.println("\n========================================");
    Serial.println("FINAL TEST RESULTS");
    Serial.println("========================================");

    Serial.println("\n--- Test Statistics ---");
    Serial.print("READ success: ");
    Serial.println(test_stats.read_success);
    Serial.print("READ timeout: ");
    Serial.println(test_stats.read_timeout);
    Serial.print("READ checksum errors: ");
    Serial.println(test_stats.read_checksum_error);
    Serial.print("WRITE success: ");
    Serial.println(test_stats.write_success);
    Serial.print("WRITE timeout: ");
    Serial.println(test_stats.write_timeout);

    uint32_t total_requests = test_stats.read_success + test_stats.read_timeout;
    if (total_requests > 0) {
        Serial.println("\n--- Latency Statistics ---");
        Serial.print("Average latency: ");
        Serial.print(test_stats.total_latency_us / test_stats.read_success);
        Serial.println(" µs");
        Serial.print("Min latency: ");
        Serial.print(test_stats.min_latency_us);
        Serial.println(" µs");
        Serial.print("Max latency: ");
        Serial.print(test_stats.max_latency_us);
        Serial.println(" µs");
    }

    Serial.println("\n--- UART3 Driver Statistics ---");
    Serial.print("RX bytes: ");
    Serial.println(uart3_stats.rx_bytes);
    Serial.print("TX bytes: ");
    Serial.println(uart3_stats.tx_bytes);
    Serial.print("RX overflow: ");
    Serial.println(uart3_stats.rx_overflow);
    Serial.print("TX overflow: ");
    Serial.println(uart3_stats.tx_overflow);
    Serial.print("ISR count: ");
    Serial.println(uart3_stats.isr_count);

    Serial.println("\n--- Test Verdict ---");
    bool pass = (test_stats.read_timeout == 0) &&
                (test_stats.read_checksum_error == 0) &&
                (uart3_stats.rx_overflow == 0) &&
                (uart3_stats.tx_overflow == 0) &&
                (test_stats.max_latency_us < 1000);

    if (pass) {
        Serial.println("✓ PASS - UART3 driver working correctly");
    } else {
        Serial.println("✗ FAIL - Issues detected:");
        if (test_stats.read_timeout > 0) Serial.println("  - Read timeouts occurred");
        if (test_stats.read_checksum_error > 0) Serial.println("  - Checksum errors detected");
        if (uart3_stats.rx_overflow > 0) Serial.println("  - RX buffer overflows");
        if (uart3_stats.tx_overflow > 0) Serial.println("  - TX buffer overflows");
        if (test_stats.max_latency_us >= 1000) Serial.println("  - Excessive latency (>1ms)");
    }

    Serial.println("========================================");
}

// ============== SETUP ==============

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  UART3 Driver Test - Servo Comm       ║");
    Serial.println("║  Teensy 4.1 + Feetech Protocol        ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Initializing UART3 driver...");

    uart3_init(SERVO_BAUD);
    delay(100);

    Serial.println("✓ UART3 initialized");
    Serial.println("  - Ring buffers: 256 bytes TX/RX");
    Serial.println("  - FIFO depth: 1 byte");
    Serial.println("  - ISR priority: 64");
    Serial.println("  - Target: LPUART2 (Serial3 pins 14/15)");
    Serial.println();
    Serial.println("WARNING: Do NOT use Serial3 after this point!");
    Serial.println();
    Serial.println("Press any key to start test...");

    while (!Serial.available()) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(200);
    }
    while (Serial.available()) Serial.read();  // Flush input

    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}

// ============== MAIN LOOP ==============

void loop() {
    // Priority 1: Service RX (parse incoming bytes)
    service_rx();

    // Priority 2: Run test sequence
    if (test_counter != 0xFF) {  // Not paused
        run_test_sequence();
    }

    // Priority 3: User commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'r' || cmd == 'R') {
            Serial.println("\n*** RESTARTING TEST ***\n");
            // Reset stats
            test_stats = {0, 0, 0, 0, 0, 0, 0, UINT32_MAX};
            current_phase = TEST_IDLE;
            test_counter = 0;
            rx_fsm.state = RX_WAIT_HEADER1;
            delay(500);
        } else if (cmd == 's' || cmd == 'S') {
            print_final_stats();
        }
    }

    // Heartbeat LED
    static uint32_t last_blink = 0;
    if (millis() - last_blink > 500) {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
