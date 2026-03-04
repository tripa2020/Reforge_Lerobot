/*
 * Feetech SYNC WRITE Test
 *
 * Minimal test sketch for Feetech STS3215 SYNC WRITE command.
 * Sends all 6 joints to center (2048), waits, then moves +25 degrees.
 *
 * Hardware: Teensy 4.1, Serial3 @ 1Mbaud to servo bus
 *
 * Protocol Reference: https://www.waveshare.com/wiki/STS3215_Serial_Bus_Servo
 * SYNC WRITE: ID=0xFE (broadcast), Instruction=0x83
 *
 * Date: 2025-12-13
 */

#include <Arduino.h>

// ============== CONFIGURATION ==============
#define SERVO_BAUD 1000000  // 1 Mbps

// Servo IDs (5 servos, no gripper)
const uint8_t SERVO_IDS[] = {1, 2, 3, 4, 5};
const uint8_t NUM_SERVOS = 5;

// Position constants
// STS3215: 4096 counts/revolution, center = 2048
// 25 degrees = 25/360 * 4096 ≈ 284 counts
const uint16_t POS_CENTER = 2048;
const uint16_t POS_PLUS_25_DEG = 2048 + 284;  // 2332

// Speed (steps/second: higher = faster, lower = slower, max ~3073)
uint16_t MOVE_SPEED = 150;  // Set to 150 for calibration testing

// Sequential test state
uint8_t current_test_servo = 0;  // Which servo we're currently testing

// Feetech Protocol Constants
#define FEETECH_HEADER_1 0xFF
#define FEETECH_HEADER_2 0xFF
#define BROADCAST_ID 0xFE
#define INSTR_WRITE 0x03
#define INSTR_READ 0x02
#define INSTR_SYNC_WRITE 0x83
#define REG_TORQUE_ENABLE 0x28
#define REG_GOAL_POSITION_L 0x2A
#define REG_PRESENT_POSITION_L 0x38  // Current position register

// ============== TORQUE ENABLE FUNCTION ==============

/**
 * Enable or disable torque for a single servo
 *
 * @param id Servo ID
 * @param enable true to enable torque, false to disable
 * @return true if command sent successfully
 */
bool set_torque_enable(uint8_t id, bool enable) {
    uint8_t pkt[8];
    pkt[0] = FEETECH_HEADER_1;
    pkt[1] = FEETECH_HEADER_2;
    pkt[2] = id;
    pkt[3] = 4;  // Length: INSTR + ADDR + DATA + CHECKSUM
    pkt[4] = INSTR_WRITE;
    pkt[5] = REG_TORQUE_ENABLE;
    pkt[6] = enable ? 1 : 0;

    // Calculate checksum
    uint8_t sum = 0;
    for (uint8_t i = 2; i < 7; i++) {
        sum += pkt[i];
    }
    pkt[7] = ~sum;

    // Debug
    Serial.print("Torque ");
    Serial.print(enable ? "ENABLE" : "DISABLE");
    Serial.print(" ID");
    Serial.print(id);
    Serial.print(": ");
    for (uint8_t i = 0; i < 8; i++) {
        if (pkt[i] < 0x10) Serial.print("0");
        Serial.print(pkt[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Flush RX buffer
    while (Serial3.available()) Serial3.read();

    // Send packet
    size_t sent = Serial3.write(pkt, 8);
    Serial3.flush();

    // Wait for response (non-broadcast, so servo will reply)
    delay(5);

    // Read and discard response
    while (Serial3.available()) Serial3.read();

    return (sent == 8);
}

/**
 * Enable torque for all servos
 */
void enable_all_torque() {
    Serial.println("Enabling torque on all servos...");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        set_torque_enable(SERVO_IDS[i], true);
        delay(10);
    }
}

// ============== READ POSITION FUNCTION ==============

/**
 * Read current position from a single servo
 *
 * Packet structure (TX):
 *   FF FF ID 04 02 ADDR LEN CHECKSUM
 *
 * Response (RX):
 *   FF FF ID LEN ERR DATA_L DATA_H CHECKSUM
 *
 * @param id Servo ID
 * @return Position (0-4095) or -1 on error
 */
int16_t read_position(uint8_t id) {
    // Build read request packet
    uint8_t pkt[8];
    pkt[0] = FEETECH_HEADER_1;
    pkt[1] = FEETECH_HEADER_2;
    pkt[2] = id;
    pkt[3] = 4;  // Length: INSTR + ADDR + LEN + CHECKSUM
    pkt[4] = INSTR_READ;
    pkt[5] = REG_PRESENT_POSITION_L;
    pkt[6] = 2;  // Read 2 bytes (position low + high)

    // Calculate checksum
    uint8_t sum = 0;
    for (uint8_t i = 2; i < 7; i++) {
        sum += pkt[i];
    }
    pkt[7] = ~sum;

    // Flush RX buffer before sending
    while (Serial3.available()) Serial3.read();

    // Send request
    Serial3.write(pkt, 8);
    Serial3.flush();

    // Wait for response (timeout 10ms)
    uint32_t start = millis();
    while (Serial3.available() < 8 && (millis() - start) < 10) {
        delayMicroseconds(100);
    }

    if (Serial3.available() < 8) {
        return -1;  // Timeout
    }

    // Read response
    uint8_t resp[8];
    for (uint8_t i = 0; i < 8; i++) {
        resp[i] = Serial3.read();
    }

    // Verify header and ID
    if (resp[0] != 0xFF || resp[1] != 0xFF || resp[2] != id) {
        return -1;  // Invalid response
    }

    // Check for error (byte 4)
    if (resp[4] != 0) {
        return -1;  // Servo reported error
    }

    // Extract position (little-endian)
    uint16_t position = resp[5] | (resp[6] << 8);
    return position;
}

/**
 * Read and print positions of all servos
 */
void print_all_positions() {
    Serial.print("Positions: ");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        int16_t pos = read_position(SERVO_IDS[i]);
        Serial.print("ID");
        Serial.print(SERVO_IDS[i]);
        Serial.print("=");
        if (pos >= 0) {
            Serial.print(pos);
            Serial.print(" (");
            Serial.print(position_to_degrees(pos), 1);
            Serial.print("deg)");
        } else {
            Serial.print("ERR");
        }
        if (i < NUM_SERVOS - 1) Serial.print(", ");
    }
    Serial.println();
}

// ============== SYNC WRITE FUNCTION ==============

/**
 * Send SYNC WRITE command to multiple servos
 *
 * Packet structure:
 *   FF FF FE LEN INSTR ADDR DATA_LEN [ID1 D1 D2 D3 D4 D5 D6] [ID2...] CHECKSUM
 *
 * Where:
 *   FE = Broadcast ID (no response expected)
 *   LEN = (DATA_LEN + 1) * N + 4
 *   INSTR = 0x83 (SYNC WRITE)
 *   ADDR = Starting register address (0x2A for goal position)
 *   DATA_LEN = Bytes per servo (6: pos_lo, pos_hi, time_lo, time_hi, speed_lo, speed_hi)
 *   [IDn D1 D2 D3 D4 D5 D6] = Servo ID + data block
 *
 * @param ids Array of servo IDs
 * @param positions Array of goal positions (0-4095)
 * @param speeds Array of speeds (0-1023)
 * @param count Number of servos
 * @return true if packet sent successfully
 */
bool sync_write_positions(const uint8_t* ids, const uint16_t* positions,
                          const uint16_t* speeds, uint8_t count) {
    if (count == 0 || count > 6) return false;

    // Calculate packet size
    const uint8_t DATA_LEN = 6;  // pos_lo, pos_hi, time_lo, time_hi, speed_lo, speed_hi
    uint8_t pkt_len = (DATA_LEN + 1) * count + 4;
    uint8_t pkt[64];  // Max 6 servos * 7 bytes + 7 header = 49 bytes

    // Build packet header
    pkt[0] = FEETECH_HEADER_1;
    pkt[1] = FEETECH_HEADER_2;
    pkt[2] = BROADCAST_ID;
    pkt[3] = pkt_len;
    pkt[4] = INSTR_SYNC_WRITE;
    pkt[5] = REG_GOAL_POSITION_L;
    pkt[6] = DATA_LEN;

    // Add servo data blocks
    uint8_t idx = 7;
    for (uint8_t i = 0; i < count; i++) {
        pkt[idx++] = ids[i];
        pkt[idx++] = positions[i] & 0xFF;         // pos_lo
        pkt[idx++] = (positions[i] >> 8) & 0xFF;  // pos_hi
        pkt[idx++] = 0x00;                        // time_lo (0 = no acceleration control)
        pkt[idx++] = 0x00;                        // time_hi
        pkt[idx++] = speeds[i] & 0xFF;            // speed_lo
        pkt[idx++] = (speeds[i] >> 8) & 0xFF;     // speed_hi
    }

    // Calculate checksum (sum of bytes from ID to data, inverted)
    uint8_t sum = 0;
    for (uint8_t i = 2; i < idx; i++) {
        sum += pkt[i];
    }
    pkt[idx++] = ~sum;

    // Debug: print packet
    Serial.print("TX [");
    Serial.print(idx);
    Serial.print(" bytes]: ");
    for (uint8_t i = 0; i < idx; i++) {
        if (pkt[i] < 0x10) Serial.print("0");
        Serial.print(pkt[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Flush RX buffer before sending
    while (Serial3.available()) Serial3.read();

    // Send packet
    size_t sent = Serial3.write(pkt, idx);
    Serial3.flush();  // Wait for TX complete

    // SYNC WRITE has no response (broadcast)
    return (sent == idx);
}

// ============== HELPER FUNCTIONS ==============

void move_all_to_position(uint16_t position) {
    uint16_t positions[NUM_SERVOS];
    uint16_t speeds[NUM_SERVOS];

    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        positions[i] = position;
        speeds[i] = MOVE_SPEED;
    }

    Serial.print("Moving all servos to position: ");
    Serial.println(position);

    bool ok = sync_write_positions(SERVO_IDS, positions, speeds, NUM_SERVOS);
    Serial.print("Result: ");
    Serial.println(ok ? "OK" : "FAIL");
}

/**
 * Move a SINGLE servo to zero position (2048)
 * Uses individual WRITE command (not SYNC WRITE) for single servo
 */
void move_single_servo_to_zero(uint8_t id) {
    uint8_t pkt[15];
    uint16_t position = POS_CENTER;  // 2048 = zero
    uint16_t time_val = 0;           // No time-based acceleration
    uint16_t speed = MOVE_SPEED;     // 150

    // Build WRITE DATA packet
    // Write 6 bytes starting at 0x2A: pos_l, pos_h, time_l, time_h, speed_l, speed_h
    pkt[0] = FEETECH_HEADER_1;
    pkt[1] = FEETECH_HEADER_2;
    pkt[2] = id;
    pkt[3] = 9;  // Length: INSTR + ADDR + 6 data bytes + checksum overhead = 9
    pkt[4] = INSTR_WRITE;
    pkt[5] = REG_GOAL_POSITION_L;
    pkt[6] = position & 0xFF;         // pos_lo
    pkt[7] = (position >> 8) & 0xFF;  // pos_hi
    pkt[8] = time_val & 0xFF;         // time_lo
    pkt[9] = (time_val >> 8) & 0xFF;  // time_hi
    pkt[10] = speed & 0xFF;           // speed_lo
    pkt[11] = (speed >> 8) & 0xFF;    // speed_hi

    // Calculate checksum
    uint8_t sum = 0;
    for (uint8_t i = 2; i < 12; i++) {
        sum += pkt[i];
    }
    pkt[12] = ~sum;

    // Debug output
    Serial.print(">>> Moving SERVO ID ");
    Serial.print(id);
    Serial.print(" to ZERO (2048) at speed ");
    Serial.println(speed);
    Serial.print("TX: ");
    for (uint8_t i = 0; i < 13; i++) {
        if (pkt[i] < 0x10) Serial.print("0");
        Serial.print(pkt[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Flush RX buffer
    while (Serial3.available()) Serial3.read();

    // Send packet
    Serial3.write(pkt, 13);
    Serial3.flush();

    // Wait for response (non-broadcast)
    delay(10);

    // Read and print any response
    if (Serial3.available()) {
        Serial.print("RX: ");
        while (Serial3.available()) {
            uint8_t b = Serial3.read();
            if (b < 0x10) Serial.print("0");
            Serial.print(b, HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

/**
 * Run sequential test: move each servo to zero one at a time
 */
void run_sequential_zero_test() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("SEQUENTIAL ZERO POSITION TEST");
    Serial.println("Moving each servo to 2048 (zero) one at a time");
    Serial.println("Speed: 150");
    Serial.println("========================================");
    Serial.println();

    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        uint8_t id = SERVO_IDS[i];

        Serial.println("----------------------------------------");
        Serial.print("Testing servo ");
        Serial.print(i + 1);
        Serial.print(" of ");
        Serial.print(NUM_SERVOS);
        Serial.print(" (ID ");
        Serial.print(id);
        Serial.println(")");

        // Read position BEFORE move
        int16_t pos_before = read_position(id);
        Serial.print("  Position BEFORE: ");
        if (pos_before >= 0) {
            Serial.print(pos_before);
            Serial.print(" (");
            Serial.print(position_to_degrees(pos_before), 1);
            Serial.println(" deg)");
        } else {
            Serial.println("READ ERROR");
        }

        // Move to zero
        move_single_servo_to_zero(id);

        // Wait for servo to reach target position (within 10 counts tolerance)
        const int16_t POSITION_TOLERANCE = 10;  // counts
        const uint32_t TIMEOUT_MS = 10000;      // 10 second timeout
        uint32_t start_time = millis();
        int16_t current_pos;
        bool reached = false;

        Serial.println("  Waiting for servo to reach 2048...");
        uint32_t last_progress_print = 0;
        while ((millis() - start_time) < TIMEOUT_MS) {
            current_pos = read_position(id);
            if (current_pos >= 0) {
                int16_t error = abs(current_pos - POS_CENTER);
                if (error <= POSITION_TOLERANCE) {
                    reached = true;
                    Serial.print("  Reached target! Position: ");
                    Serial.println(current_pos);
                    break;
                }
                // Print progress every 500ms
                if (millis() - last_progress_print > 500) {
                    last_progress_print = millis();
                    Serial.print("  Current: ");
                    Serial.print(current_pos);
                    Serial.print(" (error: ");
                    Serial.print(current_pos - POS_CENTER);
                    Serial.println(")");
                }
            }
            delay(50);  // Poll every 50ms
        }

        if (!reached) {
            Serial.println("  TIMEOUT - servo did not reach target!");
        }

        // Small settle delay
        delay(200);

        // Read position AFTER move
        int16_t pos_after = read_position(id);
        Serial.print("  Position AFTER:  ");
        if (pos_after >= 0) {
            Serial.print(pos_after);
            Serial.print(" (");
            Serial.print(position_to_degrees(pos_after), 1);
            Serial.println(" deg)");

            // Calculate difference from expected 2048
            int16_t diff = pos_after - POS_CENTER;
            Serial.print("  Offset from 2048: ");
            Serial.print(diff);
            Serial.print(" counts (");
            Serial.print((float)diff * 360.0f / 4096.0f, 2);
            Serial.println(" deg)");
        } else {
            Serial.println("READ ERROR");
        }

        Serial.println();
        Serial.println("  Press any key to continue to next servo...");
        while (!Serial.available()) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(100);
        }
        while (Serial.available()) Serial.read();
    }

    Serial.println("========================================");
    Serial.println("SEQUENTIAL TEST COMPLETE");
    Serial.println("========================================");
}

float position_to_degrees(uint16_t pos) {
    // Convert encoder position to degrees from center
    return (float)(pos - 2048) * 360.0f / 4096.0f;
}

uint16_t degrees_to_position(float degrees) {
    // Convert degrees from center to encoder position
    int32_t pos = 2048 + (int32_t)(degrees * 4096.0f / 360.0f);
    if (pos < 0) pos = 0;
    if (pos > 4095) pos = 4095;
    return (uint16_t)pos;
}

// ============== SETUP ==============

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // USB Serial for debug
    Serial.begin(115200);
    delay(2000);  // Wait for serial monitor

    Serial.println("\n========================================");
    Serial.println("Feetech SYNC WRITE Test");
    Serial.println("========================================");
    Serial.println();

    // Servo UART
    Serial3.begin(SERVO_BAUD);
    delay(100);

    Serial.print("Serial3 initialized @ ");
    Serial.print(SERVO_BAUD);
    Serial.println(" baud");
    Serial.println();

    // Print configuration
    Serial.print("Servo IDs: ");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        Serial.print(SERVO_IDS[i]);
        if (i < NUM_SERVOS - 1) Serial.print(", ");
    }
    Serial.println();
    Serial.print("Center position: ");
    Serial.print(POS_CENTER);
    Serial.print(" (");
    Serial.print(position_to_degrees(POS_CENTER), 1);
    Serial.println(" deg)");
    Serial.print("+25 deg position: ");
    Serial.print(POS_PLUS_25_DEG);
    Serial.print(" (");
    Serial.print(position_to_degrees(POS_PLUS_25_DEG), 1);
    Serial.println(" deg)");
    Serial.println();

    // Wait for user
    Serial.println("Press any key to start test sequence...");
    while (!Serial.available()) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(200);
    }
    while (Serial.available()) Serial.read();  // Clear buffer

    Serial.println();
    Serial.println("Entering interactive mode...");
    Serial.println("Commands:");
    Serial.println("  Z       - Run SEQUENTIAL ZERO TEST (main calibration test)");
    Serial.println("  P       - Print current positions");
    Serial.println("  T       - Toggle torque enable");
    Serial.println("  C       - Move ALL to center (2048)");
    Serial.println("  1-5     - Move single servo ID to zero");
    Serial.println("  +       - Move all +25 deg");
    Serial.println("  -       - Move all -25 deg");
    Serial.println();

    // Enable torque by default
    enable_all_torque();

    // Print initial positions
    print_all_positions();

    Serial.println();
    Serial.println("Press 'Z' to start the sequential zero test...");
}

// ============== MAIN LOOP ==============

void loop() {
    // Interactive commands
    if (Serial.available()) {
        char c = Serial.read();

        switch (c) {
            case 'Z':
            case 'z':
                // Run the main sequential zero test
                run_sequential_zero_test();
                break;

            case 'P':
            case 'p':
                print_all_positions();
                break;

            case 'C':
            case 'c':
                move_all_to_position(POS_CENTER);
                break;

            case '+':
            case '=':
                move_all_to_position(degrees_to_position(25.0f));
                break;

            case '-':
            case '_':
                move_all_to_position(degrees_to_position(-25.0f));
                break;

            case '1':
                Serial.println("Moving servo ID 1 to zero...");
                move_single_servo_to_zero(1);
                break;

            case '2':
                Serial.println("Moving servo ID 2 to zero...");
                move_single_servo_to_zero(2);
                break;

            case '3':
                Serial.println("Moving servo ID 3 to zero...");
                move_single_servo_to_zero(3);
                break;

            case '4':
                Serial.println("Moving servo ID 4 to zero...");
                move_single_servo_to_zero(4);
                break;

            case '5':
                Serial.println("Moving servo ID 5 to zero...");
                move_single_servo_to_zero(5);
                break;

            default:
                break;
        }
    }

    // Periodic position display (every 2 seconds - less spam)
    static uint32_t last_pos_print = 0;
    if (millis() - last_pos_print > 2000) {
        last_pos_print = millis();
        print_all_positions();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
