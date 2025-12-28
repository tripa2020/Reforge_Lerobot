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

// Servo IDs (adjust to match your robot)
const uint8_t SERVO_IDS[] = {1, 2, 3, 4, 5};
const uint8_t NUM_SERVOS = 5;

// Position constants
// STS3215: 4096 counts/revolution, center = 2048
// 25 degrees = 25/360 * 4096 ≈ 284 counts
const uint16_t POS_CENTER = 2048;
const uint16_t POS_PLUS_25_DEG = 2048 + 284;  // 2332

// Speed (steps/second: higher = faster, lower = slower, max ~3073)
const uint16_t MOVE_SPEED = 5;  // Very slow movement (~50 steps/sec ≈ 0.732 RPM)

// Feetech Protocol Constants
#define FEETECH_HEADER_1 0xFF
#define FEETECH_HEADER_2 0xFF
#define BROADCAST_ID 0xFE
#define INSTR_SYNC_WRITE 0x83
#define REG_GOAL_POSITION_L 0x2A

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
    Serial.println("========================================");
    Serial.println("TEST SEQUENCE START");
    Serial.println("========================================");
    Serial.println();

    // Step 1: Move to center
    Serial.println("Step 1: Moving all joints to CENTER (0 deg)");
    move_all_to_position(POS_CENTER);
    Serial.println("Waiting 3 seconds...");
    delay(3000);
    Serial.println();

    // Step 2: Move to +25 degrees
    Serial.println("Step 2: Moving all joints to +25 degrees");
    move_all_to_position(POS_PLUS_25_DEG);
    Serial.println("Waiting 3 seconds...");
    delay(3000);
    Serial.println();

    // Step 3: Return to center
    Serial.println("Step 3: Returning to CENTER");
    move_all_to_position(POS_CENTER);
    Serial.println();

    Serial.println("========================================");
    Serial.println("TEST SEQUENCE COMPLETE");
    Serial.println("========================================");
    Serial.println();
    Serial.println("Entering interactive mode...");
    Serial.println("Commands:");
    Serial.println("  C       - Move to center");
    Serial.println("  +       - Move +25 deg");
    Serial.println("  -       - Move -25 deg");
    Serial.println("  0-9     - Move to position (0=0, 9=4095)");
    Serial.println();
}

// ============== MAIN LOOP ==============

void loop() {
    // Interactive commands
    if (Serial.available()) {
        char c = Serial.read();

        switch (c) {
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

            case '0':
                move_all_to_position(0);
                break;

            case '5':
                move_all_to_position(2048);
                break;

            case '9':
                move_all_to_position(4095);
                break;

            default:
                if (c >= '1' && c <= '8') {
                    uint16_t pos = (c - '0') * 512;  // 0-4096 in 9 steps
                    move_all_to_position(pos);
                }
                break;
        }
    }

    // Heartbeat
    static uint32_t last_blink = 0;
    if (millis() - last_blink > 500) {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
