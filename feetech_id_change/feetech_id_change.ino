/*
 * feetech_id_change.ino
 *
 * Changes a single servo's ID
 * IMPORTANT: Only ONE servo should be connected at a time!
 *
 * Procedure for swapping ID 1 <-> ID 6:
 *   1. Disconnect all servos except the one currently ID 1
 *   2. Upload with FROM_ID=1, TO_ID=6
 *   3. Run, verify success
 *   4. Disconnect that servo, connect the one currently ID 6
 *   5. Upload with FROM_ID=6, TO_ID=1
 *   6. Run, verify success
 *   7. Reconnect all servos
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial3
#define SERVO_BAUD 1000000

// ============================================
// CONFIGURATION
// ============================================
#define READ_ONLY false   // true = just read ID, false = change ID

#define FROM_ID 4    // Current ID of the connected servo (or ID to scan)
#define TO_ID   3    // New ID to assign (ignored if READ_ONLY)
// ============================================

#define FEETECH_HEADER_1 0xFF
#define FEETECH_HEADER_2 0xFF
#define INSTR_READ_DATA 0x02
#define INSTR_WRITE_DATA 0x03
#define INSTR_PING 0x01

#define REG_ID 0x05
#define REG_LOCK 0x37

uint8_t calculate_checksum(uint8_t* data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {
        sum += data[i];
    }
    return ~sum;
}

bool ping_servo(uint8_t id) {
    uint8_t request[6];
    request[0] = FEETECH_HEADER_1;
    request[1] = FEETECH_HEADER_2;
    request[2] = id;
    request[3] = 0x02;
    request[4] = INSTR_PING;
    request[5] = calculate_checksum(request, 6);

    while (SERVO_SERIAL.available()) SERVO_SERIAL.read();
    SERVO_SERIAL.write(request, 6);

    unsigned long timeout = millis() + 50;
    while (SERVO_SERIAL.available() < 6 && millis() < timeout);

    if (SERVO_SERIAL.available() < 6) return false;

    uint8_t response[6];
    SERVO_SERIAL.readBytes(response, 6);

    if (response[0] != 0xFF || response[1] != 0xFF) return false;
    if (response[2] != id) return false;

    return true;
}

void write_register(uint8_t id, uint8_t reg, uint8_t value) {
    uint8_t request[8];
    request[0] = FEETECH_HEADER_1;
    request[1] = FEETECH_HEADER_2;
    request[2] = id;
    request[3] = 0x04;
    request[4] = INSTR_WRITE_DATA;
    request[5] = reg;
    request[6] = value;
    request[7] = calculate_checksum(request, 8);

    while (SERVO_SERIAL.available()) SERVO_SERIAL.read();
    SERVO_SERIAL.write(request, 8);
    delay(100);
}

uint8_t read_register(uint8_t id, uint8_t reg) {
    uint8_t request[8];
    request[0] = FEETECH_HEADER_1;
    request[1] = FEETECH_HEADER_2;
    request[2] = id;
    request[3] = 0x04;
    request[4] = INSTR_READ_DATA;
    request[5] = reg;
    request[6] = 0x01;
    request[7] = calculate_checksum(request, 8);

    while (SERVO_SERIAL.available()) SERVO_SERIAL.read();
    SERVO_SERIAL.write(request, 8);

    unsigned long timeout = millis() + 100;
    while (SERVO_SERIAL.available() < 7 && millis() < timeout);

    if (SERVO_SERIAL.available() < 7) return 0xFF;

    uint8_t response[7];
    SERVO_SERIAL.readBytes(response, 7);

    if (calculate_checksum(response, 7) != response[6]) return 0xFF;

    return response[5];
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    delay(1000);

    Serial.println("\n========================================");
    if (READ_ONLY) {
        Serial.println("Feetech Servo ID Reader");
        Serial.print("Scanning for ID ");
        Serial.println(FROM_ID);
    } else {
        Serial.print("Feetech Servo ID Change: ");
        Serial.print(FROM_ID);
        Serial.print(" -> ");
        Serial.println(TO_ID);
    }
    Serial.println("========================================");

    SERVO_SERIAL.begin(SERVO_BAUD);
    delay(100);

    // Step 1: Ping the expected ID
    Serial.print("\n[1] Pinging ID ");
    Serial.print(FROM_ID);
    Serial.print("... ");

    if (!ping_servo(FROM_ID)) {
        Serial.println("NOT FOUND!");
        Serial.println("\n    Scanning IDs 1-10...");

        // Scan common IDs
        for (uint8_t id = 1; id <= 10; id++) {
            Serial.print("    ID ");
            Serial.print(id);
            Serial.print(": ");
            if (ping_servo(id)) {
                Serial.println("FOUND!");
            } else {
                Serial.println("-");
            }
            delay(20);
        }

        Serial.println("\n[ERROR] Target ID not found.");
        Serial.println("        Check wiring and ensure only ONE servo connected.");
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(100);
        }
    }
    Serial.println("FOUND");

    // Step 2: Read current ID register to confirm
    Serial.print("\n[2] Reading ID register (addr 0x05)... ");
    uint8_t current_id = read_register(FROM_ID, REG_ID);
    Serial.println(current_id);

    // If read-only mode, stop here
    if (READ_ONLY) {
        Serial.println("\n========================================");
        Serial.println("READ ONLY MODE - No changes made");
        Serial.print("Servo ID confirmed: ");
        Serial.println(current_id);
        Serial.println("========================================");
        Serial.println("\nTo change ID, set READ_ONLY to false");
        return;
    }

    // Step 3: Unlock EEPROM
    Serial.print("\n[3] Unlocking EEPROM... ");
    write_register(FROM_ID, REG_LOCK, 0x00);
    delay(50);
    Serial.println("OK");

    // Step 4: Write new ID
    Serial.print("\n[4] Writing new ID ");
    Serial.print(TO_ID);
    Serial.print("... ");
    write_register(FROM_ID, REG_ID, TO_ID);
    delay(200);
    Serial.println("OK");

    // Step 5: Lock EEPROM (using new ID now)
    Serial.print("\n[5] Locking EEPROM... ");
    write_register(TO_ID, REG_LOCK, 0x01);
    delay(50);
    Serial.println("OK");

    // Step 6: Verify new ID responds
    Serial.print("\n[6] Verifying new ID ");
    Serial.print(TO_ID);
    Serial.print("... ");

    if (ping_servo(TO_ID)) {
        Serial.println("OK");
        Serial.println("\n========================================");
        Serial.println("SUCCESS!");
        Serial.print("Servo ID changed: ");
        Serial.print(FROM_ID);
        Serial.print(" -> ");
        Serial.println(TO_ID);
        Serial.println("========================================");
        Serial.println("\nPower cycle the servo for changes to take full effect.");
    } else {
        Serial.println("FAIL!");
        Serial.println("\n[ERROR] New ID not responding.");
        Serial.println("        Try power cycling the servo.");
    }

    // Confirm old ID no longer responds
    Serial.print("\n[7] Confirming old ID ");
    Serial.print(FROM_ID);
    Serial.print(" no longer responds... ");
    if (!ping_servo(FROM_ID)) {
        Serial.println("OK (not found, as expected)");
    } else {
        Serial.println("WARNING: Old ID still responds!");
    }
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last > 500) {
        last = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
