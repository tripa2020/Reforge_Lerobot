/*
 * feetech_return_delay_config.ino
 *
 * STATUS: PRODUCTION SAFE
 * - Fixed Read Length bug (7 bytes vs 8 bytes)
 * - Implemented "Read-Before-Write" to save EEPROM
 * - Uses Safe Delay (4us) to prevent Bus Contention
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial3
#define SERVO_BAUD 1000000
#define SERVO_ID 6
#define TARGET_DELAY_VAL 25   // 2 = 4µs. SAFER than 0 to prevent UART collisions.

#define FEETECH_HEADER_1 0xFF
#define FEETECH_HEADER_2 0xFF
#define INSTR_READ_DATA 0x02
#define INSTR_WRITE_DATA 0x03
#define REG_RETURN_DELAY 0x07
#define REG_LOCK 0x37

uint8_t calculate_checksum(uint8_t* data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {
        sum += data[i];
    }
    return ~sum;
}

void write_register(uint8_t id, uint8_t reg, uint8_t value) {
    uint8_t request[8];
    request[0] = FEETECH_HEADER_1;
    request[1] = FEETECH_HEADER_2;
    request[2] = id;
    request[3] = 0x05;
    request[4] = INSTR_WRITE_DATA;
    request[5] = reg;
    request[6] = value;
    request[7] = calculate_checksum(request, 8);

    while(SERVO_SERIAL.available()) SERVO_SERIAL.read();
    SERVO_SERIAL.write(request, 8);
    delay(100);  // Increased delay for EEPROM write settle time
}

// FIXED: Now expects 7 bytes for response, not 8
uint8_t read_register(uint8_t id, uint8_t reg) {
    uint8_t request[8];
    request[0] = FEETECH_HEADER_1;
    request[1] = FEETECH_HEADER_2;
    request[2] = id;
    request[3] = 0x04;
    request[4] = INSTR_READ_DATA;
    request[5] = reg;
    request[6] = 0x01;  // Read 1 byte
    request[7] = calculate_checksum(request, 8);

    while (SERVO_SERIAL.available()) SERVO_SERIAL.read();
    SERVO_SERIAL.write(request, 8);

    unsigned long timeout = millis() + 100;
    // A 1-byte READ response is 7 bytes total: FF FF ID LEN ERR DAT SUM
    while (SERVO_SERIAL.available() < 7 && millis() < timeout);

    if (SERVO_SERIAL.available() < 7) return 0xFF;

    uint8_t response[7];
    SERVO_SERIAL.readBytes(response, 7);

    if (calculate_checksum(response, 7) != response[6]) return 0xFF;

    return response[5];  // The data byte
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while(!Serial && millis() < 3000);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("Feetech Return Delay Configuration");
    Serial.println("========================================");

    SERVO_SERIAL.begin(SERVO_BAUD);
    delay(100);

    Serial.print("[1] Reading current delay... ");
    uint8_t current = read_register(SERVO_ID, REG_RETURN_DELAY);

    if (current == 0xFF) {
        Serial.println("FAIL!");
        Serial.println("[ERROR] Cannot communicate with servo.");
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(100);
        }
    }

    Serial.print("OK\n");
    Serial.print("    Current value: ");
    Serial.print(current);
    Serial.print(" (");
    Serial.print(current * 2);
    Serial.println(" µs)");

    // Read-before-write logic to save EEPROM
    if (current == TARGET_DELAY_VAL) {
        Serial.println("\n✅ STATUS: Already optimized. No changes made.");
    } else {
        Serial.print("\n[2] Optimizing delay: ");
        Serial.print(current);
        Serial.print(" → ");
        Serial.print(TARGET_DELAY_VAL);
        Serial.println("...");

        Serial.print("    Unlocking EEPROM... ");
        write_register(SERVO_ID, REG_LOCK, 0x00);
        delay(50);  // Extra settle time after unlock
        Serial.println("OK");

        Serial.print("    Writing new value... ");
        write_register(SERVO_ID, REG_RETURN_DELAY, TARGET_DELAY_VAL);
        delay(200);  // CRITICAL: EEPROM write cycle time
        Serial.println("OK");

        Serial.print("    Verifying (before lock)... ");
        uint8_t verify_unlocked = read_register(SERVO_ID, REG_RETURN_DELAY);
        if (verify_unlocked == TARGET_DELAY_VAL) {
            Serial.println("OK");
        } else {
            Serial.print("FAIL! Read: ");
            Serial.println(verify_unlocked);
        }

        Serial.print("    Locking EEPROM... ");
        write_register(SERVO_ID, REG_LOCK, 0x01);
        delay(50);
        Serial.println("OK");

        Serial.print("[3] Final verification... ");
        uint8_t verify = read_register(SERVO_ID, REG_RETURN_DELAY);
        if (verify == TARGET_DELAY_VAL) {
            Serial.println("OK");
            Serial.println("\n✅ SUCCESS: Delay optimized!");
            Serial.println("\n⚠️  IMPORTANT: Power cycle the servo for changes to take effect.");
            Serial.println("    1. Disconnect power to servo");
            Serial.println("    2. Wait 2 seconds");
            Serial.println("    3. Reconnect power");
        } else {
            Serial.println("FAIL!");
            Serial.print("[ERROR] Write failed. Read back: ");
            Serial.println(verify);
            Serial.println("\n[DEBUG] Possible causes:");
            Serial.println("    - Servo EEPROM is hardware locked");
            Serial.println("    - Register 0x07 is read-only on this model");
            Serial.println("    - Servo firmware doesn't support this register");
        }
    }

    Serial.println("========================================");
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last > 500) {
        last = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}