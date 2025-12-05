/*
 * ISM330DHCX Bare Metal Driver Test for Teensy 4.1
 * -------------------------------------------------
 * ZERO external dependencies - just <SPI.h>
 *
 * Refactored from Adafruit hybrid to pure bare metal per:
 * - ISM330DHCX Datasheet DS13012 Rev 7
 * - Forum research on SPI_MODE3 and Teensy 4.x timing
 *
 * WIRING (Default SPI pins):
 *   Sensor CS   -> Teensy pin 10 (with 10k pull-down to GND) **CRITICAL**
 *   Sensor SCL  -> Teensy pin 13 (SCK)
 *   Sensor SDA  -> Teensy pin 11 (MOSI)
 *   Sensor SDO  -> Teensy pin 12 (MISO)
 *   Sensor VCC  -> 3.3V
 *   Sensor GND  -> GND
 *
 * CRITICAL: 10k ohm resistor REQUIRED from CS (pin 10) to GND
 *           ISM330DHCX samples CS pin at power-up ONLY to determine interface mode
 *           CS HIGH/floating → I2C mode lock (no SPI recovery until power cycle)
 *           CS LOW → SPI mode (correct operation)
 *           Per datasheet DS13012 Rev 7, Section 9
 */

#include <SPI.h>

// ============== HARDWARE CONFIGURATION ==============
#define PIN_CS    10   // Default SPI CS
#define PIN_SCK   13   // Default SPI SCK
#define PIN_MOSI  11   // Default SPI MOSI
#define PIN_MISO  12   // Default SPI MISO

// ============== ISM330DHCX REGISTER MAP (DS13012 Rev 7, Section 8) ==============
#define REG_WHO_AM_I    0x0F  // Device ID (should return 0x6B)
#define REG_CTRL1_XL    0x10  // Accel ODR and FS
#define REG_CTRL2_G     0x11  // Gyro ODR and FS
#define REG_CTRL3_C     0x12  // BDU, IF_INC, SW_RESET
#define REG_CTRL4_C     0x13  // I2C_disable
#define REG_CTRL9_XL    0x18  // DEVICE_CONF
#define REG_STATUS_REG  0x1E  // Data ready status
#define REG_OUT_TEMP_L  0x20  // Temperature low byte
#define REG_OUTX_L_G    0x22  // Gyro X low byte (start of 6-byte gyro block)
#define REG_OUTX_L_A    0x28  // Accel X low byte (start of 6-byte accel block)

// ============== SENSITIVITY FACTORS ==============
// Accel: ±2g = 0.061 mg/LSB [cite: Table 2]
// Gyro: ±125dps = 4.375 mdps/LSB [cite: Table 3]
const float ACCEL_SENSITIVITY = 0.061f / 1000.0f * 9.80665f;  // mg -> m/s²
const float GYRO_SENSITIVITY  = 4.375f / 1000.0f * (PI / 180.0f);  // mdps -> rad/s

// ============== SPI CONFIGURATION ==============
// MODE0 (CPOL=0, CPHA=0) chosen over MODE3 based on comprehensive diagnostic testing
// Diagnostic results: MODE0 6/6 success (100% reliable), MODE3 5/6 success (first read fails)
// MODE3 shows intermittent first-read failure: [0x0, 0x6B, 0x6B] pattern
// Root cause: Teensy 4.1's 600MHz CPU + MODE3's clock-idle-HIGH leaves insufficient settling time
// See ISM330DHCX.md and CHANGELOG.md [2025-12-03] for full diagnostic details
SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);

// ============== SENSOR DATA STRUCTURE ==============
struct SensorData {
    float accelX, accelY, accelZ;  // m/s²
    float gyroX, gyroY, gyroZ;     // rad/s
    float temperature;              // °C
    bool valid;
};

// ============== LOW-LEVEL SPI PRIMITIVES ==============
// These are timing-safe for Teensy 4.x (600MHz)

uint8_t readRegister(uint8_t reg) {
    uint8_t value;

    SPI.beginTransaction(spiSettings);
    digitalWriteFast(PIN_CS, LOW);
    delayNanoseconds(50);  // t_su(CS) setup time [cite: Table 6]

    SPI.transfer(reg | 0x80);  // MSB=1 for read
    value = SPI.transfer(0x00);

    digitalWriteFast(PIN_CS, HIGH);
    delayNanoseconds(50);  // t_h(CS) hold time
    SPI.endTransaction();

    return value;
}

void writeRegister(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(spiSettings);
    digitalWriteFast(PIN_CS, LOW);
    delayNanoseconds(50);

    SPI.transfer(reg & 0x7F);  // MSB=0 for write
    SPI.transfer(value);

    digitalWriteFast(PIN_CS, HIGH);
    delayNanoseconds(50);
    SPI.endTransaction();
}

// Burst read for synchronized multi-byte reads (essential for data coherency)
void readRegisterBurst(uint8_t startReg, uint8_t* buffer, uint8_t len) {
    SPI.beginTransaction(spiSettings);
    digitalWriteFast(PIN_CS, LOW);
    delayNanoseconds(50);

    SPI.transfer(startReg | 0x80);  // Read mode with auto-increment (IF_INC=1)
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = SPI.transfer(0x00);
    }

    digitalWriteFast(PIN_CS, HIGH);
    delayNanoseconds(50);
    SPI.endTransaction();
}

// ============== INITIALIZATION SEQUENCE ==============
// Per DS13012 Rev 7, Section 7 [cite: 1073-1076]

bool initSensor() {
    Serial.println("\n=== ISM330DHCX BARE METAL INIT ===");

    // Step 1: Initialize SPI hardware
    SPI.begin();
    pinMode(PIN_CS, OUTPUT);
    digitalWriteFast(PIN_CS, HIGH);  // Deselect

    // Step 2: Power-on stabilization [cite: Table 4]
    // Turn-on time is 35ms typical, increased to 100ms for MODE0 reliability
    // (Originally 50ms - increased during MODE3→MODE0 diagnostic troubleshooting)
    delay(100);

    // Step 3: Verify WHO_AM_I [cite: 387]
    uint8_t whoami = readRegister(REG_WHO_AM_I);
    Serial.print("WHO_AM_I: 0x");
    Serial.print(whoami, HEX);
    if (whoami != 0x6B) {
        Serial.println(" [FAIL - expected 0x6B]");
        return false;
    }
    Serial.println(" [OK]");

    // Step 4: Software Reset [cite: 404]
    Serial.print("Software reset... ");
    writeRegister(REG_CTRL3_C, 0x01);  // SW_RESET=1
    delay(50);  // Wait for reset to complete

    // Verify reset cleared
    uint8_t ctrl3 = readRegister(REG_CTRL3_C);
    if (ctrl3 & 0x01) {
        Serial.println("[FAIL - reset bit stuck]");
        return false;
    }
    Serial.println("[OK]");

    // Step 5: Configure CTRL3_C [cite: 404]
    // BDU=1 (bit 6): Block Data Update - prevents reading mixed samples
    // IF_INC=1 (bit 2): Auto-increment address for burst reads
    // 0x44 = 0100 0100
    writeRegister(REG_CTRL3_C, 0x44);
    Serial.println("CTRL3_C: 0x44 (BDU=1, IF_INC=1)");

    // Step 6: Disable I2C [cite: 422]
    // I2C_disable=1 (bit 2): Prevents I2C block from interfering
    // 0x04 = 0000 0100
    writeRegister(REG_CTRL4_C, 0x04);
    Serial.println("CTRL4_C: 0x04 (I2C_disable=1)");

    // Step 7: Set DEVICE_CONF [cite: 1455]
    // DEVICE_CONF=1 (bit 1): "Recommended to always set"
    // 0x02 = 0000 0010
    writeRegister(REG_CTRL9_XL, 0x02);
    Serial.println("CTRL9_XL: 0x02 (DEVICE_CONF=1)");

    // Step 8: Configure Accelerometer [cite: 395]
    // ODR_XL[7:4] = 0101 (208 Hz)
    // FS_XL[3:2] = 00 (±2g)
    // 0x50 = 0101 0000
    writeRegister(REG_CTRL1_XL, 0x50);
    Serial.println("CTRL1_XL: 0x50 (208Hz, +/-2g)");

    // Step 9: Configure Gyroscope [cite: 402]
    // ODR_G[7:4] = 0101 (208 Hz)
    // FS_G[3:1] = 100 (±125 dps)
    // 0x58 = 0101 1000
    writeRegister(REG_CTRL2_G, 0x58);
    Serial.println("CTRL2_G: 0x58 (208Hz, +/-125dps)");

    // Step 10: Verification readback
    Serial.println("\n--- Register Verification ---");

    uint8_t v_ctrl3 = readRegister(REG_CTRL3_C);
    uint8_t v_ctrl4 = readRegister(REG_CTRL4_C);
    uint8_t v_ctrl9 = readRegister(REG_CTRL9_XL);
    uint8_t v_ctrl1 = readRegister(REG_CTRL1_XL);
    uint8_t v_ctrl2 = readRegister(REG_CTRL2_G);

    Serial.print("CTRL3_C: 0x"); Serial.print(v_ctrl3, HEX);
    Serial.println((v_ctrl3 == 0x44) ? " [OK]" : " [MISMATCH]");

    Serial.print("CTRL4_C: 0x"); Serial.print(v_ctrl4, HEX);
    Serial.println((v_ctrl4 == 0x04) ? " [OK]" : " [MISMATCH]");

    Serial.print("CTRL9_XL: 0x"); Serial.print(v_ctrl9, HEX);
    Serial.println((v_ctrl9 == 0x02) ? " [OK]" : " [MISMATCH]");

    Serial.print("CTRL1_XL: 0x"); Serial.print(v_ctrl1, HEX);
    Serial.println((v_ctrl1 == 0x50) ? " [OK]" : " [MISMATCH]");

    Serial.print("CTRL2_G: 0x"); Serial.print(v_ctrl2, HEX);
    Serial.println((v_ctrl2 == 0x58) ? " [OK]" : " [MISMATCH]");

    bool allOk = (v_ctrl3 == 0x44) && (v_ctrl4 == 0x04) &&
                 (v_ctrl9 == 0x02) && (v_ctrl1 == 0x50) && (v_ctrl2 == 0x58);

    if (allOk) {
        Serial.println("\n=== INIT SUCCESS ===");
    } else {
        Serial.println("\n[WARN] Some registers did not verify!");
    }

    return allOk;
}

// ============== SENSOR READ FUNCTION ==============
// Burst reads all sensor data in one SPI transaction for coherency

SensorData readSensor() {
    SensorData data;
    uint8_t buffer[14];  // Temp(2) + Gyro(6) + Accel(6)

    // Burst read starting from OUT_TEMP_L (0x20)
    // Registers are contiguous: TEMP(2), GYRO(6), ACCEL(6) = 14 bytes
    readRegisterBurst(REG_OUT_TEMP_L, buffer, 14);

    // Parse temperature (16-bit signed, 256 LSB/°C, offset 25°C) [cite: 42]
    int16_t rawTemp = (int16_t)(buffer[1] << 8 | buffer[0]);
    data.temperature = 25.0f + (rawTemp / 256.0f);

    // Parse gyroscope (bytes 2-7)
    int16_t rawGyroX = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t rawGyroY = (int16_t)(buffer[5] << 8 | buffer[4]);
    int16_t rawGyroZ = (int16_t)(buffer[7] << 8 | buffer[6]);

    data.gyroX = rawGyroX * GYRO_SENSITIVITY;
    data.gyroY = rawGyroY * GYRO_SENSITIVITY;
    data.gyroZ = rawGyroZ * GYRO_SENSITIVITY;

    // Parse accelerometer (bytes 8-13)
    int16_t rawAccelX = (int16_t)(buffer[9] << 8 | buffer[8]);
    int16_t rawAccelY = (int16_t)(buffer[11] << 8 | buffer[10]);
    int16_t rawAccelZ = (int16_t)(buffer[13] << 8 | buffer[12]);

    data.accelX = rawAccelX * ACCEL_SENSITIVITY;
    data.accelY = rawAccelY * ACCEL_SENSITIVITY;
    data.accelZ = rawAccelZ * ACCEL_SENSITIVITY;

    // Validation: check for stuck-at-zero (bus failure indicator)
    data.valid = !(rawAccelX == 0 && rawAccelY == 0 && rawAccelZ == 0 &&
                   rawGyroX == 0 && rawGyroY == 0 && rawGyroZ == 0);

    return data;
}

// ============== CHECK DATA READY ==============
bool isDataReady() {
    uint8_t status = readRegister(REG_STATUS_REG);
    // Bit 0: XLDA (accel), Bit 1: GDA (gyro), Bit 2: TDA (temp)
    return (status & 0x07) == 0x07;  // All three ready
}

// ============== RANGE QUERY FUNCTIONS ==============
// Returns accelerometer full-scale range bits [3:2] from CTRL1_XL
// 00 = ±2g, 01 = ±16g, 10 = ±4g, 11 = ±8g
uint8_t getAccelRange() {
    uint8_t ctrl1 = readRegister(REG_CTRL1_XL);
    return (ctrl1 >> 2) & 0x03;  // Extract bits [3:2]
}

// Returns gyroscope full-scale range bits [3:1] from CTRL2_G
// 000 = ±250 dps, 001 = ±500 dps, 010 = ±1000 dps, 011 = ±2000 dps, 100 = ±125 dps
uint8_t getGyroRange() {
    uint8_t ctrl2 = readRegister(REG_CTRL2_G);
    return (ctrl2 >> 1) & 0x07;  // Extract bits [3:1]
}

// ============== SETUP ==============
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("\n========================================");
    Serial.println("ISM330DHCX Bare Metal Test (Teensy 4.1)");
    Serial.println("========================================");
    Serial.println("SPI pins: CS=10, SCK=13, MOSI=11, MISO=12");
    Serial.println("Config: SPI_MODE0, 4MHz");
    Serial.println("Ensure 10k pull-down on CS to GND!\n");

    if (!initSensor()) {
        Serial.println("\n*** INIT FAILED ***");
        Serial.println("Troubleshooting:");
        Serial.println("1. Power cycle the sensor");
        Serial.println("2. Check 10k pull-down on CS pin");
        Serial.println("3. Verify wiring: CS=10, SCK=13, MOSI=11, MISO=12");
        Serial.println("4. Check 3.3V power supply");
        while (1) {
            digitalToggle(LED_BUILTIN);
            delay(100);
        }
    }

    // Display configured ranges
    Serial.println("");
    uint8_t accel_range = getAccelRange();
    Serial.print("Accel Range: ");
    switch(accel_range) {
        case 0b00: Serial.println("+/-2G"); break;
        case 0b01: Serial.println("+/-16G"); break;
        case 0b10: Serial.println("+/-4G"); break;
        case 0b11: Serial.println("+/-8G"); break;
    }

    uint8_t gyro_range = getGyroRange();
    Serial.print("Gyro Range: ");
    switch(gyro_range) {
        case 0b000: Serial.println("250 DPS"); break;
        case 0b001: Serial.println("500 DPS"); break;
        case 0b010: Serial.println("1000 DPS"); break;
        case 0b011: Serial.println("2000 DPS"); break;
        case 0b100: Serial.println("125 DPS"); break;
        default: Serial.println("Unknown"); break;
    }

    Serial.println("\nStarting sensor reads...\n");
    pinMode(LED_BUILTIN, OUTPUT);
}

// ============== MAIN LOOP ==============
void loop() {
    // Wait for new data to be ready (like Adafruit library does)
    uint32_t wait_start = micros();
    while (!isDataReady()) {
        // Poll STATUS register until all data ready
        // At 208Hz, new sample every ~4.8ms
    }
    uint32_t wait_time_us = micros() - wait_start;

    // Read sensor data with timing measurement
    uint32_t read_start = micros();
    SensorData data = readSensor();
    uint32_t read_time_us = micros() - read_start;

    if (!data.valid) {
        Serial.println("WARNING: All readings zero - SPI bus failure?");
    }

    // Display timing (target: <35µs for Phase2 integration)
    Serial.print("Wait for data ready: ");
    Serial.print(wait_time_us);
    Serial.print(" us | Read time: ");
    Serial.print(read_time_us);
    Serial.println(" us");

    // Display temperature
    Serial.print("Temp: ");
    Serial.print(data.temperature, 1);
    Serial.println(" C");

    // Display accelerometer (m/s²)
    Serial.print("Accel [m/s2]: X=");
    Serial.print(data.accelX, 2);
    Serial.print("  Y=");
    Serial.print(data.accelY, 2);
    Serial.print("  Z=");
    Serial.println(data.accelZ, 2);

    // Display gyroscope (rad/s)
    Serial.print("Gyro [rad/s]: X=");
    Serial.print(data.gyroX, 4);
    Serial.print("  Y=");
    Serial.print(data.gyroY, 4);
    Serial.print("  Z=");
    Serial.println(data.gyroZ, 4);

    Serial.println();

    // Blink LED to show activity
    digitalToggle(LED_BUILTIN);

    // 208 Hz ODR = ~4.8ms period, delay 100ms for readable output
    delay(100);
}
