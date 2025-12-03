// Basic demo for accelerometer/gyro readings from Adafruit ISM330DHCX
// CONFIGURED FOR SPI1 PERIPHERAL (Teensy 4.1)
//
// WIRING (SPI1 pins - tested and working):
//   Sensor CS   -> Teensy pin 0 (with 10k pull-down to GND)
//   Sensor SCL  -> Teensy pin 27 (SCK1)
//   Sensor SDA  -> Teensy pin 26 (MOSI1)
//   Sensor SDO  -> Teensy pin 1 (MISO1)
//   Sensor VCC  -> 3.3V
//   Sensor GND  -> GND
//
// CRITICAL: 10k ohm resistor required from CS (pin 0) to GND
//           This ensures CS is LOW during sensor power-up (forces SPI mode)

#include <SPI.h>
#include <Adafruit_ISM330DHCX.h>

// SPI1 pins (tested and verified working)
#define LSM_CS 0       // SPI1 CS
#define LSM_SCK 27     // SPI1 SCK
#define LSM_MISO 1     // SPI1 MISO
#define LSM_MOSI 26    // SPI1 MOSI

// ISM330DHCX register addresses (DS13012 Rev 7)
#define ISM330_CTRL3_C   0x12  // Control register 3 (BDU, etc.)
#define ISM330_CTRL4_C   0x13  // Control register 4 (I2C_disable, etc.)
#define ISM330_CTRL9_XL  0x18  // Control register 9 (DEVICE_CONF, etc.)
#define ISM330_WHO_AM_I  0x0F  // Device ID register (should return 0x6B)

Adafruit_ISM330DHCX ism330dhcx;

// Direct SPI register access (required because Adafruit library doesn't expose register writes)
// Updated per forum research:
//   - SPI_MODE3: CPOL=1 (clock idles HIGH), CPHA=1 (data on falling edge)
//   - digitalWriteFast(): Faster CS toggling on Teensy
//   - delayNanoseconds(125): Teensy 4.x is too fast, slave needs setup time
uint8_t imu_read_register(uint8_t reg) {
    uint8_t value;
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    digitalWriteFast(LSM_CS, LOW);
    #if defined(__IMXRT1062__)
    delayNanoseconds(125);  // CRITICAL: Teensy 4.x setup time
    #endif
    SPI1.transfer(reg | 0x80);  // Read operation: MSB = 1
    value = SPI1.transfer(0x00);
    digitalWriteFast(LSM_CS, HIGH);
    #if defined(__IMXRT1062__)
    delayNanoseconds(125);  // Hold time before next transaction
    #endif
    SPI1.endTransaction();
    return value;
}

void imu_write_register(uint8_t reg, uint8_t value) {
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    digitalWriteFast(LSM_CS, LOW);
    #if defined(__IMXRT1062__)
    delayNanoseconds(125);  // CRITICAL: Teensy 4.x setup time
    #endif
    SPI1.transfer(reg & 0x7F);  // Write operation: MSB = 0
    SPI1.transfer(value);
    digitalWriteFast(LSM_CS, HIGH);
    #if defined(__IMXRT1062__)
    delayNanoseconds(125);  // Hold time
    #endif
    SPI1.endTransaction();
}

// Configure ISM330DHCX registers per datasheet DS13012 Rev 7 Section 7.3/9
void imu_configure_registers() {
    Serial.println("\n=== ISM330DHCX REGISTER CONFIGURATION ===");

    // Verify WHO_AM_I (0x6B for ISM330DHCX)
    uint8_t who_am_i = imu_read_register(ISM330_WHO_AM_I);
    Serial.print("WHO_AM_I: 0x");
    Serial.print(who_am_i, HEX);
    Serial.println(who_am_i == 0x6B ? " [OK]" : " [UNEXPECTED]");

    // 1. Enable BDU (Block Data Update) - CTRL3_C bit 6
    uint8_t ctrl3 = imu_read_register(ISM330_CTRL3_C);
    ctrl3 |= (1 << 6);  // Set BDU bit
    imu_write_register(ISM330_CTRL3_C, ctrl3);
    Serial.print("CTRL3_C: 0x");
    Serial.print(ctrl3, HEX);
    Serial.println(" (BDU=1)");

    // 2. Disable I2C interface - CTRL4_C bit 2
    uint8_t ctrl4 = imu_read_register(ISM330_CTRL4_C);
    ctrl4 |= (1 << 2);  // Set I2C_disable bit
    imu_write_register(ISM330_CTRL4_C, ctrl4);
    Serial.print("CTRL4_C: 0x");
    Serial.print(ctrl4, HEX);
    Serial.println(" (I2C_disable=1)");

    // 3. Set DEVICE_CONF - CTRL9_XL bit 1
    uint8_t ctrl9 = imu_read_register(ISM330_CTRL9_XL);
    ctrl9 |= (1 << 1);  // Set DEVICE_CONF bit
    imu_write_register(ISM330_CTRL9_XL, ctrl9);
    Serial.print("CTRL9_XL: 0x");
    Serial.print(ctrl9, HEX);
    Serial.println(" (DEVICE_CONF=1)");

    // Verify all writes
    Serial.println("--- Verification ---");
    uint8_t v_ctrl3 = imu_read_register(ISM330_CTRL3_C);
    uint8_t v_ctrl4 = imu_read_register(ISM330_CTRL4_C);
    uint8_t v_ctrl9 = imu_read_register(ISM330_CTRL9_XL);

    bool bdu_ok = (v_ctrl3 & (1 << 6)) != 0;
    bool i2c_ok = (v_ctrl4 & (1 << 2)) != 0;
    bool dev_ok = (v_ctrl9 & (1 << 1)) != 0;

    Serial.print("BDU: ");
    Serial.println(bdu_ok ? "ENABLED" : "FAILED");
    Serial.print("I2C_disable: ");
    Serial.println(i2c_ok ? "ENABLED" : "FAILED");
    Serial.print("DEVICE_CONF: ");
    Serial.println(dev_ok ? "ENABLED" : "FAILED");

    if (bdu_ok && i2c_ok && dev_ok) {
        Serial.println("=== ALL REGISTERS CONFIGURED ===");
    } else {
        Serial.println("[WARN] Some registers failed to configure!");
    }
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ISM330DHCX test!");
  Serial.println("Using SPI1 peripheral (pins 0/27/26/1)");
  Serial.println("Ensure 10k pull-down resistor on CS (pin 0) to GND\n");

  // CS pin setup: 10kΩ pull-down resistor ensures SPI mode at power-up
  // Per ISM330DHCX datasheet DS13012 Rev 7, Section 7.3 and 9
  // The pull-down ensures CS is LOW during power-up -> SPI mode selected
  pinMode(LSM_CS, OUTPUT);
  digitalWriteFast(LSM_CS, HIGH);  // Idle high for SPI transactions

  Serial.println("SPI1: 4 MHz (pins 0/27/26/1)");

  // Use SPI1 peripheral (hardware SPI on pins 0/27/26/1)
  // Signature: begin_SPI(cs_pin, SPIClass*, sensorID, frequency)
  if (!ism330dhcx.begin_SPI(LSM_CS, &SPI1, 0, 4000000)) {
    Serial.println("Failed to find ISM330DHCX chip");
    Serial.println("\nTroubleshooting checklist:");
    Serial.println("1. POWER CYCLE sensor (CS must be LOW at power-on for SPI mode!)");
    Serial.println("2. Check all SPI1 connections:");
    Serial.println("   - CS (pin 0) -> Sensor CS");
    Serial.println("   - SCK (pin 27) -> Sensor SCL/SCK");
    Serial.println("   - MOSI (pin 26) -> Sensor SDA/SDI");
    Serial.println("   - MISO (pin 1) -> Sensor SDO");
    Serial.println("3. Verify 3.3V power and GND connections");
    Serial.println("4. Ensure SDO pin connected to MISO (NOT to VCC!)");
    Serial.println("5. Check for loose wires or cold solder joints");
    Serial.println("6. Verify 10k pull-down resistor on CS (pin 0) to GND");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ISM330DHCX Found!");

  // === CRITICAL: Configure registers per datasheet BEFORE high-rate sampling ===
  // This configures BDU, I2C_disable, and DEVICE_CONF as mandated by datasheet
  imu_configure_registers();

  // ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (ism330dhcx.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (ism330dhcx.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  default:
    Serial.println("Unknown");
    break;
  }

  Serial.print("Setting accelerometer data rate to 104 Hz...");
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_104_HZ);
  delay(10);
  Serial.println("done");
  Serial.print("Accelerometer data rate set to: ");
  lsm6ds_data_rate_t accelRate = ism330dhcx.getAccelDataRate();
  switch (accelRate) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    Serial.println("ERROR: Accelerometer failed to activate! SPI write may not be working.");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  Serial.print("Setting gyroscope data rate to 104 Hz...");
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_104_HZ);
  delay(10);
  Serial.println("done");
  Serial.print("Gyro data rate set to: ");
  lsm6ds_data_rate_t gyroRate = ism330dhcx.getGyroDataRate();
  switch (gyroRate) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    Serial.println("ERROR: Gyro failed to activate! SPI write may not be working.");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
  ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2
}

void loop() {
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  // Check if readings are suspiciously all zero (except temp)
  if (accel.acceleration.x == 0 && accel.acceleration.y == 0 && accel.acceleration.z == 0 &&
      gyro.gyro.x == 0 && gyro.gyro.y == 0 && gyro.gyro.z == 0) {
    Serial.println("WARNING: All sensor readings are zero - possible communication issue!");
  }

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(100);

  //  // serial plotter friendly format

  //  Serial.print(temp.temperature);
  //  Serial.print(",");

  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //  Serial.print(",");

  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.print(gyro.gyro.z);
  // Serial.println();
  //  delayMicroseconds(10000);
}
