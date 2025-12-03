/*
 * LeRobot Rate Monotonic Data Acquisition - Phase 1
 *
 * Target: 500Hz IMU (LSM6DS3TR-C) logging to CSV via Serial4
 *
 * Architecture:
 *   - IntervalTimer @ 500Hz triggers ISR
 *   - ISR reads IMU, writes to double-buffer
 *   - Main loop drains buffer to CSV ring buffer
 *   - CSV logger streams to Serial4 @ 921600 baud
 *
 * Phase: 1 (Minimal RM Core - 500Hz IMU Only)
 */

#include <Arduino.h>
#include <Adafruit_ISM330DHCX.h>
#include <SPI.h>

// ============== CONFIGURATION ==============
#define SAMPLE_RATE_HZ 500
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_RATE_HZ)  // 2000 µs

#define SERIAL_CSV_BAUD 2000000  // 2 Mbaud (was 921600)
#define ENABLE_DEBUG_OUTPUT 1  // 0 = silent, 1 = startup + errors

// ============== HARDWARE ==============
// SPI1 pins (tested and verified working with ISM330DHCX)
#define IMU_CS 0       // SPI1 CS (requires 10kΩ pull-down to GND)
#define IMU_SCK 27     // SPI1 SCK
#define IMU_MISO 1     // SPI1 MISO
#define IMU_MOSI 26    // SPI1 MOSI

Adafruit_ISM330DHCX imu;

// ============== DATA STRUCTURES ==============

// Single sensor snapshot (64 bytes, cache-aligned)
struct SensorData {
    uint32_t timestamp_us;

    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;

    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;

    int32_t servo_position;    // Phase 2
    int32_t servo_velocity;    // Phase 2

    uint32_t generation;       // Sample counter
    uint16_t imu_read_us;      // SPI read time (Phase 1: IMU; Phase 2: combined)
    uint16_t servo_read_us;    // Phase 1: SPI timing validation; Phase 2: servo read time

    uint8_t reserved[20];
} __attribute__((aligned(32)));

typedef SensorData CSVSample;  // Alias for consistency

// ============== DOUBLE BUFFER ==============
volatile SensorData buffers[2];
volatile uint8_t write_idx = 0;
volatile uint8_t read_idx = 1;
volatile bool newData = false;
volatile uint32_t sample_generation = 0;

// ============== CSV RING BUFFER ==============
#define CSV_BUFFER_SIZE 4096  // 4096 samples × 64 bytes = 256KB

__attribute__((section(".bss.octram"))) __attribute__((aligned(32)))
static CSVSample csv_ring[CSV_BUFFER_SIZE];

volatile uint32_t csv_head = 0;  // Write index
volatile uint32_t csv_tail = 0;  // Read index
volatile uint32_t csv_drops = 0; // Overflow counter

// ============== STATISTICS ==============
volatile uint32_t isr_count = 0;
volatile uint32_t isr_overrun = 0;  // ISR called before previous completed
volatile uint32_t max_isr_duration_us = 0;

uint32_t csv_bytes_sent = 0;
uint32_t last_stats_print_ms = 0;

// ============== ISR FLAG ==============
volatile bool isr_active = false;  // Detect ISR overruns
volatile bool logging_active = false;  // Controls whether ISR logs data

// ============== INTERVAL TIMER ==============
IntervalTimer sensorTimer;

// ============== FORWARD DECLARATIONS ==============
void sensorTimer_ISR();
void csv_push_sample(const CSVSample& sample);
bool csv_buffer_full();
void csv_service();
void print_stats();

// ============== ISR: SENSOR SAMPLING @ 500Hz ==============
void sensorTimer_ISR() {
    // Don't log until handshake complete
    if (!logging_active) {
        return;
    }

    // CRITICAL: Check for overrun (previous ISR didn't finish)
    if (isr_active) {
        isr_overrun++;
        return;  // Skip this sample to prevent corruption
    }
    isr_active = true;

    uint32_t isr_start = micros();

    // Get write buffer
    SensorData* wr = (SensorData*)&buffers[write_idx];

    // 1. CAPTURE TIMESTAMP FIRST (critical for sync)
    wr->timestamp_us = micros();

    // 2. READ IMU (SPI1 @ 4MHz)
    uint32_t spi_start = micros();
    sensors_event_t accel, gyro, temperature;
    imu.getEvent(&accel, &gyro, &temperature);
    uint32_t spi_end = micros();

    wr->accel_x_mps2 = accel.acceleration.x;
    wr->accel_y_mps2 = accel.acceleration.y;
    wr->accel_z_mps2 = accel.acceleration.z;

    wr->gyro_x_rads = gyro.gyro.x;
    wr->gyro_y_rads = gyro.gyro.y;
    wr->gyro_z_rads = gyro.gyro.z;

    // 3. METADATA & TIMING INSTRUMENTATION
    wr->generation = sample_generation++;
    wr->imu_read_us = spi_end - spi_start;       // SPI read duration
    wr->servo_read_us = spi_end - spi_start;     // Phase 1: SPI validation

    wr->servo_position = 0;      // Phase 2
    wr->servo_velocity = 0;

    // 4. ATOMIC BUFFER SWAP
    uint8_t temp_idx = read_idx;
    read_idx = write_idx;
    write_idx = temp_idx;

    newData = true;

    // Track ISR duration
    uint32_t isr_duration = micros() - isr_start;
    if (isr_duration > max_isr_duration_us) {
        max_isr_duration_us = isr_duration;
    }

    isr_count++;
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

        #if ENABLE_DEBUG_OUTPUT
        // CRITICAL ERROR: Buffer overflow - HALT system
        Serial.println("[FATAL] CSV buffer overflow - HALTING");
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(100);
        }
        #endif

        return;
    }

    csv_ring[csv_head] = sample;
    csv_head = (csv_head + 1) % CSV_BUFFER_SIZE;
}

void csv_service() {
    // NOTE: This will be replaced with DMA in final implementation
    // Current version intentionally simple for DMA migration
    const uint32_t MAX_DRAIN = 64;
    uint32_t drained = 0;

    while (csv_tail != csv_head && drained < MAX_DRAIN) {
        CSVSample& sample = csv_ring[csv_tail];

        // Write binary sample to Serial4 (BLOCKING - to be replaced with DMA)
        size_t written = Serial4.write((uint8_t*)&sample, sizeof(CSVSample));

        if (written != sizeof(CSVSample)) {
            #if ENABLE_DEBUG_OUTPUT
            Serial.println("[WARN] Serial4 write incomplete");
            #endif
        }

        csv_bytes_sent += written;
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

    Serial.println("========== STATS ==========");
    Serial.print("Samples: "); Serial.println(isr_count);
    Serial.print("Drops: "); Serial.println(csv_drops);
    Serial.print("Overruns: "); Serial.println(isr_overrun);
    Serial.print("Max ISR: "); Serial.print(max_isr_duration_us); Serial.println(" us");
    Serial.print("Buffer: "); Serial.print(buffer_pct, 1); Serial.println(" %");
    Serial.print("CSV Sent: "); Serial.print(csv_bytes_sent / 1024); Serial.println(" KB");
    Serial.println("===========================");
}

// ============== SETUP ==============

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // Debug serial
    Serial.begin(115200);
    #if ENABLE_DEBUG_OUTPUT
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("LeRobot RM Data Acquisition - Phase 1");
    Serial.println("========================================");
    #endif

    // CSV output serial
    Serial4.begin(SERIAL_CSV_BAUD);
    #if ENABLE_DEBUG_OUTPUT
    Serial.print("Serial4: "); Serial.print(SERIAL_CSV_BAUD); Serial.println(" baud");
    #endif

    // ============== HANDSHAKE PROTOCOL ==============
    #if ENABLE_DEBUG_OUTPUT
    Serial.println("\n=== WAITING FOR PYTHON SCRIPT ===");
    Serial.println("Sending READY signal on Serial4...");
    Serial.println("Start Python script: parse_phase1_csv.py");
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

    // Initialize SPI1 - CS pin setup (per ISM330DHCX datasheet pg 18)
    // NOTE: 10kΩ pull-down resistor on CS (pin 0) to GND ensures SPI mode at power-up
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, LOW);   // Force SPI mode selection
    delay(50);                    // Hold during power-up
    digitalWrite(IMU_CS, HIGH);   // Idle high for transactions
    delay(50);

    #if ENABLE_DEBUG_OUTPUT
    Serial.println("SPI1: 4 MHz (pins 0/27/26/1)");
    #endif

    // Initialize IMU on SPI1 @ 4MHz (validated in diagnostics)
    if (!imu.begin_SPI(IMU_CS, &SPI1, 0, 4000000)) {
        #if ENABLE_DEBUG_OUTPUT
        Serial.println("[FATAL] ISM330DHCX not found!");
        Serial.println("Check: 10kΩ pull-down on CS, SPI1 wiring, power cycle sensor");
        #endif
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(500);
        }
    }

    // Configure IMU for 833Hz ODR (same as LSM6DS3TRC, supported by ISM330DHCX)
    imu.setAccelDataRate(LSM6DS_RATE_833_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_833_HZ);

    #if ENABLE_DEBUG_OUTPUT
    Serial.println("IMU: ISM330DHCX @ 833Hz");
    Serial.print("Accel Range: ±");
    switch(imu.getAccelRange()) {
        case LSM6DS_ACCEL_RANGE_2_G: Serial.println("2G"); break;
        case LSM6DS_ACCEL_RANGE_4_G: Serial.println("4G"); break;
        case LSM6DS_ACCEL_RANGE_8_G: Serial.println("8G"); break;
        case LSM6DS_ACCEL_RANGE_16_G: Serial.println("16G"); break;
    }

    Serial.print("Gyro Range: ");
    switch(imu.getGyroRange()) {
        case LSM6DS_GYRO_RANGE_125_DPS: Serial.println("125 DPS"); break;
        case LSM6DS_GYRO_RANGE_250_DPS: Serial.println("250 DPS"); break;
        case LSM6DS_GYRO_RANGE_500_DPS: Serial.println("500 DPS"); break;
        case LSM6DS_GYRO_RANGE_1000_DPS: Serial.println("1000 DPS"); break;
        case LSM6DS_GYRO_RANGE_2000_DPS: Serial.println("2000 DPS"); break;
    }
    #endif

    // Start sensor timer
    sensorTimer.begin(sensorTimer_ISR, SAMPLE_PERIOD_US);
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
    #endif
}

// ============== MAIN LOOP ==============

void loop() {
    // 1. Check for new sensor data
    if (newData) {
        // Create non-volatile copy from volatile buffer
        SensorData copy;
        uint8_t idx = read_idx;  // Read volatile index once
        memcpy(&copy, (void*)&buffers[idx], sizeof(SensorData));
        newData = false;
        csv_push_sample(copy);
    }

    // 2. Drain CSV buffer to Serial4
    csv_service();

    // 3. Handle user commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 's' || cmd == 'S') {
            print_stats();
        }
    }

    // 4. Periodic stats
    #if ENABLE_DEBUG_OUTPUT
    print_stats();
    #endif

    // Heartbeat LED: 5 Hz when logging active, 1 Hz otherwise
    static uint32_t last_blink = 0;
    uint32_t blink_interval = logging_active ? 100 : 500;  // 5 Hz : 1 Hz

    if (millis() - last_blink > blink_interval) {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
