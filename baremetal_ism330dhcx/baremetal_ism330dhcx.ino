#include <Arduino.h>
#include <SPI.h>
#include "ISM330_Bare.h"

IntervalTimer imuTimer;

// Shared sample + seqlock-ish generation counter
struct SharedSample {
    uint32_t seq;
    uint32_t ts_us;
    ISM330::RawSample raw;
};

volatile SharedSample g_shared;

void imuISR() {
    // 1) timestamp
    uint32_t t = micros();

    // 2) read raw sample
    ISM330::RawSample s;
    ISM330::readRaw(s);

    // 3) publish with simple sequence counter
    // Manual field-by-field copy to avoid volatile struct assignment issues
    g_shared.seq++;
    g_shared.ts_us = t;
    g_shared.raw.temp  = s.temp;
    g_shared.raw.gx    = s.gx;
    g_shared.raw.gy    = s.gy;
    g_shared.raw.gz    = s.gz;
    g_shared.raw.ax    = s.ax;
    g_shared.raw.ay    = s.ay;
    g_shared.raw.az    = s.az;
    g_shared.raw.valid = s.valid;
    g_shared.seq++;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    ISM330::Config cfg;
    cfg.spi    = &SPI;
    cfg.cs_pin = 10;
    cfg.spi_hz = 400000;

    if (!ISM330::init(cfg)) {
        Serial.println("ISM330 init FAILED");
        while (1) {}
    }

    delay(5); // give SW_RESET a moment

    g_shared.seq = 0;

    imuTimer.begin(imuISR, 2000.0); // 500 Hz
}

void loop() {
    // Snapshot with simple seqlock
    SharedSample snap;
    uint32_t s1, s2;
    do {
        s1 = g_shared.seq;
        // Manual field-by-field copy to avoid volatile struct assignment issues
        snap.ts_us      = g_shared.ts_us;
        snap.raw.temp   = g_shared.raw.temp;
        snap.raw.gx     = g_shared.raw.gx;
        snap.raw.gy     = g_shared.raw.gy;
        snap.raw.gz     = g_shared.raw.gz;
        snap.raw.ax     = g_shared.raw.ax;
        snap.raw.ay     = g_shared.raw.ay;
        snap.raw.az     = g_shared.raw.az;
        snap.raw.valid  = g_shared.raw.valid;
        s2 = g_shared.seq;
    } while (s1 != s2 || (s1 & 1));

    // Scale in loop() (float is fine here)
    // Using same sensitivities you had:
    static const float ACCEL_SENS =
        0.061f / 1000.0f * 9.80665f;    // ±2 g mode, mg/LSB -> m/s²
    static const float GYRO_SENS  =
        4.375f / 1000.0f * (PI / 180.0f); // ±125 dps, mdps/LSB -> rad/s

    float ax = snap.raw.ax * ACCEL_SENS;
    float ay = snap.raw.ay * ACCEL_SENS;
    float az = snap.raw.az * ACCEL_SENS;

    float gx = snap.raw.gx * GYRO_SENS;
    float gy = snap.raw.gy * GYRO_SENS;
    float gz = snap.raw.gz * GYRO_SENS;

    Serial.print("t_us=");
    Serial.print(snap.ts_us);
    Serial.print(" A[");
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.print(az, 3); Serial.print("]  G[");
    Serial.print(gx, 3); Serial.print(", ");
    Serial.print(gy, 3); Serial.print(", ");
    Serial.print(gz, 3); Serial.println("]");

    delay(50); // just for readability
}
