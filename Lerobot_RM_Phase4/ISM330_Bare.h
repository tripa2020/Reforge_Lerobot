#pragma once
#include <Arduino.h>
#include <SPI.h>

namespace ISM330 {

// ===== Register map (DS13012 Rev 7) =====
static constexpr uint8_t REG_WHO_AM_I    = 0x0F;  // should be 0x6B
static constexpr uint8_t REG_CTRL1_XL    = 0x10;
static constexpr uint8_t REG_CTRL2_G     = 0x11;
static constexpr uint8_t REG_CTRL3_C     = 0x12;
static constexpr uint8_t REG_CTRL4_C     = 0x13;
static constexpr uint8_t REG_CTRL9_XL    = 0x18;
static constexpr uint8_t REG_STATUS_REG  = 0x1E;
static constexpr uint8_t REG_OUT_TEMP_L  = 0x20;
static constexpr uint8_t REG_OUTX_L_G    = 0x22;  // Gx..Gz
static constexpr uint8_t REG_OUTX_L_A    = 0x28;  // Ax..Az

// ===== Public config/sample types =====

struct Config {
    SPIClass* spi;        // usually &SPI
    uint8_t   cs_pin;     // chip select pin
    uint32_t  spi_hz;     // e.g. 4'000'000
};

struct RawSample {
    // raw register contents (no scaling), for ISR use
    int16_t temp;   // OUT_TEMP
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    bool    valid;  // all-zero guard
};

// ===== Internal static state =====

inline SPIClass*   g_spi      = nullptr;
inline uint8_t     g_cs       = 10;
inline SPISettings g_settings(4000000, MSBFIRST, SPI_MODE0); // MODE0, 4 MHz

// ===== Low-level helpers =====

// CS setup/hold delay for long wires (PJRC recommends ~100Ω series for >25cm)
inline void csDelay() { delayMicroseconds(1); }

inline uint8_t readReg(uint8_t reg) {
    uint8_t v;
    g_spi->beginTransaction(g_settings);
    digitalWriteFast(g_cs, LOW);
    csDelay();

    g_spi->transfer(reg | 0x80);   // MSB=1 → read
    v = g_spi->transfer(0x00);

    csDelay();
    digitalWriteFast(g_cs, HIGH);
    g_spi->endTransaction();
    return v;
}

inline void writeReg(uint8_t reg, uint8_t value) {
    g_spi->beginTransaction(g_settings);
    digitalWriteFast(g_cs, LOW);
    csDelay();

    g_spi->transfer(reg & 0x7F);   // MSB=0 → write
    g_spi->transfer(value);

    csDelay();
    digitalWriteFast(g_cs, HIGH);
    g_spi->endTransaction();
}

inline void readBurst(uint8_t startReg, uint8_t* buf, uint8_t len) {
    g_spi->beginTransaction(g_settings);
    digitalWriteFast(g_cs, LOW);
    csDelay();

    g_spi->transfer(startReg | 0x80);  // read, IF_INC handles increment
    for (uint8_t i = 0; i < len; ++i) {
        buf[i] = g_spi->transfer(0x00);
    }

    csDelay();
    digitalWriteFast(g_cs, HIGH);
    g_spi->endTransaction();
}

// ===== Init: match your current bare-metal sequence =====
//
// - SW_RESET
// - CTRL3_C = 0x44 (BDU=1, IF_INC=1)  [datasheet CTRL3_C: BDU & IF_INC bits]
// - CTRL4_C = 0x04 (I2C_disable=1)   [SPI-only recommended init]
// - CTRL9_XL = 0x02 (DEVICE_CONF=1)
// - CTRL1_XL = 0x50 (ODR 208 Hz, ±2 g)
// - CTRL2_G  = 0x58 (ODR 208 Hz, ±125 dps)

inline bool init(const Config& cfg) {
    g_spi = cfg.spi ? cfg.spi : &SPI;
    g_cs  = cfg.cs_pin;
    if (cfg.spi_hz != 0) {
        g_settings = SPISettings(cfg.spi_hz, MSBFIRST, SPI_MODE0);
    }

    g_spi->begin();
    pinMode(g_cs, OUTPUT);
    digitalWriteFast(g_cs, HIGH);   // deselect

    delay(10);  // Boot time after power-on

    // WHO_AM_I check
    uint8_t who = readReg(REG_WHO_AM_I);
    if (who != 0x6B) {              // expected 0x6B per datasheet
        return false;
    }

    // Software reset (SW_RESET=1)
    writeReg(REG_CTRL3_C, 0x01);

    // Wait for reset to complete (poll SW_RESET bit or just delay)
    delay(10);  // ISM330DHCX needs time after SW_RESET

    // Verify WHO_AM_I again after reset
    who = readReg(REG_WHO_AM_I);
    if (who != 0x6B) {
        return false;  // Reset may have failed
    }

    // Re-write CTRL3_C: BDU=1, IF_INC=1
    writeReg(REG_CTRL3_C, 0x44);

    // Disable I2C, enable SPI only
    writeReg(REG_CTRL4_C, 0x04);

    // DEVICE_CONF = 1
    writeReg(REG_CTRL9_XL, 0x02);

    // Accel: 208 Hz, ±2 g
    writeReg(REG_CTRL1_XL, 0x50);

    // Gyro: 208 Hz, ±125 dps
    writeReg(REG_CTRL2_G, 0x58);

    return true;
}

// Optional STATUS_REG poll (if you ever want it in ISR)
inline bool dataReady() {
    uint8_t status = readReg(REG_STATUS_REG);
    // XLDA (bit 0) + GDA (bit 1) both set → accel+gyro new data
    return (status & 0x03) == 0x03;
}

// ===== ISR-safe raw read =====
//
// Single burst from OUT_TEMP_L: TEMP(2) + G(6) + A(6) = 14 bytes.
// No floats, no Serial, no dynamic allocation.

inline void readRaw(RawSample& out) {
    uint8_t buf[14];
    readBurst(REG_OUT_TEMP_L, buf, 14);

    out.temp = (int16_t)((buf[1] << 8) | buf[0]);

    out.gx   = (int16_t)((buf[3] << 8) | buf[2]);
    out.gy   = (int16_t)((buf[5] << 8) | buf[4]);
    out.gz   = (int16_t)((buf[7] << 8) | buf[6]);

    out.ax   = (int16_t)((buf[9]  << 8) | buf[8]);
    out.ay   = (int16_t)((buf[11] << 8) | buf[10]);
    out.az   = (int16_t)((buf[13] << 8) | buf[12]);

    // very cheap sanity guard
    out.valid = !(
        out.ax == 0 && out.ay == 0 && out.az == 0 &&
        out.gx == 0 && out.gy == 0 && out.gz == 0
    );
}

} // namespace ISM330
