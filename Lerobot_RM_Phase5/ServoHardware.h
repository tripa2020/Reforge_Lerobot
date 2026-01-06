/**
 * ServoHardware.h
 *
 * Deep module for Feetech STS3215 servo bus communication.
 * Phase 1: MODE_MOVE blocking operations only.
 *
 * Design: All Feetech protocol details hidden. Caller sees only:
 *   - init(port)
 *   - readPosition(id) → position or SERVO_ERROR
 *   - syncWrite(ids, pos, count)
 *
 * Reference: Phase5_docs/Modular_design.md
 */

#ifndef SERVO_HARDWARE_H
#define SERVO_HARDWARE_H

#include <Arduino.h>

class ServoHardware {
public:
    // =========================================================================
    // Constants
    // =========================================================================

    static constexpr int16_t SERVO_ERROR = -9999;  // Unambiguous error sentinel
    static constexpr uint8_t MAX_SERVOS = 6;

    // =========================================================================
    // Lifecycle
    // =========================================================================

    /**
     * Initialize servo hardware.
     * Stores reference to serial port, sets baud rate to 1Mbaud.
     *
     * @param port  Reference to HardwareSerial (typically Serial3)
     */
    void init(HardwareSerial& port);

    // =========================================================================
    // Blocking Operations (MODE_MOVE only)
    // =========================================================================

    /**
     * Read current position from one servo.
     * Blocks up to 2ms waiting for response.
     *
     * @param id  Servo ID (1-6)
     * @return    Position (0-4095) or SERVO_ERROR (-9999) on timeout/error
     */
    int16_t readPosition(uint8_t id);

    /**
     * Write goal positions to multiple servos simultaneously.
     * Uses SYNC_WRITE command (broadcast, no response expected).
     * Times and speeds default to 0 (immediate, max speed).
     *
     * @param ids    Array of servo IDs
     * @param pos    Array of goal positions (0-4095)
     * @param count  Number of servos (1-6)
     * @return       true if packet sent successfully
     */
    bool syncWrite(const uint8_t* ids, const int16_t* pos, uint8_t count);

    /**
     * Write goal positions with time and speed control.
     * Uses SYNC_WRITE command with full parameter set.
     *
     * @param ids    Array of servo IDs
     * @param pos    Array of goal positions (0-4095)
     * @param times  Array of move times in ms (0 = max speed)
     * @param speeds Array of speed limits (0 = no limit)
     * @param count  Number of servos (1-6)
     * @return       true if packet sent successfully
     */
    bool syncWriteFull(const uint8_t* ids, const int16_t* pos,
                       const uint16_t* times, const uint16_t* speeds,
                       uint8_t count);

    // =========================================================================
    // Cached State
    // =========================================================================

    /**
     * Get last successfully read position for a servo.
     * Returns cached value (does NOT perform bus transaction).
     *
     * @param id  Servo ID (1-6)
     * @return    Last known position, or SERVO_ERROR if never read
     */
    int16_t getLastPosition(uint8_t id) const;

private:
    // =========================================================================
    // State
    // =========================================================================

    HardwareSerial* _port;               // Stored reference to serial port
    int16_t _lastPosition[MAX_SERVOS];   // Cached positions (index = id-1)

    // =========================================================================
    // Protocol Constants (Feetech STS3215)
    // =========================================================================

    static constexpr uint8_t HEADER1 = 0xFF;
    static constexpr uint8_t HEADER2 = 0xFF;

    // Instructions
    static constexpr uint8_t INST_READ       = 0x02;
    static constexpr uint8_t INST_WRITE      = 0x03;
    static constexpr uint8_t INST_SYNC_WRITE = 0x83;

    // Registers
    static constexpr uint8_t REG_GOAL_POSITION    = 0x2A;  // 2 bytes
    static constexpr uint8_t REG_GOAL_TIME        = 0x2C;  // 2 bytes
    static constexpr uint8_t REG_GOAL_SPEED       = 0x2E;  // 2 bytes
    static constexpr uint8_t REG_PRESENT_POSITION = 0x38;  // 2 bytes (read-only)

    // Timing
    static constexpr uint32_t BAUD_RATE  = 1000000;
    static constexpr uint32_t TIMEOUT_US = 2000;

    // =========================================================================
    // Transport Layer
    // =========================================================================

    void flushRx();
    void writeBus(const uint8_t* data, uint8_t len);
    int readByte();

    // =========================================================================
    // Protocol Layer
    // =========================================================================

    uint8_t checksum(const uint8_t* data, uint8_t len);
    uint8_t buildReadPacket(uint8_t* buf, uint8_t id, uint8_t reg, uint8_t readLen);
    uint8_t buildSyncWritePacket(uint8_t* buf,
                                  const uint8_t* ids,
                                  const int16_t* pos,
                                  const uint16_t* times,
                                  const uint16_t* speeds,
                                  uint8_t count);

    // =========================================================================
    // Transaction Layer
    // =========================================================================

    uint8_t transact(const uint8_t* txBuf, uint8_t txLen,
                     uint8_t* rxBuf, uint8_t expectedRxLen);
    bool parseResponse(const uint8_t* buf, uint8_t len, uint8_t expectedId);
};

#endif // SERVO_HARDWARE_H
