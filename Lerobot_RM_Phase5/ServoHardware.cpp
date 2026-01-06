/**
 * ServoHardware.cpp
 *
 * Implementation of Feetech STS3215 servo bus driver.
 * Phase 1: MODE_MOVE blocking operations only.
 *
 * All protocol knowledge (packets, checksums, timing) hidden here.
 * Caller sees only: init(), readPosition(), syncWrite().
 */

#include "ServoHardware.h"

// =============================================================================
// Lifecycle
// =============================================================================

void ServoHardware::init(HardwareSerial& port) {
    _port = &port;
    _port->begin(BAUD_RATE);

    // Initialize cache to error state
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        _lastPosition[i] = SERVO_ERROR;
    }

    // Flush any garbage on bus after power-up
    delay(10);
    flushRx();
}

// =============================================================================
// Public API: Read Position
// =============================================================================

int16_t ServoHardware::readPosition(uint8_t id) {
    if (id < 1 || id > MAX_SERVOS) return SERVO_ERROR;

    uint8_t txBuf[8];
    uint8_t rxBuf[8];

    // Build read packet for present position register
    uint8_t txLen = buildReadPacket(txBuf, id, REG_PRESENT_POSITION, 2);

    // Expected response: FF FF ID LEN ERR DATA_L DATA_H CHECKSUM = 8 bytes
    uint8_t rxLen = transact(txBuf, txLen, rxBuf, 8);

    if (rxLen < 8) {
        return SERVO_ERROR;  // Timeout
    }

    if (!parseResponse(rxBuf, rxLen, id)) {
        return SERVO_ERROR;  // Invalid response
    }

    // Extract position (bytes 5 and 6, little-endian)
    int16_t position = rxBuf[5] | (rxBuf[6] << 8);

    // Update cache
    _lastPosition[id - 1] = position;

    return position;
}

// =============================================================================
// Public API: Sync Write
// =============================================================================

bool ServoHardware::syncWrite(const uint8_t* ids, const int16_t* pos, uint8_t count) {
    if (count == 0 || count > MAX_SERVOS) return false;

    uint8_t txBuf[64];  // Max: 7 header + 6 * 7 data = 49 bytes

    // Build sync write packet (no times/speeds - use 0)
    uint8_t txLen = buildSyncWritePacket(txBuf, ids, pos, nullptr, nullptr, count);

    // Send (no response expected for broadcast)
    flushRx();
    writeBus(txBuf, txLen);

    return true;
}

bool ServoHardware::syncWriteFull(const uint8_t* ids, const int16_t* pos,
                                   const uint16_t* times, const uint16_t* speeds,
                                   uint8_t count) {
    if (count == 0 || count > MAX_SERVOS) return false;

    uint8_t txBuf[64];

    uint8_t txLen = buildSyncWritePacket(txBuf, ids, pos, times, speeds, count);

    flushRx();
    writeBus(txBuf, txLen);

    return true;
}

// =============================================================================
// Public API: Cached State
// =============================================================================

int16_t ServoHardware::getLastPosition(uint8_t id) const {
    if (id < 1 || id > MAX_SERVOS) return SERVO_ERROR;
    return _lastPosition[id - 1];
}

// =============================================================================
// Transport Layer
// =============================================================================

void ServoHardware::flushRx() {
    while (_port->available() > 0) {
        _port->read();
    }
}

void ServoHardware::writeBus(const uint8_t* data, uint8_t len) {
    _port->write(data, len);
    _port->flush();  // Block until TX complete (half-duplex requirement)
}

int ServoHardware::readByte() {
    if (_port->available() > 0) {
        return _port->read();
    }
    return -1;
}

// =============================================================================
// Protocol Layer: Checksum
// =============================================================================

uint8_t ServoHardware::checksum(const uint8_t* data, uint8_t len) {
    // Feetech checksum: ~(ID + Length + Instruction + Params)
    // Sum bytes from index 2 to len-1 (skip headers, exclude checksum itself)
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {
        sum += data[i];
    }
    return ~sum;  // Bitwise NOT
}

// =============================================================================
// Protocol Layer: Build Read Packet
// =============================================================================

uint8_t ServoHardware::buildReadPacket(uint8_t* buf, uint8_t id,
                                        uint8_t reg, uint8_t readLen) {
    // Feetech READ packet: FF FF ID LEN INST REG READ_LEN CHECKSUM
    buf[0] = HEADER1;           // 0xFF
    buf[1] = HEADER2;           // 0xFF
    buf[2] = id;                // Servo ID
    buf[3] = 4;                 // Length: INST + REG + READ_LEN + CHECKSUM
    buf[4] = INST_READ;         // 0x02
    buf[5] = reg;               // Register address
    buf[6] = readLen;           // How many bytes to read
    buf[7] = checksum(buf, 8);  // Checksum
    return 8;                   // Packet length
}

// =============================================================================
// Protocol Layer: Build Sync Write Packet
// =============================================================================

uint8_t ServoHardware::buildSyncWritePacket(uint8_t* buf,
                                             const uint8_t* ids,
                                             const int16_t* pos,
                                             const uint16_t* times,
                                             const uint16_t* speeds,
                                             uint8_t count) {
    // SYNC_WRITE: FF FF FE LEN INST START_REG DATA_LEN [ID DATA...]... CHECKSUM
    // DATA per servo: ID + 6 bytes (pos:2 + time:2 + speed:2)

    const uint8_t DATA_LEN = 6;  // Bytes per servo after ID

    buf[0] = HEADER1;                        // 0xFF
    buf[1] = HEADER2;                        // 0xFF
    buf[2] = 0xFE;                           // Broadcast ID
    buf[3] = 4 + count * (1 + DATA_LEN);     // Length
    buf[4] = INST_SYNC_WRITE;                // 0x83
    buf[5] = REG_GOAL_POSITION;              // Start register (0x2A)
    buf[6] = DATA_LEN;                       // Data length per servo

    uint8_t idx = 7;
    for (uint8_t i = 0; i < count; i++) {
        buf[idx++] = ids[i];                 // Servo ID
        buf[idx++] = pos[i] & 0xFF;          // Position low byte
        buf[idx++] = (pos[i] >> 8) & 0xFF;   // Position high byte
        buf[idx++] = times ? (times[i] & 0xFF) : 0;
        buf[idx++] = times ? ((times[i] >> 8) & 0xFF) : 0;
        buf[idx++] = speeds ? (speeds[i] & 0xFF) : 0;
        buf[idx++] = speeds ? ((speeds[i] >> 8) & 0xFF) : 0;
    }

    buf[idx] = checksum(buf, idx + 1);
    return idx + 1;
}

// =============================================================================
// Transaction Layer: Transact
// =============================================================================

uint8_t ServoHardware::transact(const uint8_t* txBuf, uint8_t txLen,
                                 uint8_t* rxBuf, uint8_t expectedRxLen) {
    // 1. Flush any stale RX data
    flushRx();

    // 2. Send request
    writeBus(txBuf, txLen);

    // 3. Wait for response with timeout
    uint32_t startUs = micros();
    uint8_t rxIdx = 0;

    while (rxIdx < expectedRxLen) {
        if ((micros() - startUs) > TIMEOUT_US) {
            return rxIdx;  // Timeout - return partial count
        }

        int byte = readByte();
        if (byte >= 0) {
            rxBuf[rxIdx++] = (uint8_t)byte;
        }
    }

    return rxIdx;  // Full response received
}

// =============================================================================
// Transaction Layer: Parse Response
// =============================================================================

bool ServoHardware::parseResponse(const uint8_t* buf, uint8_t len, uint8_t expectedId) {
    // Minimum response: FF FF ID LEN ERR CHECKSUM = 6 bytes
    if (len < 6) return false;

    // Check headers
    if (buf[0] != HEADER1 || buf[1] != HEADER2) return false;

    // Check ID matches
    if (buf[2] != expectedId) return false;

    // Check length field consistency
    uint8_t declaredLen = buf[3];
    if (len != declaredLen + 4) return false;  // +4 for headers + ID + LEN

    // Validate checksum
    if (buf[len - 1] != checksum(buf, len)) return false;

    // Check error byte (buf[4])
    if (buf[4] != 0x00) return false;  // Any error bit set

    return true;
}
