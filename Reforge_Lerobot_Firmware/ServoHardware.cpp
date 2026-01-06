/**
 * ServoHardware.cpp
 *
 * Implementation of Feetech STS3215 servo bus driver.
 *
 * Phase 1: MODE_MOVE blocking operations
 * Phase 2: MODE_DATA async operations with ISR-safe state access
 *
 * All protocol knowledge (packets, checksums, timing) hidden here.
 * Caller sees only the public API.
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

    // Initialize async state
    resetAsyncState();

    // Flush any garbage on bus after power-up
    delay(10);
    flushRx();
}

void ServoHardware::resetAsyncState() {
    _rxState = RxState::WaitHeader1;
    _rxIdx = 0;
    _rxExpectedLen = 0;

    _busState = BusState::Idle;

    _requestInFlight = false;
    _requestQueued = false;
    _requestTimeUs = 0;
    _requestId = 1;

    _writeInFlight = false;
    _writeTimeUs = 0;

    // Don't reset _latest - preserve last known state
    // Don't reset _diag - preserve diagnostic history
}

// =============================================================================
// Phase 1: Blocking Read Position
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
// Phase 1: Sync Write
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
// Phase 1: Cached State
// =============================================================================

int16_t ServoHardware::getLastPosition(uint8_t id) const {
    if (id < 1 || id > MAX_SERVOS) return SERVO_ERROR;
    return _lastPosition[id - 1];
}

// =============================================================================
// Phase 2: Queue Read (Non-blocking)
// =============================================================================

void ServoHardware::queueRead(uint8_t id) {
    if (id < 1 || id > MAX_SERVOS) return;

    // Don't queue if already in flight or write pending
    if (_requestInFlight) return;
    if (_busState == BusState::WriteWait) return;

    _requestId = id;
    _requestQueued = true;
}

// =============================================================================
// Phase 2: Write Goal (Non-blocking, single servo)
// =============================================================================

void ServoHardware::writeGoal(uint8_t id, int16_t pos, uint16_t time_ms, uint16_t speed) {
    if (id < 1 || id > MAX_SERVOS) return;

    // Rate limit writes
    uint32_t now = micros();
    if (now - _lastWriteUs < WRITE_INTERVAL_US) return;

    // Don't write if bus is busy with read
    if (_busState == BusState::ReadWait) return;

    // Build single-servo WRITE packet
    uint8_t txBuf[16];
    uint8_t txLen = buildWritePacket(txBuf, id, pos, time_ms, speed);

    flushRx();
    writeBus(txBuf, txLen);

    _writeTimeUs = now;
    _lastWriteUs = now;
    _writeInFlight = true;
    _busState = BusState::WriteWait;

    // Reset RX FSM for response
    _rxState = RxState::WaitHeader1;
    _rxIdx = 0;
}

// =============================================================================
// Phase 2: Poll (Main service function)
// =============================================================================

void ServoHardware::poll() {
    // 1. Handle timeouts
    handleTimeout();

    // 2. Send queued read if bus is idle
    if (_requestQueued && !_requestInFlight && _busState == BusState::Idle) {
        sendQueuedRead();
    }

    // 3. Process incoming bytes
    int b;
    while ((b = readByte()) >= 0) {
        processByte((uint8_t)b);
    }
}

// =============================================================================
// Phase 2: Send Queued Read
// =============================================================================

void ServoHardware::sendQueuedRead() {
    uint8_t txBuf[8];
    uint8_t txLen = buildReadPacket(txBuf, _requestId, REG_PRESENT_POSITION, 2);

    flushRx();
    writeBus(txBuf, txLen);

    _requestTimeUs = micros();
    _requestInFlight = true;
    _requestQueued = false;
    _busState = BusState::ReadWait;

    // Store request time in latest for ISR snapshot
    _latest.t_req_us = _requestTimeUs;
}

// =============================================================================
// Phase 2: Process Byte (RX FSM)
// =============================================================================

void ServoHardware::processByte(uint8_t b) {
    switch (_rxState) {

    case RxState::WaitHeader1:
        if (b == HEADER1) {
            _rxBuf[0] = b;
            _rxState = RxState::WaitHeader2;
        }
        break;

    case RxState::WaitHeader2:
        if (b == HEADER2) {
            _rxBuf[1] = b;
            _rxState = RxState::WaitId;
        } else {
            _rxState = RxState::WaitHeader1;
            _diag.resync_count++;
        }
        break;

    case RxState::WaitId:
        _rxBuf[2] = b;
        _rxState = RxState::WaitLen;
        break;

    case RxState::WaitLen:
        _rxBuf[3] = b;
        _rxExpectedLen = b;
        _rxIdx = 4;
        _rxState = RxState::WaitBody;
        break;

    case RxState::WaitBody:
        _rxBuf[_rxIdx++] = b;
        // Complete when we have header(4) + body(expectedLen)
        if (_rxIdx >= (uint8_t)(_rxExpectedLen + 4)) {
            parseAsyncResponse();
            _rxState = RxState::WaitHeader1;
        }
        break;
    }
}

// =============================================================================
// Phase 2: Parse Async Response
// =============================================================================

void ServoHardware::parseAsyncResponse() {
    uint8_t len = _rxIdx;
    uint32_t t_rx = micros();

    // Check if this is a WRITE ACK (6 bytes, len field = 0x02)
    if (len == 6 && _rxBuf[3] == 0x02) {
        // Write acknowledged, bus is now idle
        _writeInFlight = false;
        _busState = BusState::Idle;
        return;
    }

    // This should be a READ response
    bool valid = true;
    uint8_t error_code = 0;

    // Minimum length check
    if (len < 6) {
        valid = false;
        error_code = 3;  // Framing
        _diag.framing_errors++;
    }

    // ID check
    if (valid && _rxBuf[2] != _requestId) {
        valid = false;
        error_code = 3;  // Wrong servo
        _diag.framing_errors++;
    }

    // Checksum check
    if (valid) {
        uint8_t calc = checksum(_rxBuf, len);
        if (_rxBuf[len - 1] != calc) {
            valid = false;
            error_code = 2;  // Checksum
            _diag.checksum_errors++;
        }
    }

    // Error byte check
    if (valid && _rxBuf[4] != 0x00) {
        valid = false;
        error_code = 3;  // Servo reported error
        _diag.framing_errors++;
    }

    // Extract position if valid read response (8 bytes)
    if (valid && len >= 8) {
        int32_t pos = _rxBuf[5] | (_rxBuf[6] << 8);

        // Update latest with generation counter (for ISR safety)
        _latest.position = pos;
        _latest.t_rx_us = t_rx;
        _latest.error_code = 0;
        SERVO_BARRIER();
        _latest.generation++;

        // Update cache
        if (_requestId >= 1 && _requestId <= MAX_SERVOS) {
            _lastPosition[_requestId - 1] = (int16_t)pos;
        }

        _diag.successful_reads++;
    } else {
        // Error case
        if (error_code == 0) error_code = 3;
        _latest.error_code = error_code;
        SERVO_BARRIER();
        _latest.generation++;
    }

    _requestInFlight = false;
    _busState = BusState::Idle;
}

// =============================================================================
// Phase 2: Handle Timeout
// =============================================================================

void ServoHardware::handleTimeout() {
    uint32_t now = micros();

    // Check read timeout
    if (_requestInFlight) {
        uint32_t elapsed = now - _requestTimeUs;
        if (elapsed > TIMEOUT_ASYNC) {
            // Timeout - update latest with error
            _latest.error_code = 1;  // Timeout
            SERVO_BARRIER();
            _latest.generation++;

            _diag.timeouts++;

            _requestInFlight = false;
            _busState = BusState::Idle;
            _rxState = RxState::WaitHeader1;
        }
    }

    // Check write timeout
    if (_writeInFlight) {
        uint32_t elapsed = now - _writeTimeUs;
        if (elapsed > TIMEOUT_ASYNC) {
            // Write timeout - just reset state
            _writeInFlight = false;
            _busState = BusState::Idle;
            _rxState = RxState::WaitHeader1;
        }
    }
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
// Protocol Layer: Build Write Packet (single servo)
// =============================================================================

uint8_t ServoHardware::buildWritePacket(uint8_t* buf, uint8_t id,
                                         int16_t pos, uint16_t time_ms, uint16_t speed) {
    // Feetech WRITE packet: FF FF ID LEN INST ADDR DATA... CHECKSUM
    // Writing 6 bytes to 0x2A: pos(2) + time(2) + speed(2)
    buf[0] = HEADER1;                       // 0xFF
    buf[1] = HEADER2;                       // 0xFF
    buf[2] = id;                            // Servo ID
    buf[3] = 9;                             // Length: INST(1) + ADDR(1) + DATA(6) + CHECKSUM(1)
    buf[4] = INST_WRITE;                    // 0x03
    buf[5] = REG_GOAL_POSITION;             // 0x2A
    buf[6] = pos & 0xFF;                    // Position low byte
    buf[7] = (pos >> 8) & 0xFF;             // Position high byte
    buf[8] = time_ms & 0xFF;                // Time low byte
    buf[9] = (time_ms >> 8) & 0xFF;         // Time high byte
    buf[10] = speed & 0xFF;                 // Speed low byte
    buf[11] = (speed >> 8) & 0xFF;          // Speed high byte
    buf[12] = checksum(buf, 13);            // Checksum
    return 13;                              // Packet length
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
// Transaction Layer: Transact (Blocking)
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
        if ((micros() - startUs) > TIMEOUT_BLOCKING) {
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
// Transaction Layer: Parse Response (Blocking)
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
