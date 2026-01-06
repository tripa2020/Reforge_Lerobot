/**
 * ServoHardware.h
 *
 * Deep module for Feetech STS3215 servo bus communication.
 *
 * Phase 1: MODE_MOVE blocking operations
 *   - readPosition(id) - blocking 2ms read
 *   - syncWrite() / syncWriteFull() - broadcast write
 *
 * Phase 2: MODE_DATA async operations
 *   - queueRead(id) - non-blocking request
 *   - writeGoal() - non-blocking single servo write
 *   - poll() - service RX/TX/timeouts
 *   - snapshot() - ISR-safe state access
 *
 * Design: All Feetech protocol details hidden.
 * Reference: Phase5_docs/Phase2_ISR_Integration.md
 */

#ifndef SERVO_HARDWARE_H
#define SERVO_HARDWARE_H

#include <Arduino.h>

// ARM Cortex-M7 memory barrier for ISR safety
#if defined(__arm__) && defined(__IMXRT1062__)
  #define SERVO_BARRIER() asm volatile("dmb" ::: "memory")
#else
  #define SERVO_BARRIER() __sync_synchronize()
#endif

class ServoHardware {
public:
    // =========================================================================
    // Constants
    // =========================================================================

    static constexpr int16_t SERVO_ERROR = -9999;  // Unambiguous error sentinel
    static constexpr uint8_t MAX_SERVOS = 6;

    // =========================================================================
    // Phase 2 Types
    // =========================================================================

    /**
     * ISR-safe snapshot of latest servo state.
     * Returned by snapshot() - guaranteed coherent via generation counter.
     */
    struct Snapshot {
        int32_t position;      // Raw encoder value (0-4095) or -1 on error
        uint32_t t_req_us;     // When READ request was sent
        uint32_t t_rx_us;      // When response was received
        uint8_t error_code;    // 0=OK, 1=timeout, 2=checksum, 3=framing
        bool coherent;         // True if snapshot was consistent
    };

    /**
     * Diagnostic counters for debugging servo communication.
     */
    struct Diagnostics {
        uint32_t timeouts;
        uint32_t checksum_errors;
        uint32_t framing_errors;
        uint32_t resync_count;
        uint32_t successful_reads;
    };

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
    // Phase 1: Blocking Operations (MODE_MOVE only)
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
     */
    bool syncWrite(const uint8_t* ids, const int16_t* pos, uint8_t count);

    /**
     * Write goal positions with time and speed control.
     */
    bool syncWriteFull(const uint8_t* ids, const int16_t* pos,
                       const uint16_t* times, const uint16_t* speeds,
                       uint8_t count);

    /**
     * Get last successfully read position for a servo.
     * Returns cached value (does NOT perform bus transaction).
     */
    int16_t getLastPosition(uint8_t id) const;

    // =========================================================================
    // Phase 2: Async Operations (MODE_DATA)
    // =========================================================================

    /**
     * Queue a non-blocking read request.
     * Call from main loop after ISR sets request flag.
     * Actual TX happens in poll().
     *
     * @param id  Servo ID (1-6)
     */
    void queueRead(uint8_t id);

    /**
     * Queue a non-blocking write (single servo goal position).
     * Used for trajectory streaming in MODE_DATA_RUN.
     *
     * @param id       Servo ID (1-6)
     * @param pos      Goal position (0-4095)
     * @param time_ms  Interpolation time in ms (0 = immediate)
     * @param speed    Speed limit (0 = no limit)
     */
    void writeGoal(uint8_t id, int16_t pos, uint16_t time_ms, uint16_t speed);

    /**
     * Service function - call from main loop in MODE_DATA_RUN.
     * Handles:
     *   - Sending queued read requests
     *   - Parsing incoming RX bytes
     *   - Timeout detection
     *   - Bus state management
     */
    void poll();

    /**
     * ISR-safe snapshot of latest servo reading.
     * Uses generation counter to ensure coherent multi-word read.
     * Guaranteed <500ns, no blocking.
     *
     * @return Coherent snapshot of latest state
     */
    inline Snapshot snapshot() const __attribute__((always_inline));

    /**
     * Check if a read request is currently in flight.
     */
    bool isRequestPending() const { return _requestInFlight; }

    /**
     * Check if a read request is queued but not yet sent.
     */
    bool isRequestQueued() const { return _requestQueued; }

    /**
     * Get bus state for debugging.
     */
    uint8_t getBusState() const { return static_cast<uint8_t>(_busState); }

    /**
     * Get diagnostic counters.
     */
    const Diagnostics& getDiagnostics() const { return _diag; }

    /**
     * Reset async state (call when transitioning modes).
     */
    void resetAsyncState();

private:
    // =========================================================================
    // State
    // =========================================================================

    HardwareSerial* _port;               // Stored reference to serial port
    int16_t _lastPosition[MAX_SERVOS];   // Cached positions (index = id-1)

    // =========================================================================
    // Phase 2: Async State
    // =========================================================================

    // RX FSM state
    enum class RxState : uint8_t {
        WaitHeader1,
        WaitHeader2,
        WaitId,
        WaitLen,
        WaitBody
    };

    RxState _rxState = RxState::WaitHeader1;
    uint8_t _rxBuf[16];
    uint8_t _rxIdx = 0;
    uint8_t _rxExpectedLen = 0;

    // Bus state machine
    enum class BusState : uint8_t {
        Idle,
        ReadWait,
        WriteWait
    };

    BusState _busState = BusState::Idle;

    // Request tracking
    volatile bool _requestInFlight = false;
    volatile bool _requestQueued = false;
    volatile uint32_t _requestTimeUs = 0;
    uint8_t _requestId = 1;

    // Write tracking
    volatile bool _writeInFlight = false;
    volatile uint32_t _writeTimeUs = 0;
    uint32_t _lastWriteUs = 0;

    // Latest reading (ISR-accessible via snapshot())
    struct Latest {
        volatile int32_t position = -1;
        volatile uint32_t t_req_us = 0;
        volatile uint32_t t_rx_us = 0;
        volatile uint8_t error_code = 0xFF;  // 0xFF = not initialized
        volatile uint32_t generation = 0;
    };
    Latest _latest;

    // Diagnostics
    Diagnostics _diag = {0, 0, 0, 0, 0};

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
    static constexpr uint32_t BAUD_RATE         = 1000000;
    static constexpr uint32_t TIMEOUT_BLOCKING  = 2000;    // MODE_MOVE: 2ms
    static constexpr uint32_t TIMEOUT_ASYNC     = 800;     // MODE_DATA: 800µs
    static constexpr uint32_t WRITE_INTERVAL_US = 10000;   // 100Hz max write rate

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
    uint8_t buildWritePacket(uint8_t* buf, uint8_t id, int16_t pos,
                              uint16_t time_ms, uint16_t speed);
    uint8_t buildSyncWritePacket(uint8_t* buf,
                                  const uint8_t* ids,
                                  const int16_t* pos,
                                  const uint16_t* times,
                                  const uint16_t* speeds,
                                  uint8_t count);

    // =========================================================================
    // Transaction Layer (Phase 1: Blocking)
    // =========================================================================

    uint8_t transact(const uint8_t* txBuf, uint8_t txLen,
                     uint8_t* rxBuf, uint8_t expectedRxLen);
    bool parseResponse(const uint8_t* buf, uint8_t len, uint8_t expectedId);

    // =========================================================================
    // Async Layer (Phase 2: Non-blocking)
    // =========================================================================

    void sendQueuedRead();
    void processByte(uint8_t b);
    void parseAsyncResponse();
    void handleTimeout();
};

// =============================================================================
// Inline Implementation: snapshot()
// Must be in header for inlining in ISR
// =============================================================================

inline ServoHardware::Snapshot ServoHardware::snapshot() const {
    Snapshot s;
    uint32_t g1, g2;
    uint8_t retries = 0;

    do {
        g1 = _latest.generation;
        SERVO_BARRIER();

        s.position = _latest.position;
        s.t_req_us = _latest.t_req_us;
        s.t_rx_us = _latest.t_rx_us;
        s.error_code = _latest.error_code;

        SERVO_BARRIER();
        g2 = _latest.generation;
        retries++;
    } while (g1 != g2 && retries < 3);

    s.coherent = (g1 == g2);
    return s;
}

#endif // SERVO_HARDWARE_H
