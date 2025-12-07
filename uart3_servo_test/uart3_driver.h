/*
 * UART3 Driver for Phase 3 Decoupled Architecture
 *
 * Core Principle: "ISR = move bytes to/from ring buffers. Protocol/logic = non-ISR code."
 *
 * This driver provides:
 *   - RX ring buffer (ISR writes, main loop reads via uart3_get_byte)
 *   - TX ring buffer (main loop writes via uart3_send_bytes, ISR transmits)
 *   - Custom ISR for LPUART3 (Teensy 4.1)
 *   - FIFO depth = 1 for predictable ISR timing
 *
 * Priority: 64 (lower than Frame ISR @ 32)
 * Baud: 1 Mbaud (Feetech servo default)
 *
 * !! CRITICAL WARNING !!
 * After uart3_init(), you must NEVER use Serial3 (write/read/print/available).
 * This driver hijacks LPUART3 hardware from Teensyduino's Serial3.
 * Serial3.begin() is called only for pin mux setup, then we install our own ISR.
 * Using Serial3 after init will corrupt servo communication with undefined behavior.
 */

#ifndef UART3_DRIVER_H
#define UART3_DRIVER_H

#include <Arduino.h>

// Ring buffer size (must be power of 2 for efficient modulo)
#define UART3_RING_SIZE 256
#define UART3_RING_MASK (UART3_RING_SIZE - 1)

// Statistics for diagnostics
struct UART3Stats {
    volatile uint32_t rx_bytes;         // Total bytes received
    volatile uint32_t tx_bytes;         // Total bytes transmitted
    volatile uint32_t rx_overflow;      // Bytes dropped due to full RX buffer
    volatile uint32_t tx_overflow;      // Bytes dropped due to full TX buffer
    volatile uint32_t isr_count;        // ISR invocations
};

extern UART3Stats uart3_stats;

// ============== API ==============

/**
 * Initialize UART3 with custom ISR
 * @param baud Baud rate (default: 1000000 for Feetech)
 */
void uart3_init(uint32_t baud);

/**
 * Get one byte from RX ring buffer (non-blocking)
 * @return Byte value (0-255) or -1 if buffer empty
 */
int uart3_get_byte(void);

/**
 * Check if RX data is available
 * @return Number of bytes available in RX ring
 */
uint16_t uart3_available(void);

/**
 * Send bytes via UART3 (non-blocking, queues to TX ring)
 * @param data Pointer to data
 * @param len Number of bytes to send
 * @return Number of bytes actually queued (may be less if TX buffer full)
 */
uint8_t uart3_send_bytes(const uint8_t* data, uint8_t len);

/**
 * Flush RX buffer (discard all pending data)
 * Use for resynchronization after errors
 */
void uart3_flush_rx(void);

/**
 * Check if TX is complete (all bytes transmitted)
 * @return true if TX buffer empty and UART idle
 */
bool uart3_tx_complete(void);

#endif // UART3_DRIVER_H
