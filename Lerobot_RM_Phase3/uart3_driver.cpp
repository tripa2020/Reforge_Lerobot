/*
 * UART3 Driver Implementation for Phase 3 Decoupled Architecture
 *
 * Target: Teensy 4.1 (IMXRT1062 / LPUART3)
 * Baud: 1 Mbaud
 * FIFO: Depth = 1 for predictable ISR timing
 * Priority: 64 (lower than Frame ISR @ 32)
 */

#include "uart3_driver.h"

// ============== RING BUFFERS ==============

struct RingBuffer {
    volatile uint8_t buf[UART3_RING_SIZE];
    volatile uint16_t head;  // Producer writes here
    volatile uint16_t tail;  // Consumer reads here
};

static RingBuffer rx_ring = {{0}, 0, 0};
static RingBuffer tx_ring = {{0}, 0, 0};

// Global stats
UART3Stats uart3_stats = {0, 0, 0, 0, 0};

// ============== LPUART3 ISR ==============

// Forward declaration for ISR
static void lpuart3_isr(void);

static void lpuart3_isr(void) {
    uart3_stats.isr_count++;

    uint32_t stat = LPUART3_STAT;

    // RX: Data available in hardware FIFO?
    if (stat & LPUART_STAT_RDRF) {
        // Read all available bytes (FIFO depth = 1, so usually just 1)
        while (LPUART3_STAT & LPUART_STAT_RDRF) {
            uint8_t byte = LPUART3_DATA & 0xFF;
            uint16_t next_head = (rx_ring.head + 1) & UART3_RING_MASK;

            if (next_head != rx_ring.tail) {
                // Buffer not full: store byte
                rx_ring.buf[rx_ring.head] = byte;
                rx_ring.head = next_head;
                uart3_stats.rx_bytes++;
            } else {
                // Buffer full: drop byte
                uart3_stats.rx_overflow++;
            }
        }
    }

    // TX: Hardware ready for next byte?
    if (stat & LPUART_STAT_TDRE) {
        if (tx_ring.head != tx_ring.tail) {
            // Data to send: write next byte to hardware
            LPUART3_DATA = tx_ring.buf[tx_ring.tail];
            tx_ring.tail = (tx_ring.tail + 1) & UART3_RING_MASK;
            uart3_stats.tx_bytes++;
        } else {
            // TX buffer empty: disable TX interrupt
            LPUART3_CTRL &= ~LPUART_CTRL_TIE;
        }
    }

    // Clear any error flags (overrun, noise, framing, parity)
    if (stat & (LPUART_STAT_OR | LPUART_STAT_NF | LPUART_STAT_FE | LPUART_STAT_PF)) {
        LPUART3_STAT = stat;  // Write 1 to clear flags
    }
}

// ============== INITIALIZATION ==============

void uart3_init(uint32_t baud) {
    // Use Teensyduino Serial3.begin() for pin mux and basic setup
    Serial3.begin(baud);

    // Disable interrupts while configuring
    __disable_irq();

    // CRITICAL: Set FIFO depth = 1 for predictable ISR timing
    // This ensures each byte triggers an interrupt immediately
    // Reference: IMXRT1060 Reference Manual, Chapter 47.4.2
    LPUART3_WATER = 0;  // RX watermark = 0, TX watermark = 0

    // Disable FIFOs, then re-enable with minimal depth
    LPUART3_FIFO &= ~(LPUART_FIFO_RXFE | LPUART_FIFO_TXFE);
    LPUART3_FIFO |= (LPUART_FIFO_RXFE | LPUART_FIFO_TXFE);

    // Clear any pending data in hardware FIFOs
    LPUART3_FIFO |= (LPUART_FIFO_RXFLUSH | LPUART_FIFO_TXFLUSH);

    // Attach custom ISR (replaces Teensyduino's default)
    attachInterruptVector(IRQ_LPUART3, lpuart3_isr);

    // Set priority: 64 (lower than Frame ISR @ 32)
    // Lower number = higher priority on ARM Cortex-M7
    NVIC_SET_PRIORITY(IRQ_LPUART3, 64);
    NVIC_ENABLE_IRQ(IRQ_LPUART3);

    // Enable RX interrupt (TX interrupt enabled on demand in uart3_send_bytes)
    LPUART3_CTRL |= LPUART_CTRL_RIE;

    // Clear ring buffers
    rx_ring.head = rx_ring.tail = 0;
    tx_ring.head = tx_ring.tail = 0;

    // Reset stats
    uart3_stats = {0, 0, 0, 0, 0};

    __enable_irq();
}

// ============== RX API ==============

int uart3_get_byte(void) {
    if (rx_ring.head == rx_ring.tail) {
        return -1;  // Empty
    }

    uint8_t byte = rx_ring.buf[rx_ring.tail];
    rx_ring.tail = (rx_ring.tail + 1) & UART3_RING_MASK;
    return byte;
}

uint16_t uart3_available(void) {
    return (rx_ring.head - rx_ring.tail) & UART3_RING_MASK;
}

void uart3_flush_rx(void) {
    __disable_irq();
    rx_ring.head = rx_ring.tail = 0;
    // Also flush hardware FIFO
    LPUART3_FIFO |= LPUART_FIFO_RXFLUSH;
    __enable_irq();
}

// ============== TX API ==============

uint8_t uart3_send_bytes(const uint8_t* data, uint8_t len) {
    uint8_t sent = 0;

    __disable_irq();

    for (uint8_t i = 0; i < len; i++) {
        uint16_t next_head = (tx_ring.head + 1) & UART3_RING_MASK;

        if (next_head == tx_ring.tail) {
            // Buffer full: stop queuing
            uart3_stats.tx_overflow += (len - i);
            break;
        }

        tx_ring.buf[tx_ring.head] = data[i];
        tx_ring.head = next_head;
        sent++;
    }

    // Enable TX interrupt to start transmission
    // (ISR will send bytes and disable when done)
    if (sent > 0) {
        LPUART3_CTRL |= LPUART_CTRL_TIE;
    }

    __enable_irq();

    return sent;
}

bool uart3_tx_complete(void) {
    // Check: TX ring empty AND hardware TX complete
    if (tx_ring.head != tx_ring.tail) {
        return false;  // Still have data to send
    }

    // Check hardware TX complete flag
    return (LPUART3_STAT & LPUART_STAT_TC) != 0;
}
