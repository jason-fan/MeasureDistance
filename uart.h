/**
 * uart.h - STM8S UART1 Driver (Object-Oriented, Debug Output)
 *
 * Provides UART1 register access macros and a polymorphic UART_t
 * abstraction with method binding, mirroring LED_t in led.h.
 * Default: 115200 baud, 8N1. TX pin: PD5, RX pin: PD6.
 * (Reused verbatim from project2/project3.)
 *
 * Usage:
 *   #include "uart.h"
 *   UART_t uart = UART_Create(UART1_BASE);
 *   uart.init(&uart);
 *   uart.puts(&uart, "hello\r\n");
 *   uart.printf(&uart, "tick=%u led=%d\r\n", 1000, 1);
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

/* ========== STM8S103 UART1 Register Addresses ==========
 * Base 0x5230 (verified against STM8S103 datasheet map):
 *   +0: SR     Status Register
 *   +1: DR     Data Register
 *   +2: BRR1   Baud Rate Register 1 (mantissa LSB [7:0])
 *   +3: BRR2   Baud Rate Register 2 (mantissa MSB [11:8] + fraction [3:0])
 *   +4: CR1    Control Register 1
 *   +5: CR2    Control Register 2
 *   +6: CR3    Control Register 3
 */

#define UART1_BASE  0x5230
#define UART1_SR    (*(volatile uint8_t *)0x5230)
#define UART1_DR    (*(volatile uint8_t *)0x5231)
#define UART1_BRR1  (*(volatile uint8_t *)0x5232)
#define UART1_BRR2  (*(volatile uint8_t *)0x5233)
#define UART1_CR1   (*(volatile uint8_t *)0x5234)
#define UART1_CR2   (*(volatile uint8_t *)0x5235)
#define UART1_CR3   (*(volatile uint8_t *)0x5236)

/* SR bits */
#define UART1_SR_TXE   0x80    /* transmit data register empty */
#define UART1_SR_TC    0x40    /* transmission complete */
#define UART1_SR_RXNE  0x20    /* read data register not empty */
#define UART1_SR_OR    0x08    /* overrun error */
#define UART1_SR_PE    0x04    /* parity error */

/* CR1 bits */
#define UART1_CR1_UARTD  0x20  /* UART disable (1 = disable) */
#define UART1_CR1_M      0x10  /* word length (0 = 8 data bits) */
#define UART1_CR1_PCEN   0x08  /* parity control enable */

/* CR2 bits */
#define UART1_CR2_TIEN   0x80  /* transmit interrupt enable */
#define UART1_CR2_RIEN   0x20  /* receive interrupt enable (RXNE -> IRQ18) */
#define UART1_CR2_TEN    0x08  /* transmitter enable */
#define UART1_CR2_REN    0x04  /* receiver enable */

/* CR3 bits */
#define UART1_CR3_STOP1  0x20  /* stop bits: bit1 (00 = 1 stop bit) */
#define UART1_CR3_STOP0  0x10  /* stop bits: bit0 (00 = 1 stop bit) */

/* ========== GPIOD Register Addresses (PD5 = UART1_TX, PD6 = UART1_RX) ==========
 * PD base 0x500F: ODR+0, IDR+1, DDR+2, CR1+3, CR2+4.
 * STM8S alternate-function outputs (UART TX) still require DDR=1
 * (output direction), otherwise the TX signal cannot drive the pin.
 */
#define GPIO_PD_ODR   (*(volatile uint8_t *)0x500F)
#define GPIO_PD_IDR   (*(volatile uint8_t *)0x5010)
#define GPIO_PD_DDR   (*(volatile uint8_t *)0x5011)
#define GPIO_PD_CR1   (*(volatile uint8_t *)0x5012)
#define GPIO_PD_CR2   (*(volatile uint8_t *)0x5013)
#define UART1_TX_PIN  0x20     /* PD5 bit mask */
#define UART1_RX_PIN  0x40     /* PD6 bit mask */

/* ========== Clock Register Addresses ==========
 * CLK_CKDIVR (0x50C6): HSIDIV[5:3] + CPUDIV[2:0]. uart_init must force
 * 0x00 so the UART clock is the full 16MHz HSI.
 * CLK_PCKENR1 bit2 = UART1EN: peripheral clock gate. */
#define CLK_CKDIVR           (*(volatile uint8_t *)0x50C6)
#define CLK_PCKENR1          (*(volatile uint8_t *)0x50C7)
#define CLK_PCKENR1_UART1EN  0x04   /* bit2 = UART1 clock enable (ST SPL: 0x04) */

/* ========== TX Ring Buffer ==========
 * All uart.putc/puts/printf write into this software ring buffer
 * (non-blocking, safe to call from ISR / callbacks); the actual
 * byte transmission is drained by uart.tx_service() in the main loop.
 *
 * TX_BUF_SIZE must be a power of two (e.g. 128). Wrap-around is done
 * with an explicit mask `(idx + 1) & (TX_BUF_SIZE - 1)`.
 * head/tail stay 8-bit atomic on STM8, so the ISR producer and the
 * main-loop consumer never tear a shared pointer.
 */
#define TX_BUF_SIZE  128

/* ========== UART Class Definition ========== */

typedef struct UART UART_t;

struct UART {
    uint16_t base;              /* UART base address (0x5230) */

    /* --- RX state (filled by UART1 RX ISR in uart.c, consumed by main) --- */
    volatile uint8_t rx_byte;   /* last received byte */
    volatile uint8_t rx_flag;   /* 1 = a new byte arrived; clear after use */

    /* --- TX ring buffer (producer = putc/puts/printf, consumer = tx_service) ---
     * head: next write index, written ONLY by producers (ISR-safe)
     * tail: next read index,  written ONLY by tx_service (main loop)
     * empty: head == tail ; full: (uint8_t)(head + 1) == tail
     */
    volatile uint8_t tx_buf[TX_BUF_SIZE];
    volatile uint8_t tx_head;
    volatile uint8_t tx_tail;

    /* --- Public methods --- */
    void (*init)  (UART_t *self);                 /* 115200 8N1, TX+RX, RXNE IRQ */
    void (*putc)  (UART_t *self, char c);         /* enqueue one char (non-blocking) */
    void (*puts)  (UART_t *self, const char *s);  /* enqueue C string (non-blocking) */
    void (*printf)(UART_t *self, const char *fmt, ...); /* %c %s %d %u %x (enqueue) */
    void (*tx_service)(UART_t *self);             /* drain ring buffer (main loop) */
};

/* ========== Constructor ========== */

/**
 * Create a UART object bound to a UART peripheral base address.
 * Uses static storage — safe for single-instance use on STM8S103.
 * The constructor does NOT configure the peripheral; call init() first.
 *
 * @param base  UART base address (use UART1_BASE on STM8S103)
 * @return      fully initialized UART_t object
 */
UART_t UART_Create(uint16_t base);

/* RX ISR helper: called from the __interrupt(18) vector stub in the
 * main compilation unit (SDCC stm8 only emits the interrupt vector
 * table from the module that contains main()). Reads DR into the
 * UART_t instance and sets rx_flag; actual echo is done by main loop. */
void uart_rx_irq_handler(void);

#endif /* UART_H */
