/**
 * STM8S103F3P6 HC-SR04 Ultrasonic + 2-Digit 7-Segment LED Display
 * (Object-Oriented, common-anode)
 *
 * Target MCU:   STM8S103F3P6
 * Compiler:     SDCC (Small Device C Compiler)
 * Clock:        HSI 16MHz
 *
 * Peripherals / pins (TSSOP20):
 *   HC-SR04  PB5=Trig (out, open-drain + ext 10k pull-up),
 *            PB4=Echo (in)                   -> ultrasonic distance
 *   7-SEG    segments a..g,dp -> PA1 PC5 PA3 PC3 PC4 PA2 PC6 PC7
 *            (common-ANODE, active-low segment, 2 digits)
 *            digits 1..2      -> PD2 PD3 (common anode, active high)
 *            (10 GPIO total; HC-SR04 occupies PB4/PB5 (former I2C
 *             pins, I2C not used), UART PD5/PD6 and BEEP PD4 avoided;
 *             PD0 not brought out on TSSOP20; PD1 shared with SWIM
 *             kept free for debug)
 *   UART     PD5 TX / PD6 RX, 115200 8N1 debug output
 *   BEEP     PD4 (hardware BEEP peripheral, LSI clock, 1/2/4 kHz)
 *
 * Display behaviour (internal precision = MILLIMETRES, display in cm):
 *   Every HCSR04_PERIOD_MS (500ms) the main loop starts a new
 *   non-blocking HC-SR04 measurement. service() advances the sensor
 *   state machine once per loop iteration; when a cycle finishes the
 *   2-digit display is updated:
 *     - valid 100..990mm (10..99cm) -> integer cm, DP lit on ones digit
 *     - valid < 100mm (<10cm)       -> one decimal cm, DP lit (x.x cm)
 *     - valid > 990mm (>99cm, OVR)  -> "--" (cannot fit 3 digits)
 *     - invalid / no echo / timeout -> "--"
 *   The result is printed over UART; the beep policy is:
 *     2kHz 50ms for valid 0..99cm, 1kHz 100ms for over-range,
 *     NO beep for invalid / no echo.
 *
 * Event loop: TIM1 hard interrupt drives seg.onTime() dynamic scan
 * (1ms per digit, 2ms full refresh) and beep.update() (auto-off timed
 * beeps). The non-blocking HC-SR04 state machine runs from the main
 * loop (never inside an ISR) and never blocks the loop. UART1 RX
 * (IRQ 18) is serviced by the main loop (echo back only).
 */

#include <stdint.h>
#include "hcsr04.h"
#include "segled.h"
#include "uart.h"
#include "beep.h"

/* ========== User Configuration ========== */

#define HCSR04_PERIOD_MS  500    /* start a new measurement every 500ms */

/* ========== STM8S103 Register Definitions ========== */

#define CLK_CKDIVR      (*(volatile uint8_t *)0x50C6)

/* ========== Clock Configuration ========== */

static void clock_setup(void)
{
    CLK_CKDIVR = 0x00;  /* HSI 16MHz, no division */
}

/* ========== Global Objects (file-level, accessible from ISR) ========== */

static SEGLED_t   seg;                    /* 2-digit common-anode 7-seg */
static HCSR04_t   hcsr04;                 /* ultrasonic sensor PB5/PB4 */
static UART_t     uart;                   /* UART1 debug link (PD5/PD6) */
static Beep_t     beep;                   /* hardware BEEP on PD4 */

/* Global 1ms tick, incremented by TIM1 ISR (used by hcsr04 timing) */
volatile uint16_t g_tick = 0;

/* ========== TIM1 1ms Time-Slice Timer (STM8S103) ========== */

#define TIM1_CR1    (*(volatile uint8_t *)0x5250)
#define TIM1_IER    (*(volatile uint8_t *)0x5254)
#define TIM1_SR1    (*(volatile uint8_t *)0x5255)
#define TIM1_EGR    (*(volatile uint8_t *)0x5257)
#define TIM1_PSCRH  (*(volatile uint8_t *)0x5260)
#define TIM1_PSCRL  (*(volatile uint8_t *)0x5261)
#define TIM1_ARRH   (*(volatile uint8_t *)0x5262)
#define TIM1_ARRL   (*(volatile uint8_t *)0x5263)
#define CLK_PCKENR1 (*(volatile uint8_t *)0x50C7)

/* HSI 16MHz, prescaler /16 -> 1MHz counter, ARR=1000 -> 1ms update */
static void tim1_setup(void)
{
    CLK_PCKENR1 |= 0x80;   /* bit7 = TIM1 clock gate, explicit enable */
    TIM1_PSCRH = 0x00;   /* prescaler high byte (16-bit prescaler = 16) */
    TIM1_PSCRL = 15;     /* /16 */
    TIM1_ARRH  = 0x03;   /* auto-reload 1000 (0x03E8) */
    TIM1_ARRL  = 0xE8;
    TIM1_EGR  |= 0x01;   /* UG: generate update to load prescaler/ARR */
    TIM1_IER  |= 0x01;   /* UIE: enable update interrupt */
    TIM1_CR1  |= 0x01;   /* CEN: start counter */
}

/* ========== TIM1 Update ISR ==========
 * STM8S103F3 TIM1_UPD_OVF_TRG_BRK vector = 0x8034 (IRQ 11)
 * -> SDCC __interrupt(11). Drives the non-blocking 1ms time slices:
 *   1ms dynamic scan of the 2-digit display + BEEP auto-off update.
 *   g_tick is the global 1ms counter also used by hcsr04 for timing. */
void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(11)
{
    TIM1_SR1 &= (uint8_t)~0x01;      /* clear UIF (must do before exit) */

    g_tick++;                        /* advance global 1ms tick */

    /* 1ms dynamic scan: one digit lit per call, full refresh every 2ms. */
    seg.onTime(&seg, g_tick);

    /* 1ms tick for the BEEP module (auto-off timed beeps). */
    beep.update(&beep);
}

/* ========== UART1 RX ISR Stub (must live in the main unit!) ==========
 * SDCC 4.5.0 stm8 后端只在"含 main 的编译单元"生成 __interrupt_vect 向量表。
 * UART1_RX vector = 0x8050 (IRQ 18) -> __interrupt(18)。
 * 向量壳放在本文件，实际处理委托给 uart 模块的 uart_rx_irq_handler()。
 */
void UART1_RX_IRQHandler(void) __interrupt(18)
{
    uart_rx_irq_handler();
}

/* ========== Display a Distance Value ==========
 * Input is a MILLIMETRE distance; display strategy on the 2-digit
 * common-anode display (right digit = ones / least significant,
 * DP = PC7):
 *   >= 100mm  -> integer cm: tens = mm/100 (dp OFF),
 *                ones = (mm/10)%10 WITH dp lit (e.g. 123mm -> "12.");
 *   <  100mm  -> one decimal cm: tens = mm/10 WITH dp lit,
 *                ones = mm%10 (e.g. 35mm -> "3.5", 9mm -> "0.9");
 *   >  990mm  -> "--" (over-range, cannot fit 3 digits).
 * Display buffer update happens in the main loop; the actual scanning
 * runs in seg.onTime() from the TIM1 ISR. */
static void seg_show_distance(uint16_t dist_mm)
{
    if (dist_mm > 990) {
        seg.setDigit(&seg, 0, SEG_DIGIT_MINUS);
        seg.setDigit(&seg, 1, SEG_DIGIT_MINUS);
    } else if (dist_mm >= 100) {
        /* >= 10cm: integer cm, decimal point lit on the ones digit */
        seg.setDigit(&seg, 0, (uint8_t)(dist_mm / 100));
        seg.setDigit(&seg, 1, (uint8_t)((dist_mm / 10) % 10) | SEG_FLAG_DP);
    } else {
        /* < 10cm: one decimal place, tens digit with DP lit */
        seg.setDigit(&seg, 0, (uint8_t)(dist_mm / 10) | SEG_FLAG_DP);
        seg.setDigit(&seg, 1, (uint8_t)(dist_mm % 10));
    }
}

/* ========== Handle One Finished HC-SR04 Cycle ==========
 * Called from the main loop exactly once when service() returns
 * HCSR04_DONE. Updates the display, prints the result over UART and
 * plays the configured beep:
 *   2kHz 50ms  -> valid 0..99cm (mm <= 990)
 *   1kHz 100ms -> valid but over-range (>99cm, mm > 990)
 *   (no beep)  -> invalid / no echo / timeout */
static void hcsr04_handle_done(void)
{
    if (hcsr04.valid) {
        uint16_t d = hcsr04.last_dist_mm;
        seg_show_distance(d);
        if (d <= 990) {
            if (d >= 100) {
                /* >= 10cm: integer cm, no decimal point */
                uart.printf(&uart, "DIST=%u cm\r\n",
                            (unsigned int)(d / 10));
            } else {
                /* < 10cm: one decimal cm, e.g. 35mm -> "DIST=3.5 cm" */
                uart.printf(&uart, "DIST=%u.%u cm\r\n",
                            (unsigned int)(d / 10),
                            (unsigned int)(d % 10));
            }
            beep.once(&beep, 2, 50);
        } else {
            /* > 99cm: over-range, display "--" */
            uart.printf(&uart, "DIST=%u cm (OVR>99)\r\n",
                        (unsigned int)(d / 10));
            beep.once(&beep, 1, 100);
        }
    } else {
        seg_show_distance(0xFFFF);        /* "--" (invalid) */
        uart.puts(&uart, "DIST=-- cm (invalid / out of range)\r\n");
        /* invalid / no echo: do NOT beep */
    }
}

/* ========== Main Entry ========== */

int main(void)
{
    uint16_t last_measure_tick = 0;
    uint8_t  done_handled = 0;      /* edge-triggered DONE consumption */

    /* Create objects (bind methods + defaults) */
    seg    = SEGLED_Create();   /* 2-digit common-anode 7-seg */
    hcsr04 = HCSR04_Create();   /* Trig=PB5, Echo=PB4 */
    uart   = UART_Create(UART1_BASE);
    beep   = BEEP_Create();     /* hardware BEEP on PD4 */

    clock_setup();              /* HSI 16MHz */

    /* Init peripherals */
    uart.init(&uart);           /* UART1 115200 8N1, TX PD5 / RX PD6 */
    seg.init(&seg);             /* 10 GPIO outputs for 2x7-seg (common-anode) */
    hcsr04.init(&hcsr04);       /* PB5=Trig out (open-drain+10k pull), PB4=Echo in */
    beep.init(&beep);           /* LSI + PD4 BEEP output, idle off */

    tim1_setup();               /* TIM1: 1ms update interrupt */
    __asm__("rim");             /* enable global interrupts */

    /* Show "--" until the first measurement completes. */
    seg.setDigit(&seg, 0, SEG_DIGIT_MINUS);
    seg.setDigit(&seg, 1, SEG_DIGIT_MINUS);

    uart.puts(&uart, "STM8S103 HC-SR04 + 2x7SEG UART ready\r\n");
    uart.puts(&uart, "Trig=PB5 Echo=PB4, period=500ms (non-blocking), mm precision, display cm\r\n");

    /* Event-driven main loop.
     * NOTE: 不能用 halt（TIM1 无法唤醒 halt）；用 wfi：CPU 暂停但外设时钟
     * 继续，任何中断（含 TIM1 1ms / UART1 RX）唤醒后自动进入 ISR。 */
    while (1) {
        /* Drain pending TX bytes so startup / previous results reach
         * the console promptly. */
        uart.tx_service(&uart);

        /* UART RX: echo back only (display is HC-SR04 driven now). */
        if (uart.rx_flag) {
            uint8_t c;
            uart.rx_flag = 0;
            c = uart.rx_byte;
            uart.putc(&uart, (char)c);      /* echo back */
        }

        /* Non-blocking HC-SR04 state machine: advance one step per
         * loop iteration. When a cycle finishes, consume the result
         * exactly once (display + UART + beep). */
        if (hcsr04.service(&hcsr04) == HCSR04_DONE) {
            if (!done_handled) {
                done_handled = 1;
                hcsr04_handle_done();
            }
        } else {
            done_handled = 0;
        }

        /* Start a new measurement every HCSR04_PERIOD_MS (500ms). */
        if ((uint16_t)(g_tick - last_measure_tick) >= HCSR04_PERIOD_MS) {
            last_measure_tick = g_tick;
            hcsr04.start(&hcsr04);      /* ignored while busy */
        }

        uart.tx_service(&uart);     /* drain TX after the measurement */

        __asm__("wfi");             /* wait-for-interrupt */
    }

    // return 0;
}
