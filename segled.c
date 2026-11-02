/**
 * segled.c - STM8S 2-Digit 7-Segment LED Driver Implementation (Common-Anode)
 *
 * Dynamic scanning: onTime() is called every 1ms from the TIM1 ISR
 * and lights exactly one digit (segment code + digit select HIGH for
 * common-anode). A full 2-digit refresh takes 2ms (~500Hz per digit),
 * which is flicker-free for human eyes.
 *
 * Common-anode wiring notes:
 *   - Segment pins drive the shared segment ANODEs through
 *     current-limiting resistors (e.g. 220R per segment), active-LOW
 *     to light a segment (segment code table is complemented).
 *   - Digit pins drive the shared CATHODEs, active-HIGH to select a
 *     digit. For higher brightness / larger displays buffer the digit
 *     select with NPN transistors; direct GPIO is fine for small
 *     displays (common-anode: digit HIGH turns the digit on).
 *
 * DP extension (kept from project4): digits[] values may carry
 * SEG_FLAG_DP (0x20) ORed with a digit 0..15. onTime() then lights
 * the dp segment together with the digit pattern.
 */

#include <stdint.h>
#include "segled.h"

/* ========== GPIO Port Bases (STM8S103F3P6) ========== */
#define GPIO_PA_BASE   0x5000
#define GPIO_PB_BASE   0x5005
#define GPIO_PC_BASE   0x500A
#define GPIO_PD_BASE   0x500F

/* ========== Common-Cathode Segment Code Table ==========
 * bit0..bit6 = a..g, bit7 = dp. Active-HIGH segment outputs
 * (reference table, kept for readability; common-anode output is the
 * bitwise complement of these codes).
 */
static const uint8_t seg_code_table_cc[] = {
    0x3F,  /* 0  */
    0x06,  /* 1  */
    0x5B,  /* 2  */
    0x4F,  /* 3  */
    0x66,  /* 4  */
    0x6D,  /* 5  */
    0x7D,  /* 6  */
    0x07,  /* 7  */
    0x7F,  /* 8  */
    0x6F,  /* 9  */
    0x77,  /* A  (10) */
    0x7C,  /* b  (11) */
    0x39,  /* C  (12) */
    0x5E,  /* d  (13) */
    0x79,  /* E  (14) */
    0x71,  /* F  (15) */
    0x00,  /* blank (16) */
    0x40,  /* '-' (17) */
    0x80   /* '.' (18): dp only */
};

/* ========== SEGLED Method Implementations ========== */

static void segled_init(SEGLED_t *self)
{
    uint8_t i;

    /* Segments: push-pull outputs, all off (HIGH for common-anode). */
    for (i = 0; i < 8; i++) {
        uint16_t port = self->seg_port[i];
        uint8_t  mask = self->seg_mask[i];
        GPIO_DDR(port) |=  mask;
        GPIO_CR1(port) |=  mask;   /* push-pull */
        GPIO_CR2(port) &= ~mask;   /* 2MHz slew (default) */
        GPIO_ODR(port) |=  mask;   /* segment off (HIGH = anode released) */
    }

    /* Digits: push-pull outputs, all inactive (LOW = cathode released
     * for common-anode displays, i.e. no digit selected). */
    for (i = 0; i < SEG_DIGIT_COUNT; i++) {
        uint16_t port = self->dig_port[i];
        uint8_t  mask = self->dig_mask[i];
        GPIO_DDR(port) |=  mask;
        GPIO_CR1(port) |=  mask;   /* push-pull */
        GPIO_CR2(port) &= ~mask;
        GPIO_ODR(port) &= ~mask;   /* digit off (low) */
    }

    self->cur_digit = 0;
}

static void segled_setDigit(SEGLED_t *self, uint8_t pos, uint8_t value)
{
    if (pos < SEG_DIGIT_COUNT) {
        self->digits[pos] = value;
    }
}

static void segled_show(SEGLED_t *self, const char *str)
{
    uint8_t i;
    for (i = 0; i < SEG_DIGIT_COUNT && str && str[i]; i++) {
        char c = str[i];
        if (c >= '0' && c <= '9') {
            self->digits[i] = (uint8_t)(c - '0');
        } else if (c >= 'A' && c <= 'F') {
            self->digits[i] = (uint8_t)(c - 'A' + 10);
        } else if (c >= 'a' && c <= 'f') {
            self->digits[i] = (uint8_t)(c - 'a' + 10);
        } else if (c == '-') {
            self->digits[i] = SEG_DIGIT_MINUS;
        } else {
            self->digits[i] = SEG_DIGIT_BLANK;
        }
    }
}

/* Scan one digit per call. Non-blocking, ISR-safe. */
static void segled_onTime(SEGLED_t *self, uint16_t tick)
{
    (void)tick;                     /* fixed 1ms cadence from TIM1 */
    uint8_t i;
    uint8_t code;
    uint8_t cur;
    uint8_t val;

    /* 1. Blank all digits (all digit selects LOW for common-anode). */
    for (i = 0; i < SEG_DIGIT_COUNT; i++) {
        GPIO_ODR(self->dig_port[i]) &= (uint8_t)~self->dig_mask[i];
    }

    /* 2. Drive segment pattern for the current digit (active-LOW). */
    cur = self->cur_digit;
    val = self->digits[cur];
    code = seg_code_table_cc[val & 0x1F];       /* guard index */
    if (val & SEG_FLAG_DP) {
        code |= 0x80;                           /* light dp segment */
    }
    code = (uint8_t)~code;                      /* common-anode: invert */
    for (i = 0; i < 8; i++) {
        if (code & (1u << i)) {
            GPIO_ODR(self->seg_port[i]) |= self->seg_mask[i];
        } else {
            GPIO_ODR(self->seg_port[i]) &= (uint8_t)~self->seg_mask[i];
        }
    }

    /* 3. Select the current digit (cathode high). */
    GPIO_ODR(self->dig_port[cur]) |= self->dig_mask[cur];

    /* 4. Advance to the next digit. */
    self->cur_digit = (uint8_t)((cur + 1) % SEG_DIGIT_COUNT);
}

/* ========== SEGLED Constructor ========== */

SEGLED_t SEGLED_Create(void)
{
    SEGLED_t seg;

    /* Default pin map (TSSOP20 actual GPIO, 10 pins):
     *   segments PA1 PC5 PA3 PC3 PC4 PA2 PC6 PC7  (a b c d e f g dp)
     *   digits   PD2 PD3                          (DIG1 DIG2)
     * PD0 is NOT brought out on TSSOP20; PD1 = SWIM (kept free for
     * debug), PD4 = BEEP, PD5/PD6 = UART1, PB5/PB4 = HC-SR04. */
    static const uint16_t seg_ports[8] = {
        GPIO_PA_BASE, GPIO_PC_BASE, GPIO_PA_BASE, GPIO_PC_BASE,
        GPIO_PC_BASE, GPIO_PA_BASE, GPIO_PC_BASE, GPIO_PC_BASE
    };
    static const uint8_t  seg_pins[8]  = { 1, 5, 3, 3, 4, 2, 6, 7 };
    static const uint16_t dig_ports[SEG_DIGIT_COUNT] = {
        GPIO_PD_BASE, GPIO_PD_BASE
    };
    static const uint8_t  dig_pins[SEG_DIGIT_COUNT]  = { 2, 3 };

    uint8_t i;

    for (i = 0; i < 8; i++) {
        seg.seg_port[i] = seg_ports[i];
        seg.seg_pin[i]  = seg_pins[i];
        seg.seg_mask[i] = (uint8_t)(1 << seg_pins[i]);
    }
    for (i = 0; i < SEG_DIGIT_COUNT; i++) {
        seg.dig_port[i] = dig_ports[i];
        seg.dig_pin[i]  = dig_pins[i];
        seg.dig_mask[i] = (uint8_t)(1 << dig_pins[i]);
    }

    /* Power-on display: "88" (all segments self-test). */
    seg.digits[0] = 8;
    seg.digits[1] = 8;
    seg.cur_digit = 0;

    /* Bind methods. */
    seg.init     = segled_init;
    seg.setDigit = segled_setDigit;
    seg.show     = segled_show;
    seg.onTime   = segled_onTime;

    return seg;
}
