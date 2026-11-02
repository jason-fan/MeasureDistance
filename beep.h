/**
 * beep.h - STM8S103 BEEP Driver (Object-Oriented, PD4 / pin 1)
 *
 * The BEEP peripheral outputs a fixed-frequency square wave on PD4
 * (TSSOP20 pin 1). Clock source is the internal LSI (128 kHz nominal).
 * The LS clock is divided by BEEPDIV[4:0] to ~8 kHz, then BEEPSEL[1:0]
 * selects 1 / 2 / 4 kHz at the pin.
 *
 * Usage:
 *   #include "beep.h"
 *   Beep_t beep = BEEP_Create();
 *   beep.init(&beep);                // LSI on, PD4 push-pull, 1kHz off
 *   ... in ~1ms main-loop tick ...
 *   beep.update(&beep);              // auto-off timed beeps
 *   ... anywhere (also in ISR callbacks) ...
 *   beep.once(&beep, 2, 100);        // 2kHz for 100ms then auto-off
 */

#ifndef BEEP_H
#define BEEP_H

#include <stdint.h>

/* ========== BEEP Register (RM0016, BEEP_CSR @ 0x50F3) ========== */
#define BEEP_CSR    (*(volatile uint8_t *)0x50F3)

#define BEEP_CSR_BEEPSEL  0xC0   /* bits 7:6: output freq selection */
#define BEEP_CSR_BEEPEN   0x20   /* bit  5: BEEP enable */
#define BEEP_CSR_BEEPDIV  0x1F   /* bits 4:0: LS->8kHz divider, code = div-2 */

/* BEEPSEL values (with BEEPDIV yielding ~8kHz internal clock) */
#define BEEP_FREQ_1K   0x00
#define BEEP_FREQ_2K   0x40
#define BEEP_FREQ_4K   0x80

/* BEEPDIV for fLS = 128 kHz: 128k/8k = 16 -> code 16-2 = 0x0E.
 * NOTE: reset value 0x1F is an INVALID code and must not be kept. */
#define BEEP_DIV_128K  0x0E

/* ========== BEEP Class Definition ========== */

typedef struct BEEP Beep_t;

struct BEEP {
    uint16_t counter;    /* remaining ms of current timed beep (0 = idle) */
    uint8_t  freq_code;  /* current BEEP_FREQ_* */
    uint8_t  enabled;    /* current BEEPEN state (1 = outputting) */

    /* --- Public methods --- */
    void (*init)     (Beep_t *self);              /* LSI + PD4 + BEEP off */
    void (*set_freq) (Beep_t *self, uint8_t khz); /* 1 / 2 / 4 kHz */
    void (*on)       (Beep_t *self);              /* enable output (current freq) */
    void (*off)      (Beep_t *self);              /* disable output, cancel timer */
    void (*once)     (Beep_t *self, uint8_t khz, uint16_t ms); /* timed tone */
    void (*update)   (Beep_t *self);              /* ~1ms tick: auto-off */
};

/* ========== Constructor ========== */
Beep_t BEEP_Create(void);

#endif /* BEEP_H */
