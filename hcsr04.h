/**
 * hcsr04.h - STM8S HC-SR04 Ultrasonic Distance Sensor Driver
 *           (Object-Oriented, PB5=Trig / PB4=Echo)
 *
 * Measures distance with the HC-SR04 module:
 *   - Trig  : 20us HIGH pulse starts a measurement cycle
 *   - Echo  : HIGH pulse width proportional to distance
 *             (58us per cm, max range ~4..5m)
 *   Validity window: ~2.3cm .. ~5m (134us .. 30000us). Anything shorter
 *   falls inside the HC-SR04 acoustic blind zone and is rejected as
 *   invalid (main.c then shows "--" and does not beep).
 *   Internal precision is MILLIMETRES: mm = echo_us * 10 / 58.
 *
 * Timing is measured with the TIM1 1MHz counter (1us resolution)
 * combined with the global 1ms tick (g_tick, maintained in main.c's
 * TIM1 ISR).
 *
 * The driver is NON-BLOCKING and state-machine driven: start() begins
 * a measurement cycle, then service() is called once per main-loop
 * iteration to advance the state machine (wait echo idle -> trigger
 * 20us pulse -> wait echo high -> wait echo low -> compute). service()
 * never blocks longer than the short 20us Trig pulse; all waits use
 * the TIM1 microsecond clock with timeout protection. main.c starts a
 * new measurement every 200ms and retries a failed attempt (no echo)
 * up to 2 times inside the same period.
 *
 * Pins (TSSOP20):
 *   TRIG PB5 (open-drain output + external 10k pull-up to VDD),
 *   ECHO PB4 (floating input).
 *   NOTE: HC-SR04 Echo output is 5V logic — add a resistor divider
 *   (e.g. 3.3k/2.2k) between Echo and PB4 to keep 3.3V safe levels.
 *   PB0/PB1 are not brought out on the TSSOP20 package.
 *
 * Echo falling-edge de-bounce (half-reading fix):
 *   The Echo HIGH level seen by PB4 depends on the resistor divider. If
 *   the divided level lands close to the input VIH threshold of the pin,
 *   the input can be read as LOW in the middle of a still-valid HIGH
 *   pulse. Such a false falling edge truncates the measured width and the
 *   distance is reported at roughly HALF of the real value.
 *   The driver therefore never accepts a falling edge on the first
 *   sample: WAIT_LOW only latches a *candidate* timestamp and switches to
 *   WAIT_CONFIRM, where the pin is sampled again 200us later.
 *     - still LOW  -> real falling edge, accepted; the candidate
 *                     timestamp (not the confirmation time) is used for
 *                     the width, so the added delay costs no accuracy.
 *     - HIGH again -> glitch, discarded; the driver returns to WAIT_LOW
 *                     and keeps waiting for the true edge.
 *   A 200us window (HCSR04_CONFIRM_US, defined in hcsr04.c) can never
 *   reject the real edge, because the Echo line stays LOW for milliseconds
 *   after it, while short LOW noise bursts (the ~15% "half reading" jitter
 *   at long range) do not last that long and are dropped.
 *   The confirm state carries both a time bound and a retry bound, so a
 *   stalled microsecond clock cannot wedge the state machine.
 *
 * Usage (non-blocking, from the main loop):
 *   #include "hcsr04.h"
 *   HCSR04_t hc = HCSR04_Create();   // PB5=Trig, PB4=Echo
 *   hc.init(&hc);
 *   ...
 *   if (hc.start(&hc)) {             // begin a cycle (rejected if busy)
 *       // measurement is now running in the background
 *   }
 *   if (hc.service(&hc) == HCSR04_DONE) {
 *       if (hc.valid) {
 *           uint16_t d = hc.last_dist_mm;  // valid distance (mm)
 *       } else {
 *           // invalid / no echo / out of range
 *       }
 *   }
 */

#ifndef HCSR04_H
#define HCSR04_H

#include <stdint.h>

/* ========== HC-SR04 Class Definition ========== */

/* --- Non-blocking state machine ---
 * start() begins a cycle; service() is called once per main-loop
 * iteration and advances the state machine without blocking. */
#define HCSR04_STATE_IDLE         0   /* idle, waiting for start() */
#define HCSR04_STATE_WAIT_IDLE    1   /* waiting for Echo line to go low */
#define HCSR04_STATE_WAIT_HIGH    2   /* waiting for Echo rising edge */
#define HCSR04_STATE_WAIT_LOW     3   /* waiting for Echo falling edge */
#define HCSR04_STATE_WAIT_CONFIRM 4   /* falling-edge candidate latched,
                                       * re-sample after 200us to reject a
                                       * glitch (near-VIH false edge or a
                                       * short LOW noise burst) */
#define HCSR04_STATE_DONE         5   /* measurement finished, result ready */

/* service() return codes */
#define HCSR04_BUSY             0   /* measurement still in progress */
#define HCSR04_DONE             1   /* measurement finished (see valid) */

typedef struct HCSR04 HCSR04_t;

struct HCSR04 {
    uint16_t trig_port;         /* GPIO base of Trig pin (PB) */
    uint8_t  trig_pin;          /* pin number (5) */
    uint8_t  trig_mask;         /* precomputed bit mask (0x20) */
    uint16_t echo_port;         /* GPIO base of Echo pin (PB) */
    uint8_t  echo_pin;          /* pin number (4) */
    uint8_t  echo_mask;         /* precomputed bit mask (0x10) */

    /* --- State machine (set by start(), advanced by service()) --- */
    uint8_t  state;             /* HCSR04_STATE_* */
    uint16_t t0;                /* Echo rising-edge CNTR snapshot (1us) */
    uint16_t t1;                /* Echo falling-edge CNTR snapshot (1us).
                                 * In WAIT_CONFIRM this holds the *candidate*
                                 * edge time, which is what gets used for
                                 * the width once the edge is confirmed. */
    uint16_t t_wait;            /* timeout anchor (CNTR snapshot); re-used as
                                 * the confirm-delay anchor in WAIT_CONFIRM */
    uint16_t confirm_retry;     /* service() calls spent in WAIT_CONFIRM;
                                 * bounds the state even if the microsecond
                                 * clock stops advancing (16-bit because
                                 * the 200us window spans far more calls
                                 * than a uint8_t can count) */

    /* --- Last measurement result (valid after service()==HCSR04_DONE) --- */
    uint16_t last_dist_mm;      /* last valid distance in mm (0 if invalid) */
    uint32_t last_echo_us;      /* raw Echo HIGH pulse width captured in the
                                 * last cycle (us). Written immediately at
                                 * the falling edge; cleared to 0 by start()
                                 * so "no echo / timeout" reads 0. Kept
                                 * below 65536 by the 40ms wait timeout, so
                                 * it can be printed with the 16-bit UART
                                 * "%u". Diagnostic aid for range issues. */
    uint8_t  valid;             /* 1 = last measurement valid, 0 = invalid */

    /* --- Optional display hook (interrupt-masked window) -------------
     * During the pulse capture the global interrupt mask is closed, so the
     * TIM2 1ms display-scan ISR cannot run and whichever digit happens to
     * be selected stays lit for the whole window while the other stays
     * dark. That long unbalanced brightness (4..32ms) is what the eye
     * reads as "the digit scan is slow / stuck".
     * main.c binds blank_cb to a routine that switches BOTH digit selects
     * OFF just before the mask is closed, so the frozen window becomes a
     * uniform, barely visible dimming instead of one digit frozen on.
     * NULL is allowed: no hook, old behaviour. */
    void    (*blank_cb)(void);  /* blank every digit (optional, may be 0) */

    /* --- Public methods --- */
    void    (*init)   (HCSR04_t *self);   /* PB5 out / PB4 in */
    uint8_t (*start)  (HCSR04_t *self);   /* start one cycle: 1=ok, 0=busy */
    uint8_t (*service)(HCSR04_t *self);   /* advance: HCSR04_BUSY or DONE */
};

/* ========== Constructor ========== */

/**
 * Create an HC-SR04 object with the default pin map:
 *   Trig = PB5 (open-drain output + ext 10k pull-up), Echo = PB4
 *   (floating input).
 * The constructor does NOT configure the GPIOs; call init() first.
 *
 * @return  fully initialized HCSR04_t object
 */
HCSR04_t HCSR04_Create(void);

#endif /* HCSR04_H */
