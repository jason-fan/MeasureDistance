/**
 * hcsr04.h - STM8S HC-SR04 Ultrasonic Distance Sensor Driver
 *           (Object-Oriented, PB5=Trig / PB4=Echo)
 *
 * Measures distance with the HC-SR04 module:
 *   - Trig  : 10us HIGH pulse starts a measurement cycle
 *   - Echo  : HIGH pulse width proportional to distance
 *             (58us per cm, max range ~4..5m)
 *   Internal precision is MILLIMETRES: mm = echo_us * 10 / 58.
 *
 * Timing is measured with the TIM1 1MHz counter (1us resolution)
 * combined with the global 1ms tick (g_tick, maintained in main.c's
 * TIM1 ISR).
 *
 * The driver is NON-BLOCKING and state-machine driven: start() begins
 * a measurement cycle, then service() is called once per main-loop
 * iteration to advance the state machine (wait echo idle -> trigger
 * 10us pulse -> wait echo high -> wait echo low -> compute). service()
 * never blocks longer than the short 12us Trig pulse; all waits use
 * the TIM1 microsecond clock with timeout protection. main.c starts a
 * new measurement every 500ms.
 *
 * Pins (TSSOP20):
 *   TRIG PB5 (open-drain output + external 10k pull-up to VDD),
 *   ECHO PB4 (floating input).
 *   NOTE: HC-SR04 Echo output is 5V logic — add a resistor divider
 *   (e.g. 3.3k/2.2k) between Echo and PB4 to keep 3.3V safe levels.
 *   PB0/PB1 are not brought out on the TSSOP20 package.
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
#define HCSR04_STATE_IDLE       0   /* idle, waiting for start() */
#define HCSR04_STATE_WAIT_IDLE  1   /* waiting for Echo line to go low */
#define HCSR04_STATE_WAIT_HIGH  2   /* waiting for Echo rising edge */
#define HCSR04_STATE_WAIT_LOW   3   /* waiting for Echo falling edge */
#define HCSR04_STATE_DONE       4   /* measurement finished, result ready */

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
    uint32_t t0;                /* Echo rising-edge timestamp (us clock) */
    uint32_t t1;                /* Echo falling-edge timestamp (us clock) */
    uint32_t t_wait;            /* timeout anchor (us clock) */

    /* --- Last measurement result (valid after service()==HCSR04_DONE) --- */
    uint16_t last_dist_mm;      /* last valid distance in mm (0 if invalid) */
    uint8_t  valid;             /* 1 = last measurement valid, 0 = invalid */

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
