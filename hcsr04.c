/**
 * hcsr04.c - STM8S HC-SR04 Ultrasonic Distance Sensor Driver Implementation
 *
 * Measurement principle (HC-SR04):
 *   Trig 10us HIGH -> module sends 8x 40kHz bursts -> Echo goes HIGH
 *   when the echo is received, stays HIGH for a time proportional to
 *   the round-trip distance:  distance(cm) = echo_width_us / 58.
 *
 * Wiring (TSSOP20):
 *   Trig = PB5 (open-drain output + external 10k pull-up to VDD),
 *   Echo = PB4 (floating input). PB0/PB1 are not brought out on the
 *   TSSOP20 package.
 *
 * Time base:
 *   TIM1 runs at 1MHz (PSC=15, HSI 16MHz) with ARR=1000 -> 1ms update
 *   interrupt. The free-running 16-bit counter value CNTR (0..999,
 *   1us resolution) plus the global 1ms tick (g_tick) form a monotonic
 *   microsecond clock used for echo width measurement and timeouts.
 *
 * Non-blocking state machine:
 *   start()  begins one measurement cycle (accepted only when idle or
 *            when the previous cycle has been consumed).
 *   service() advances the state machine; it is called once per
 *            main-loop iteration and never blocks longer than the
 *            12us Trig pulse. All waits (echo idle, echo high, echo
 *            low) are polled against the TIM1 microsecond clock with
 *            timeout protection; between two calls the sensor state
 *            is simply held, so the main loop stays fully responsive.
 *   Return:  HCSR04_BUSY while in progress, HCSR04_DONE when the
 *            current cycle finished (result in valid / last_dist_mm).
 *
 * NOTE: g_tick is defined in main.c and incremented by the TIM1 ISR.
 */

#include <stdint.h>
#include "hcsr04.h"

/* ========== GPIO Port Bases (STM8S103F3P6) ========== */
#define GPIO_PB_BASE   0x5005

/* ========== GPIO Register Access (same macros as segled.h) ========== */
#define GPIO_REG(port, off)     (*(volatile uint8_t *)((port) + (off)))
#define GPIO_ODR(port)          GPIO_REG(port, 0)
#define GPIO_IDR(port)          GPIO_REG(port, 1)
#define GPIO_DDR(port)          GPIO_REG(port, 2)
#define GPIO_CR1(port)          GPIO_REG(port, 3)
#define GPIO_CR2(port)          GPIO_REG(port, 4)

/* ========== TIM1 Counter Registers (1MHz, 0..999) ========== */
#define TIM1_CNTRH     (*(volatile uint8_t *)0x525C)
#define TIM1_CNTRL     (*(volatile uint8_t *)0x525D)

/* ========== Global 1ms Tick (defined in main.c) ========== */
extern volatile uint16_t g_tick;

/* ========== Measurement Constants ========== */
#define HCSR04_MIN_US      116UL   /* ~2cm  (58us/cm * 2) */
#define HCSR04_MAX_US    30000UL   /* ~5m   (58us/cm * 500) */
#define HCSR04_US_PER_CM    58UL   /* 58us echo width = 1cm; mm = us*10/58 */

/* ========== State-Machine Timeouts ==========
 * Bounds for the non-blocking waits (microsecond clock):
 *   echo idle    5ms  - echo line stuck HIGH for too long
 *   echo high   30ms  - no echo received (out of range / no target)
 *   echo low    40ms  - echo stuck HIGH too long (> ~6.8m) */
#define HCSR04_WAIT_IDLE_US  5000UL
#define HCSR04_WAIT_HIGH_US 30000UL
#define HCSR04_WAIT_LOW_US  40000UL

/* ========== Microsecond Clock ==========
 * Combines the 1ms tick with the 1MHz TIM1 counter. Reads CNTR first,
 * then tick; if the tick changed between the reads (1ms boundary
 * crossed) the counter is re-read so both values belong to the same
 * millisecond window. */
static uint32_t hcsr04_now_us(void)
{
    uint16_t t, c;

    c = (uint16_t)((uint16_t)TIM1_CNTRH << 8) | TIM1_CNTRL;
    t = g_tick;
    if (t != g_tick) {
        /* tick advanced while reading: re-read counter for new period */
        t = g_tick;
        c = (uint16_t)((uint16_t)TIM1_CNTRH << 8) | TIM1_CNTRL;
    }
    return (uint32_t)t * 1000u + (uint32_t)c;
}

/* ========== Microsecond Delay (bounded, us must be < 1000) ========== */
static void hcsr04_delay_us(uint16_t us)
{
    uint16_t start = (uint16_t)((uint16_t)TIM1_CNTRH << 8) | TIM1_CNTRL;
    uint16_t c;
    do {
        c = (uint16_t)((uint16_t)TIM1_CNTRH << 8) | TIM1_CNTRL;
    } while ((uint16_t)(c - start) < us);   /* wrap-safe unsigned diff */
}

/* ========== HC-SR04 Method Implementations ========== */

static void hcsr04_init(HCSR04_t *self)
{
    /* PB5 = Trig: open-drain output, idle LOW.
     * Open-drain (DDR=1, CR1=0, CR2=0) needs an external 10k pull-up
     * resistor to VDD to drive a HIGH level; a LOW is driven by the
     * output stage directly. */
    GPIO_DDR(self->trig_port) |=  self->trig_mask;
    GPIO_CR1(self->trig_port) &= (uint8_t)~self->trig_mask;
    GPIO_CR2(self->trig_port) &= (uint8_t)~self->trig_mask;
    GPIO_ODR(self->trig_port) &= (uint8_t)~self->trig_mask;

    /* PB4 = Echo: floating input (DDR=0, CR1=0, CR2=0). */
    GPIO_DDR(self->echo_port) &= (uint8_t)~self->echo_mask;
    GPIO_CR1(self->echo_port) &= (uint8_t)~self->echo_mask;
    GPIO_CR2(self->echo_port) &= (uint8_t)~self->echo_mask;

    /* Reset the non-blocking state machine to idle. */
    self->state = HCSR04_STATE_IDLE;
    self->t0 = 0;
    self->t1 = 0;
    self->t_wait = 0;
    self->valid = 0;
    self->last_dist_mm = 0;
}

/* Send a Trig HIGH pulse >= 10us (12us for margin).
 * This is the only short busy-wait in the driver (bounded 12us). */
static void hcsr04_trigger(HCSR04_t *self)
{
    GPIO_ODR(self->trig_port) |= self->trig_mask;
    hcsr04_delay_us(12);
    GPIO_ODR(self->trig_port) &= (uint8_t)~self->trig_mask;
}

/* Mark the current cycle finished with an invalid result. */
static void hcsr04_fail(HCSR04_t *self)
{
    self->valid = 0;
    self->last_dist_mm = 0;
    self->state = HCSR04_STATE_DONE;
}

static uint8_t hcsr04_start(HCSR04_t *self)
{
    /* Accept a new cycle only when idle or after the previous result
     * has been delivered (main loop consumed service()==HCSR04_DONE). */
    if (self->state != HCSR04_STATE_IDLE &&
        self->state != HCSR04_STATE_DONE) {
        return 0;   /* measurement already in progress */
    }

    /* Enter WAIT_IDLE: wait until the Echo line is low before
     * sending the trigger pulse (avoids a stale high edge). */
    self->valid = 0;
    self->state = HCSR04_STATE_WAIT_IDLE;
    self->t_wait = hcsr04_now_us();
    return 1;
}

static uint8_t hcsr04_service(HCSR04_t *self)
{
    uint32_t now;

    switch (self->state) {

    case HCSR04_STATE_IDLE:
        /* Nothing running. */
        return HCSR04_BUSY;

    case HCSR04_STATE_WAIT_IDLE:
        /* 1. Wait for the Echo line to be idle (low), bounded 5ms. */
        if (!(GPIO_IDR(self->echo_port) & self->echo_mask)) {
            hcsr04_trigger(self);               /* send the 10us pulse */
            self->state = HCSR04_STATE_WAIT_HIGH;
            self->t_wait = hcsr04_now_us();
            return HCSR04_BUSY;
        }
        now = hcsr04_now_us();
        if ((now - self->t_wait) > HCSR04_WAIT_IDLE_US) {
            hcsr04_fail(self);                  /* echo stuck high */
            return HCSR04_DONE;
        }
        return HCSR04_BUSY;

    case HCSR04_STATE_WAIT_HIGH:
        /* 2. Wait for the Echo rising edge, bounded 30ms (no echo). */
        if (GPIO_IDR(self->echo_port) & self->echo_mask) {
            self->t0 = hcsr04_now_us();         /* latch rising edge */
            self->state = HCSR04_STATE_WAIT_LOW;
            self->t_wait = self->t0;
            return HCSR04_BUSY;
        }
        now = hcsr04_now_us();
        if ((now - self->t_wait) > HCSR04_WAIT_HIGH_US) {
            hcsr04_fail(self);                  /* no echo / timeout */
            return HCSR04_DONE;
        }
        return HCSR04_BUSY;

    case HCSR04_STATE_WAIT_LOW:
        /* 3. Wait for the Echo falling edge, bounded 40ms. */
        if (!(GPIO_IDR(self->echo_port) & self->echo_mask)) {
            uint32_t us;
            self->t1 = hcsr04_now_us();         /* latch falling edge */
            us = self->t1 - self->t0;           /* echo width (us) */

            /* 4. Plausibility window: ~2cm .. ~5m, then convert to mm.
             *    mm = us*10/58; the 32-bit intermediate keeps us*10
             *    (<= 300000) from overflowing the 16-bit field. */
            if (us < HCSR04_MIN_US || us > HCSR04_MAX_US) {
                hcsr04_fail(self);
            } else {
                self->valid = 1;
                self->last_dist_mm = (uint16_t)(us * 10UL / HCSR04_US_PER_CM);
                self->state = HCSR04_STATE_DONE;
            }
            return HCSR04_DONE;
        }
        now = hcsr04_now_us();
        if ((now - self->t_wait) > HCSR04_WAIT_LOW_US) {
            hcsr04_fail(self);                  /* echo stuck high */
            return HCSR04_DONE;
        }
        return HCSR04_BUSY;

    case HCSR04_STATE_DONE:
        /* Result ready and waiting for the main loop to consume it.
         * start() may begin a new cycle from here. */
        return HCSR04_DONE;

    default:
        self->state = HCSR04_STATE_IDLE;        /* safety reset */
        return HCSR04_BUSY;
    }
}

/* ========== HC-SR04 Constructor ========== */

HCSR04_t HCSR04_Create(void)
{
    HCSR04_t hc;

    /* Default pin map: Trig = PB5, Echo = PB4.
     * TSSOP20 does not bring out PB0/PB1; PB5 is open-drain output
     * with an external 10k pull-up to VDD, PB4 is a floating input. */
    hc.trig_port = GPIO_PB_BASE;
    hc.trig_pin  = 5;
    hc.trig_mask = 0x20;
    hc.echo_port = GPIO_PB_BASE;
    hc.echo_pin  = 4;
    hc.echo_mask = 0x10;

    /* Initial state machine / result defaults. */
    hc.state = HCSR04_STATE_IDLE;
    hc.t0 = 0;
    hc.t1 = 0;
    hc.t_wait = 0;
    hc.last_dist_mm = 0;
    hc.valid = 0;

    /* Bind methods. */
    hc.init    = hcsr04_init;
    hc.start   = hcsr04_start;
    hc.service = hcsr04_service;

    return hc;
}
