/**
 * hcsr04.c - STM8S HC-SR04 Ultrasonic Distance Sensor Driver Implementation
 *
 * Measurement principle (HC-SR04):
 *   Trig 20us HIGH -> module sends 8x 40kHz bursts -> Echo goes HIGH
 *   when the echo is received, stays HIGH for a time proportional to
 *   the round-trip distance:  distance(cm) = echo_width_us / 58.
 *
 * Wiring (TSSOP20):
 *   Trig = PB5 (open-drain output + external 10k pull-up to VDD),
 *   Echo = PB4 (floating input). PB0/PB1 are not brought out on the
 *   TSSOP20 package.
 *
 * Time base (pure CNTR, tick-independent):
 *   TIM1 runs free at 1MHz (PSC=15, HSI 16MHz, ARR=0xFFFF) and generates
 *   NO interrupt. Its 16-bit counter CNTR is therefore a pure 1us clock
 *   wrapping every 65.536ms, and every interval measured here fits in
 *   one wrap, so echo width and all timeouts are plain 16-bit
 *   differences of CNTR snapshots - immune to main-loop blocking. A
 *   separate 1ms tick (g_tick, TIM2 ISR) only schedules the measurement
 *   period, the retry gap and the UART report cadence.
 *
 * Non-blocking state machine:
 *   start()  begins one measurement cycle (accepted only when idle or
 *            when the previous cycle has been consumed).
 *   service() advances the state machine; it is called once per
 *            main-loop iteration and never blocks longer than the
 *            20us Trig pulse. All waits (echo idle, echo high, echo
 *            low) are polled against the TIM1 microsecond clock with
 *            timeout protection; between two calls the sensor state
 *            is simply held, so the main loop stays fully responsive.
 *   Sequence: IDLE -> WAIT_IDLE -> WAIT_HIGH -> WAIT_LOW ->
 *            WAIT_CONFIRM -> DONE. WAIT_CONFIRM is the falling-edge
 *            de-bounce step: a LOW sample is only a candidate, and it is
 *            accepted as the real edge only if the line is still LOW
 *            HCSR04_CONFIRM_US (200us) later. This rejects the false
 *            falling edges produced when the divided Echo level approaches
 *            the pin VIH threshold mid-pulse, which used to truncate the
 *            width to roughly half the real distance, and it also drops
 *            short noise glitches that do not persist for the full window
 *            (the real edge keeps the line low for milliseconds).
 *   Return:  HCSR04_BUSY while in progress, HCSR04_DONE when the
 *            current cycle finished (result in valid / last_dist_mm).
 *
 * NOTE: g_tick is defined in main.c and incremented by the TIM2 1ms ISR.
 *       It schedules the measurement cadence only; it is NEVER part of
 *       the echo-width computation.
 */

#include <stdint.h>
#include "hcsr04.h"

/* g_tick is owned by main.c and advanced by the TIM2 1ms ISR. It is
 * referenced here only to hand back the 1ms ticks that are lost while
 * interrupts are masked during a high-resolution echo capture. */
extern volatile uint16_t g_tick;
extern volatile uint16_t g_mask_us_max;   /* TEMP DIAG (main.c) */

/* ========== GPIO Port Bases (STM8S103F3P6) ========== */
#define GPIO_PB_BASE   0x5005

/* ========== GPIO Register Access (same macros as segled.h) ========== */
#define GPIO_REG(port, off)     (*(volatile uint8_t *)((port) + (off)))
#define GPIO_ODR(port)          GPIO_REG(port, 0)
#define GPIO_IDR(port)          GPIO_REG(port, 1)
#define GPIO_DDR(port)          GPIO_REG(port, 2)
#define GPIO_CR1(port)          GPIO_REG(port, 3)
#define GPIO_CR2(port)          GPIO_REG(port, 4)

/* ========== TIM1 Counter Registers (1MHz, counts 0..1000) ==========
 * STM8S103 TIM1 register map (RM0016 / datasheet Table 8):
 *   0x5250 CR1, 0x5251 CR2, 0x5252 SMCR, 0x5253 ETR,
 *   0x5254 IER, 0x5255 SR1, 0x5256 SR2, 0x5257 EGR,
 *   0x5258..0x525B CCMR1..CCMR4,
 *   0x525C CCER1, 0x525D CCER2,
 *   0x525E CNTRH, 0x525F CNTRL,
 *   0x5260 PSCRH, 0x5261 PSCRL, 0x5262 ARRH, 0x5263 ARRL.
 * WARNING: 0x525C / 0x525D are CCER1 / CCER2, which stay 0x00 in this
 * project. Driving the microsecond clock from them made the
 * hcsr04_delay_us() comparison a constant "0 < us" and spun forever,
 * freezing the CPU right after the first Trig pulse. */
#define TIM1_CNTRH     (*(volatile uint8_t *)0x525E)
#define TIM1_CNTRL     (*(volatile uint8_t *)0x525F)

/* ========== Measurement Constants ========== */
#define HCSR04_MIN_US      134UL   /* ~2.3cm (58us/cm * 2.3). Below this the
                                    * HC-SR04 sits in its acoustic blind zone
                                    * and the width is unreliable -> invalid */
#define HCSR04_MAX_US     6000UL   /* ~103cm (58us/cm * 103). The display tops
                                    * out at 99cm, so any echo longer than this
                                    * is treated as out of range and is shown as
                                    * "--" instead of waiting for the real width.
                                    * 6000us instead of the exact 5742us keeps a
                                    * ~250us margin so a target sitting right at
                                    * 99cm is not cut off at the edge and does
                                    * not flicker between a number and "--";
                                    * anything past 1000mm is still shown as "--"
                                    * by the display layer. */
#define HCSR04_US_PER_CM    58UL   /* 58us echo width = 1cm; mm = us*10/58 */

/* ========== State-Machine Timeouts ==========
 * Bounds for the non-blocking waits (microsecond clock):
 *   echo idle    5ms  - echo line stuck HIGH for too long
 *   echo high    8ms  - no rising edge (out of range / no target). The echo
 *                       of a 99cm target arrives ~5.8ms after the trigger, so
 *                       8ms covers the whole useful range with margin.
 *   echo low    40ms  - echo stuck HIGH too long */
#define HCSR04_WAIT_IDLE_US  5000UL
#define HCSR04_WAIT_HIGH_US  8000UL
#define HCSR04_WAIT_LOW_US  40000UL

/* ========== Echo Falling-Edge De-bounce ==========
 * Echo reaches PB4 through a resistor divider. When the divided HIGH
 * level sits close to the pin VIH threshold the input can be sampled LOW
 * in the middle of a still-valid HIGH pulse; accepting that sample as the
 * falling edge truncates the width and the distance comes out at roughly
 * half of the real value. Independently, short LOW glitches on the Echo
 * line produce spurious edges at long range, where the wide echo pulse
 * gives noise a long window to inject one.
 * Every falling-edge candidate is therefore latched (t1) and re-sampled
 * HCSR04_CONFIRM_US (200us) later; the edge is accepted only if the line
 * is still LOW. This is safe for the real edge: after the true falling
 * edge the Echo line stays LOW for milliseconds (until the next 200ms
 * trigger), so a 200us window can never reject it. A LOW that has already
 * gone HIGH again within the window is a glitch and is discarded, sending
 * the state machine back to WAIT_LOW to keep waiting for the true edge.
 * The candidate timestamp is used for the width, so the confirmation delay
 * does not degrade accuracy.
 * HCSR04_CONFIRM_MAX_RETRY bounds the confirm state by service() call
 * count as well, so a stalled microsecond clock cannot wedge the state
 * machine. At roughly 3..5us per service() call the 200us window costs
 * ~40..70 calls, so a bound of 1500 never truncates the window in normal
 * operation; it only fires if the microsecond clock has stopped. */
#define HCSR04_CONFIRM_US          200UL   /* re-sample delay (us) */
#define HCSR04_CONFIRM_MAX_RETRY   1500u   /* fallback bound on re-samples */

/* ========== High-Resolution Pulse Capture ==========
 * The Echo edges used to be sampled once per service() call, i.e. once
 * per main-loop pass. That pass is ~18us long, so every edge was snapped
 * to an 18us grid and the width could be off by one or more grid steps
 * (18us = 3.1mm, 55us = 9.5mm). Because the grid is deterministic and not
 * random, averaging could not remove it - the readings parked on fixed
 * steps such as 774/775 vs 830/831 vs 849 instead of covering the range
 * continuously, and the step pattern (always ~55us apart) was mistaken
 * for a genuine two-reflector echo.
 * The pulse is now captured in one pass, spinning on the pin at ~1us, and
 * global interrupts are masked for the duration so the TIM2 1ms ISR
 * cannot delay an edge sample. Masking is bounded by HCSR04_CAPTURE_US,
 * which is now tied to the 99cm display ceiling (6.0ms) instead of the old
 * 32ms / 5.5m window. Out-of-range targets therefore no longer hold the
 * scan interrupt off for 32ms: the pulse is abandoned as soon as it passes
 * the 99cm width, so the worst-case blanking is ~6.0ms instead of ~32ms. */
#define HCSR04_CAPTURE_US       6000UL   /* max masked pulse window (us), ~103cm */

/* ========== Read the 16-bit TIM1 Counter ==========
 * On STM8S the high byte must be read first: reading CNTRH latches
 * CNTRL, so the following CNTRL read returns the matching low byte and
 * the pair forms an atomic 16-bit snapshot of the 1MHz counter. */
static uint16_t hcsr04_read_cntr(void)
{
    uint8_t hi1, lo, hi2;

    /* Double-read the high byte around the low byte. If CNTRL carried into
     * CNTRH between the two byte accesses, hi1 belongs to the old period
     * while lo already belongs to the new one, and the assembled value is
     * short by 256 counts = 256us = 4.4cm. A changed high byte means a
     * carry did happen, so the pair is fetched again (one extra iteration
     * at most, and it costs only a few CPU cycles).
     *
     * This matters more now than before: the echo width is a plain
     * difference of two such reads, so a torn read would show up directly
     * as a ~4cm spike instead of being averaged away. */
    do {
        hi1 = TIM1_CNTRH;
        lo  = TIM1_CNTRL;
        hi2 = TIM1_CNTRH;
    } while (hi1 != hi2);

    return (uint16_t)(((uint16_t)hi1 << 8) | lo);
}

/* ========== Microsecond Timebase: pure CNTR ==========
 * TIM1 now runs free over its whole 16-bit range (PSC=15 -> 1MHz,
 * ARR=0xFFFF), so CNTR alone is a 1us-resolution clock that wraps every
 * 65536us (65.536ms). Every interval this driver measures is shorter
 * than one wrap - echo width <= 30ms, the 5/30/40ms timeouts and the
 * 200us confirm window - so a plain 16-bit subtraction always yields the
 * true duration:
 *
 *     delta = (uint16_t)(now - then)     // wrap-safe by construction
 *
 * g_tick takes no part in timing any more. It is a 1ms software tick
 * (driven by TIM2) used ONLY for scheduling: measurement period, retry
 * gap and UART report cadence. A late or missed tick can therefore no
 * longer shift a distance reading by 17cm the way the old
 * g_tick*1000+CNTR clock did. */
static uint16_t hcsr04_now_us(void)
{
    return hcsr04_read_cntr();
}

/* ========== Microsecond Delay (bounded, us must be < 1000) ==========
 * Guard: at 1MHz the counter advances once per microsecond, so a 20us
 * pulse needs only ~20 iterations. If TIM1 is not counting (clock gate
 * off, CEN cleared, wrong register address, ...) the counter would
 * never reach us and the loop would hang the CPU forever, so the
 * iteration count is capped and the function returns instead. */
#define HCSR04_DELAY_MAX_LOOP  2000u

static void hcsr04_delay_us(uint16_t us)
{
    uint16_t start = hcsr04_read_cntr();
    uint16_t guard = HCSR04_DELAY_MAX_LOOP;

    while (guard--) {
        if ((uint16_t)(hcsr04_read_cntr() - start) >= us) {
            return;                 /* requested delay elapsed */
        }
    }
    /* Guard expired (timer not running): return without blocking so the
     * caller keeps the main loop responsive. */
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
    self->confirm_retry = 0;
    self->valid = 0;
    self->last_dist_mm = 0;
    self->last_echo_us = 0;
}

/* Send a Trig HIGH pulse >= 10us (20us for margin).
 * 20us instead of the datasheet minimum 10us: this board drives Trig
 * from a 3.3V open-drain pin with an external pull-up, while the
 * HC-SR04 input threshold is ~3.0..3.5V, i.e. the 3.3V level sits
 * right at the edge and a short 12us pulse was missed by the sensor
 * roughly half the time (symptom: many cycles with us=0). A longer
 * pulse rides out the slow pull-up rise and the threshold uncertainty.
 * This is the only short busy-wait in the driver (bounded 20us). */
static void hcsr04_trigger(HCSR04_t *self)
{
    GPIO_ODR(self->trig_port) |= self->trig_mask;
    hcsr04_delay_us(20);
    GPIO_ODR(self->trig_port) &= (uint8_t)~self->trig_mask;
}

/* Mark the current cycle finished with an invalid result. */
static void hcsr04_fail(HCSR04_t *self)
{
    self->valid = 0;
    self->last_dist_mm = 0;
    self->state = HCSR04_STATE_DONE;
}

/* Close the cycle from the confirmed falling edge in self->t1.
 * Called once the edge is trustworthy (de-bounce passed):
 *   3. Publish the raw width for diagnostics (before any filtering, so an
 *      out-of-window capture still shows what the sensor returned).
 *   4. Apply the plausibility window ~2.3cm..~5m and convert to mm.
 *      Below HCSR04_MIN_US (134us, ~2.3cm) the sensor is inside its
 *      acoustic blind zone and the reported width is unreliable, so
 *      such captures are rejected as invalid ("--", no beep).
 *      mm = us*10/58; the 32-bit intermediate keeps us*10 (<= 300000)
 *      from overflowing the 16-bit field.
 * Always returns HCSR04_DONE. */
static uint8_t hcsr04_finish_echo(HCSR04_t *self)
{
    uint32_t us = (uint16_t)(self->t1 - self->t0); /* width (us), wrap-safe */

    self->last_echo_us = us;

    if (us < HCSR04_MIN_US || us > HCSR04_MAX_US) {
        hcsr04_fail(self);
    } else {
        self->valid = 1;
        self->last_dist_mm = (uint16_t)(us * 10UL / HCSR04_US_PER_CM);
        self->state = HCSR04_STATE_DONE;
    }
    return HCSR04_DONE;
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
    self->last_echo_us = 0;     /* cleared per cycle: stays 0 when no echo
                                 * arrives or a wait times out */
    self->state = HCSR04_STATE_WAIT_IDLE;
    self->t_wait = hcsr04_now_us();
    return 1;
}

static uint8_t hcsr04_service(HCSR04_t *self)
{
    uint16_t now;

    switch (self->state) {

    case HCSR04_STATE_IDLE:
        /* Nothing running. */
        return HCSR04_BUSY;

    case HCSR04_STATE_WAIT_IDLE:
        /* 1. Wait for the Echo line to be idle (low), bounded 5ms. */
        if (!(GPIO_IDR(self->echo_port) & self->echo_mask)) {
            hcsr04_trigger(self);               /* send the 20us pulse */
            self->state = HCSR04_STATE_WAIT_HIGH;
            self->t_wait = hcsr04_now_us();
            return HCSR04_BUSY;
        }
        now = hcsr04_now_us();
        if ((uint16_t)(now - self->t_wait) > HCSR04_WAIT_IDLE_US) {
            hcsr04_fail(self);                  /* echo stuck high */
            return HCSR04_DONE;
        }
        return HCSR04_BUSY;

    case HCSR04_STATE_WAIT_HIGH:
    {
        uint16_t t0;
        uint16_t t1;
        uint16_t cand;
        uint16_t span;

        /* 2. Wait for the Echo rising edge, bounded 30ms (no echo).
         *    Spin here instead of re-entering service(): the main-loop
         *    period is ~18us, which is far too coarse for a 1us clock. */
        for (;;) {
            if (GPIO_IDR(self->echo_port) & self->echo_mask) {
                break;                          /* rising edge seen */
            }
            now = hcsr04_now_us();
            if ((uint16_t)(now - self->t_wait) > HCSR04_WAIT_HIGH_US) {
                hcsr04_fail(self);              /* no echo / timeout */
                return HCSR04_DONE;
            }
        }
        t0 = hcsr04_now_us();                   /* rising-edge stamp */

        /* 2b. Capture the whole pulse in one pass, with interrupts
         *     masked so the TIM2 1ms ISR (display scan) cannot delay an
         *     edge sample. The masked window is bounded by
         *     HCSR04_CAPTURE_US.
         *     Before masking, blank every display digit: the scan ISR is
         *     frozen for the whole window, so leaving a digit selected
         *     would hold that one digit lit (unbalanced brightness = the
         *     visible "slow scan"). Blanked, the window only dims the
         *     whole display slightly.
         *     g_tick is NOT compensated by hand any more: the 1ms ISR
         *     simply resumes one tick behind, which keeps the digit phase
         *     perfectly continuous. (Hand compensation on top of the
         *     pending UIF that fires right after rim() counted the same
         *     millisecond twice and made the phase jump every cycle.) */
        if (self->blank_cb != 0) {
            self->blank_cb();
        }
        __asm__("sim");
        for (;;) {
            now = hcsr04_now_us();
            if ((uint16_t)(now - t0) > HCSR04_CAPTURE_US) {
                span = (uint16_t)(now - t0);
                __asm__("rim");
                if (span > g_mask_us_max) {         /* TEMP DIAG */
                    g_mask_us_max = span;
                }
                hcsr04_fail(self);              /* pulse far too long */
                return HCSR04_DONE;
            }
            if (!(GPIO_IDR(self->echo_port) & self->echo_mask)) {
                cand = hcsr04_now_us();         /* candidate falling edge */
                do {                            /* confirm 200us later */
                    span = (uint16_t)(hcsr04_now_us() - cand);
                } while (span < HCSR04_CONFIRM_US);
                if (!(GPIO_IDR(self->echo_port) & self->echo_mask)) {
                    t1 = cand;                  /* real edge, no added error */
                    break;
                }
                /* LOW glitch: keep hunting for the true edge. */
            }
        }
        span = (uint16_t)(hcsr04_now_us() - t0);    /* masked time (us) */
        __asm__("rim");
        if (span > g_mask_us_max) {                 /* TEMP DIAG */
            g_mask_us_max = span;
        }

        self->t0 = t0;
        self->t1 = t1;
        return hcsr04_finish_echo(self);
    }

    case HCSR04_STATE_WAIT_LOW:
        /* 3. Legacy edge-sampling path. No longer entered: step 2b above
         *    captures the full pulse in a single pass, so the width never
         *    gets quantised to the main-loop period again. Kept as a
         *    safety net in case a future change routes back here.
         *
         *    Original behaviour: a LOW sample is NOT taken as the falling
         *    edge yet: with the Echo level near the pin VIH a HIGH pulse
         *    can read LOW in the middle, which would truncate the width.
         *    Latch it as a candidate and go confirm it. */
        if (!(GPIO_IDR(self->echo_port) & self->echo_mask)) {
            self->t1 = hcsr04_now_us();         /* candidate falling edge */
            self->state = HCSR04_STATE_WAIT_CONFIRM;
            self->t_wait = self->t1;            /* confirm-delay anchor */
            self->confirm_retry = 0;
            return HCSR04_BUSY;
        }
        now = hcsr04_now_us();
        if ((uint16_t)(now - self->t_wait) > HCSR04_WAIT_LOW_US) {
            hcsr04_fail(self);                  /* echo stuck high */
            return HCSR04_DONE;
        }
        return HCSR04_BUSY;

    case HCSR04_STATE_WAIT_CONFIRM:
        /* 3b. De-bounce the falling-edge candidate latched in t1.
         *     Re-sample 200us later: still LOW -> real edge; HIGH again ->
         *     glitch, drop the candidate and keep waiting. The real edge
         *     holds the line LOW for milliseconds, so the wide window is
         *     safe, while a short noise glitch cannot survive it. */
        now = hcsr04_now_us();
        if ((uint16_t)(now - self->t_wait) < HCSR04_CONFIRM_US) {
            /* Not yet time to re-sample. The retry bound covers the case
             * where the microsecond clock has stopped advancing: the delay
             * would then never elapse and the state machine would hang. */
            if (++self->confirm_retry < HCSR04_CONFIRM_MAX_RETRY) {
                return HCSR04_BUSY;
            }
        }
        if (GPIO_IDR(self->echo_port) & self->echo_mask) {
            /* Glitch: the line is HIGH again, so t1 was not a real falling
             * edge. Discard the candidate and restore the 40ms anchor of
             * the low-wait (it is measured from the rising edge t0). */
            self->state = HCSR04_STATE_WAIT_LOW;
            self->t_wait = self->t0;
            return HCSR04_BUSY;
        }
        /* Confirmed: keep the candidate timestamp t1 as the falling edge,
         * so the confirmation delay adds no error to the width. */
        return hcsr04_finish_echo(self);

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
    hc.confirm_retry = 0;
    hc.last_dist_mm = 0;
    hc.last_echo_us = 0;
    hc.valid = 0;
    hc.blank_cb = 0;            /* display-blank hook not bound yet */

    /* Bind methods. */
    hc.init    = hcsr04_init;
    hc.start   = hcsr04_start;
    hc.service = hcsr04_service;

    return hc;
}
