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
 *   Every HCSR04_PERIOD_MS (100ms) the main loop starts a new
 *   non-blocking HC-SR04 measurement. service() advances the sensor
 *   state machine once per loop iteration; when a cycle finishes the
 *   2-digit display is updated:
 *     - valid 100..999mm (10..99cm) -> integer cm, DP lit on ones digit
 *     - valid < 100mm (<10cm)       -> one decimal cm, DP lit (x.x cm)
 *     - valid >= 1000mm (>= 1m)     -> "--" (out of display range)
 *     - invalid / no echo / timeout -> "--"
 *     - echo width < 134us (~2.3cm, inside the HC-SR04 acoustic blind
 *       zone) -> treated as invalid: "--", no beep
 *   The validity window is therefore ~2.3cm..~5m; the display range
 *   stops at 1m, and anything longer is shown as "--" exactly like an
 *   invalid reading.
 *   The result is printed over UART (the REAL echo width is always
 *   printed, e.g. "DIST=-- cm (OVR>=1m, us=25523)"); beep policy uses
 *   two escalating warning bands (2kHz 50ms each), so the distance can
 *   be judged by ear alone:
 *     - < HCSR04_CRIT_MM (10cm)  -> ALARM: one beep per
 *       HCSR04_CRIT_BEEP_EVERY (2) accepted readings
 *       (200ms cadence, ~5Hz);
 *     - < HCSR04_NEAR_MM (20cm)  -> one beep per HCSR04_NEAR_BEEP_EVERY
 *       (4) accepted readings (400ms cadence, ~2.5Hz);
 *     - NO beep for >= 20cm, >= 1m, suppressed or rejected readings,
 *       and invalid / no echo. Both counters are cleared whenever a
 *       reading is >= 20cm or invalid.
 *
 * Filter chain (CURRENTLY ACTIVE - see hcsr04_handle_done):
 *   Three stages used to be cascaded here: a median window, a 300mm
 *   jump suppression and a 10mm display hysteresis. Two of them are now
 *   bypassed (functions kept as dead code: dist_filter / display_update)
 *   because the 10mm hysteresis made the panel physically unable to
 *   change by less than a full centimetre - a slow approach stepped
 *   5.5 -> 4.5 with nothing in between - and the jump filter held every
 *   reading it had not seen confirmed.
 *   What runs now is exactly two stages, median FIRST and rounding LAST:
 *       med      = median_push(raw);       5-point sliding median
 *       shown_mm = (med + 2) / 5 * 5;      round to the nearest 5mm
 *   The median is what removes the ±60..90mm spikes seen in the log
 *   (214 -> 296 -> 309 -> 214mm): two adjacent outliers are now rejected
 *   because the median of {214, 296, 309, 220, 211} is 220. A 3-point
 *   window could not do this - the median of {214, 296, 309} is 296.
 *   Rounding only snaps the result onto a 5mm grid, which is simply the
 *   declared display resolution (0.5cm below 10cm, whole cm above it),
 *   so a slow approach still produces every intermediate step
 *   (5.5 -> 5.0 -> 4.5 ...) while the last digit does not change on
 *   every cycle from the ±2-3mm of HC-SR04 noise.
 *   Lag budget: 5 samples x 100ms = 500ms to fill the window after a
 *   reset, then (5-1)/2 x 100ms = 200ms of centre lag - i.e. less than
 *   the 500ms-to-1s the previous configurations settled in.
 *   The log prints the whole chain in one line so both stages stay
 *   auditable: shown (mm) / raw=... / med=...
 *   Invalid cycles (no echo / out of range) still show "--", print the
 *   original invalid line and reset the median window.
 *
 * Event loop: the TIM2 hard interrupt drives seg.onTime() dynamic scan
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

/* Measurement period. Shortened from 200ms to 100ms so that the restored
 * 5-sample median window still settles within 500ms (5 x 100ms) instead
 * of 1s. 100ms is also the HC-SR04 recommended minimum cycle time, so
 * the sensor is never re-triggered while the previous burst may still
 * be ringing. */
#define HCSR04_PERIOD_MS  100    /* start a new measurement every 100ms */

/* UART report policy: CHANGE-DRIVEN, not time-driven.
 * A line is printed only when the value the panel shows has moved by at
 * least HCSR04_UART_CHG_MM since the last printed line, plus the very
 * first line after boot and every transition to/from "--". A still
 * target therefore goes completely silent instead of repeating the same
 * distance several times per second, while a moving one is logged at up
 * to 10 lines/s (one per 100ms cycle) - no event is lost, because any
 * change big enough to matter forces a line by itself.
 * HCSR04_UART_EVERY_N is the old time divider, kept and set to 1: every
 * cycle is now CHECKED, and the change test decides what reaches the
 * console. The divider existed because one line is ~45..70 bytes and at
 * 115200 baud that is ~4..6ms of TX work drained in small slices, so an
 * unconditional line per cycle would keep the loop busy. Change-gating
 * gives the same loop relief (silence while the target is still) without
 * throwing away the events that actually matter. */
#define HCSR04_UART_EVERY_N  1   /* check every cycle (was 10) */

/* Minimum panel change (mm) that justifies a new line. The panel itself
 * is quantised to 5mm, so one display step always passes this test. */
#define HCSR04_UART_CHG_MM   5

/* Timebase self-check line ("TICK=... +5000"). Off: the log is now
 * change-driven only. Set to 1 when the 1ms cadence is in doubt - it is
 * the first thing to check if the display starts lagging or the beep
 * rhythm drifts. */
#define HCSR04_UART_TICK     0

/* Retry policy (fix for ~50% of cycles returning us=0):
 * Trig is driven by a 3.3V open-drain pin while the HC-SR04 input
 * threshold is ~3.0..3.5V, so the trigger pulse is dropped now and
 * then. Inside one HCSR04_PERIOD_MS window a failed attempt
 * (valid == 0 or last_echo_us == 0) is retried after
 * HCSR04_RETRY_GAP_MS, up to HCSR04_RETRY_MAX extra attempts, i.e. at
 * most 3 attempts per period. The retry counter and the gap are both
 * driven by g_tick, so retrying never blocks the main loop and the
 * sensor state machine keeps running normally. A successful attempt
 * ends the retries for that period; if all 3 attempts fail the result
 * is reported as invalid exactly like before ("--", no beep). */
#define HCSR04_RETRY_MAX     2   /* up to 2 retries -> 3 attempts/period */
#define HCSR04_RETRY_GAP_MS  20  /* idle gap between two attempts (ms) */

/* Jump suppression (fix for ~17% spurious echoes at the 4.4m limit):
 * at maximum range the weak echo sometimes returns a much shorter width,
 * e.g. a real 439cm target reads 124/125cm for a single period, so the
 * display appears to jump several metres towards the sensor and back.
 * A new reading that differs from the last ADOPTED one by more than
 * HCSR04_JUMP_THRESHOLD_MM is therefore not shown right away:
 *   - no adopted value yet            -> adopt the reading at once;
 *   - |new - last_good| <= threshold   -> adopt at once (incl. the return
 *                                         to the real value after a glitch);
 *   - |new - pending|  <= threshold    -> the previous outlier repeats, so
 *                                         the target really moved: adopt the
 *                                         pending value;
 *   - otherwise                        -> store the reading as pending and
 *                                         keep the previous display value;
 *                                         the raw value is still printed over
 *                                         UART, marked "suppressed", but no
 *                                         beep is played.
 * Dead-man rule: after HCSR04_PENDING_MAX consecutive rejected cycles the
 * newest reading is adopted anyway, so a target that really moved to a
 * completely different distance cannot freeze the display on a stale
 * value. Invalid cycles (no echo / out of range) never enter this logic
 * and still take the original invalid branch. */
#define HCSR04_JUMP_THRESHOLD_MM   300  /* NOT CALLED right now - kept for
                                         * reference, see dist_filter().
                                         * max plausible step between two
                                         * samples (mm); unrelated to the
                                         * 1000mm DISPLAY limit below.
                                         * Was 1000: that "100cm" bound was
                                         * wider than any real target can
                                         * move, so the filter never
                                         * rejected anything and every raw
                                         * jitter sample went straight to
                                         * the panel. 300mm still passes a
                                         * genuine hand move (fast sweep
                                         * ~1m/s = 100mm per 100ms cycle)
                                         * while rejecting far-reflector
                                         * spikes - but it also swallowed
                                         * legitimate slow approaches, so
                                         * the median window covers the
                                         * spike case instead. */
#define HCSR04_PENDING_MAX           3  /* rejected cycles before forcing */

/* ========== Median Pre-Filter ==========
 * Low-lag jitter rejection: the HC-SR04 occasionally returns spikes
 * (multi-path / weak echo) that would otherwise reach the panel. A
 * sliding median removes them without the lag of a moving average and
 * without ever showing a value that was not measured.
 *
 * Window size history, in order:
 *   - 5 samples: reliable, but with the old 200ms period the settling
 *     time was 5 x 200ms = 1s, so a slowly approaching target appeared
 *     to "stick" while the raw reading kept creeping;
 *   - 3 samples: 200ms lag, but it CANNOT reject two consecutive spikes
 *     (median of {a, spike, spike} is still a spike). The UART log
 *     showed exactly that: 214 -> 296 -> 309 -> 214mm, two adjacent
 *     outliers, which a 3-point window passes through untouched;
 *   - 5 samples again (current), now with the period shortened to
 *     100ms, so the settling time is 5 x 100ms = 500ms - shorter than
 *     the 1s it had originally, while two adjacent spikes are now
 *     rejected: the median of {214, 296, 309, 220, 211} is 220mm.
 *
 * Cost: steady-state response delayed by (HCSR04_MEDIAN_WIN-1)/2 cycles
 * = 2 x 100ms = 200ms. A hand sweeping at ~1m/s moves 20mm per 100ms,
 * i.e. ~40mm across the window - still far below one display centimetre
 * of lag in the perceived value.
 *
 * WHY IT IS BACK TO 3 (after being raised to 5):
 * The two adjacent outliers that forced 5 points (214 -> 296 -> 309) were
 * NOT random noise - they were the edge-sampling grid of the old
 * per-main-loop echo capture. That root cause is gone (hcsr04.c now
 * captures the whole pulse in one pass with interrupts masked), so what
 * is left is isolated single-sample outliers, e.g. one us=982 among a
 * cluster of us=926/927. A 3-point median rejects an isolated spike just
 * as well as a 5-point one, at half the delay. */
#define HCSR04_MEDIAN_WIN             3  /* samples in the median window */

/* ========== Mean Stage (two-peak echo killer) ==========
 * A still target produced a BIMODAL raw distribution, not noise:
 *   us = 1355 (+-1) -> raw = 233mm   } both clusters are rock steady
 *   us = 1410 (+-2) -> raw = 243mm   } (identical us repeated many times)
 * The two clusters are 10mm apart, and neither dominates, so the median
 * of any window is a coin flip between 233 and 243 -> the panel flip-
 * flopped between 24 and 25 forever. A median is only a stable estimate
 * for a SINGLE-peak distribution; with two peaks it is the worst possible
 * filter. Averaging, on the other hand, sits exactly between them:
 * mean(233,243) = 238mm, which quantises to 240 -> a rock-steady "24".
 *
 * So the chain becomes: raw -> median5 (kills spikes) -> mean4 (kills
 * the two-peak flip-flop) -> round to 5mm. The mean is fed with the
 * MEDIAN OUTPUT, not with raw, so an isolated spike has already been
 * removed before it can bias the average.
 *
 * Extra lag: (HCSR04_MEAN_WIN-1)/2 = 1.5 cycles = 150ms, on top of the
 * median's 200ms. Total ~350ms, still well inside human tolerance.
 *
 * WHY IT IS NOW 2 (was 4):
 * The bimodal distribution above was itself an artefact of the old
 * per-main-loop edge sampling - 233mm and 243mm were two SAMPLING GRID
 * STEPS 55us apart, not two physical reflectors (the same 55us split
 * showed up at 133mm/143mm against a bare wall). With the grid gone the
 * mean stage no longer has a two-peak flip-flop to fix; all it does now
 * is shave the residual +-0.3mm dither, so a 2-sample window is enough.
 * A hand sweeping at ~1m/s moves 4.5mm per 100ms, and the whole chain
 * (median3 + mean2) now spans 3 + 2 - 1 = 4 cycles = 400ms, i.e. half
 * the lag it had before. Raise back to 4 only if a genuine two-peak echo
 * reappears with the capture fix in place. */
#define HCSR04_MEAN_WIN               2  /* samples in the mean window */

/* ========== Display Hysteresis ==========
 * The panel shows whole cm (or 0.1cm below 10cm), so millimetre-level
 * noise makes the last digit flicker even when the target is still.
 * The displayed value is only refreshed when the newly accepted
 * distance differs from the one on the panel by at least this much:
 *   - noise below 10mm -> panel stays put (that is one display step);
 *   - real movement   -> the panel steps exactly once per cm crossed, so
 *                        a slowly approaching target is followed 1cm at
 *                        a time instead of jumping 2cm at once.
 * 20mm (the previous value) equals TWO display steps, so a slow approach
 * left the panel frozen for 3-4 cycles and then jumped two digits - that
 * is exactly the "loses precision / does not follow the movement"
 * symptom. 10mm matches the display resolution (1cm, or 0.1cm below
 * 10cm), which is the smallest band that cannot make the last digit
 * flicker while the reading stays inside one step.
 * Raise to 20~30 if the environment is still noisy; must stay well
 * below HCSR04_JUMP_THRESHOLD_MM. */
#define HCSR04_DISPLAY_HYST_MM       10  /* one display step (1cm) */

/* ========== Display / Beep Range Limit ==========
 * The panel only has 2 digits, so distances >= 1000mm (1m) cannot be
 * shown meaningfully: they are displayed as "--" and do not beep. The
 * sensor keeps measuring up to HCSR04_MAX_US (about 5m) and the real
 * echo width is still printed over UART for diagnostics. */
#define DIST_DISPLAY_MAX_MM        1000  /* display range: < 1m only */

/* ========== Near-Range Beep Escalation (two warning bands) ==========
 * The beeper has to tell a CONTINUOUS close-range alarm apart from a
 * plain proximity warning, without the operator looking at the panel.
 * Two bands are used, both 2kHz / 50ms, and BOTH are throttled by an
 * accepted-reading counter because the cycle period is now 100ms - a
 * beep on every cycle would be 10Hz, i.e. the 50ms pulses would merge
 * into a continuous softer tone instead of a chirp:
 *   < HCSR04_CRIT_MM (10cm) -> one beep every HCSR04_CRIT_BEEP_EVERY (2)
 *      accepted readings = once per 200ms: a fast ~5Hz chirp,
 *      unmistakable alarm;
 *   < HCSR04_NEAR_MM (20cm) -> one beep every HCSR04_NEAR_BEEP_EVERY (4)
 *      accepted readings = once per 400ms: audible but not frantic;
 *   >= 20cm up to the 1m display limit -> silent (the display still
 *      shows the distance).
 * Each band owns its own counter and both are cleared as soon as the
 * reading leaves the band or turns invalid, so a partial count can never
 * spill over into a later approach - and the 5Hz alarm cadence can never
 * inherit a half-filled 2.5Hz warning counter. */
#define HCSR04_CRIT_MM              100  /* < 10cm: alarm band */
#define HCSR04_CRIT_BEEP_EVERY        2  /* beep per 2 reads -> 200ms/5Hz */
#define HCSR04_NEAR_MM              200  /* < 20cm: throttled warning */
#define HCSR04_NEAR_BEEP_EVERY        4  /* beep per 4 reads -> 400ms/2.5Hz */

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

/* Global 1ms tick, incremented by the TIM2 ISR.
 * SCOPE: scheduling only - measurement period, retry gap, UART cadence.
 * It is NOT part of any distance computation: the echo width is timed
 * purely from the TIM1 CNTR snapshots (see hcsr04.c). */
volatile uint16_t g_tick = 0;

/* ---- TEMP DIAG: display-scan timing probe (remove after diagnosis) ----
 * TIM1 is a free-running 1us timebase, so its counter gives a cheap
 * microsecond stamp usable from inside an ISR.
 *   g_isr_per_min/max : spacing between consecutive TIM2 ISR entries (us)
 *   g_isr_exec_max    : time spent inside the TIM2 ISR (us)
 *   g_mask_us_max     : longest window with interrupts masked by the
 *                       HC-SR04 echo capture (display scan frozen) (us)
 * All are peak-hold and cleared after each report. */
#define TIM1_CNTRH_D (*(volatile uint8_t *)0x525E)
#define TIM1_CNTRL_D (*(volatile uint8_t *)0x525F)

static uint16_t diag_cntr(void)
{
    uint8_t h1, l, h2;
    do {
        h1 = TIM1_CNTRH_D;
        l  = TIM1_CNTRL_D;
        h2 = TIM1_CNTRH_D;
    } while (h1 != h2);
    return (uint16_t)(((uint16_t)h1 << 8) | l);
}

volatile uint16_t g_isr_per_min  = 0xFFFFu;
volatile uint16_t g_isr_per_max  = 0u;
volatile uint16_t g_isr_exec_max = 0u;
volatile uint16_t g_mask_us_max  = 0u;
static   uint16_t s_diag_prev    = 0u;
static   uint16_t s_diag_report  = 0u;

/* ========== TIM1: Free-Running 1us Timebase (no interrupt) ==========
 * ARR = 0xFFFF (was 1000). With the old 1ms auto-reload the counter only
 * ever held 0..1000, i.e. CNTR could not express an interval longer than
 * 1ms - while an Echo pulse lasts up to 30ms. Running the counter over
 * its full 16-bit range makes CNTR a free 1us clock wrapping every
 * 65.536ms, so the driver can take plain 16-bit differences of two CNTR
 * snapshots and stop caring about g_tick entirely.
 * The update interrupt is DISABLED here: TIM1 is now a pure timebase,
 * and the 1ms time slice lives on TIM2. */

#define TIM1_CR1    (*(volatile uint8_t *)0x5250)
#define TIM1_IER    (*(volatile uint8_t *)0x5254)
#define TIM1_SR1    (*(volatile uint8_t *)0x5255)
#define TIM1_EGR    (*(volatile uint8_t *)0x5257)
#define TIM1_PSCRH  (*(volatile uint8_t *)0x5260)
#define TIM1_PSCRL  (*(volatile uint8_t *)0x5261)
#define TIM1_ARRH   (*(volatile uint8_t *)0x5262)
#define TIM1_ARRL   (*(volatile uint8_t *)0x5263)

/* ========== TIM2: 1ms Time-Slice Timer (display scan / BEEP / g_tick) ========== */

/* WARNING: TIM2's register map is NOT the same as TIM1's. TIM2 has no ETR
 * register and only CCMR1..CCMR3 (TIM1 has ETR + CCMR1..CCMR4), so every
 * register after CR1 is shifted one address lower, and TIM2's prescaler is
 * a SINGLE 8-bit PSCR (there is no PSCRH/PSCRL). Using TIM1-style offsets
 * here leaves UIE inherited by the wrong register (update IRQ never fires)
 * and lands PSC/ARR inside CNTR/ARRH - the counter then reloads at a far
 * slower rate instead of 1ms.
 * Verified against the STM8S103F3 datasheet register map:
 *   CR1 0x5300, IER 0x5303, SR1 0x5304, SR2 0x5305, EGR 0x5306,
 *   CNTRH 0x530C, CNTRL 0x530D, PSCR 0x530E, ARRH 0x530F, ARRL 0x5310 */
#define TIM2_CR1    (*(volatile uint8_t *)0x5300)
#define TIM2_IER    (*(volatile uint8_t *)0x5303)
#define TIM2_SR1    (*(volatile uint8_t *)0x5304)
#define TIM2_SR2    (*(volatile uint8_t *)0x5305)
#define TIM2_EGR    (*(volatile uint8_t *)0x5306)
#define TIM2_CNTRH  (*(volatile uint8_t *)0x530C)
#define TIM2_CNTRL  (*(volatile uint8_t *)0x530D)
#define TIM2_PSCR   (*(volatile uint8_t *)0x530E)
#define TIM2_ARRH   (*(volatile uint8_t *)0x530F)
#define TIM2_ARRL   (*(volatile uint8_t *)0x5310)

#define CLK_PCKENR1 (*(volatile uint8_t *)0x50C7)

/* HSI 16MHz, prescaler /16 -> 1MHz counter, ARR=999 -> 1000 ticks = 1ms */
static void tim1_setup(void)
{
    CLK_PCKENR1 |= 0x80;   /* bit7 = TIM1 clock gate, explicit enable */
    TIM1_PSCRH = 0x00;   /* prescaler high byte (16-bit prescaler = 16) */
    TIM1_PSCRL = 15;     /* /16 -> 1MHz, i.e. 1us per count */
    TIM1_ARRH  = 0xFF;   /* auto-reload 65535: free-running 16-bit */
    TIM1_ARRL  = 0xFF;
    TIM1_EGR  |= 0x01;   /* UG: generate update to load prescaler/ARR */
    TIM1_IER   = 0x00;   /* UIE: NO interrupt - pure timebase */
    TIM1_SR1  &= (uint8_t)~0x01;  /* drop the UIF set by UG */
    TIM1_CR1  |= 0x01;   /* CEN: start counter */
}

/* TIM2 carries the 1ms time slice that TIM1 used to provide. */
static void tim2_setup(void)
{
    CLK_PCKENR1 |= 0x20;   /* bit5 = TIM2 clock gate */
    /* CRITICAL: TIM2/TIM4 prescaler is an EXPONENT, not a divider value.
     * f_CK_CNT = f_CK_PSC / 2^PSCR, so PSCR=4 gives /16 -> 1MHz.
     * Writing 15 here (the TIM1 style "N+1" mental model) selected
     * /32768 -> 488Hz -> with ARR=1000 an update every 2.05 SECONDS:
     * g_tick crawled, the 200ms measure period became ~410s, the 5s
     * TICK report ~2.8h (UART looked dead) and the display advanced
     * one digit every ~2s. TIM1 uses PSCR+1, TIM2 uses 2^PSCR - never
     * copy one to the other. */
    TIM2_PSCR  = 4;      /* 2^4 = /16 -> 1MHz counting clock */
    TIM2_ARRH  = 0x03;   /* auto-reload 1000 -> 1ms update */
    TIM2_ARRL  = 0xE8;
    TIM2_EGR  |= 0x01;   /* UG: load prescaler/ARR */
    TIM2_SR1  &= (uint8_t)~0x01;  /* drop the UIF set by UG */
    TIM2_IER  |= 0x01;   /* UIE: enable update interrupt */
    TIM2_CR1  |= 0x01;   /* CEN: start counter */
}

/* ========== TIM2 Update ISR ==========
 * STM8S103F3 TIM2_UPD_OVF_BRK -> SDCC __interrupt(13).
 * Drives the non-blocking 1ms time slices:
 *   1ms dynamic scan of the 2-digit display + BEEP auto-off update.
 *   g_tick is the global 1ms counter used for scheduling only. */
void TIM2_UPD_OVF_BRK_IRQHandler(void) __interrupt(13)
{
    uint16_t d_t0;
    uint16_t d_t1;
    uint16_t d_gap;

    TIM2_SR1 &= (uint8_t)~0x01;      /* clear UIF (must do before exit) */

    /* ---- TEMP DIAG: ISR period + execution time ---- */
    d_t0  = diag_cntr();
    d_gap = (uint16_t)(d_t0 - s_diag_prev);
    s_diag_prev = d_t0;
    if (d_gap < g_isr_per_min) { g_isr_per_min = d_gap; }
    if (d_gap > g_isr_per_max) { g_isr_per_max = d_gap; }

    g_tick++;                        /* advance global 1ms tick */

    /* 1ms dynamic scan: one digit lit per call, full refresh every 2ms. */
    seg.onTime(&seg, g_tick);

    /* 1ms tick for the BEEP module (auto-off timed beeps). */
    beep.update(&beep);

    /* ---- TEMP DIAG: ISR body duration ---- */
    d_t1 = diag_cntr();
    d_gap = (uint16_t)(d_t1 - d_t0);
    if (d_gap > g_isr_exec_max) { g_isr_exec_max = d_gap; }
}

/* ========== UART1 RX ISR Stub (must live in the main unit!) ==========
 * SDCC 4.5.0 stm8 后端只在"�?main 的编译单�?生成 __interrupt_vect 向量表�? * UART1_RX vector = 0x8050 (IRQ 18) -> __interrupt(18)�? * 向量壳放在本文件，实际处理委托给 uart 模块�?uart_rx_irq_handler()�? */
void UART1_RX_IRQHandler(void) __interrupt(18)
{
    uart_rx_irq_handler();
}

/* ========== Display a Distance Value ==========
 * Input is a MILLIMETRE distance; display strategy on the 2-digit
 * common-anode display (right digit = ones / least significant,
 * DP = PC7):
 *   >= 100mm  -> integer cm, ROUNDED to the nearest cm: tens = r/100
 *                (dp OFF), ones = (r/10)%10 WITH dp lit
 *                (e.g. 123mm -> "12.", 148mm -> "15.");
 *   <  100mm  -> one decimal cm: tens = mm/10 WITH dp lit,
 *                ones = mm%10 (e.g. 35mm -> "3.5", 9mm -> "0.9");
 *   >= 1000mm -> outside the display range: both digits show "-"
 *                ("--", no DP). Any distance of 1m or more is treated
 *                as "no usable reading" on the display, so the panel
 *                never shows an ambiguous compressed cm value.
 * Display buffer update happens in the main loop; the actual scanning
 * runs in seg.onTime() from the TIM1 ISR. */
static void seg_show_distance(uint16_t dist_mm)
{
    if (dist_mm >= DIST_DISPLAY_MAX_MM) {
        /* >= 1m: beyond the display range -> "--" (no decimal point).
         * 0xFFFF (invalid sentinel) also lands here. */
        seg.setDigit(&seg, 0, SEG_DIGIT_MINUS);
        seg.setDigit(&seg, 1, SEG_DIGIT_MINUS);
    } else if (dist_mm >= 100) {
        /* >= 10cm: integer cm, ROUNDED to nearest cm (truncation used to
         * lose up to 9mm: 148mm was shown as "14." although the target
         * was 1.5cm away from that). Clamp at 999 so that e.g. 996mm
         * cannot round up to 100cm and overflow the tens digit. */
        uint16_t r = (uint16_t)(dist_mm + 5);

        if (r >= DIST_DISPLAY_MAX_MM) {
            r = (uint16_t)(DIST_DISPLAY_MAX_MM - 1);
        }
        seg.setDigit(&seg, 0, (uint8_t)(r / 100));
        seg.setDigit(&seg, 1, (uint8_t)((r / 10) % 10) | SEG_FLAG_DP);
    } else {
        /* < 10cm: one decimal place, tens digit with DP lit. The value
         * is already at millimetre resolution, no rounding needed. */
        seg.setDigit(&seg, 0, (uint8_t)(dist_mm / 10) | SEG_FLAG_DP);
        seg.setDigit(&seg, 1, (uint8_t)(dist_mm % 10));
    }
}

/* ========== Distance Jump Filter State ==========
 * Pure state, no hardware. Only touched from the main loop. */
static uint16_t last_good_mm    = 0;   /* last distance shown (mm) */
static uint8_t  have_good       = 0;   /* 1 = last_good_mm is meaningful */
static uint16_t pending_mm      = 0;   /* unconfirmed outlier (mm) */
static uint8_t  have_pending    = 0;   /* 1 = an outlier is waiting */
static uint8_t  pending_rejects = 0;   /* consecutive rejected cycles */

/* ========== Distance Jump Filter ==========
 * Returns 1 and writes the value to show into *out_mm when the reading
 * is accepted (fresh value, confirmed move or dead-man override);
 * returns 0 when the reading is an unconfirmed jump, in which case the
 * display keeps its previous value and nothing is beeped. */
static uint8_t dist_filter(uint16_t d, uint16_t *out_mm)
{
    uint16_t diff;

    /* Very first reading: nothing to compare against, adopt it. */
    if (!have_good) {
        have_good       = 1;
        last_good_mm    = d;
        have_pending    = 0;
        pending_rejects = 0;
        *out_mm = d;
        return 1;
    }

    diff = (d > last_good_mm) ? (uint16_t)(d - last_good_mm)
                              : (uint16_t)(last_good_mm - d);
    if (diff <= HCSR04_JUMP_THRESHOLD_MM) {
        /* Plausible step: adopt now and forget any pending outlier
         * (this is also the path that returns the display to the real
         * 439cm value after a single 124/125cm glitch). */
        last_good_mm    = d;
        have_pending    = 0;
        pending_rejects = 0;
        *out_mm = d;
        return 1;
    }

    if (have_pending) {
        diff = (d > pending_mm) ? (uint16_t)(d - pending_mm)
                                : (uint16_t)(pending_mm - d);
        if (diff <= HCSR04_JUMP_THRESHOLD_MM) {
            /* The same outlier repeats -> the target really moved.
             * Adopt the PENDING value so the shown number stays inside
             * the confirmed cluster instead of following the newest
             * sample. */
            last_good_mm    = pending_mm;
            have_pending    = 0;
            pending_rejects = 0;
            *out_mm = pending_mm;
            return 1;
        }
    }

    /* Unconfirmed jump: remember it, keep the old value on the display. */
    pending_mm     = d;
    have_pending   = 1;
    if (pending_rejects < 0xFF) {
        pending_rejects++;
    }

    if (pending_rejects >= HCSR04_PENDING_MAX) {
        /* Dead-man rule: consecutive outliers that also disagree with
         * each other mean the stored value is stale -> adopt newest. */
        last_good_mm    = d;
        have_pending    = 0;
        pending_rejects = 0;
        *out_mm = d;
        return 1;
    }

    return 0;
}

/* ========== Jump Filter Reset ==========
 * Called for every reading outside the display range (>= 1m). Such a
 * value is never shown as a number, so it must NOT become the filter
 * reference: keeping it would (a) let a real approach from 4.4m down
 * into the display range be misjudged as a jump, and (b) keep a stale
 * far reference alive forever. After the reset the next in-range
 * reading is adopted directly. */
static void dist_filter_reset(void)
{
    have_good       = 0;
    last_good_mm    = 0;
    have_pending    = 0;
    pending_mm      = 0;
    pending_rejects = 0;
}

/* ========== Median Window State ========== */
static uint16_t med_buf[HCSR04_MEDIAN_WIN];
static uint8_t  med_idx = 0;    /* next slot to write */
static uint8_t  med_cnt = 0;    /* valid samples, fills up to the window */

/* ========== Display Hysteresis State ========== */
static uint16_t shown_mm   = 0; /* value currently on the panel (mm) */
static uint8_t  have_shown = 0; /* 1 = shown_mm is meaningful */

/* Drop the median history (out-of-range / invalid readings). Without
 * this a later in-range measurement would be median-averaged together
 * with stale far-away samples and take several cycles to settle. */
static void median_reset(void)
{
    med_idx = 0;
    med_cnt = 0;
}

/* Push one sample, return the median of the samples gathered so far
 * (the window is fully populated after HCSR04_MEDIAN_WIN cycles).
 * Insertion sort over at most 5 entries: no recursion, ~10 bytes of
 * stack, called once per 100ms - negligible. The median never returns
 * a value that was not measured, so no phantom distance is displayed.
 *
 * While the window is still filling (1..WIN-1 samples, i.e. the first
 * 400ms after a reset) the newest sample is passed straight through.
 * Taking tmp[med_cnt/2] on a partial window would return the LOWER
 * median: with 2 samples that is the older/bigger one and with 4 samples
 * it is the third of four, so a fresh reading would be dragged towards
 * one end of the cluster for a few cycles. */
static uint16_t median_push(uint16_t v)
{
    uint16_t tmp[HCSR04_MEDIAN_WIN];
    uint16_t key;
    uint8_t  i, j;

    med_buf[med_idx] = v;
    med_idx = (uint8_t)((med_idx + 1) % HCSR04_MEDIAN_WIN);
    if (med_cnt < HCSR04_MEDIAN_WIN) {
        med_cnt++;
        if (med_cnt < HCSR04_MEDIAN_WIN) {
            return v;       /* window filling: no median yet */
        }
    }

    for (i = 0; i < med_cnt; i++) {
        tmp[i] = med_buf[i];
    }
    for (i = 1; i < med_cnt; i++) {
        key = tmp[i];
        j   = i;
        while (j > 0 && tmp[j - 1] > key) {
            tmp[j] = tmp[j - 1];
            j--;
        }
        tmp[j] = key;
    }
    return tmp[med_cnt / 2];
}

/* ========== Mean Window State ==========
 * Fed with the MEDIAN OUTPUT (not with raw), so spikes are already gone
 * when the average is formed. Its job is the two-peak echo described at
 * HCSR04_MEAN_WIN: it parks the output between the two clusters instead
 * of letting it hop from one to the other.
 *
 * Like the median, a partially filled window averages only the samples
 * it already has - averaging against zeros would drag the first readings
 * towards 0 and show a phantom target at 2cm. */
static uint16_t mean_buf[HCSR04_MEAN_WIN];
static uint8_t  mean_idx = 0;   /* next slot to write */
static uint8_t  mean_cnt = 0;   /* valid samples, fills up to the window */

/* Drop the mean history together with the median history: both are keyed
 * to the same physical target, so a drop-out must clear both, otherwise
 * the mean would blend a fresh in-range reading with stale far-away ones. */
static void mean_reset(void)
{
    mean_idx = 0;
    mean_cnt = 0;
}

/* Push one (already median-filtered) sample and return the mean of the
 * samples gathered so far. uint32_t accumulator: 4 x 1000mm = 4000 fits
 * in 16 bits, but the explicit 32-bit type keeps the division exact even
 * if HCSR04_MEAN_WIN is later raised beyond 16 samples. */
static uint16_t mean_push(uint16_t v)
{
    uint32_t sum = 0;
    uint8_t  i;

    mean_buf[mean_idx] = v;
    mean_idx = (uint8_t)((mean_idx + 1) % HCSR04_MEAN_WIN);
    if (mean_cnt < HCSR04_MEAN_WIN) {
        mean_cnt++;
    }
    for (i = 0; i < mean_cnt; i++) {
        sum += mean_buf[i];
    }
    return (uint16_t)(sum / mean_cnt);
}

/* Console output: ONE line per real change, nothing else.
 * Two fields per line, and the gap between them is the whole point:
 *   the first is the PANEL TEXT - exactly what the two digits show,
 *     whole centimetres from 10cm up ("20 cm") and one decimal below
 *     ("6.0 cm"), so a line can be checked against the board by eye;
 *   the second is the INTERNAL value in millimetres (nearest 5mm after
 *     the median/mean chain). It is never rounded here, so a line like
 *     "SHOW 20 cm | 198 mm" shows the panel truncating 2mm; a systematic
 *     offset between the two columns is rounding, not measurement error.
 * Blank states carry a short tag so a "--" is never ambiguous:
 *   OVR    - echo width >= 1m (outside the display range)
 *   NOECHO - no echo / invalid reading (trigger missed, nothing back) */
static void log_show(uint16_t mm)
{
    if (mm >= 100) {
        uint16_t pr = (uint16_t)(mm + 5);

        if (pr >= DIST_DISPLAY_MAX_MM) {
            pr = (uint16_t)(DIST_DISPLAY_MAX_MM - 1);
        }
        uart.printf(&uart, "SHOW %u cm | %u mm\r\n",
                    (unsigned int)(pr / 10), (unsigned int)mm);
    } else {
        uart.printf(&uart, "SHOW %u.%u cm | %u mm\r\n",
                    (unsigned int)(mm / 10), (unsigned int)(mm % 10),
                    (unsigned int)mm);
    }
}

static void log_blank(const char *why)
{
    uart.printf(&uart, "SHOW -- cm | %s\r\n", why);
}

/* Decide what the panel should show. Returns 1 when it must be
 * refreshed, 0 when the change stays inside the hysteresis band and the
 * current value is kept - this is what stops the last digit from
 * flickering on a still target. */
static uint8_t display_update(uint16_t v)
{
    uint16_t diff;

    if (!have_shown) {
        shown_mm   = v;
        have_shown = 1;
        return 1;
    }
    diff = (v > shown_mm) ? (uint16_t)(v - shown_mm)
                          : (uint16_t)(shown_mm - v);
    if (diff >= HCSR04_DISPLAY_HYST_MM) {
        shown_mm = v;
        return 1;
    }
    return 0;
}

/* ========== Handle One Finished HC-SR04 Cycle ==========
 * Called from the main loop exactly once when service() returns
 * HCSR04_DONE. Updates the display, prints the result over UART and
 * plays the configured beep in one of two escalating bands. Both bands
 * count ACCEPTED readings, because one cycle is only 100ms:
 *   alarm      -> accepted reading < 10cm (HCSR04_CRIT_MM): one beep
 *                 every HCSR04_CRIT_BEEP_EVERY (2) accepted readings,
 *                 i.e. 200ms / 5Hz; its counter is cleared whenever the
 *                 reading leaves the band, so a retreat into the 10-20cm
 *                 band starts counting from scratch
 *   throttled  -> accepted reading < 20cm (HCSR04_NEAR_MM): one beep
 *                 every HCSR04_NEAR_BEEP_EVERY (4) accepted readings,
 *                 counter then restarts from 0 (400ms / 2.5Hz cadence)
 *   (no beep)  -> >= 20cm, >= 1m, suppressed / rejected readings,
 *                 invalid or no echo (both counters cleared on >= 20cm
 *                 and on invalid samples) */
static void hcsr04_handle_done(void)
{
    /* NOTE: the raw echo width (hcsr04.last_echo_us) is deliberately NOT
     * read here any more - the console only carries the panel text and
     * the internal mm value. Re-add a local for it if a per-sample us
     * trace is ever needed again (it must stay 16-bit safe: bounded by
     * the 40ms WAIT_LOW timeout, < 65536). */
    /* Beep throttle counters, one per warning band: only meaningful for
     * accepted readings inside the display range. Held across calls. */
    static uint8_t near_beep_cnt = 0;
    static uint8_t crit_beep_cnt = 0;
    /* UART log divider: 0 = this cycle prints, see HCSR04_UART_EVERY_N. */
    static uint8_t uart_div = 0;
    uint8_t do_print;

    /* Last value that actually reached the console, so the change test has
     * something to compare against:
     *   0xFFFF = nothing printed since boot (forces the very first line)
     *   0xFFFE = the last line was a blank "--" (forces a line as soon as
     *            a real distance comes back, whatever that distance is) */
    static uint16_t printed_mm = 0xFFFF;

    do_print = 0;
    if (++uart_div >= HCSR04_UART_EVERY_N) {
        uart_div = 0;
        do_print = 1;
    }

    if (hcsr04.valid) {
        uint16_t raw = hcsr04.last_dist_mm;  /* raw measurement (mm) */
        uint16_t med;                        /* median-window output (mm) */
        uint16_t avg;                        /* mean-window output (mm) */

        /* Range check FIRST, before the jump filter.
         * >= 1m is outside the display range: the display shows "--"
         * and the raw echo width is printed for observation, but the
         * value is never shown as a number. Deciding this BEFORE
         * dist_filter() is essential - otherwise a legitimate far
         * reading (e.g. 125cm against a stale 4390mm reference) is
         * classified as an "unconfirmed jump" and printed as a plain
         * cm value, which contradicts the >= 1m -> "--" rule.
         * Out-of-range values also reset the filter and the median
         * window: they are never displayed, so they must not pollute
         * the reference for the next in-range measurement. */
        if (raw >= DIST_DISPLAY_MAX_MM) {
            seg_show_distance(0xFFFF);  /* "--" */
            if (do_print && printed_mm != 0xFFFE) {
                log_blank("OVR");       /* one line per entry into "--" */
                printed_mm = 0xFFFE;
            }
            near_beep_cnt = 0;          /* >= 1m: silent, drop the counters */
            crit_beep_cnt = 0;
            dist_filter_reset();
            median_reset();
            mean_reset();
            have_shown = 0;
            return;
        }

        /* Filter chain (in this exact order):
         *   1. 3-point sliding median -> rejects isolated spikes and, at
         *      3 samples, still tracks a slow approach (a 5-point window
         *      held the target 2 cycles behind);
         *   2. 2-point sliding mean on the median output -> shaves the
         *      residual dither left after the median;
         *   3. nearest-5mm rounding    -> snaps to the display grid,
         *      0.5cm below 10cm, whole cm above.
         * Median FIRST, mean SECOND, rounding LAST: a spike must be gone
         * before it can bias the average, and the average must be formed
         * before the value is quantised (otherwise the 5mm grid just
         * carries the flip-flop along in a slightly different size).
         *
         * Settling time: median3 fills in 3 x 100ms and mean2 adds ~50ms
         * of centre lag, i.e. ~250ms to first valid output and ~150ms of
         * steady-state lag - the shortest chain that still rejects a
         * single bad echo. The 300mm jump suppression and the 10mm
         * display hysteresis remain bypassed: both swallowed legitimate
         * slow movement (a real approach was held for several cycles and
         * then jumped). */
        med        = median_push(raw);
        avg        = mean_push(med);
        shown_mm   = (uint16_t)(((avg + 2) / 5) * 5);  /* nearest 5mm */
        have_shown = 1;
        seg_show_distance(shown_mm);

        if (do_print) {
            uint8_t report;

            if (printed_mm == 0xFFFF || printed_mm == 0xFFFE) {
                report = 1;     /* first line, or recovering from "--" */
            } else {
                uint16_t d = (shown_mm > printed_mm)
                                 ? (uint16_t)(shown_mm - printed_mm)
                                 : (uint16_t)(printed_mm - shown_mm);

                report = (uint8_t)(d >= HCSR04_UART_CHG_MM);
            }
            if (report) {
                log_show(shown_mm);
                printed_mm = shown_mm;
            }
        }

        /* Beep policy keyed on the value the panel actually shows, so
         * the beep and the display always agree. One cycle is 100ms, so
         * BOTH bands are throttled by their own counter (a beep on every
         * cycle would be 10Hz and the 50ms pulses would merge into a
         * continuous tone):
         *   <  HCSR04_CRIT_MM (10cm) -> ALARM, one beep per
         *      HCSR04_CRIT_BEEP_EVERY (2) readings = 200ms / 5Hz. The
         *      near counter is pinned to 0 so a retreat into the 10-20cm
         *      band restarts from scratch;
         *   <  HCSR04_NEAR_MM (20cm) -> throttled, one beep per
         *      HCSR04_NEAR_BEEP_EVERY (4) readings = 400ms / 2.5Hz;
         *   >= HCSR04_NEAR_MM (20cm) -> never beeps, both counters are
         *      cleared so a later close approach starts from 0. */
        if (shown_mm < HCSR04_CRIT_MM) {
            near_beep_cnt = 0;          /* pin: alarm has priority */
            if (crit_beep_cnt < HCSR04_CRIT_BEEP_EVERY) {
                crit_beep_cnt++;
            }
            if (crit_beep_cnt >= HCSR04_CRIT_BEEP_EVERY) {
                crit_beep_cnt = 0;
                beep.once(&beep, 2, 50);
            }
        } else if (shown_mm < HCSR04_NEAR_MM) {
            crit_beep_cnt = 0;
            if (near_beep_cnt < HCSR04_NEAR_BEEP_EVERY) {
                near_beep_cnt++;
            }
            if (near_beep_cnt >= HCSR04_NEAR_BEEP_EVERY) {
                near_beep_cnt = 0;
                beep.once(&beep, 2, 50);
            }
        } else {
            near_beep_cnt = 0;      /* >= 20cm: silent */
            crit_beep_cnt = 0;
        }
    } else {
        seg_show_distance(0xFFFF);        /* "--" (invalid) */
        if (do_print && printed_mm != 0xFFFE) {
            log_blank("NOECHO");          /* one line per entry into "--" */
            printed_mm = 0xFFFE;
        }
        /* invalid / no echo: do NOT beep, and drop BOTH band counters
         * so a stale partial count can never trigger later.
         * The median window and the panel value are dropped too, so a
         * long drop-out cannot drag the next real reading around. */
        near_beep_cnt = 0;
        crit_beep_cnt = 0;
        median_reset();
        mean_reset();
        have_shown = 0;
    }
}

/* ========== Main Entry ========== */

/**
 * Blank BOTH digit selects at once.
 *
 * Bound to hcsr04.blank_cb and called right before the driver closes the
 * global interrupt mask to time the Echo pulse. The TIM2 1ms scan ISR is
 * frozen for the whole masked window (4ms at 60..70cm, up to 32ms when
 * there is no echo), so without this the digit that happened to be
 * selected stays lit for the entire window while the other digit stays
 * dark: a fixed, one-sided brightness error on a 10Hz cadence, which is
 * exactly what reads as "the digit scan is visibly slow".
 * Blanked, the frozen window only dims the whole display by that
 * percentage, which the eye barely registers.
 * Common-anode: digit select HIGH lights a digit, so pulling both selects
 * LOW switches both off. Segment lines are left as-is; the next scan ISR
 * rewrites them anyway.
 */
static void seg_blank_all(void)
{
    uint8_t i;

    for (i = 0; i < SEG_DIGIT_COUNT; i++) {
        GPIO_ODR(seg.dig_port[i]) &= (uint8_t)~seg.dig_mask[i];
    }
}

int main(void)
{
    uint16_t last_measure_tick = 0;
    uint8_t  done_handled = 0;      /* edge-triggered DONE consumption */
#if HCSR04_UART_TICK
    uint16_t last_tick_report = 0;  /* time-base self-check (5s), see
                                     * HCSR04_UART_TICK: the variable
                                     * only exists when the check is on,
                                     * otherwise SDCC warns about an
                                     * unused local */
#endif
    uint8_t  retry_count = 0;       /* retries used in the current period */
    uint8_t  retry_wait = 0;        /* 1 = retry gap pending (armed) */
    uint16_t retry_tick = 0;        /* g_tick at the last failed attempt */
    uint8_t  idle_backoff = 0;      /* 0..2 -> period 100/200/400ms.
                                     * Raised while no echo comes back: a
                                     * failed capture keeps the interrupts
                                     * masked for the full HCSR04_CAPTURE_US
                                     * (32ms), i.e. 32% of a 100ms period,
                                     * which is what makes the "--" screen
                                     * flicker. At 800ms the same freeze is
                                     * only 4%, as calm as the numeric case
                                     * (2.4ms / 100ms). Any valid reading
                                     * snaps it straight back to 100ms. */

    /* Create objects (bind methods + defaults) */
    seg    = SEGLED_Create();   /* 2-digit common-anode 7-seg */
    hcsr04 = HCSR04_Create();   /* Trig=PB5, Echo=PB4 */
    hcsr04.blank_cb = seg_blank_all;    /* blank digits during masked capture */
    uart   = UART_Create(UART1_BASE);
    beep   = BEEP_Create();     /* hardware BEEP on PD4 */

    clock_setup();              /* HSI 16MHz */

    /* Init peripherals */
    uart.init(&uart);           /* UART1 115200 8N1, TX PD5 / RX PD6 */
    seg.init(&seg);             /* 10 GPIO outputs for 2x7-seg (common-anode) */
    hcsr04.init(&hcsr04);       /* PB5=Trig out (open-drain+10k pull), PB4=Echo in */
    beep.init(&beep);           /* LSI + PD4 BEEP output, idle off */

    tim1_setup();               /* TIM1: free-running 1us timebase, no IRQ */
    tim2_setup();               /* TIM2: 1ms slice (scan/BEEP/g_tick) */
    __asm__("rim");             /* enable global interrupts */

    /* Show "--" until the first measurement completes. */
    seg.setDigit(&seg, 0, SEG_DIGIT_MINUS);
    seg.setDigit(&seg, 1, SEG_DIGIT_MINUS);

    /* The TX ring only holds TX_BUF_SIZE-1 = 127 bytes; uart_putc() DROPS
     * silently once full. The three banner lines together exceed that, so
     * the tail used to vanish after "Timeb". Flush after every line. */
    uart.puts(&uart, "STM8S103F3 HC-SR04 + 2x7SEG UART ready\r\n");
    uart.tx_service(&uart);
    uart.puts(&uart, "Trig=PB5 Echo=PB4, 100ms period, retry x2, mm in / cm out\r\n");
    uart.tx_service(&uart);
    uart.puts(&uart, "Filter: median3 -> mean2 -> round 5mm. Timebase: TIM1 1us, TIM2 1ms\r\n");
    uart.tx_service(&uart);
    uart.puts(&uart, "Log: one line per >=5mm change: SHOW <panel> | <mm>\r\n");
    uart.tx_service(&uart);

    /* Event-driven main loop.
     * NOTE: 不能�?halt（外设中断无法唤�?halt）；�?wfi：CPU 暂停但外�?     * 时钟继续，任何中断（�?TIM2 1ms / UART1 RX）唤醒后自动进入 ISR�?*/
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
         * exactly once. A failed attempt (no echo: valid==0 or
         * last_echo_us==0) is NOT reported immediately while retries
         * are still available - it is retried HCSR04_RETRY_GAP_MS
         * later, so a transient missed trigger does not reach the
         * display. Only when all HCSR04_RETRY_MAX+1 attempts of the
         * period have failed is the result handled the old way
         * ("--" + no beep). */
        if (hcsr04.service(&hcsr04) == HCSR04_DONE) {
            if (!done_handled) {
                done_handled = 1;
                if (hcsr04.valid && hcsr04.last_echo_us != 0) {
                    retry_wait = 0;         /* success: stop retrying */
                    idle_backoff = 0;       /* echo back: 100ms cadence */
                    hcsr04_handle_done();
                } else if (retry_count < HCSR04_RETRY_MAX) {
                    retry_count++;          /* arm one more attempt */
                    retry_wait = 1;
                    retry_tick = g_tick;
                } else {
                    retry_wait = 0;         /* all attempts failed */
                    if (idle_backoff < 2) {
                        idle_backoff++;     /* no echo: slow down so the
                                             * 6.0ms mask stops eating scan
                                             * frames (ceiling 400ms) */
                    }
                    hcsr04_handle_done();   /* original invalid branch */
                }
            }
        } else {
            done_handled = 0;
        }

        /* Start a new measurement every HCSR04_PERIOD_MS (100ms),
         * stretched up to 4x while no echo is coming back (idle_backoff).
         * A new period also resets the retry budget. */
        if ((uint16_t)(g_tick - last_measure_tick) >=
            (uint16_t)(HCSR04_PERIOD_MS << idle_backoff)) {
            last_measure_tick = g_tick;
            retry_count = 0;
            retry_wait = 0;
            hcsr04.start(&hcsr04);      /* ignored while busy */
        } else if (retry_wait &&
                   (uint16_t)(g_tick - retry_tick) >= HCSR04_RETRY_GAP_MS) {
            /* Retry gap elapsed: re-trigger. start() also clears
             * last_echo_us for the new attempt and is ignored if the
             * state machine is still busy, so this never disturbs an
             * in-flight measurement. */
            retry_wait = 0;
            hcsr04.start(&hcsr04);
        }

        /* Time-base self-check: every 5s report the raw 1ms tick
         * counter AND the delta since the previous report. g_tick is a
         * uint16 and wraps every 65.536s, so the absolute value alone is
         * misleading (e.g. "4464" is just 70000 mod 65536 = the 14th
         * report). The delta is what must read ~5000; a different value
         * means the TIM2 1ms cadence (or the HSI clock) is off, which
         * would bias the measurement period and the UART cadence.
         * NOTE: since the timebase is now pure TIM1 CNTR, a wrong tick
         * no longer affects distance accuracy - it only skews scheduling. */
#if HCSR04_UART_TICK
        if ((uint16_t)(g_tick - last_tick_report) >= 5000u) {
            uint16_t dt = (uint16_t)(g_tick - last_tick_report);

            last_tick_report = g_tick;
            uart.printf(&uart, "TICK=%u (+%u, expect +5000 per report)\r\n",
                        (unsigned int)g_tick, (unsigned int)dt);
        }
#endif

        /* ---- TEMP DIAG report (~2s): what the TIM2 ISR really does ----
         * per  : spacing between ISR entries (expect ~1000 us)
         * isr  : ISR body duration (big -> low duty cycle -> dim/flicker)
         * mask : longest HC-SR04 capture window with IRQs masked, during
         *        which seg.onTime() cannot run at all (scan frozen) */
        if ((uint16_t)(g_tick - s_diag_report) >= 2000u) {
            s_diag_report = g_tick;
            uart.printf(&uart, "SCAN per=%u..%u us, isr=%u us, mask=%u us\r\n",
                        (unsigned int)g_isr_per_min,
                        (unsigned int)g_isr_per_max,
                        (unsigned int)g_isr_exec_max,
                        (unsigned int)g_mask_us_max);
            g_isr_per_min  = 0xFFFFu;
            g_isr_per_max  = 0u;
            g_isr_exec_max = 0u;
            g_mask_us_max  = 0u;
        }

        uart.tx_service(&uart);     /* drain TX after the measurement */

        /* Low-power wait: ONLY when no measurement is in flight.
         * 无条�?wfi 会把 service() 的推进节拍锁死在 1ms 中断上，Echo �?         * 上升�?/ 下降沿都要等到下一�?wfi 唤醒才被采样，两次边沿的采样
         * 延迟各差最�?1ms，测出的宽度就带�?±1000us（≈ ±17cm）误差�?         * 时基本身已是�?CNTR�?us 分辨率），误差不来自计数精度而来�?         * 边沿采样延迟，因此测量窗口内仍需全速轮询�?         * 因此测量进行中（WAIT_IDLE / WAIT_HIGH / WAIT_LOW）必须全�?         * 轮询，才能采到真实的微秒级边沿时刻；只有 IDLE（等下一�?100ms
         * start）和 DONE（结果已就绪、等�?start 复用）才允许 wfi 省电�?*/
        if (hcsr04.state == HCSR04_STATE_IDLE ||
            hcsr04.state == HCSR04_STATE_DONE) {
            __asm__("wfi");         /* idle/done: sleep until next IRQ */
        }
    }

    // return 0;
}
