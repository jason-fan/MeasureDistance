/**
 * beep.c - STM8S103 BEEP Driver (Object-Oriented, PD4 / pin 1)
 *
 * BEEP output chain: LSI (128 kHz nominal) -> BEEPDIV[4:0] -> ~8 kHz
 *                    -> BEEPSEL[1:0] (3-bit counter) -> 1/2/4 kHz on PD4.
 *
 * Key registers (RM0016):
 *   BEEP_CSR @ 0x50F3  (BEEPSEL bits 7:6, BEEPEN bit5, BEEPDIV bits 4:0)
 *   CLK_ICKR @ 0x50C0  (LSION bit2, LSIRDY bit3)
 *
 * GPIO note: PD4 is the BEEP alternate function; configure it as
 * push-pull output (DDR=1, CR1=1, CR2=0) like any STM8S muxed output.
 */

#include "beep.h"

#define CLK_ICKR    (*(volatile uint8_t *)0x50C0)
#define CLK_ICKR_LSION   0x04   /* bit2: LSI oscillator enable */
#define CLK_ICKR_LSIRDY  0x08   /* bit3: LSI oscillator ready */

#define GPIO_PD      0x500F
#define PD_ODR       (*(volatile uint8_t *)(GPIO_PD + 0))
#define PD_DDR       (*(volatile uint8_t *)(GPIO_PD + 2))
#define PD_CR1       (*(volatile uint8_t *)(GPIO_PD + 3))
#define PD_CR2       (*(volatile uint8_t *)(GPIO_PD + 4))
#define PD4_MASK     0x10

#define LSI_READY_TIMEOUT  100000UL

static Beep_t s_beep;   /* constructor backing storage */

static void beep_init(Beep_t *self)
{
    uint32_t wait = 0;

    /* PD4 = BEEP alternate output: push-pull, no external interrupt */
    PD_DDR |= PD4_MASK;
    PD_CR1 |= PD4_MASK;
    PD_CR2 &= (uint8_t)~PD4_MASK;
    PD_ODR &= (uint8_t)~PD4_MASK;   /* idle low until BEEP drives it */

    /* Enable LSI (BEEP clock source) and wait for ready (bounded) */
    CLK_ICKR |= CLK_ICKR_LSION;
    while (!(CLK_ICKR & CLK_ICKR_LSIRDY)) {
        if (++wait >= LSI_READY_TIMEOUT) break;
    }

    /* BEEPDIV = 0x0E (128k/8k = 16, code 14), BEEPSEL = 1kHz, BEEPEN = 0 */
    BEEP_CSR = (uint8_t)(BEEP_DIV_128K | BEEP_FREQ_1K);

    self->counter   = 0;
    self->freq_code = BEEP_FREQ_1K;
    self->enabled   = 0;
}

static void beep_set_freq(Beep_t *self, uint8_t khz)
{
    uint8_t csr;

    if (khz >= 4) {
        self->freq_code = BEEP_FREQ_4K;
    } else if (khz >= 2) {
        self->freq_code = BEEP_FREQ_2K;
    } else {
        self->freq_code = BEEP_FREQ_1K;
    }

    /* Update BEEPSEL/BEEPEN while preserving BEEPDIV */
    csr = (uint8_t)((BEEP_CSR & (uint8_t)~(BEEP_CSR_BEEPSEL | BEEP_CSR_BEEPEN))
                    | self->freq_code);
    if (self->enabled) {
        csr |= BEEP_CSR_BEEPEN;
    }
    BEEP_CSR = csr;
}

static void beep_on(Beep_t *self)
{
    self->enabled = 1;
    BEEP_CSR |= BEEP_CSR_BEEPEN;   /* frequency already loaded */
}

static void beep_off(Beep_t *self)
{
    self->enabled = 0;
    self->counter = 0;
    BEEP_CSR &= (uint8_t)~BEEP_CSR_BEEPEN;
}

static void beep_once(Beep_t *self, uint8_t khz, uint16_t ms)
{
    beep_set_freq(self, khz);
    beep_on(self);
    self->counter = ms;
}

static void beep_update(Beep_t *self)
{
    if (self->counter) {
        self->counter--;
        if (self->counter == 0) {
            beep_off(self);
        }
    }
}

Beep_t BEEP_Create(void)
{
    s_beep.counter   = 0;
    s_beep.freq_code = BEEP_FREQ_1K;
    s_beep.enabled   = 0;
    s_beep.init      = &beep_init;
    s_beep.set_freq  = &beep_set_freq;
    s_beep.on        = &beep_on;
    s_beep.off       = &beep_off;
    s_beep.once      = &beep_once;
    s_beep.update    = &beep_update;
    return s_beep;
}
