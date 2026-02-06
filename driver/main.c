#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static volatile uint32_t system_millis = 0;

void sys_tick_handler(void) {
    system_millis++;
}

static void delay_ms(uint32_t ms) {
    uint32_t start = system_millis;
    while ((system_millis - start) < ms) {
        __asm__("nop");
    }
}

static void clock_setup(void) {
    /* NUCLEO-F446RE runs from HSI by default; use a libopencm3 helper config.
       This sets up clocks + 168MHz-ish typical for F4 parts (depends on helper).
       If this function isn’t available in your libopencm3 version, tell me and
       I’ll give the explicit PLL setup. */
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_168MHZ]);

    /* 1ms SysTick */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_counter_enable();
    systick_interrupt_enable();
}

static void gpio_setup(void) {
    /* LD2 on NUCLEO-F446RE is PA5 */
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
}

int main(void) {
    clock_setup();
    gpio_setup();

    while (1) {
        gpio_toggle(GPIOA, GPIO5);
        delay_ms(200);
    }
}

