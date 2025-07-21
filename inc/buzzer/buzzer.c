#include "buzzer.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

int buzzer_setup(uint gpio_pin, float clk_div) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clk_div);
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(gpio_pin, 0);

    return slice_num;
}

void buzzer_play(uint gpio_pin, uint frequency) {
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t top = clock_freq / frequency - 1;

    pwm_set_wrap(slice_num, top);
    pwm_set_gpio_level(gpio_pin, top / 2);
}

void buzzer_stop(uint gpio_pin) {
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_gpio_level(gpio_pin, 0);
}
