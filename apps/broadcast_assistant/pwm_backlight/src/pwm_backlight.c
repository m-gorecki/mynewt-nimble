#include <syscfg/syscfg.h>
#include <bsp/bsp.h>
#include <hal/hal_gpio.h>

#if MYNEWT_VAL(PWM_0) && MYNEWT_VAL(LCD_BL_PIN) >= 0
#include <pwm/pwm.h>

struct pwm_dev *pwm;
int pwm_top_value;
int pwm_inc;
uint16_t pwm_fraction = 2000;
void up(void *arg)
{
    uint16_t fr = pwm_fraction + pwm_inc;
    if (fr > pwm_top_value) {
        fr = 0;
    }
    pwm_fraction = fr;
    pwm_set_duty_cycle(pwm, 0, pwm_fraction);
}

void
pwm_backlight_init(void)
{
    pwm = (struct pwm_dev *)os_dev_open("pwm0", 0, NULL);
    struct pwm_chan_cfg pwm_cfg = {
        .pin = MYNEWT_VAL(LCD_BL_PIN),
    };
    pwm_top_value = pwm_get_top_value(pwm);
    pwm_inc = pwm_top_value / 10;
    pwm_fraction = pwm_inc;
    pwm_configure_channel(pwm, 0, &pwm_cfg);
    pwm_enable(pwm);
    pwm_set_duty_cycle(pwm, 0, pwm_fraction);
    if (MYNEWT_VAL(PWM_BACKLIGHT_BUTTON) >= 0) {
        hal_gpio_irq_init(MYNEWT_VAL(PWM_BACKLIGHT_BUTTON), up, NULL, HAL_GPIO_TRIG_FALLING, HAL_GPIO_PULL_UP);
        hal_gpio_irq_enable(MYNEWT_VAL(PWM_BACKLIGHT_BUTTON));
    }
}

#endif
