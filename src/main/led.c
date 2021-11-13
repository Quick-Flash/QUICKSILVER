#include "led.h"

#include "drv_gpio.h"
#include "drv_time.h"
#include "project.h"
#include "util.h"

#define LEDALL 15

void ledon(uint8_t val) {
  for (uint32_t i = 1; i <= LED_PIN_MAX; i++) {
    if (val & i) {
      if (led_defs[i - 1].invert) {
        gpio_pin_reset(led_defs[i - 1].pin);
      } else {
        gpio_pin_set(led_defs[i - 1].pin);
      }
    }
  }
}

void ledoff(uint8_t val) {
  for (uint32_t i = 1; i <= LED_PIN_MAX; i++) {
    if (val & i) {
      if (led_defs[i - 1].invert) {
        gpio_pin_set(led_defs[i - 1].pin);
      } else {
        gpio_pin_reset(led_defs[i - 1].pin);
      }
    }
  }
}

void ledflash(uint32_t period, int duty) {
  if (timer_micros() % period > (period * duty) >> 4) {
    ledon(LEDALL);
  } else {
    ledoff(LEDALL);
  }
}

uint8_t led_pwm2(uint8_t pwmval) {
  static int loopcount = 0;
  static int ledlevel = 0;

  ledlevel = pwmval;
  loopcount++;
  loopcount &= 0xF;
  if (ledlevel > loopcount) {
    ledon(255);
  } else {
    ledoff(255);
  }
  return ledlevel;
}

// delta- sigma first order modulator.
uint8_t led_pwm(uint8_t pwmval) {
  static uint32_t lastledtime = 0;
  static float lastledbrightness = 0;
  static float ds_integrator = 0;

  const uint32_t time = timer_micros();
  const uint32_t ledtime = time - lastledtime;

  lastledtime = time;

  float desiredbrightness = pwmval * (1.0f / 15.0f);
  limitf(&ds_integrator, 2);

  ds_integrator += (desiredbrightness - lastledbrightness) * ledtime * (1.0f / LOOPTIME);

  if (ds_integrator > 0.49f) {
    ledon(255);
    lastledbrightness = 1.0f;
  } else {
    ledoff(255);
    lastledbrightness = 0;
  }
  return 0;
}
