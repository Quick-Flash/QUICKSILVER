#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <stm32f4xx_ll_gpio.h>

struct gpio_pin_def {
  GPIO_TypeDef *port;
  uint8_t index;
  uint32_t pin_mask;
};

struct gpio_af_pin_def {
  const struct gpio_pin_def *pin;
  uint32_t af;
};

struct led_pin_def {
  uint8_t index;
  const struct gpio_pin_def *pin;
  bool invert;
};

struct battery_adc_def {
  const struct gpio_pin_def *pin;
  uint32_t adc_channel;
  uint32_t divider_r1;
  uint32_t divider_r2;
  float ref_voltage;
};

#define MAKE_PIN_DEF(port, pin) static const struct gpio_pin_def P##port##pin = {GPIO##port, pin, (1U << pin)};
#include "gpio_pin.in"
#undef MAKE_PIN_DEF