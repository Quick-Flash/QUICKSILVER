#pragma once

#include "gpio_pin.h"
#include "project.h"

typedef struct gpio_pin_def gpio_pin_def_t;
typedef struct gpio_af_pin_def gpio_af_pin_def_t;

void gpio_init();

void gpio_pin_init(LL_GPIO_InitTypeDef *init, const gpio_pin_def_t *pin);
void gpio_pin_init_af(LL_GPIO_InitTypeDef *init, const gpio_af_pin_def_t *pin_af);
void gpio_pin_set(const gpio_pin_def_t *pin);
void gpio_pin_reset(const gpio_pin_def_t *pin);
uint32_t gpio_pin_read(const gpio_pin_def_t *pin);

int gpio_init_fpv(uint8_t mode);
