#pragma once

#include "gpio_pin.h"

struct uart_port_def {
  uint8_t index;
  USART_TypeDef *channel;

  IRQn_Type irq;

  struct gpio_af_pin_def rx;
  struct gpio_af_pin_def tx;
};