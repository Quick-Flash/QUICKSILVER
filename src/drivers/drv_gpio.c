#include "drv_gpio.h"

#include "project.h"

static volatile uint8_t fpv_init_done = 0;

void gpio_init() {
// clocks on to all ports
#ifdef STM32F4
  SET_BIT(
      RCC->AHB1ENR,
      RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
          RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOHEN);
#endif

  LL_GPIO_InitTypeDef init;
  init.Mode = LL_GPIO_MODE_OUTPUT;
  init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  init.Pull = LL_GPIO_PULL_NO;
  init.Speed = LL_GPIO_SPEED_FREQ_HIGH;

#ifdef ENABLE_VREG_PIN
  gpio_pin_init(&init, VREG_PIN_1);
  gpio_pin_set(VREG_PIN_1);
#endif

  for (uint32_t i = 0; i < LED_PIN_MAX; i++) {
    gpio_pin_init(&init, led_defs[i].pin);
  }

#if defined(FPV_SWITCH) && defined(FPV_PIN)
  if (FPV_PIN == PIN_A13 || FPV_PIN == PIN_A14) {
    //skip repurpose of swd pin @boot
  } else {
    gpio_pin_init(&init, FPV_PIN);
    gpio_pin_reset(FPV_PIN);
    fpv_init_done = 1;
  }
#endif
}

// init fpv pin separately because it may use SWDAT/SWCLK don't want to enable it right away
int gpio_init_fpv(uint8_t mode) {
#if defined(FPV_SWITCH) && defined(FPV_PIN)
  // only repurpose the pin after rx/tx have bound if it is swd
  // common settings to set ports
  LL_GPIO_InitTypeDef init;
  init.Mode = LL_GPIO_MODE_OUTPUT;
  init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  init.Pull = LL_GPIO_PULL_NO;
  init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  if (mode == 1 && fpv_init_done == 0) {
    // set gpio pin as output no matter what
    gpio_pin_init(&init, FPV_PIN);
    return 1;
  }
  if (mode == 1 && fpv_init_done == 1) {
    return 1;
  }
#endif
  return 0;
}

void gpio_pin_init(LL_GPIO_InitTypeDef *init, const gpio_pin_def_t *pin) {
  init->Pin = pin->pin_mask;
  LL_GPIO_Init(pin->port, init);
}

void gpio_pin_init_af(LL_GPIO_InitTypeDef *init, const gpio_af_pin_def_t *pin_af) {
  init->Alternate = pin_af->af;
  gpio_pin_init(init, pin_af->pin);
}

void gpio_pin_set(const gpio_pin_def_t *pin) {
  LL_GPIO_SetOutputPin(pin->port, pin->pin_mask);
}

void gpio_pin_reset(const gpio_pin_def_t *pin) {
  LL_GPIO_ResetOutputPin(pin->port, pin->pin_mask);
}

uint32_t gpio_pin_read(const gpio_pin_def_t *pin) {
  return LL_GPIO_IsInputPinSet(pin->port, pin->pin_mask);
}