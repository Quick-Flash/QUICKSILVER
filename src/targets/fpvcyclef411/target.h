#include "config.h"
#include "config_helper.h"

#define FPVCycleF411

#define F4
#define F411

//PORTS
#define SPI_PORTS \
  SPI1_PA5PA6PA7  \
  SPI2_PB13PB14PB15

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2

//LEDS
#define LED_NUMBER 1
#define LED1PIN PIN_C13

#define BUZZER_PIN PIN_B2

//GYRO
#define MPU6XXX_SPI_PORT SPI_PORT1
#define MPU6XXX_NSS PIN_A4
#define MPU6XXX_INT PIN_A1
//#define SENSOR_ROTATE_NONE
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x71

//RADIO
#ifdef SERIAL_RX
#define RX_USART USART_PORT2
#define SOFTSPI_NONE
#endif

#ifndef SOFTSPI_NONE
#define RADIO_CHECK
#define SPI_MISO_PIN LL_GPIO_PIN_10
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN LL_GPIO_PIN_9
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN LL_GPIO_PIN_3
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN LL_GPIO_PIN_2
#define SPI_SS_PORT GPIOA
#endif

// OSD
#define ENABLE_OSD
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

//VOLTAGE DIVIDER
#define BATTERYPIN PIN_B0
#define BATTERY_ADC_CHANNEL LL_ADC_CHANNEL_8

#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif

#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 1000
#endif

#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PB6
#define MOTOR_PIN1 MOTOR_PIN_PB7
#define MOTOR_PIN2 MOTOR_PIN_PB4
#define MOTOR_PIN3 MOTOR_PIN_PB5
