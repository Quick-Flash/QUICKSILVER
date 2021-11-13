#pragma once

#include "drv_gpio.h"
#include "project.h"

typedef enum {
  RX_SERIAL_PROTOCOL_INVALID,
  RX_SERIAL_PROTOCOL_DSM,
  RX_SERIAL_PROTOCOL_SBUS,
  RX_SERIAL_PROTOCOL_IBUS,
  RX_SERIAL_PROTOCOL_FPORT,
  RX_SERIAL_PROTOCOL_CRSF,
  RX_SERIAL_PROTOCOL_REDPINE,
  RX_SERIAL_PROTOCOL_SBUS_INVERTED,
  RX_SERIAL_PROTOCOL_FPORT_INVERTED,
  RX_SERIAL_PROTOCOL_REDPINE_INVERTED,
} rx_serial_protocol_t;

// potentially a debug tool to limit the detection sequence of universal serial
// todo:  purge this if deemed unnecessary
//#define RX_SERIAL_PROTOCOL_MAX RX_SERIAL_PROTOCOL_CRSF
#define RX_SERIAL_PROTOCOL_MAX RX_SERIAL_PROTOCOL_REDPINE_INVERTED

typedef struct uart_port_def uart_port_def_t;
typedef enum uart_port_names uart_port_names_t;

extern uart_port_names_t serial_rx_port;
extern uart_port_names_t serial_smart_audio_port;

void serial_enable_rcc(const uart_port_def_t *port);
void serial_enable_isr(const uart_port_def_t *port);

void serial_debug_init(void);
void serial_rx_init(rx_serial_protocol_t rx_serial_protocol);
void serial_smart_audio_init(void);
