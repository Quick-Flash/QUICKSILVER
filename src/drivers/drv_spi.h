#pragma once

#include <stdint.h>

#include "drv_gpio.h"

#include "project.h"

typedef struct spi_port_def spi_port_def_t;
typedef struct spi_device_def spi_device_def_t;

void spi_enable_rcc(const spi_port_def_t *port);
void spi_init_pins(const spi_device_def_t *dev);

void spi_csn_enable(const spi_device_def_t *dev);
void spi_csn_disable(const spi_device_def_t *dev);

uint8_t spi_transfer_byte(const spi_port_def_t *port, uint8_t data);
uint8_t spi_transfer_byte_timeout(const spi_port_def_t *port, uint8_t data, uint32_t timeout);

void spi_dma_init(const spi_port_def_t *port);
void spi_dma_enable_rcc(const spi_port_def_t *port);

void spi_dma_receive_init(const spi_port_def_t *port, uint8_t *base_address_in, uint32_t buffer_size);
void spi_dma_transmit_init(const spi_port_def_t *port, uint8_t *base_address_out, uint32_t buffer_size);

uint8_t spi_dma_is_ready(const spi_port_def_t *port);
void spi_dma_wait_for_ready(const spi_port_def_t *port);
void spi_dma_transfer_begin(const spi_port_def_t *port, uint8_t *buffer, uint32_t length);
void spi_dma_transfer_bytes(const spi_port_def_t *port, uint8_t *buffer, uint32_t length);

// soft spi  header file
void spi_init(void);
void spi_cson(void);
void spi_csoff(void);
void spi_sendbyte(int);
int spi_sendrecvbyte(int);
int spi_sendzerorecvbyte(void);
