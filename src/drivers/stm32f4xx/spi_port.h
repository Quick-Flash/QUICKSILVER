#pragma once

#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_spi.h>

#include "drv_gyro.h"

#include "gpio_pin.h"

typedef struct {
  DMA_TypeDef *dma;
  uint8_t dma_index;
  uint32_t channel;
  uint8_t channel_index;

  uint8_t rx_stream_index;
  uint32_t rx_tci_flag;
  IRQn_Type rx_it;
  uint32_t rx_it_flag;

  uint8_t tx_stream_index;
  uint32_t tx_tci_flag;
  IRQn_Type tx_it;
  uint32_t tx_it_flag;
} spi_dma_def_t;

static const spi_dma_def_t spi_dma_defs[] = {
    {
        .dma = DMA2,
        .dma_index = 2,
        .channel = LL_DMA_CHANNEL_3,
        .channel_index = 3,

        .rx_stream_index = LL_DMA_STREAM_2,
        .rx_tci_flag = DMA_LISR_TCIF2,
        .rx_it = DMA2_Stream2_IRQn,
        .rx_it_flag = DMA_LIFCR_CTCIF2,

        .tx_stream_index = LL_DMA_STREAM_3,
        .tx_tci_flag = DMA_LISR_TCIF3,
        .tx_it = DMA2_Stream3_IRQn,
        .tx_it_flag = DMA_LIFCR_CTCIF3,
    },
    {
        .dma = DMA1,
        .dma_index = 1,
        .channel = LL_DMA_CHANNEL_0,
        .channel_index = 0,

        .rx_stream_index = LL_DMA_STREAM_3,
        .rx_tci_flag = DMA_LISR_TCIF3,
        .rx_it = DMA1_Stream3_IRQn,
        .rx_it_flag = DMA_LIFCR_CTCIF3,

        .tx_stream_index = LL_DMA_STREAM_4,
        .tx_tci_flag = DMA_HISR_TCIF4,
        .tx_it = DMA1_Stream4_IRQn,
        .tx_it_flag = DMA_HIFCR_CTCIF4,
    },
    {
        .dma = DMA1,
        .dma_index = 1,
        .channel = LL_DMA_CHANNEL_0,
        .channel_index = 0,

        .rx_stream_index = LL_DMA_STREAM_0,
        .rx_tci_flag = DMA_LISR_TCIF0,
        .rx_it = DMA1_Stream0_IRQn,
        .rx_it_flag = DMA_LIFCR_CTCIF0,

        .tx_stream_index = LL_DMA_STREAM_7,
        .tx_tci_flag = DMA_HISR_TCIF7,
        .tx_it = DMA1_Stream7_IRQn,
        .tx_it_flag = DMA_HIFCR_CTCIF7,
    },
};

struct spi_port_def {
  uint8_t index;
  SPI_TypeDef *channel;

  const struct gpio_af_pin_def sck;
  const struct gpio_af_pin_def miso;
  const struct gpio_af_pin_def mosi;

  const spi_dma_def_t *dma;
};

struct spi_device_def {
  const struct spi_port_def *port;
  const struct gpio_pin_def *nss;
};

struct gyro_device_def {
  const gyro_types_t type;
  const struct spi_device_def spi;
  const uint8_t orientation;
  const uint8_t ids[4];
};

struct cc2500_device_def {
  const struct spi_device_def spi;
  const struct gpio_pin_def *gdo0;
  const struct gpio_pin_def *tx_en;
  const struct gpio_pin_def *lna_en;
  const struct gpio_pin_def *ant_sel;
};