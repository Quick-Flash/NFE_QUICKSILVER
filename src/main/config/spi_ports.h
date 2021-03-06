#pragma once

#include "target.h"

#ifdef F4

//                      spi_port, sck_pin, miso_pin, mosi_pin
#define SPI1_PA5PA6PA7 SPI_PORT(1, PIN_A5, PIN_A6, PIN_A7)
#define SPI1_PB3PB4PB5 SPI_PORT(1, PIN_B3, PIN_B4, PIN_B5)
#define SPI2_PB13PB14PB15 SPI_PORT(2, PIN_B13, PIN_B14, PIN_B15)
#define SPI3_PB3PB4PB5 SPI_PORT(3, PIN_B3, PIN_B4, PIN_B5)
#define SPI3_PC10PC11PC12 SPI_PORT(3, PIN_C10, PIN_C11, PIN_C12)

//                      spi_port, dma_port, channel, rx_stream, tx_stream
#define SPI_DMA1 SPI_DMA(1, 2, 3, 2, 3)
#define SPI_DMA2 SPI_DMA(2, 1, 0, 3, 4)
#define SPI_DMA3 SPI_DMA(3, 1, 0, 0, 7)

#endif

#define SPI_IDENT(channel) SPI_PORT##channel
typedef enum {
  SPI_PORT_INVALID,
  SPI_PORT1,
  SPI_PORT2,
  SPI_PORT3,
  SPI_PORTS_MAX,
} spi_ports_t;