#pragma once

#include <stdint.h>

#include "data_flash.h"

enum m25p16_commands {
  M25P16_WRITE_ENABLE = 0x06,
  M25P16_WRITE_DISABLE = 0x04,

  M25P16_READ_IDENTIFICATION = 0x9F,

  M25P16_READ_STATUS_REGISTER = 0x05,
  M25P16_WRITE_STATUS_REGISTER = 0x01,

  M25P16_READ_DATA_BYTES = 0x03,
  M25P16_READ_DATA_BYTES_BURST = 0x0B,

  M25P16_PAGE_PROGRAM = 0x02,

  M25P16_SECTOR_ERASE = 0xD8,
  M25P16_BULK_ERASE = 0xC7,
};

void m25p16_init();
uint8_t m25p16_read_status();
void m25p16_wait_for_ready();
void m25p16_get_bounds(data_flash_bounds_t *bounds);

uint8_t m25p16_command(const uint8_t cmd);
uint8_t m25p16_read_command(const uint8_t cmd, uint8_t *data, const uint32_t len);
uint8_t m25p16_read_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len);
uint8_t m25p16_write_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len);