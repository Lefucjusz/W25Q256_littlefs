/*
 * W25Q256.h
 *
 *  Created on: Apr 4, 2024
 *      Author: lefucjusz
 */

#pragma once

#include "stm32f4xx_hal.h"

/* Typedefs */
typedef struct
{
	uint8_t manufacturer;
	uint8_t memory_type;
	uint8_t capacity;
} W25Q256_jedec_id_t;

/* Memory opcodes */
#define W25Q256_ENABLE_WRITE 0x06
#define W25Q256_DISABLE_WRITE 0x04
#define W25Q256_READ_STATUS_REG_1 0x05
#define W25Q256_READ_STATUS_REG_2 0x35
#define W25Q256_READ_STATUS_REG_3 0x15
#define W25Q256_PROGRAM_PAGE_4BYTE_ADDR 0x12
#define W25Q256_READ_DATA_4BYTE_ADDR 0x13
#define W25Q256_ERASE_SECTOR_4BYTE_ADDR 0x21
#define W25Q256_READ_JEDEC_ID 0x9F

/* Status registers bits */
#define W25Q256_STATUS_REG_1_BUSY_MASK (1 << 0)

/* Timeouts */
#define W25Q256_PAGE_PROGRAM_TIMEOUT_MS 100
#define W25Q256_SECTOR_ERASE_TIMEOUT_MS 1000

/* Constants */
#define W25Q256_BITS_PER_BYTE 8UL
#define W25Q256_BYTES_PER_MEBIBYTE (1024UL * 1024UL)
#define W25Q256_TOTAL_SIZE ((256UL / W25Q256_BITS_PER_BYTE) * W25Q256_BYTES_PER_MEBIBYTE)

#define W25Q256_PAGE_SIZE 256UL
#define W25Q256_PAGE_COUNT (W25Q256_TOTAL_SIZE / W25Q256_PAGE_SIZE)

#define W25Q256_SECTOR_SIZE 4096UL
#define W25Q256_SECTORS_COUNT (W25Q256_TOTAL_SIZE / W25Q256_SECTOR_SIZE)

#define W25Q256_DUMMY_BYTE 0xFF

/* Configuration */
#define W25Q256_SPI_PORT hspi1
#define W25Q256_SPI_TIMEOUT_MS 1000

#define W25Q256_CS_GPIO GPIOB
#define W25Q256_CS_PIN GPIO_PIN_14

/* Variables */
extern SPI_HandleTypeDef W25Q256_SPI_PORT;

/* Functions */
int W25Q256_get_jedec_id(W25Q256_jedec_id_t *id);

int W25Q256_read(uint32_t address, void *buffer, size_t size);

int W25Q256_write_enable(void);
int W25Q256_write_disable(void);
int W25Q256_write_page(uint32_t page, uint8_t offset, const void *buffer, size_t size);
int W25Q256_write(uint32_t address, const void *buffer, size_t size);

int W25Q256_erase_sector(uint16_t sector);
