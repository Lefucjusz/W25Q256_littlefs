/*
 * W25Q256.c
 *
 *  Created on: Apr 4, 2024
 *      Author: lefucjusz
 */

#include "W25Q256.h"
#include <stdbool.h>
#include <errno.h>
#include "delay.h"

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define W25Q256_PAGE_TO_ADDRESS(x) ((x) * W25Q256_PAGE_SIZE)
#define W25Q256_ADDRESS_TO_PAGE(x) ((x) / W25Q256_PAGE_SIZE)
#define W25Q256_ADDRESS_TO_PAGE_OFFSET(x) ((x) % W25Q256_PAGE_SIZE)

static void W25Q256_select(void)
{
	HAL_GPIO_WritePin(W25Q256_CS_GPIO, W25Q256_CS_PIN, GPIO_PIN_RESET);
}

static void W25Q256_deselect(void)
{
	HAL_GPIO_WritePin(W25Q256_CS_GPIO, W25Q256_CS_PIN, GPIO_PIN_SET);
}

static bool W25Q256_spi_xfer(const void *tx_buffer, void *rx_buffer, size_t size)
{
	HAL_StatusTypeDef status;
	if (rx_buffer == NULL) {
		status = HAL_SPI_Transmit(&W25Q256_SPI_PORT, (uint8_t *)tx_buffer, size, W25Q256_SPI_TIMEOUT_MS);
	}
	else if (tx_buffer == NULL) {
		status = HAL_SPI_Receive(&W25Q256_SPI_PORT, (uint8_t *)rx_buffer, size, W25Q256_SPI_TIMEOUT_MS);
	}
	else {
		status = HAL_SPI_TransmitReceive(&W25Q256_SPI_PORT, (uint8_t *)tx_buffer, (uint8_t *)rx_buffer, size, W25Q256_SPI_TIMEOUT_MS);
	}
	return (status == HAL_OK);
}

static uint8_t W25Q256_read_status_reg(uint8_t reg)
{
	/* Prepare command */
	uint8_t tx_buffer[2];
	tx_buffer[0] = reg;
	tx_buffer[1] = W25Q256_DUMMY_BYTE;

	uint8_t rx_buffer[2];

	/* Send command and receive data */
	W25Q256_select();
	W25Q256_spi_xfer(tx_buffer, rx_buffer, sizeof(tx_buffer));
	W25Q256_deselect();

	return rx_buffer[1];
}

static bool W25Q256_wait(uint16_t timeout_ms)
{
	bool wait_status;
	const uint32_t start_tick = HAL_GetTick();

	while (1) {
		/* Check if timeout */
		const uint32_t current_tick = HAL_GetTick();
		if ((current_tick - start_tick) >= timeout_ms) {
			wait_status = false;
			break;
		}

		/* Check if ready */
		const uint8_t reg = W25Q256_read_status_reg(W25Q256_READ_STATUS_REG_1);
		if ((reg & W25Q256_STATUS_REG_1_BUSY_MASK) == 0) {
			wait_status = true;
			break;
		}
		delay_us(100);
	}

	return wait_status;
}

int W25Q256_get_jedec_id(W25Q256_jedec_id_t *id)
{
	/* Sanity check */
	if (id == NULL) {
		return -EINVAL;
	}

	/* Prepare command */
	uint8_t tx_buffer[4];
	tx_buffer[0] = W25Q256_READ_JEDEC_ID;
	tx_buffer[1] = W25Q256_DUMMY_BYTE;
	tx_buffer[2] = W25Q256_DUMMY_BYTE;
	tx_buffer[3] = W25Q256_DUMMY_BYTE;

	uint8_t rx_buffer[4];

	/* Send command and receive data */
	W25Q256_select();
	const bool xfer_status = W25Q256_spi_xfer(tx_buffer, rx_buffer, sizeof(tx_buffer));
	W25Q256_deselect();
	if (!xfer_status) {
		return -EIO;
	}

	id->manufacturer = rx_buffer[1];
	id->memory_type = rx_buffer[2];
	id->capacity = rx_buffer[3];
	return 0;
}

int W25Q256_read(uint32_t address, void *buffer, size_t size)
{
	/* Sanity checks */
	if (address >= W25Q256_TOTAL_SIZE) {
		return -EINVAL;
	}
	if (buffer == NULL) {
		return -EINVAL;
	}
	if (size > (W25Q256_TOTAL_SIZE - address)) {
		return -EINVAL;
	}

	/* Prepare command */
	uint8_t tx_buffer[5];
	tx_buffer[0] = W25Q256_READ_DATA_4BYTE_ADDR;
	tx_buffer[1] = (address & 0xFF000000) >> 24;
	tx_buffer[2] = (address & 0x00FF0000) >> 16;
	tx_buffer[3] = (address & 0x0000FF00) >> 8;
	tx_buffer[4] = (address & 0x000000FF) >> 0;

	W25Q256_select();
	/* Set read address */
	if (!W25Q256_spi_xfer(tx_buffer, NULL, sizeof(tx_buffer))) {
		W25Q256_deselect();
		return -EIO;
	}

	/* Read data */
	if (!W25Q256_spi_xfer(NULL, buffer, size)) {
		W25Q256_deselect();
		return -EIO;
	}
	W25Q256_deselect();

	return 0;
}

int W25Q256_write_enable(void)
{
	const uint8_t tx_value = W25Q256_ENABLE_WRITE;

	W25Q256_select();
	const bool xfer_status = W25Q256_spi_xfer(&tx_value, NULL, sizeof(tx_value));
	W25Q256_deselect();

	return xfer_status ? 0 : -EIO;
}

int W25Q256_write_disable(void)
{
	const uint8_t tx_value = W25Q256_DISABLE_WRITE;

	W25Q256_select();
	const bool xfer_status = W25Q256_spi_xfer(&tx_value, NULL, sizeof(tx_value));
	W25Q256_deselect();

	return xfer_status ? 0 : -EIO;
}

int W25Q256_write_page(uint32_t page, uint8_t offset, const void *buffer, size_t size)
{
	/* Sanity checks */
	if (buffer == NULL) {
		return -EINVAL;
	}
	if (page >= W25Q256_PAGE_COUNT) {
		return -EINVAL;
	}
	if (offset >= W25Q256_PAGE_SIZE) {
		return -EINVAL;
	}
	if (size > (W25Q256_PAGE_SIZE - offset)) {
		return -EINVAL;
	}

	/* Enable writing */
	const int status = W25Q256_write_enable();
	if (status != 0) {
		return status;
	}

	/* Prepare command */
	const uint32_t address = W25Q256_PAGE_TO_ADDRESS(page) + offset;
	uint8_t tx_buffer[5];
	tx_buffer[0] = W25Q256_PROGRAM_PAGE_4BYTE_ADDR;
	tx_buffer[1] = (address & 0xFF000000) >> 24;
	tx_buffer[2] = (address & 0x00FF0000) >> 16;
	tx_buffer[3] = (address & 0x0000FF00) >> 8;
	tx_buffer[4] = (address & 0x000000FF) >> 0;

	W25Q256_select();
	/* Set write address */
	if (!W25Q256_spi_xfer(tx_buffer, NULL, sizeof(tx_buffer))) {
		W25Q256_deselect();
		return -EIO;
	}

	/* Write data */
	if (!W25Q256_spi_xfer(buffer, NULL, size)) {
		W25Q256_deselect();
		return -EIO;
	}
	W25Q256_deselect();

	/* Wait until data written */
	if (!W25Q256_wait(W25Q256_PAGE_PROGRAM_TIMEOUT_MS)) {
		return -ETIME;
	}
	return 0;
}

int W25Q256_write(uint32_t address, const void *buffer, size_t size)
{
	const uint8_t *buffer_ptr = buffer;
	uint32_t bytes_written = 0;
	int status = 0;

	while (bytes_written < size) {
		const uint32_t page = W25Q256_ADDRESS_TO_PAGE(address + bytes_written);
		const uint8_t offset = W25Q256_ADDRESS_TO_PAGE_OFFSET(address + bytes_written);
		const size_t bytes_to_write = MIN(size - bytes_written, W25Q256_PAGE_SIZE - offset);

		status = W25Q256_write_page(page, offset, &buffer_ptr[bytes_written], bytes_to_write);
		if (status != 0) {
			break;
		}

		bytes_written += bytes_to_write;
	}

	return status;
}

int W25Q256_erase_sector(uint16_t sector)
{
	/* Enable writing */
	const int status = W25Q256_write_enable();
	if (status != 0) {
		return status;
	}

	/* Prepare command */
	const uint32_t address = sector * W25Q256_SECTOR_SIZE;
	uint8_t tx_buffer[5];
	tx_buffer[0] = W25Q256_ERASE_SECTOR_4BYTE_ADDR;
	tx_buffer[1] = (address & 0xFF000000) >> 24;
	tx_buffer[2] = (address & 0x00FF0000) >> 16;
	tx_buffer[3] = (address & 0x0000FF00) >> 8;
	tx_buffer[4] = (address & 0x000000FF) >> 0;

	/* Send command */
	W25Q256_select();
	const bool tx_status = W25Q256_spi_xfer(tx_buffer, NULL, sizeof(tx_buffer));
	W25Q256_deselect();
	if (!tx_status) {
		return -EIO;
	}

	/* Wait until sector erased */
	if (!W25Q256_wait(W25Q256_SECTOR_ERASE_TIMEOUT_MS)) {
		return -ETIME;
	}

	return 0;
}
