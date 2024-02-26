/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "flash.h"

#include "../../../spiflash_driver/src/spiflash.h"
#include "stm32fxxx.h"
#include "cfassert.h"
#include "deck.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define INITIAL_ERASE_SIZE  0x40000

static uint32_t current_base_addr;
static uint8_t dummy[256];

static spiflash_t spif;

static SemaphoreHandle_t mutex;
static StaticSemaphore_t mutexBuffer;

static int impl_spiflash_spi_txrx(spiflash_t *spi, const uint8_t *tx_data,
                                  uint32_t tx_len, uint8_t *rx_data,
                                  uint32_t rx_len) {
    ASSERT(tx_len <= 256);

    int res = SPIFLASH_OK;
    if (tx_len > 0) {
        //DEBUG_PRINT("[W] (%lu) %x\n", tx_len, tx_data[0]);
        res |= !spiExchange(tx_len, tx_data, dummy);
    }

    if (res == SPIFLASH_OK && rx_len > 0) {
        res |= !spiExchange(rx_len, rx_data, rx_data);
        //DEBUG_PRINT("[R] (%lu) %x\n", rx_len, rx_data[0]);
    }

    return res;
}

static void impl_spiflash_spi_cs(spiflash_t *spi, uint8_t cs) {
    if (cs) {
        //DEBUG_PRINT("[C] A\n");
        spiBeginTransaction(SPI_BAUDRATE_21MHZ);
        digitalWrite(DECK_GPIO_IO4, LOW);
    } else {
        //DEBUG_PRINT("[C] D\n");
        digitalWrite(DECK_GPIO_IO4, HIGH);
        spiEndTransaction();
    }
}

void impl_spiflash_wait(spiflash_t *spi, uint32_t ms) {
    vTaskDelay(M2T(ms));
}

const spiflash_hal_t spiflash_hal = {
        ._spiflash_spi_txrx = impl_spiflash_spi_txrx,
        ._spiflash_spi_cs = impl_spiflash_spi_cs,
        ._spiflash_wait = impl_spiflash_wait,
};

const spiflash_cmd_tbl_t spiflash_cmds = {
        .write_disable = 0x04,
        .write_enable = 0x06,
        .page_program = 0x02,
        .read_data = 0x03,
        .read_data_fast = 0x0b,
        .write_sr = 0x01,
        .read_sr = 0x05,
        .block_erase_4 = 0x20,
        .block_erase_8 = 0x00,  // not supported
        .block_erase_16 = 0x00, // not supported
        .block_erase_32 = 0x00, // not supported
        .block_erase_64 = 0xd8,
        .chip_erase = 0xc7,
        .device_id = 0x00,      // not supported
        .jedec_id = 0x9f,
        .sr_busy_bit = 0x01,
};

const spiflash_config_t spiflash_config = {
        .sz = 1024 * 1024 * 4,  // 4 MB flash
        .page_sz = 256,         // 256 byte pages
        .addr_sz = 3,           // 3 byte addressing
        .addr_dummy_sz = 0,     // using single line data
        .addr_endian = SPIFLASH_ENDIANNESS_BIG,
        .sr_write_ms = 2,
        .page_program_ms = 1,
        .block_erase_4_ms = 250,
        .block_erase_8_ms = 0,  // not supported
        .block_erase_16_ms = 0, // not supported
        .block_erase_32_ms = 0, // not supported
        .block_erase_64_ms = 600,
        .chip_erase_ms = 30000,
};

void flash_init() {

    int res;

    // Initialize mutex
    mutex = xSemaphoreCreateMutexStatic(&mutexBuffer);

    // Configure hardware
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    spiBegin();

    // Initialize driver
    SPIFLASH_init(&spif,
                  &spiflash_config,
                  &spiflash_cmds,
                  &spiflash_hal,
                  NULL,
                  SPIFLASH_SYNCHRONOUS,
                  NULL);

    // Read SPI Flash JEDEC code
    uint8_t jedec[4];
    res = SPIFLASH_read_jedec_id(&spif, (uint32_t *) jedec);
    ASSERT(res == SPIFLASH_OK);
    DEBUG_PRINT("Flash JEDEC: %x %x %x\n", jedec[0], jedec[1], jedec[2]);

    // Create some space to store data
    res = SPIFLASH_erase(&spif, 0, INITIAL_ERASE_SIZE);
    current_base_addr = 0;
    ASSERT(res == SPIFLASH_OK);

}

int flash_write(const void *data, size_t len, flash_file_t *file) {

    // Check that enough sectors have been erased
    ASSERT(current_base_addr + len <= INITIAL_ERASE_SIZE);

    // Write meta data
    file->addr = current_base_addr;
    file->len = len;

    // Increment base address by size of data
    current_base_addr += len;

    // Write the data to flash
    xSemaphoreTake(mutex, portMAX_DELAY);
    int res = SPIFLASH_write(&spif, file->addr, file->len, data);
    xSemaphoreGive(mutex);
    return res;
}

int flash_read(const flash_file_t *file, void *data) {

    // Read the data from flash
    xSemaphoreTake(mutex, portMAX_DELAY);
    int res = SPIFLASH_read(&spif, file->addr, file->len, data);
    xSemaphoreGive(mutex);
    return res;

}
