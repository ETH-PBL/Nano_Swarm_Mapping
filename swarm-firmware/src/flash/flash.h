/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t addr;
    uint32_t len;
} flash_file_t;


void flash_init();

int flash_write(const void *data, size_t len, flash_file_t *file);

int flash_read(const flash_file_t *file, void *data);

#endif //SPI_FLASH_H
