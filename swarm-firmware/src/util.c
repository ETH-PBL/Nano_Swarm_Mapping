/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "util.h"

int compare_int16(const void *a, const void *b) {
    return (*(int16_t *) a - *(int16_t *) b);
}