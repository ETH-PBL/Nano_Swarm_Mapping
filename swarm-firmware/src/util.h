/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef UTIL_H
#define  UTIL_H

#include <stdint.h>

#define MIN(a, b)   ((a) < (b) ? (a) : (b))
#define MAX(a, b)   ((a) > (b) ? (a) : (b))

#define SIZEOFARRAY(x)  (sizeof(x) / sizeof((x)[0]))

int compare_int16(const void *a, const void *b);

#endif //UTIL_H
