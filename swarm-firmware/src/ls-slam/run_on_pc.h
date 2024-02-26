/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Vlad Niculescu
 */

#ifndef DATATYPES_H
#define DATATYPES_H


#ifdef OFFDRONE

// use printf when not on the drone
#define DEBUG_PRINT printf

typedef signed short int int16_t;
typedef int int32_t;

typedef long int TickType_t;
TickType_t xTaskGetTickCount();

#endif

#endif