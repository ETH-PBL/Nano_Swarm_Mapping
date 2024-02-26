/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef SWARM_H
#define SWARM_H

#include <stdint.h>

#define SWARM_NUM_DRONES    1                       // max 14
#define SWARM_NUM_DEVICES   (SWARM_NUM_DRONES + 1)  // max 15

extern uint8_t swarm_id;

void swarm_init();

#endif //SWARM_H
