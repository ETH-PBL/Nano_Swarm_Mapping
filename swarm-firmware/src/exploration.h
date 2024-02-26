/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <stddef.h>

#include "scan.h"

#define EXP_HEIGHT              0.6f    // m
#define EXP_ACCELERATION        0.5f    // m/s/s
#define EXP_LAT_VELOCITY        2.0f    // m/s/m
#define EXP_WALL_DIST           0.5f    // m
#define EXP_SLOW_DOWN_DIST      0.75f   // m
#define EXP_WAYPOINT_INTERVAL   1.0f    // m
#define EXP_WAYPOINT_TOL        0.75f   // m

void explore(direction_t initial_dir, const direction_t dir_priority[4],
             float velocity, size_t max_poses);

#endif //EXPLORATION_H
