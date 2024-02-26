/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef SCAN_H
#define SCAN_H

#include <stdint.h>
#include <stdbool.h>

#include "swarm_graph.h"
#include "icp/icp.h"

typedef enum {
    DIR_FRONT = 0,
    DIR_BACK = 1,
    DIR_LEFT = 2,
    DIR_RIGHT = 3,
} direction_t;

#define SCAN_NUM_FRAMES 15
#define POINTS_IN_SCAN  (SCAN_NUM_FRAMES * 4 * 8)
#define SCAN_TIME_MS    2000

typedef struct {
    int16_t dists[4][8];
    float dx, dy, dyaw;
} scan_frame_t;

typedef struct scan {
    swarm_pose_id_t pose;
    scan_frame_t frames[SCAN_NUM_FRAMES];
} scan_t;

void scan_extract_points(const scan_t *scan, float x, float y, float yaw,
                         icp_points_t *points);

void scan_acquire(scan_t *scan);

#endif //SCAN_H
