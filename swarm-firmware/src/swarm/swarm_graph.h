/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef SWARM_GRAPH_H
#define SWARM_GRAPH_H

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

#include "flash.h"

typedef struct scan scan_t;

#define GRAPH_POSE_MAX_EDGES    8

typedef union {
    uint16_t raw;
    struct {
        uint16_t node_id: 12;
        uint16_t drone_id: 4;
    };
} swarm_pose_id_t;
static_assert(sizeof(swarm_pose_id_t) == sizeof(uint16_t), "pose id size");

typedef struct {
    float t_x, t_y, t_yaw;
    swarm_pose_id_t to;
} swarm_edge_t;

typedef struct {
    float x, y, yaw;
    swarm_pose_id_t id;
    uint8_t num_edges;
    swarm_edge_t edges[GRAPH_POSE_MAX_EDGES];
} swarm_pose_t;

void swarm_graph_init();

bool swarm_graph_has_pose(swarm_pose_id_t id);

void swarm_graph_store_pose(const swarm_pose_t *pose);

bool swarm_graph_load_pose(swarm_pose_t *pose);

bool swarm_graph_has_scan(swarm_pose_id_t pose_id);

void swarm_graph_store_scan(const scan_t *scan);

void swarm_graph_load_scan(scan_t *scan);

void swarm_graph_debug_print();

#endif //SWARM_GRAPH_H
