/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef SWARM_COMM_H
#define SWARM_COMM_H

#include <stdint.h>
#include <stddef.h>

#include "swarm_graph.h"

#define COMM_MSG_TAG_POSE       0
#define COMM_MSG_TAG_SCAN_REQ   1
#define COMM_MSG_TAG_SCAN_RES   2
#define COMM_MSG_TAG_CTRL       3

typedef struct {
    enum {
        CTRL_MSG_TYPE_START = 0,
        CTRL_MSG_TYPE_DONE,
        CTRL_MSG_TYPE_POSE,
    } type;
    swarm_pose_id_t pose;
} swarm_comm_ctrl_msg_t;

void swarm_comm_init();

void swarm_comm_broadcast_pose(swarm_pose_id_t id);

void swarm_comm_fetch_scan(uint8_t addr, swarm_pose_id_t id);

void swarm_comm_send_scan(uint8_t addr, swarm_pose_id_t id);

void swarm_comm_send_ctrl_msg(uint8_t addr, swarm_comm_ctrl_msg_t msg);

void swarm_comm_broadcast_ctrl_msg(swarm_comm_ctrl_msg_t msg);

void swarm_comm_handle_ctrl_msg(uint8_t src, swarm_comm_ctrl_msg_t msg);

#endif //SWARM_COMM_H
