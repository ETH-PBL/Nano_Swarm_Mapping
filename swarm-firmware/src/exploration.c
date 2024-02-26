/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "exploration.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "estimator_kalman.h"
#include "commander.h"
#include "log.h"
#include "debug.h"

#include "tof_matrix.h"
#include "scan.h"
#include "swarm.h"
#include "swarm_radio.h"
#include "swarm_comm.h"
#include "swarm_graph.h"
#include "util.h"

static float get_dist(int16_t tof_raw[8][8]) {

    // Select center pixels
    size_t num = 0;
    int16_t d[4];
    for (int i = 3; i <= 4; ++i) {
        for (int j = 3; j <= 4; ++j) {
            if (tof_raw[i][j] >= 0) {
                d[num++] = tof_raw[i][j];
            }
        }
    }
    if (num == 0) {
        return INFINITY;
    }

    // Find median distance
    qsort(d, num, sizeof(int16_t), compare_int16);
    float median;
    if (num & 1) {
        median = d[num >> 1];
    } else {
        median = (float) (d[num >> 1] + d[(num >> 1) - 1]);
        median *= 0.5f;
    }

    return median / 1000.0f;
}

static void pri_vel(float d, float target, float *out) {
    if (d <= EXP_SLOW_DOWN_DIST) {
        target = (d / EXP_SLOW_DOWN_DIST + 0.1f) * target;
    }
    float limit_p = *out + EXP_ACCELERATION * 0.067f;
    float limit_n = *out - EXP_ACCELERATION * 0.067f;
    *out = MAX(limit_n, MIN(limit_p, target));
}

static float sec_vel(float d1, float d2) {

    // Stabilise horizontally (in the center or fixed distance from the wall)
    float width = d1 + d2;
    if (width < 3 * EXP_WALL_DIST) {
        return (d1 - 0.5f * width) * EXP_LAT_VELOCITY;
    } else if (d2 < 2 * EXP_WALL_DIST) {
        return (EXP_WALL_DIST - d2) * EXP_LAT_VELOCITY;
    } else if (d1 < 2 * EXP_WALL_DIST) {
        return (d1 - EXP_WALL_DIST) * EXP_LAT_VELOCITY;
    } else {
        return 0;
    }

}


static void add_pose(uint16_t id, float out[3]) {

    DEBUG_PRINT("%d/%d\n", swarm_id, id);

    // Initialize new pose
    swarm_pose_t pose = {
            .id = {
                    .drone_id = swarm_id,
                    .node_id = id,
            },
            .x = logGetFloat(logGetVarId("stateEstimate", "x")),
            .y = logGetFloat(logGetVarId("stateEstimate", "y")),
            .yaw = logGetFloat(logGetVarId("stateEstimate", "yaw")) / 180.0f *
                   (float) M_PI,
            .num_edges = 0,
    };
    out[0] = pose.x;
    out[1] = pose.y;
    out[2] = pose.yaw;

    // Register new pose with bridge
    swarm_comm_ctrl_msg_t ctrl_msg = {
            .type = CTRL_MSG_TYPE_POSE,
            .pose = pose.id,
    };
    swarm_comm_send_ctrl_msg(RADIO_ADR_BRIDGE, ctrl_msg);

    // Store pose in graph
    swarm_graph_store_pose(&pose);

    // Acquire scan
    scan_t scan = {
            .pose = pose.id,
    };
    scan_acquire(&scan);

    // Broadcast pose to swarm (async)
    swarm_comm_broadcast_pose(pose.id);

    // Send scan to bridge (async)
    swarm_comm_send_scan(RADIO_ADR_BRIDGE, pose.id);

}

void explore(direction_t initial_dir, const direction_t dir_priority[4],
             float velocity, size_t max_poses) {

    TickType_t prevWakeTime = xTaskGetTickCount();
    direction_t cur_dir = initial_dir;
    uint16_t pose_id = 0;
    float last_pose[3] = {0};

    setpoint_t setpoint = {
            .mode = {
                    .x = modeVelocity,
                    .y = modeVelocity,
                    .z = modeAbs,
                    .yaw = modeAbs,
            },
            .position = {
                    .x = 0,
                    .y = 0,
                    .z = EXP_HEIGHT,
            },
            .attitude = {
                    .yaw = 0,
            },
    };

    // Initial pose
    add_pose(pose_id++, last_pose);
    max_poses--;

    while (true) {

        // Get current position
        point_t pos = {0};
        estimatorKalmanGetEstimatedPos(&pos);

        // Fetch distances from ToF sensors
        float dists[4];
        int16_t matrix[8][8];
        for (int i = 0; i < 4; i++) {
            while (get_tof_matrix(i, matrix) != TOF_SUCCESS) {
                vTaskDelay(M2T(10));
            }
            dists[i] = get_dist(matrix);
        }
        dists[DIR_FRONT] += 0.02f;
        dists[DIR_BACK] += 0.02f;
        dists[DIR_LEFT] += 0.025f;
        dists[DIR_RIGHT] += 0.025f;

        // Calculate possible distances to next pose
        float dist_to_corner = dists[cur_dir] - EXP_WALL_DIST;
        float dist_to_waypoint;
        {
            float diff_x = pos.x - last_pose[0];
            float diff_y = pos.y - last_pose[1];
            float since_last_pose = sqrtf(diff_x * diff_x + diff_y * diff_y);
            dist_to_waypoint = EXP_WAYPOINT_INTERVAL - since_last_pose;
        }
        bool skip_waypoint =
                dist_to_corner - dist_to_waypoint < EXP_WAYPOINT_TOL;
        float dist_to_pose = skip_waypoint
                             ? dist_to_corner
                             : MIN(dist_to_corner, dist_to_waypoint);

        // Check if we are getting too close to a corner
        if (dist_to_corner <= 0) {

            // Check for dead end
            switch (cur_dir) {
                case DIR_FRONT:
                case DIR_BACK:
                    if (dists[DIR_LEFT] < 2 * EXP_WALL_DIST &&
                        dists[DIR_RIGHT] < 2 * EXP_WALL_DIST) {
                        return;
                    }
                    break;
                case DIR_RIGHT:
                case DIR_LEFT:
                    if (dists[DIR_FRONT] < 2 * EXP_WALL_DIST &&
                        dists[DIR_BACK] < 2 * EXP_WALL_DIST) {
                        return;
                    }
                    break;
            }

            // Find a new direction to fly in
            for (int i = 0; i < 4; ++i) {
                direction_t next_dir = dir_priority[i];
                switch (next_dir) {
                    case DIR_FRONT:
                    case DIR_BACK:
                        if (cur_dir == DIR_FRONT || cur_dir == DIR_BACK) {
                            continue;
                        }
                        break;
                    case DIR_LEFT:
                    case DIR_RIGHT:
                        if (cur_dir == DIR_LEFT || cur_dir == DIR_RIGHT) {
                            continue;
                        }
                        break;
                }
                if (dists[next_dir] >= 2 * EXP_WALL_DIST) {
                    cur_dir = next_dir;
                    break;
                }
            }

            // Reset velocity
            setpoint.velocity.x = setpoint.velocity.y = 0;

        }

        // Check if we are getting too close to the next pose location
        if (dist_to_pose <= 0) {

            // Stop the drone
            setpoint.velocity.x = setpoint.velocity.y = 0;
            commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
            vTaskDelay(M2T(500));

            // Add the pose
            add_pose(pose_id++, last_pose);
            if (!--max_poses) return;

        }

        // Calculate the next setpoint
        switch (cur_dir) {
            case DIR_FRONT:
                pri_vel(dist_to_pose, velocity, &setpoint.velocity.x);
                setpoint.velocity.y = sec_vel(dists[DIR_LEFT],
                                              dists[DIR_RIGHT]);
                break;
            case DIR_BACK:
                pri_vel(dist_to_pose, -velocity, &setpoint.velocity.x);
                setpoint.velocity.y = sec_vel(dists[DIR_LEFT],
                                              dists[DIR_RIGHT]);
                break;
            case DIR_LEFT:
                pri_vel(dist_to_pose, velocity, &setpoint.velocity.y);
                setpoint.velocity.x = sec_vel(dists[DIR_FRONT],
                                              dists[DIR_BACK]);
                break;
            case DIR_RIGHT:
                pri_vel(dist_to_pose, -velocity, &setpoint.velocity.y);
                setpoint.velocity.x = sec_vel(dists[DIR_FRONT],
                                              dists[DIR_BACK]);
                break;
        }
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);

        // Delay until new ToF data is available
        vTaskDelayUntil(&prevWakeTime, M2T(67));
    }
}
