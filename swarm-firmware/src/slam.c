/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "slam.h"

#include <stdint.h>

#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"

#include "scan.h"
#include "icp.h"
#include "graph-based-slam.h"
#include "swarm.h"
#include "swarm_graph.h"
#include "swarm_comm.h"

static constraint comp_constr(icp_points_t *ref_scan, icp_points_t *cur_scan,
                              float *ref_pose, float *cur_pose,
                              int16_t ref_pose_idx, int16_t cur_pose_idx) {

    // Execute ICP
    icp_point_t icp_tr;
    float icp_rot;
    DEBUG_PRINT("Starting ICP (sizes: %d %d)\n", ref_scan->num, cur_scan->num);
    icp(ref_scan, cur_scan, 10, 0.8f, &icp_tr, &icp_rot);
    DEBUG_PRINT("ICP T:%.2f %.2f R:%.2f\n",
                (double) icp_tr.x,
                (double) icp_tr.y,
                (double) icp_rot);

    // Prepare SLAM constraint
    float c = cosf(icp_rot);
    float s = sinf(icp_rot);
    float x = cur_pose[0] * c - cur_pose[1] * s + icp_tr.x;
    float y = cur_pose[0] * s + cur_pose[1] * c + icp_tr.y;
    float corrected_pose[3] = {x, y, cur_pose[2] + icp_rot};
    constraint constraint = {
            .fromIdx = cur_pose_idx,
            .toIdx = ref_pose_idx,
    };
    measurement_from_poses(corrected_pose, ref_pose, constraint.z);

    DEBUG_PRINT("Constraint: %d -> %d: %.2f %.2f %.2f\n", cur_pose_idx,
                ref_pose_idx,
                (double) constraint.z[0],
                (double) constraint.z[1],
                (double) constraint.z[2]);
    vTaskDelay(M2T(50));

    return constraint;
}

#define TOTAL_NUM_POSES (25)
#define NUM_CONSTRAINTS 9

void run_slam() {

    const int16_t num_poses[SWARM_NUM_DRONES] = {25};
    float poses[TOTAL_NUM_POSES][3];
    size_t poses_idx;

    uint8_t skip[TOTAL_NUM_POSES] = {0};

    constraint constraints[NUM_CONSTRAINTS];

    const swarm_pose_id_t cor_ids[NUM_CONSTRAINTS][2] = {
            {
                    {.drone_id = 0, .node_id = 0},
                    {.drone_id = 0, .node_id = 8},
            },
            {
                    {.drone_id = 0, .node_id = 2},
                    {.drone_id = 0, .node_id = 10},
            },
            {
                    {.drone_id = 0, .node_id = 4},
                    {.drone_id = 0, .node_id = 12},
            },
            {
                    {.drone_id = 0, .node_id = 6},
                    {.drone_id = 0, .node_id = 14},
            },
            {
                    {.drone_id = 0, .node_id = 0},
                    {.drone_id = 0, .node_id = 16},
            },
            {
                    {.drone_id = 0, .node_id = 2},
                    {.drone_id = 0, .node_id = 18},
            },
            {
                    {.drone_id = 0, .node_id = 4},
                    {.drone_id = 0, .node_id = 20},
            },
            {
                    {.drone_id = 0, .node_id = 6},
                    {.drone_id = 0, .node_id = 22},
            },
            {
                    {.drone_id = 0, .node_id = 0},
                    {.drone_id = 0, .node_id = 24},
            },
    };

    // Write pose data
    swarm_pose_t pose;
    poses_idx = 0;
    for (int j = 0; j < SWARM_NUM_DRONES; j++) {
        for (int i = 0; i < num_poses[j]; i++) {
            pose.id.drone_id = j;
            pose.id.node_id = i;
            swarm_graph_load_pose(&pose);
            poses[poses_idx][0] = pose.x;
            poses[poses_idx][1] = pose.y;
            poses[poses_idx][2] = pose.yaw;
            poses_idx++;
        }
    }

    {

        // Allocate memory for the scans
        scan_t ref_scan;
        scan_t cur_scan;
        POINTS_STATIC_ALLOC(ref_cloud, POINTS_IN_SCAN)
        POINTS_STATIC_ALLOC(cur_cloud, POINTS_IN_SCAN)

        // Calculate constraints at intersections
        for (size_t i = 0; i < NUM_CONSTRAINTS; i++) {

            swarm_pose_id_t ref_id = cor_ids[i][0];
            swarm_pose_id_t cur_id = cor_ids[i][1];
            int16_t ref_idx = ref_id.node_id;
            for (int j = 0; j < ref_id.drone_id; j++) ref_idx += num_poses[j];
            int16_t cur_idx = cur_id.node_id;
            for (int j = 0; j < cur_id.drone_id; j++) cur_idx += num_poses[j];

            // Fetch scans
            if (ref_id.drone_id != swarm_id) {
                DEBUG_PRINT("Fetching %d/%d...\n", ref_id.drone_id,
                            ref_id.node_id);
                swarm_comm_fetch_scan(ref_id.drone_id, ref_id);
            }
            if (cur_id.drone_id != swarm_id) {
                DEBUG_PRINT("Fetching %d/%d...\n", cur_id.drone_id,
                            cur_id.node_id);
                swarm_comm_fetch_scan(cur_id.drone_id, cur_id);
            }

            // Load scans
            ref_scan.pose = ref_id;
            cur_scan.pose = cur_id;
            vTaskDelay(M2T(10));
            swarm_graph_load_scan(&ref_scan);
            vTaskDelay(M2T(10));
            swarm_graph_load_scan(&cur_scan);
            vTaskDelay(M2T(10));
            scan_extract_points(&ref_scan,
                                poses[ref_idx][0],
                                poses[ref_idx][1],
                                poses[ref_idx][2],
                                ref_cloud);
            vTaskDelay(M2T(10));
            scan_extract_points(&cur_scan,
                                poses[cur_idx][0],
                                poses[cur_idx][1],
                                poses[cur_idx][2],
                                cur_cloud);
            vTaskDelay(M2T(10));

            // Compute constraint
            constraints[i] = comp_constr(ref_cloud, cur_cloud,
                                         poses[ref_idx], poses[cur_idx],
                                         ref_idx, cur_idx);
            vTaskDelay(M2T(10));

        }

    }

    poses_idx = 0;
    for (int j = 0; j < SWARM_NUM_DRONES; j++) {
        for (int i = 0; i < num_poses[j]; i++) {
            DEBUG_PRINT("%d/%d: %.3f %.3f %.3f\n", j, i,
                        (double) poses[poses_idx][0],
                        (double) poses[poses_idx][1],
                        (double) poses[poses_idx][2]);
            vTaskDelay(M2T(20));
            poses_idx++;
        }
    }
    DEBUG_PRINT("====\n");

    ls_slam((float *) poses, skip, constraints, TOTAL_NUM_POSES,
            NUM_CONSTRAINTS);

    poses_idx = 0;
    for (int j = 0; j < SWARM_NUM_DRONES; j++) {
        for (int i = 0; i < num_poses[j]; i++) {
            DEBUG_PRINT("%d/%d: %.3f %.3f %.3f\n", j, i,
                        (double) poses[poses_idx][0],
                        (double) poses[poses_idx][1],
                        (double) poses[poses_idx][2]);
            vTaskDelay(M2T(20));
            poses_idx++;
        }
    }

    // Store updates to graph and broadcast
    poses_idx = 0;
    for (int j = 0; j < SWARM_NUM_DRONES; j++) {
        for (int i = 0; i < num_poses[j]; i++) {
            pose.id.drone_id = j;
            pose.id.node_id = i;
            swarm_graph_load_pose(&pose);
            pose.x = poses[poses_idx][0];
            pose.y = poses[poses_idx][1];
            pose.yaw = poses[poses_idx][2];
            swarm_graph_store_pose(&pose);
            swarm_comm_broadcast_pose(pose.id);
            vTaskDelay(M2T(50));
            poses_idx++;
        }
    }
}
