/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "swarm_graph.h"

#include <stddef.h>
#include <stdbool.h>

#include "debug.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "scan.h"

#define GRAPH_MAX_POSES 48

static SemaphoreHandle_t mutex;
static StaticSemaphore_t mutex_buffer;

static size_t num_poses = 0;
static swarm_pose_t poses[GRAPH_MAX_POSES] = {0};

static size_t num_scan_files = 0;
static struct {
    flash_file_t file;
    swarm_pose_id_t id;
} scan_files[GRAPH_MAX_POSES];

void swarm_graph_init() {

    // Initialize mutex
    mutex = xSemaphoreCreateMutexStatic(&mutex_buffer);
}

bool swarm_graph_has_pose(swarm_pose_id_t id) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (int i = 0; i < num_poses; i++) {
        if (poses[i].id.raw == id.raw) {
            xSemaphoreGive(mutex);
            return true;
        }
    }
    xSemaphoreGive(mutex);
    return false;
}

void swarm_graph_store_pose(const swarm_pose_t *pose) {
    xSemaphoreTake(mutex, portMAX_DELAY);

    // Search for pose in existing poses
    bool found = false;
    for (int i = 0; i < num_poses; i++) {

        // Check if the pose IDs match
        if (poses[i].id.raw == pose->id.raw) {

            // Overwrite the pose with the updated version
            poses[i] = *pose;
            found = true;
            break;
        }
    }

    // Allocate new slot if no matching pose was found
    if (!found) {
        ASSERT(num_poses < GRAPH_MAX_POSES);
        poses[num_poses++] = *pose;
    }

    xSemaphoreGive(mutex);
}

bool swarm_graph_load_pose(swarm_pose_t *pose) {
    xSemaphoreTake(mutex, portMAX_DELAY);

    // Search for pose in existing poses
    for (int i = 0; i < num_poses; i++) {

        // Check if the pose IDs match
        if (poses[i].id.raw == pose->id.raw) {
            *pose = poses[i];
            xSemaphoreGive(mutex);
            return true;
        }
    }
    xSemaphoreGive(mutex);
    return false;
}

bool swarm_graph_has_scan(swarm_pose_id_t pose_id) {
    xSemaphoreTake(mutex, portMAX_DELAY);

    // Search for existing file
    for (int i = 0; i < num_scan_files; i++) {
        if (scan_files[i].id.raw == pose_id.raw) {
            xSemaphoreGive(mutex);
            return true;
        }
    }
    xSemaphoreGive(mutex);
    return false;
}

static flash_file_t *scan_file(swarm_pose_id_t pose_id) {

    // Search for existing file
    for (int i = 0; i < num_scan_files; ++i) {
        if (scan_files[i].id.raw == pose_id.raw) {
            return &scan_files[i].file;
        }
    }

    // Allocate new slot if no match was found
    ASSERT(num_scan_files < GRAPH_MAX_POSES);
    scan_files[num_scan_files].id = pose_id;
    return &scan_files[num_scan_files++].file;
}

void swarm_graph_store_scan(const scan_t *scan) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    flash_write(scan, sizeof(scan_t), scan_file(scan->pose));
    xSemaphoreGive(mutex);
}

void swarm_graph_load_scan(scan_t *scan) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    flash_file_t *file = scan_file(scan->pose);
    xSemaphoreGive(mutex);
    flash_read(file, scan);
}

void swarm_graph_debug_print() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (int i = 0; i < num_poses; ++i) {
        DEBUG_PRINT("----\n");
        vTaskDelay(50);
        DEBUG_PRINT("%d/%d:\n", poses[i].id.drone_id, poses[i].id.node_id);
        vTaskDelay(50);
        DEBUG_PRINT("X:%.1f Y:%.1f YAW:%.1f\n", (double) poses[i].x,
                    (double) poses[i].y, (double) poses[i].yaw);
        vTaskDelay(50);
        DEBUG_PRINT("Num edges: %d\n", poses[i].num_edges);
        vTaskDelay(50);
        for (int j = 0; j < poses[i].num_edges; ++j) {
            DEBUG_PRINT(" -> %d/%d: X:%.1f Y:%.1f YAW:%.1f\n",
                        poses[i].edges[j].to.drone_id,
                        poses[i].edges[j].to.node_id,
                        (double) poses[i].edges[j].t_x,
                        (double) poses[i].edges[j].t_y,
                        (double) poses[i].edges[j].t_yaw);
            vTaskDelay(50);
        }
    }
    DEBUG_PRINT("====\n");
    xSemaphoreGive(mutex);
}
