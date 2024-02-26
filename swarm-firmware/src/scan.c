/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "scan.h"

#include <stdlib.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "crtp_commander_high_level.h"

#include "tof_matrix.h"
#include "icp/icp.h"
#include "util.h"

#define DEBUG_MODULE __FILE__

#include "debug.h"

void scan_extract_points(const scan_t *scan, float x, float y, float yaw,
                         icp_points_t *points) {

    const float offsets[4] = {
            0.02f, 0.02f, 0.025f, 0.025f,
    };
    const float step = -((float) M_PI_4) / 8;

    points->num = 0;

    for (int frame = 0; frame < SCAN_NUM_FRAMES; frame++) {

        float frame_x = x + scan->frames[frame].dx;
        float frame_y = y + scan->frames[frame].dy;
        float frame_yaw = yaw + scan->frames[frame].dyaw;

        for (int dir = 0; dir < 4; dir++) {

            float dir_yaw = frame_yaw;
            if (dir == DIR_BACK) dir_yaw += (float) M_PI;
            else if (dir == DIR_LEFT) dir_yaw += (float) M_PI_2;
            else if (dir == DIR_RIGHT) dir_yaw -= (float) M_PI_2;
            float angle = -4 * step + step / 2;
            for (int col = 0; col < 8; angle += step, col++) {

                // Check if measurement is valid
                int16_t measurement = scan->frames[frame].dists[dir][col];
                if (measurement < 0) continue;

                // Apply rotation and add point
                float dist_x = (float) measurement / 1000.0f;
                float dist_y = tanf(angle) * dist_x;
                dist_x += offsets[dir];
                icp_point_t p = {
                        .x = frame_x +
                             dist_x * cosf(dir_yaw) -
                             dist_y * sinf(dir_yaw),
                        .y = frame_y +
                             dist_x * sinf(dir_yaw) +
                             dist_y * cosf(dir_yaw),
                };
                points->items[points->num++] = p;
            }
        }
    }
}

static void extract_measurements(const int16_t matrix[8][8],
                                 int16_t measurements[8]) {

    for (int col = 0; col < 8; col++) {

        // Extract center four pixels of each column discarding invalid pixels
        int16_t rows[4];
        uint8_t num_rows = 0;
        for (int i = 2; i < 6; i++) {
            if (matrix[i][col] >= 0) {
                rows[num_rows++] = matrix[i][col];
            }
        }
        if (num_rows == 0) {
            measurements[col] = -1;
            continue;
        }

        // Find median
        qsort(rows, num_rows, sizeof(int16_t), compare_int16);
        if (num_rows & 1) {
            measurements[col] = rows[num_rows >> 1];
        } else {
            measurements[col] =
                    (rows[num_rows >> 1] + rows[(num_rows >> 1) - 1]) >> 1;
        }
    }
}

void scan_acquire(scan_t *scan) {

    // Get current pose and rotate the drone in place
    float x = logGetFloat(logGetVarId("stateEstimate", "x"));
    float y = logGetFloat(logGetVarId("stateEstimate", "y"));
    float z = logGetFloat(logGetVarId("stateEstimate", "z"));
    float yaw = logGetFloat(logGetVarId("stateEstimate", "yaw"));
    crtpCommanderHighLevelGoTo(x, y, z, M_PI_4, SCAN_TIME_MS / 1000.0f, false);

    TickType_t prev_wake_time = xTaskGetTickCount();
    for (int i = 0; i < SCAN_NUM_FRAMES; i++) {

        // Store current drone pose (in relation to initial pose)
        scan->frames[i].dx = logGetFloat(logGetVarId("stateEstimate", "x")) - x;
        scan->frames[i].dy = logGetFloat(logGetVarId("stateEstimate", "y")) - y;
        scan->frames[i].dyaw =
                (logGetFloat(logGetVarId("stateEstimate", "yaw")) - yaw) /
                180.0f * (float) M_PI;

        // Read measurements from each sensor
        int16_t matrix[8][8];
        for (int dir = 0; dir < 4; ++dir) {
            if (get_tof_matrix(dir, matrix) == TOF_SUCCESS) {
                extract_measurements(matrix, scan->frames[i].dists[dir]);
            } else {
                for (int j = 0; j < 8; ++j) {
                    scan->frames[i].dists[dir][j] = -1;
                }
            }
        }

        // Continue rotating
        vTaskDelayUntil(&prev_wake_time, M2T(SCAN_TIME_MS / SCAN_NUM_FRAMES));
    }

    // Reverse rotation
    crtpCommanderHighLevelGoTo(x, y, z, 0, SCAN_TIME_MS / 1000.0f, false);

    // Save scan to flash while rotating back
    prev_wake_time = xTaskGetTickCount();
    swarm_graph_store_scan(scan);
    vTaskDelayUntil(&prev_wake_time, M2T(SCAN_TIME_MS));

}
