/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "estimator_kalman.h"
#include "crtp_commander_high_level.h"
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "static_mem.h"

#include "swarm.h"
#include "swarm_graph.h"
#include "swarm_comm.h"
#include "swarm_radio.h"
#include "tof_matrix.h"
#include "exploration.h"
#include "slam.h"

static SemaphoreHandle_t start_semaphore;
static StaticSemaphore_t start_semaphore_buf;
static SemaphoreHandle_t done_semaphore;
static StaticSemaphore_t done_semaphore_buf;

STATIC_MEM_TASK_ALLOC(mission_task, 2560);

static void mission_task(void *parameters);

void appMain() {

    start_semaphore = xSemaphoreCreateBinaryStatic(&start_semaphore_buf);
    done_semaphore = xSemaphoreCreateBinaryStatic(&done_semaphore_buf);

    // Initialization
    flash_init();
    swarm_init();
    swarm_graph_init();
    swarm_radio_init();
    swarm_comm_init();
    tof_matrix_init();

    // Start mission task
    STATIC_MEM_TASK_CREATE(mission_task, mission_task, "mission", NULL, 2);

    if (swarm_id == 0) {

        // Wait for other drone to complete flight
        xSemaphoreTake(done_semaphore, portMAX_DELAY);

        vTaskDelay(M2T(2000));

        // Run SLAM
        DEBUG_PRINT("Run SLAM...\n");
        run_slam();

    }

    while (true) vTaskDelay(M2T(10000));

}

static void mission_task(void *parameters) {

    DEBUG_PRINT("Waiting for start command...\n");
    xSemaphoreTake(start_semaphore, portMAX_DELAY);
    DEBUG_PRINT("Start\n");

    // Start mission
    estimatorKalmanInitToPose(-1.0f, -1.0f, 0, 0);
    vTaskDelay(M2T(1000));
    crtpCommanderHighLevelTakeoffWithVelocity(EXP_HEIGHT, 0.2f, false);
    vTaskDelay(4000);

    // Begin autonomous exploration
    direction_t dir_priority[4] = {
            DIR_LEFT, DIR_BACK, DIR_RIGHT, DIR_FRONT,
    };
    explore(DIR_FRONT, dir_priority, 0.8f, 25);

    DEBUG_PRINT("Land\n");

    // Land the drone
    crtpCommanderHighLevelLand(0.0f, 4.0f);
    vTaskDelay(4000);

    // Broadcast to the swarm that the mission is complete
    swarm_comm_ctrl_msg_t ctrl_msg = {
            .type = CTRL_MSG_TYPE_DONE,
    };
    swarm_comm_broadcast_ctrl_msg(ctrl_msg);
    xSemaphoreGive(done_semaphore);

    //swarm_graph_debug_print();

    while (1) vTaskDelay(1000);

}

void swarm_comm_handle_ctrl_msg(uint8_t src, swarm_comm_ctrl_msg_t msg) {
    if (msg.type == CTRL_MSG_TYPE_START) {
        xSemaphoreGive(start_semaphore);
    }
    if (src == SWARM_NUM_DRONES - 1 && msg.type == CTRL_MSG_TYPE_DONE) {
        xSemaphoreGive(done_semaphore);
    }
}
