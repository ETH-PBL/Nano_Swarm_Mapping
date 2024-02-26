/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "swarm_comm.h"

#include "cfassert.h"
#include "radiolink.h"
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "static_mem.h"

#include "swarm.h"
#include "swarm_graph.h"
#include "swarm_radio.h"
#include "scan.h"

typedef struct {
    enum {
        COMM_TX_QUEUE_ITEM_POSE,
        COMM_TX_QUEUE_ITEM_SCAN_REQ,
        COMM_TX_QUEUE_ITEM_SCAN_RES,
        COMM_TX_QUEUE_ITEM_CTRL,
    } type;
    uint8_t addr;
    union {
        swarm_pose_id_t pose_id;
        swarm_comm_ctrl_msg_t ctrl_msg;
    };
} tx_queue_item_t;

static QueueHandle_t tx_queue;
static StaticQueue_t tx_queue_static;
static uint8_t tx_queue_buf[8 * sizeof(tx_queue_item_t)];
static SemaphoreHandle_t scan_mutex;
static StaticSemaphore_t scan_mutex_buffer;
static SemaphoreHandle_t scan_semaphore;
static StaticSemaphore_t scan_semaphore_buf;
static swarm_pose_id_t scan_req_id;

static void swarm_comm_tx_task(void *parameters);

STATIC_MEM_TASK_ALLOC(tx_task, 1536);

void swarm_comm_init() {

    // Create synchronisation primitives
    scan_mutex = xSemaphoreCreateMutexStatic(&scan_mutex_buffer);
    scan_semaphore = xSemaphoreCreateBinaryStatic(&scan_semaphore_buf);

    // Set up TX
    tx_queue = xQueueCreateStatic(
            sizeof(tx_queue_buf) / sizeof(tx_queue_item_t),
            sizeof(tx_queue_item_t), tx_queue_buf, &tx_queue_static);
    STATIC_MEM_TASK_CREATE(tx_task, swarm_comm_tx_task, "swarm_tx", NULL, 0);

}

void swarm_comm_broadcast_pose(swarm_pose_id_t id) {
    tx_queue_item_t item = {
            .type = COMM_TX_QUEUE_ITEM_POSE,
            .pose_id = id,
    };
    xQueueSend(tx_queue, &item, portMAX_DELAY);
}

void swarm_comm_fetch_scan(uint8_t addr, swarm_pose_id_t id) {
    scan_req_id = id;
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    tx_queue_item_t item = {
            .type = COMM_TX_QUEUE_ITEM_SCAN_REQ,
            .addr = addr,
            .pose_id = id,
    };
    xQueueSend(tx_queue, &item, portMAX_DELAY);
    xSemaphoreTake(scan_semaphore, portMAX_DELAY);
    xSemaphoreGive(scan_mutex);
}

void swarm_comm_send_scan(uint8_t addr, swarm_pose_id_t id) {
    tx_queue_item_t item = {
            .type = COMM_TX_QUEUE_ITEM_SCAN_RES,
            .addr = addr,
            .pose_id = id,
    };
    ASSERT(xQueueSend(tx_queue, &item, portMAX_DELAY));
}

void swarm_comm_send_ctrl_msg(uint8_t addr, swarm_comm_ctrl_msg_t msg) {
    tx_queue_item_t item = {
            .type = COMM_TX_QUEUE_ITEM_CTRL,
            .addr = addr,
            .ctrl_msg = msg,
    };
    ASSERT(xQueueSend(tx_queue, &item, portMAX_DELAY));
}

void swarm_comm_broadcast_ctrl_msg(swarm_comm_ctrl_msg_t msg) {
    swarm_comm_send_ctrl_msg(RADIO_ADR_BROADCAST, msg);
}

static void handle_tx_queue_item(tx_queue_item_t *item) {
    switch (item->type) {
        case COMM_TX_QUEUE_ITEM_POSE: {
            // Load full pose from the swarm graph
            swarm_pose_t pose = {
                    .id = item->pose_id,
            };
            swarm_graph_load_pose(&pose);

            // Broadcast the pose
            swarm_radio_send_msg(RADIO_ADR_BROADCAST, COMM_MSG_TAG_POSE,
                                 &pose, sizeof(pose));
            break;
        }
        case COMM_TX_QUEUE_ITEM_SCAN_REQ: {
            // Send the pose_id as the message
            swarm_radio_send_msg(item->addr, COMM_MSG_TAG_SCAN_REQ,
                                 &item->pose_id, sizeof(item->pose_id));
            break;
        }
        case COMM_TX_QUEUE_ITEM_SCAN_RES: {
            // Load the scan from flash
            scan_t scan = {
                    .pose = item->pose_id,
            };
            ASSERT(swarm_graph_has_scan(item->pose_id));
            swarm_graph_load_scan(&scan);

            // Send the scan to the requesting drone
            swarm_radio_send_msg(item->addr, COMM_MSG_TAG_SCAN_RES,
                                 &scan, sizeof(scan));
            break;
        }
        case COMM_TX_QUEUE_ITEM_CTRL: {
            // Send the control message
            swarm_radio_send_msg(item->addr, COMM_MSG_TAG_CTRL,
                                 &item->ctrl_msg, sizeof(item->ctrl_msg));
        }
    }
}

void swarm_comm_tx_task(void *parameters) {

    while (true) {

        // Check if there is a transmission request
        tx_queue_item_t item;
        if (xQueueReceive(tx_queue, &item, portMAX_DELAY)) {
            handle_tx_queue_item(&item);
        }
    }
}

void swarm_radio_handle_msg(uint8_t src, uint8_t tag, uint8_t *data,
                            size_t len) {

    // Handle the incoming message depending on it's tag
    switch (tag) {
        case COMM_MSG_TAG_POSE: {
            if (len == sizeof(swarm_pose_t)) {
                swarm_graph_store_pose((swarm_pose_t *) data);
            } else {
                DEBUG_PRINT("Discarding invalid pose message!");
            }
            break;
        }
        case COMM_MSG_TAG_SCAN_REQ: {
            if (len == sizeof(swarm_pose_id_t)) {
                swarm_comm_send_scan(src, *(swarm_pose_id_t *) data);
            } else {
                DEBUG_PRINT("Discarding invalid scan request message!");
            }
            break;
        }
        case COMM_MSG_TAG_SCAN_RES: {
            if (len == sizeof(scan_t)) {
                scan_t *scan = (scan_t *) data;
                swarm_graph_store_scan(scan);
                if (xSemaphoreTake(scan_mutex, 0)) {
                    xSemaphoreGive(scan_mutex);
                } else {
                    if (scan_req_id.raw == scan->pose.raw) {
                        xSemaphoreGive(scan_semaphore);
                    }
                }
            } else {
                DEBUG_PRINT("Discarding invalid scan response message!");
            }
            break;
        }
        case COMM_TX_QUEUE_ITEM_CTRL: {
            swarm_comm_handle_ctrl_msg(src, *(swarm_comm_ctrl_msg_t *) data);
        }
    }

}
