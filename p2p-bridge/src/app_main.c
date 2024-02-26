/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include <stdbool.h>
#include <string.h>

#include "app.h"
#include "radiolink.h"
#include "crtp.h"
#include "debug.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "swarm.h"
#include "swarm_radio.h"
#include "swarm_comm.h"
#include "util.h"

typedef struct {
    uint8_t addr;
    swarm_comm_ctrl_msg_t ctrl_msg;
} tx_queue_item_t;

static QueueHandle_t tx_queue;
static StaticQueue_t tx_queue_static;
static uint8_t tx_queue_buf[8 * sizeof(tx_queue_item_t)];

void p2pHandler(P2PPacket *p) {
    swarm_radio_p2p_handler(p);
    CRTPPacket pk = {
            .port = 1,
            .channel = p->size <= CRTP_MAX_DATA_SIZE ? 1 : 0,
            .size = MIN(p->size, CRTP_MAX_DATA_SIZE),
    };
    memcpy(pk.data, p->data, pk.size);
    crtpSendPacketBlock(&pk);
    if (p->size > CRTP_MAX_DATA_SIZE) {
        pk.channel = 1;
        pk.size = p->size - CRTP_MAX_DATA_SIZE;
        memcpy(pk.data, p->data + CRTP_MAX_DATA_SIZE, pk.size);
        crtpSendPacketBlock(&pk);
    }
}

void swarm_radio_handle_msg(uint8_t src, uint8_t tag, uint8_t *data,
                            size_t len) {
    CRTPPacket pk = {
            .port = 10,
            .channel = 0,
            .size = 6,
            .data = {src, tag},
    };
    memcpy(&pk.data[2], &len, sizeof(len));
    crtpSendPacketBlock(&pk);
    pk.channel = 1;
    for (size_t offset = 0; offset < len; offset += CRTP_MAX_DATA_SIZE) {
        pk.size = MIN(len - offset, CRTP_MAX_DATA_SIZE);
        memcpy(pk.data, data + offset, pk.size);
        crtpSendPacketBlock(&pk);
    }
}

void ctrl_msg_handler(CRTPPacket *pk) {
    tx_queue_item_t item = {
            .addr = pk->data[0],
            .ctrl_msg = *(swarm_comm_ctrl_msg_t *) (&pk->data[1]),
    };
    ASSERT(xQueueSend(tx_queue, &item, M2T(5)));
}

void appMain() {

    tx_queue = xQueueCreateStatic(
            sizeof(tx_queue_buf) / sizeof(tx_queue_item_t),
            sizeof(tx_queue_item_t), tx_queue_buf, &tx_queue_static);

    swarm_init();
    swarm_radio_init();

    p2pRegisterCB(p2pHandler);
    crtpRegisterPortCB(11, ctrl_msg_handler);

    // Transmit control messages
    while (true) {

        // Check if there is a transmission request
        tx_queue_item_t item;
        if (xQueueReceive(tx_queue, &item, portMAX_DELAY)) {
            swarm_radio_send_msg(item.addr, COMM_MSG_TAG_CTRL,
                                 &item.ctrl_msg, sizeof(item.ctrl_msg));
        }
    }
}
