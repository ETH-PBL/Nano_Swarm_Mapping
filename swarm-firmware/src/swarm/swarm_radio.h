/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef SWARM_RADIO_H
#define SWARM_RADIO_H

#include <stdint.h>
#include <stddef.h>

#include "radiolink.h"

#include "swarm.h"

#define RADIO_ADR_BRIDGE        (SWARM_NUM_DEVICES - 1)
#define RADIO_ADR_BROADCAST     0xF

void swarm_radio_init();

void swarm_radio_send_msg(uint8_t dst, uint8_t tag, void *data, size_t len);

void swarm_radio_handle_msg(uint8_t src, uint8_t tag, uint8_t *data,
                            size_t len);

void swarm_radio_p2p_handler(P2PPacket *p);

#endif //SWARM_RADIO_H
