/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "swarm.h"

#include "cfassert.h"
#include "debug.h"

#include "swarm_radio.h"

uint8_t swarm_id;

// Static ID map
static const struct {
    uint32_t mcu_id;
    uint8_t swarm_id;
} id_map[SWARM_NUM_DEVICES] = {
        {
                .mcu_id = 0x00240035,
                .swarm_id = 0,
        },
//        {
//                .mcu_id = 0x003A0032,
//                .swarm_id = 1,
//        },
//        {
//                .mcu_id = 0x0026001D,
//                .swarm_id = 2,
//        },
//        {
//                .mcu_id = 0x0044001B,
//                .swarm_id = 3,
//        },
        // Bridge (base station)
        {
                .mcu_id = 0x0027004D,
                .swarm_id = RADIO_ADR_BRIDGE,
        },
};

void swarm_init() {

    // Set swarm address of this drone
    swarm_id = 0xFF;
    for (int i = 0; i < SWARM_NUM_DEVICES; i++) {
        if (*((uint32_t *) (MCU_ID_ADDRESS)) == id_map[i].mcu_id) {
            swarm_id = id_map[i].swarm_id;
            break;
        }
    }
    ASSERT(swarm_id != 0xFF);
    DEBUG_PRINT("SWARM ADDR: %d\n", swarm_id);
}
