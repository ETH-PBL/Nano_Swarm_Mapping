/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Vlad Niculescu
 */

#include <stdio.h>
#include <string.h>

#include "circular-queue.h"


int16_t init_queue(circularQueue* queue) {
    memset(queue, 0, sizeof(circularQueue));
    return 1;
}


int16_t is_full(circularQueue* queue) {
    if (queue->size == MAX_SIZE)
        return 1;
    else 
        return 0;
}


int16_t is_empty(circularQueue* queue) {
    if (queue->size == 0)
        return 1;
    else 
        return 0;
}


int16_t push_to_queue(circularQueue* queue, node element) {
    if (is_full(queue))
        return 0;

    queue->nodes[queue->tail] = element;
    queue->tail = (queue->tail + 1) % MAX_SIZE;
    queue->size++;

    return 1;
}


int16_t pop_from_queue(circularQueue* queue, node* element) {
    if (is_empty(queue))
        return 0;

    *element = queue->nodes[queue->head];
    queue->head = (queue->head + 1) % MAX_SIZE;
    queue->size--;

  return 1;
}
