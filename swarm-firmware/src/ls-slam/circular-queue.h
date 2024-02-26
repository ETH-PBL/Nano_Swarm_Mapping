/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Vlad Niculescu
 */

#ifndef CIRCULARQUEUE_H
#define CIRCULARQUEUE_H

#include "node.h"
#include "config_size.h"

#include "run_on_pc.h"


typedef struct {
  node nodes[MAX_SIZE];
  int16_t head;
  int16_t tail;
  int16_t size;
}circularQueue;

int16_t init_queue(circularQueue* queue);
int16_t is_full(circularQueue* queue);
int16_t is_empty(circularQueue* queue);
int16_t push_to_queue(circularQueue* queue, node element);
int16_t pop_from_queue(circularQueue* queue, node* element);

#endif