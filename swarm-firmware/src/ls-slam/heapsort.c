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
#include <stdlib.h>

#include "node.h"
#include "heapsort.h"


void swap(node* a, node* b) {
    node temp = *a;
    *a = *b;
    *b = temp;
}
 
// To heapify a subtree rooted with node i
// which is an index in nodes[].
// n is size of heap
void heapify(node* nodes, int16_t N, int16_t i) {
    // Find largest among root, left child and right child
 
    // Initialize largest as root
    int16_t largest = i;
 
    // left = 2*i + 1
    int16_t left = 2 * i + 1;
 
    // right = 2*i + 2
    int16_t right = 2 * i + 2;
 
    // If left child is larger than root
    if (left < N && nodes[left].neighbors > nodes[largest].neighbors)
 
        largest = left;
 
    // If right child is larger than largest
    // so far
    if (right < N && nodes[right].neighbors > nodes[largest].neighbors)
 
        largest = right;
 
    // Swap and continue heapifying if root is not largest
    // If largest is not root
    if (largest != i) {
 
        swap(&nodes[i], &nodes[largest]);
 
        // Recursively heapify the affected
        // sub-tree
        heapify(nodes, N, largest);
    }
}
 
// Main function to do heap sort
void heapsort_nodes(node* nodes, int16_t N) {
 
    // Build max heap
    for (int16_t i = N / 2 - 1; i >= 0; i--)
 
        heapify(nodes, N, i);
 
    // Heap sort
    for (int16_t i = N - 1; i >= 0; i--) {
 
        swap(&nodes[0], &nodes[i]);
 
        // Heapify root element to get highest element at
        // root again
        heapify(nodes, i, 0);
    }
}





