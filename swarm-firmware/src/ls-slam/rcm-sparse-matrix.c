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
#include <string.h>

#include "node.h"
#include "circular-queue.h"
#include "heapsort.h"
#include "sparse-matrix.h"
#include "rcm-sparse-matrix.h"


int16_t is_neighbour(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j) {
   if (i == j)
      return 0;

   if (i < j) {
      int16_t aux = i;
      i = j;
      j = aux;
   }

   int16_t res = already_added(spMat, N, i, j);
      
   return res;
}

void count_all_neighbors(sparseMatrix* spMat, node* nodes, int16_t N){
   for (int16_t i=0;i<N;i++){
      nodes[i].index = i;
   }

   for (int16_t i = 0; i < N; i++)
      for (int32_t k = spMat->rowCnt[i]; k < spMat->rowCnt[i+1]; k++)
         if (spMat->col[k] < i) {
            nodes[i].neighbors++;
            nodes[spMat->col[k]].neighbors++;
         }
}

void rcm(sparseMatrix* spMat, int16_t* perm, int16_t N) {
   node nodes[N];
   memset(nodes, 0, N*sizeof(node));

   count_all_neighbors(spMat, nodes, N);

   // quickSortNodes(nodes, N);
   heapsort_nodes(nodes, N);

   // Init queue
   circularQueue q;
   init_queue(&q);

   int16_t perm_index = 0;
   int16_t nodes_index = 0;

   for (int16_t i=0;i<N; i++) {
      if (nodes[i].neighbors > 0)
         break;
      else {
         perm[perm_index++] = nodes[i].index;
         nodes[i].visited = 1;
      }
   }

   nodes_index = perm_index;

   node C;
   while (perm_index < N) {
      if (nodes[nodes_index].visited == 0) {
         push_to_queue(&q, nodes[nodes_index]);
         nodes[nodes_index].visited = 1;
      }
      nodes_index++;

      while (!is_empty(&q)) {
         pop_from_queue(&q, &C);
         perm[perm_index++] = C.index;

         int16_t neighbour_counter = 0;
         for (int16_t i=nodes_index; i<N; i++) {
            if (is_neighbour(spMat, N, C.index, nodes[i].index)) {
               if (nodes[i].visited == 0) {
                  push_to_queue(&q, nodes[i]);
                  nodes[i].visited = 1;
               }
               neighbour_counter++;
               if (neighbour_counter == C.neighbors)
                  break;
            }
         }
      }
   }

   for (int16_t i = 0; i<N/2; i++){
      int16_t temp = perm[i];
      perm[i] = perm[N-i-1];
      perm[N-i-1] = temp;
   }
}

void permutation_inverse(int16_t* perm, int N) {
   int16_t permInv[N];
   for (int16_t i = 0; i < N; i++)
      for (int16_t j = 0; j < N; j++)
         if (perm[j] == i) {
            permInv[i] = j;
            break;
         }

   memcpy(perm, permInv, sizeof(permInv));
}

void permute_array(float* array, int16_t* perm, int N) {
   float arrayInv[N];
   for (int16_t i=0; i<N; i++)
      arrayInv[i] = array[perm[i]];

   memcpy(array, arrayInv, sizeof(arrayInv));
}