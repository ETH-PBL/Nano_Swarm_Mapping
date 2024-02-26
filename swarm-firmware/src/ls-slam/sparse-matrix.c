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
#include <math.h>

#include "sparse-matrix.h"


void insert_element(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j, float element) {
   /*
   If we have a dense matrix with eleemtns of coordinates (i, j), we call as order the
   value i*N + j.
   */

    if (fabsf(element) < 0.00001f)
        return;

   // If the new element is higher order then the previous added elements
   if (spMat->idx1 == 0) {
      spMat->highestOrder = -1;
      spMat->idx0 = -1;
   }

   if (i*N + j <= spMat->highestOrder) {
      // printf("aa \n");
      int32_t rowStart = spMat->rowCnt[i];  // 1
      int32_t rowEnd = spMat->rowCnt[i+1];  // 2
      int32_t k_last = rowStart;
      for (int32_t k=rowStart; k<rowEnd; k++) {
         k_last = rowEnd;
         if (spMat->col[k] == j) {  // Already there
            spMat->data[k] = element;
            spMat->col[k] = j;
            return;
         }
         if (spMat->col[k] > j) {
            k_last = k;
            break;
         }
      }

      memmove (&(spMat->data[k_last+1]), &(spMat->data[k_last]), (spMat->idx0 - k_last + 1) * sizeof(float));
      memmove (&(spMat->col[k_last+1]), &(spMat->col[k_last]), (spMat->idx0 - k_last + 1) * sizeof(int16_t));

      spMat->data[k_last] = element;
      spMat->col[k_last] = j;
      for (int16_t l=i+1; l<=spMat->idx1; l++)
         spMat->rowCnt[l]++;
      spMat->idx0++;
      return;
   }

   spMat->idx0++;
   spMat->data[spMat->idx0] = element;
   spMat->col[spMat->idx0] = j;
   for (int16_t k=spMat->idx1; k<=i; k++)
      spMat->rowCnt[k+1] = spMat->rowCnt[k];

   spMat->rowCnt[i+1]++;
   spMat->idx1 = i+1;

   spMat->highestOrder = i*N + j;
   return;
}


void add_to_element(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j, float value) {
   int32_t rowStart = spMat->rowCnt[i];
   int32_t rowEnd = spMat->rowCnt[i+1];

   for (int32_t k=rowStart; k<rowEnd; k++) {
      if (spMat->col[k] == j) {
         spMat->data[k] += value;
         return;
      }
   }

    insert_element(spMat, N, i, j, value);
}


int16_t already_added(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j) {
   int32_t rowStart = spMat->rowCnt[i];
   int32_t rowEnd = spMat->rowCnt[i+1];

   for (int32_t k=rowStart; k<rowEnd; k++) {
      if (spMat->col[k] > j)
         return 0;
      if (spMat->col[k] == j)
         return 1;
   }
   return 0;
}

float get_element(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j) {
   int32_t rowStart = spMat->rowCnt[i];
   int32_t rowEnd = spMat->rowCnt[i+1];

   for (int32_t k=rowStart; k<rowEnd; k++) {
      if (spMat->col[k] == j)
         return spMat->data[k];
      if (spMat->col[k] > j)
         break;
   }

   return 0.0f;
}

float get_element_sym(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j) {
   if (i < j) {
      int16_t aux = i;
      i = j;
      j = aux;
   }

   int32_t rowStart = spMat->rowCnt[i];
   int32_t rowEnd = spMat->rowCnt[i+1];

   for (int32_t k=rowStart; k<rowEnd; k++) {
      if (spMat->col[k] == j)
         return spMat->data[k];
      if (spMat->col[k] > j)
         break;
   }

   return 0.0f;
}


void sparse_from_dense(int16_t N, float mat[N][N], sparseMatrix* spMat) {
      for (int16_t i=0; i<N; i++)
         for (int16_t j=0; j<N; j++)
            if (mat[i][j] > 0)
               insert_element(spMat, N, i, j, mat[i][j]);
}


void transpose_lower_triangular(sparseMatrix* spMat, int16_t N) {
   sparseMatrix spMatTr;
   memset(&spMatTr, 0, sizeof(sparseMatrix));

   for (int16_t i = 0; i < N; i++)
      for (int32_t k = spMat->rowCnt[i]; k < spMat->rowCnt[i+1]; k++)
         insert_element(&spMatTr, N, spMat->col[k], i, spMat->data[k]);

   memcpy(spMat, &spMatTr, sizeof(sparseMatrix));
}

