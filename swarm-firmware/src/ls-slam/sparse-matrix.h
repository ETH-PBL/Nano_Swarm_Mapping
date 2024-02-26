/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Vlad Niculescu
 */

#ifndef SPARSEMAT_H
#define SPARSEMAT_H

#include "config_size.h"

#include "run_on_pc.h"


typedef struct {
   float data[MAX_SIZE_SP_MAT];
   int16_t col[MAX_SIZE_SP_MAT];
   int16_t rowCnt[MAX_SIZE+1];
   int32_t idx0;
   int16_t idx1;
   int32_t highestOrder;
}sparseMatrix;


void insert_element(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j, float element);
void add_to_element(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j, float value);
int16_t already_added(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j);
void sparse_from_dense(int16_t N, float mat[N][N], sparseMatrix* spMat);
float get_element(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j);
float get_element_sym(sparseMatrix* spMat, int16_t N, int16_t i, int16_t j);
void transpose_lower_triangular(sparseMatrix* spMat, int16_t N);

#endif