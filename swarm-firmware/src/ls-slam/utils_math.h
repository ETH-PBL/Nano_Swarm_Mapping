/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Vlad Niculescu
 */

#ifndef UTILS_MATH_H
#define UTILS_MATH_H

#include "run_on_pc.h"


void matinv_3x3(float A[3][3], float A_inv[3][3]);
void matmul_3x3(float A[3][3], float B[3][3], float R[3][3]);
void transpose_and_mult_3x3(float A[3][3], float B[3][3], float R[3][3]);
void transpose_and_mult_3x3x1(float A[3][3], float B[3], float R[3]);
void matmul_2x2(float A[2][2], float B[2][2], float R[2][2]);
void matmul_2x2x1(float A[2][2], float B[2], float R[2]);


#endif