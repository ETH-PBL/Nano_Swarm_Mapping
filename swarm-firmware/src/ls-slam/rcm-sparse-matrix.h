/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Vlad Niculescu
 */

#ifndef RCMSPARSE_H
#define RCMSPARSE_H

#include "run_on_pc.h"


void rcm(sparseMatrix* spMat, int16_t* perm, int16_t N);
void permutation_inverse(int16_t* perm, int N);
void permute_array(float* array, int16_t* perm, int N);

#endif