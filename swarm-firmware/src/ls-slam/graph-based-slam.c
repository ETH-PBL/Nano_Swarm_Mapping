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
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef OFFDRONE
   #include "debug.h"
   #include "system.h"
   #include "FreeRTOS.h"
   #include "task.h"
#endif

#include "utils_math.h"
#include "config_size.h"
#include "sparse-matrix.h"
#include "rcm-sparse-matrix.h"
#include "graph-based-slam.h"


void v2t(float pose[3], float T[3][3]) {
   /* This function converts SE2 pose from a vector to transformation

   Parameters
   ----------
   pose : 3x1 vector
     (x, y, theta) of the robot pose

   Returns
   -------
   T : 3x3 matrix
     Transformation matrix corresponding to the vector
   */

   float c = cosf(pose[2]);
   float s = sinf(pose[2]);
   T[0][0] = c; T[0][1] = -s; T[0][2] = pose[0];
   T[1][0] = s; T[1][1] = c;  T[1][2] = pose[1];
   T[2][0] = 0; T[2][1] = 0;  T[2][2] = 1;
}


void t2v(float T[3][3], float v[3]) {
   /* This function converts SE2 transformation to vector for

   Parameters
   ----------
   T : 3x3 matrix
     Transformation matrix for 2D pose

   Returns
   -------
   pose : 3x1 vector
     (x, y, theta) of the robot pose
   */

   float x = T[0][2];
   float y = T[1][2];
   float theta = atan2f(T[1][0], T[0][0]);
   v[0] = x; v[1] = y; v[2] = theta;
}


void linearize_pose_pose_constraint(float* x1, float* x2, float* z, float A[3][3], float B[3][3], float* e) {
   /* Compute the error and the Jacobian for pose-pose constraint

   Parameters
   ----------
   x1 : 3x1 vector
      (x,y,theta) of the first robot pose
   x2 : 3x1 vector
      (x,y,theta) of the second robot pose
   z :  3x1 vector
      (x,y,theta) of the measurement

   Returns
   -------
   e  : 3x1
      error of the constraint
   A  : 3x3
      Jacobian wrt x1
   B  : 3x3
      Jacobian wrt x2
   */

   // Compute e
   float Zij[3][3];
   float Xi[3][3];
   float Xj[3][3];

   v2t(z, Zij);
   v2t(x1, Xi);
   v2t(x2, Xj);

   matinv_3x3(Zij, Zij);
   matinv_3x3(Xi, Xi);
   matmul_3x3(Zij, Xi, Xi);
   matmul_3x3(Xi, Xj, Xi);

   t2v(Xi, e);

   // compute A & B
   float s = sinf(x1[2]);
   float c = cosf(x1[2]);

   float s2 = sinf(z[2]);
   float c2 = cosf(z[2]);

   float RiT[2][2] = {{c, s}, {-s, c}};
   float RijT[2][2] = {{c2, s2}, {-s2, c2}};
   float dRiT_dTheta[2][2] = {{-s, c}, {-c, -s}};
   float delta_t[2] = {x2[0] - x1[0], x2[1] - x1[1]};

   float aux0[2][2];
   float aux1[2];

   matmul_2x2(RijT, dRiT_dTheta, aux0);
   matmul_2x2x1(aux0, delta_t, aux1);
   matmul_2x2(RijT, RiT, aux0);

   A[0][0] = -aux0[0][0]; A[0][1] = -aux0[0][1]; A[0][2] = aux1[0];
   A[1][0] = -aux0[1][0]; A[1][1] = -aux0[1][1]; A[1][2] = aux1[1];
   A[2][0] = 0;           A[2][1] = 0;           A[2][2] = -1;

   B[0][0] = aux0[0][0]; B[0][1] = aux0[0][1]; B[0][2] = 0;
   B[1][0] = aux0[1][0]; B[1][1] = aux0[1][1]; B[1][2] = 0;
   B[2][0] = 0;          B[2][1] = 0;          B[2][2] = 1;

}


void build_H_and_b(float* x, 
                   sparseMatrix* H, 
                   float* b, 
                   measurement* measurements,
                   const uint8_t *skip,
                   constraint* constraints, 
                   int16_t nr_of_poses, 
                   int16_t nr_of_constr) {
   float z[3];
   float A[3][3], B[3][3], e[3];

   int16_t dim = 3*nr_of_poses;

   insert_element(H, dim, 0, 0, 1000);
   insert_element(H, dim, 1, 1, 1000);
   insert_element(H, dim, 2, 0, 0);
   insert_element(H, dim, 2, 1, 0);
   insert_element(H, dim, 2, 2, 1000);

   for (int16_t i=1; i<nr_of_poses; i++) {
      if (skip[i-1]) continue;

      float xi[3] = {x[3*i-3], x[3*i-2], x[3*i-1]};
      float xj[3] = {x[3*i], x[3*i+1], x[3*i+2]};
      memcpy(z, measurements[i-1].z, sizeof(z));

      linearize_pose_pose_constraint(xi, xj, z, A, B, e);
      float bi[3], bj[3];
      float Hii[3][3], Hjj[3][3], Hij[3][3];
      transpose_and_mult_3x3x1(A, e, bi);
      transpose_and_mult_3x3x1(B, e, bj);
      transpose_and_mult_3x3(A, A, Hii);
      transpose_and_mult_3x3(A, B, Hij);
      transpose_and_mult_3x3(B, B, Hjj);

      add_to_element(H, dim, 3*(i-1) + 0, 3*(i-1) + 0, Hii[0][0]); // Hii
      add_to_element(H, dim, 3*(i-1) + 1, 3*(i-1) + 1, Hii[1][1]); // Hii
      add_to_element(H, dim, 3*(i-1) + 2, 3*(i-1) + 0, Hii[2][0]); // Hii
      add_to_element(H, dim, 3*(i-1) + 2, 3*(i-1) + 1, Hii[2][1]); // Hii
      add_to_element(H, dim, 3*(i-1) + 2, 3*(i-1) + 2, Hii[2][2]); // Hii

      insert_element(H, dim, 3*i + 0, 3*(i-1) + 0, -1); // Hij

      insert_element(H, dim, 3*i + 0, 3*i + 0, Hjj[0][0]); // Hjj

      insert_element(H, dim, 3*i + 1, 3*(i-1) + 1, -1); // Hij

      insert_element(H, dim, 3*i + 1, 3*i + 1, Hjj[1][1]); // Hjj

      insert_element(H, dim, 3*i + 0, 3*(i-1) + 2, Hij[2][0]); // Hij
      insert_element(H, dim, 3*i + 1, 3*(i-1) + 2, Hij[2][1]); // Hij
      insert_element(H, dim, 3*i + 2, 3*(i-1) + 2, -1); // Hij

      insert_element(H, dim, 3*i + 2, 3*i + 0, 0); // Hjj
      insert_element(H, dim, 3*i + 2, 3*i + 1, 0); // Hjj
      insert_element(H, dim, 3*i + 2, 3*i + 2, Hjj[2][2]); // Hjj

      for (int16_t i0=0; i0<3; i0++) {
         b[3*(i-1)+i0] += -bi[i0];
         b[3*i+i0] += -bj[i0];
      }
   }

   for (int16_t i=0; i<nr_of_constr; i++) {
      int fromIdx = constraints[i].fromIdx;
      int toIdx = constraints[i].toIdx;
      float info = 20.0f;
      memcpy(z, constraints[i].z, sizeof(z));

      float xi[3] = {x[3*fromIdx], x[3*fromIdx+1], x[3*fromIdx+2]};
      float xj[3] = {x[3*toIdx], x[3*toIdx+1], x[3*toIdx+2]};

      linearize_pose_pose_constraint(xi, xj, z, A, B, e);
      float bi[3], bj[3];
      float Hii[3][3], Hjj[3][3], Hij[3][3];
      transpose_and_mult_3x3x1(A, e, bi);
      transpose_and_mult_3x3x1(B, e, bj);
      transpose_and_mult_3x3(A, A, Hii);
      transpose_and_mult_3x3(A, B, Hij);
      transpose_and_mult_3x3(B, B, Hjj);

      for (int16_t i0=0; i0<3; i0++) {
         add_to_element(H, dim, 3*fromIdx + i0, 3*fromIdx + i0, info * Hii[i0][i0]); // Hii
         add_to_element(H, dim, 3*toIdx + i0, 3*toIdx + i0, info * Hjj[i0][i0]); // Hjj
         insert_element(H, dim, 3*fromIdx + i0, 3*toIdx + i0, info * (-1.0f)); // Hij
      }
      add_to_element(H, dim, 3*fromIdx + 2, 3*fromIdx + 0, info * Hii[2][0]); // Hii
      add_to_element(H, dim, 3*fromIdx + 2, 3*fromIdx + 1, info * Hii[2][1]); // Hii

      insert_element(H, dim, 3*fromIdx + 2, 3*toIdx + 0, info * Hij[2][0]); // Hij
      insert_element(H, dim, 3*fromIdx + 2, 3*toIdx + 1, info * Hij[2][1]); // Hij

      for (int16_t i0=0; i0<3; i0++) {
         b[3*fromIdx+i0] += -bi[i0] * info;
         b[3*toIdx+i0] += -bj[i0] * info;
      }
   }
}


void forward_substitution(sparseMatrix* L, float* b, int N) {
   float sol[N];
   memset(sol, 0, sizeof(sol));
   sol[0] = b[0] / get_element(L, N, 0, 0);
   for (int i = 1; i < N; i++) {
      float sum = 0;
      for (int32_t k = L->rowCnt[i]; k < L->rowCnt[i+1]; k++)
         if (i > L->col[k])
            sum += L->data[k] * sol[L->col[k]];
      sol[i] = (1.0f / get_element(L, N, i, i)) * (b[i] - sum);
   }

   memcpy(b, sol, sizeof(sol));
}


void backward_substitution(sparseMatrix* L, float* b, int N) {
   float sol[N];
   memset(sol, 0, sizeof(sol));
   sol[N-1] = b[N-1] / get_element(L, N, N-1, N-1);
   for (int i = N-2; i >= 0; i--) {
      float sum = 0;
      for (int32_t k = L->rowCnt[i]; k < L->rowCnt[i+1]; k++)
         if (i < L->col[k])
            sum += L->data[k] * sol[L->col[k]];
      sol[i] = (1.0f / get_element(L, N, i, i)) * (b[i] - sum);
   }

   memcpy(b, sol, sizeof(sol));
}


int16_t bandwidth_test(sparseMatrix* spMat, int N, int16_t* perm) {
   int16_t perm_inv[N];
   for (int16_t i = 0; i < N; i++)
      for (int16_t j = 0; j < N; j++)
         if (perm[j] == i) {
            perm_inv[i] = j;
            break;
         }

   // Bandwidth test
   int16_t bw_len = 0;
   for (int16_t i = 0; i < N; i++) {
      for (int32_t k = spMat->rowCnt[i]; k < spMat->rowCnt[i+1]; k++) {
         int j = spMat->col[k];
         int indx = perm_inv[i];
         int indy = perm_inv[j];
         if (abs(indx - indy) > bw_len) {
            bw_len = abs(indx - indy);
         }
      }
   }
   DEBUG_PRINT("Bandwidth is %d \n", bw_len);
   return bw_len;
}


void cholesky_sparse(sparseMatrix* H, int16_t N, int16_t perm[N]) {
   int16_t i, j;
   int32_t k;
   sparseMatrix L;
   memset(&L, 0, sizeof(sparseMatrix));

   float threshold = 0.00001f;

   for (i = 0; i < N; i++) {
      for (j = 0; j <= i; j++) {
         float sum = 0;
         float start = L.rowCnt[i];
         for (k = L.rowCnt[j]; k < L.rowCnt[j+1]; k++) {
            int16_t colIdxJ = L.col[k];
            if (colIdxJ >= i)
               break;

            for (int32_t o = start; o < L.rowCnt[i+1]; o++) {
               int16_t colIdxI = L.col[o];
               if (colIdxJ == colIdxI) {
                  sum += L.data[k] * L.data[o];
                  start = o;
                  break;
               }
               if (colIdxI > colIdxJ) {
                  start = o;
                  break;
               }
            }
         }
 
         float value = 0;
         if (i == j) 
            value = sqrtf(get_element_sym(H, N, perm[i], perm[i]) - sum);
         else {
            float value2 = get_element_sym(H, N, perm[i], perm[j]) - sum;
            if (value2 == 0.0f)
               value = 0.0f;
            else
               value = (1.0f / get_element(&L, N, j, j)) * value2;
         }
         
         if (fabs(value) > (double)threshold)
            insert_element(&L, N, i, j, value);
      }
   }


   DEBUG_PRINT("Size of L is %d\n", L.idx0);
   memcpy(H, &L, sizeof(sparseMatrix));
}

static float sparse_dot_product(sparseMatrix* S, int16_t row_a, int16_t row_b, int16_t max_col) {
    float sum = 0;
    int16_t i = row_a;
    int16_t j = row_b;

    int16_t start = S->rowCnt[i];
    for (int32_t k = S->rowCnt[j]; k < S->rowCnt[j+1]; k++) {
        int16_t colIdxJ = S->col[k];
        if (colIdxJ > max_col)
            break;
        for (int32_t o = start; o < S->rowCnt[i+1]; o++) {
            int16_t colIdxI = S->col[o];
            if (colIdxJ == colIdxI) {
                sum += S->data[k] * S->data[o];
                start = o;
                break;
            }
            if (colIdxI > colIdxJ) {
                start = o;
                break;
            }
        }
    }
    return sum;
}

static void cholesky_sparse_2(sparseMatrix* H, int16_t N, int16_t* perm) {
    int16_t i, j;
    int32_t k;
    sparseMatrix L;
    memset(&L, 0, sizeof(sparseMatrix));

    for (j = 0; j < N; j++) {
        float sum = 0;
        for (k = 0; k < j; k++)
            sum += powf(get_element(&L, N, j, k), 2);

        float element = sqrtf(get_element_sym(H, N, perm[j], perm[j]) - sum);
        insert_element(&L, N, j, j, element);

        for (i = j+1; i < N; i++) {
            sum = sparse_dot_product(&L, j, i, j-1);

            float Hij = get_element_sym(H, N, perm[i], perm[j]) - sum;
            if (Hij == 0.0f)
                element = 0.0f;
            else
                element = (1.0f / get_element(&L, N, j, j)) * Hij;

            insert_element(&L, N, i, j, element);
        }
    }
    DEBUG_PRINT("Size of L is %d\n", L.idx0);
    memcpy(H, &L, sizeof(sparseMatrix));
}

float test_linear_system(sparseMatrix* H, float* x, float* b, int N) {
   // H * x = b
   float res[N];
   memset(res, 0, sizeof(res));
   for (int16_t i = 0; i < N; i++) {
      for (int32_t k = H->rowCnt[i]; k < H->rowCnt[i+1]; k++) {
         res[i] += H->data[k] * x[H->col[k]];
         if (i > H->col[k])
            res[H->col[k]] += H->data[k] * x[i];
      }
      res[i] -= b[i];
   }

   float error = 0.0f;
   for (int16_t i = 0; i < N; i++)
      error += (float)fabs(res[i]);

   DEBUG_PRINT("\nError is %.8f \n", error);
   return error;
}


void solve_for_x(sparseMatrix* H, float* b, int16_t* perm, int16_t size) {
   bandwidth_test(H, size, perm);

   // Cholesky Decomposition
//   TickType_t t0 = xTaskGetTickCount();
   cholesky_sparse_2(H, size, perm);
    DEBUG_PRINT("Size of H is %d\n", H->idx0);
//    DEBUG_PRINT("Cholesky time: %ld \n", xTaskGetTickCount() - t0);

   permute_array(b, perm, size);

   /* L * L.T * x = b;   we note L.T * x = y
      Solves L * y = b; */
   forward_substitution(H, b, size);
   float* y = b;

   // Solves L.T * x = y
   transpose_lower_triangular(H, size);
   backward_substitution(H, y, size);
   float* x = y;

   // Inverse permute the solution
   permutation_inverse(perm, size);
   permute_array(x, perm, size);   
}

void measurement_from_poses(float* x_from, float* x_to, float* z) {
   for (int16_t k=0; k<3; k++)
      z[k] = x_to[k] - x_from[k];

   float s = sinf(-x_from[2]);
   float c = cosf(-x_from[2]); 
   float R[2][2] = {{c, -s}, {s, c}};
   matmul_2x2x1(R, z, z);
}

void measurements_from_poses(float* x, measurement* measurements, int nr_of_poses) {
   float z[3];
   for (int16_t i=1; i<nr_of_poses; i++) {
      float xi[3] = {x[3*i-3], x[3*i-2], x[3*i-1]};
      float xj[3] = {x[3*i], x[3*i+1], x[3*i+2]};
      measurement_from_poses(xi, xj, z);

      measurements[i-1].z[0] = z[0];
      measurements[i-1].z[1] = z[1];
      measurements[i-1].z[2] = z[2];
   }
}

void ls_slam(float* x, uint8_t *skip, constraint* constraints, int16_t nr_of_poses, int16_t nr_of_constr) {
   int16_t R[3*nr_of_poses];

   measurement measurements[nr_of_poses-1];
   measurements_from_poses(x, measurements, nr_of_poses);

   sparseMatrix H;
   float b[3*nr_of_poses];

   for (int16_t k=0; k<3; k++) {
//      TickType_t t0 = xTaskGetTickCount();
      memset(&H, 0, sizeof(sparseMatrix));
      memset(b, 0, sizeof(b));

      build_H_and_b(x, &H, b, measurements, skip, constraints, nr_of_poses, nr_of_constr);
//      TickType_t t1 = xTaskGetTickCount();
      rcm(&H, R, 3*nr_of_poses);
//      DEBUG_PRINT("RCM time: %ld \n", xTaskGetTickCount() - t1);
      solve_for_x(&H, b, R, 3*nr_of_poses);

      for (int16_t i=0; i<3*nr_of_poses; i++)
         x[i] += b[i];

//      DEBUG_PRINT("Loop time: %ld \n\n", xTaskGetTickCount() - t0);

   }
}