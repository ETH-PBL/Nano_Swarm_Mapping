/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Vlad Niculescu
 */

#ifndef GRAPHBASEDSLAM_H
#define GRAPHBASEDSLAM_H

#include "sparse-matrix.h"

#include "run_on_pc.h"


typedef struct {
   int16_t fromIdx;
   int16_t toIdx;
   float z[3];
}constraint;

typedef struct {
   float z[3];
}measurement;


/**
 * Calculates the jacobians of A, B and the error.
 * 
 * Calculates the cumulative contribution of the jacobians.
 * @param 
 * @param
 * @param 
 * @param 
 * @param 
 * @param 
 **/
void linearize_pose_pose_constraint(float* x1, float* x2, float* z, float A[3][3], float B[3][3], float* e);


/**
 * Calculate the H matrix and the b array
 * 
 * Calculates the cumulative contribution of the jacobians.
 * @param 
 * @param
 * @param 
 * @param 
 * @param 
 * @param 
 * @param 
 **/
void build_H_and_b(float* x,
                   sparseMatrix* H,
                   float* b,
                   measurement* measurements,
                   const uint8_t *skip,
                   constraint* constraints,
                   int16_t nr_of_poses,
                   int16_t nr_of_constr);


/**
 * Solve the equation H * x = b
 * 
 * This function solves the linearized equation system H * x = b, which is the most complex part of a LS_SLAM iteration.
 * @param H pointer to the sparse matrix that contains the non-zero elements of H.
 * @param b one column array.
 * @param perm permutation array which contains the reordering information for the rows/columns of H.
 * @param size dimension of the equation system: size = 3*nr_of_poses. The dense equivalent of H is (size x size). b is (size, 1).
 * @return x the function return is void, but the value of x is found in b after the function executes (for memory saving).
 **/
void solve_for_x(sparseMatrix* H, float* b, int16_t* perm, int16_t size);


/**
 * Computes a measurement out of two poses
 * 
 * This function calculates the measurement that drives the drone from pose x_from to pose x_to. 
 * The measurement is represented in the frame of x_from. A pose has the structure (x, y, yaw).
 * @param x_from array of dimension 3 containing the first pose.
 * @param x_to array of dimension 3 containing the second pose.
 * @param z array of dimension 3 containing the measurement.
 **/
void measurement_from_poses(float* x_from, float* x_to, float* z);


/**
 * Computes the measurement given a series of consecutive poses
 * 
 * This function calculates all the measurements given a series of consecutive poses. 
 * Given a series of N poses: x0, x1, x2, ... xN, N-1 measurements are provided: (x0, x1) -> m0, (x1, x2) -> m1, (x(N-1), xN) -> m(N-1).
 * The measurement calculation is based on the function above.
 * @param x array of dimension (3*nr_of_poses x 1). Pose i is consisted of (x[i], x[i+1], x[i+2]).
 * @param measurements pointer to the structure containing the measurements. After running the function, the measurements are found here.
 * @param nr_of_poses Number of poses in the trajectory.
 **/
void measurements_from_poses(float* x, measurement* measurements, int nr_of_poses);


/**
 * Forward substitution
 * 
 * This function solves an equation system of the shape L * y = b, given that L is a lower triangular matrix. 
 * @param L sparse matrix of the lower triangular Cholesky matrix (L * L.T = H).
 * @param b array of dimension (N x 1).
 * @param N dimension of the equation system.
 * @return x the function return is void, but the solution x is found in the array b after the function executes.
 **/
void forward_substitution(sparseMatrix* L, float* b, int N);


/**
 * Backward substitution
 * 
 * This function solves an equation system of the shape L.T * x = y, given that L is a lower triangular matrix. 
 * @param L sparse matrix of the lower triangular Cholesky matrix (L * L.T = H).
 * @param b array of dimension (N x 1).
 * @param N dimension of the equation system.
 * @return x the function return is void, but the solution x is found in the array b after the function executes.
 **/
void backward_substitution(sparseMatrix* L, float* b, int N);


/**
 * Run LS-SLAM
 * 
 * This function solves an equation system of the shape L.T * x = y, given that L is a lower triangular matrix. 
 * @param L sparse matrix of the lower triangular Cholesky matrix (L * L.T = H).
 * @param b array of dimension (N x 1).
 * @param N dimension of the equation system.
 * @return x the function return is void, but the solution x is found in the array b after the function executes.
 **/
void ls_slam(float* x, uint8_t *skip, constraint* constraints, int16_t nr_of_poses, int16_t nr_of_constr);


/**
 * Test the solution
 * 
 * This function is not meant to be used in the real-time operation. Is is only used for debugging/testing purposes. 
 * The function calculates the sum of the absolute values of the elements of H * x - b. If the solution x is correct, the error is 0.
 * @param H sparse matrix.
 * @param b array of dimension (N x 1).
 * @param x array of dimension (N x 1) containing the solution.
 * @return the error of H * x - b. Calculated as sum(abs(H * x - b)).
 **/
float test_linear_system(sparseMatrix* H, float* x, float* b, int N);

#endif
