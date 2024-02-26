/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "icp.h"

#include <stdint.h>
#include <stdio.h>

#include "cfassert.h"

#define H_ZERO  {1, 0, 0, 0, 1, 0, 0, 0, 1}

static void print_mat(const float m[4]) {
    printf("%f\t%f\n%f\t%f\n\n", m[0], m[1], m[2], m[3]);
}

static void matmul_2x2(const float a[4], const float b[4], float c[4]) {
    c[0] = a[0] * b[0] + a[1] * b[2];
    c[1] = a[0] * b[1] + a[1] * b[3];
    c[2] = a[2] * b[0] + a[3] * b[2];
    c[3] = a[2] * b[1] + a[3] * b[3];
}

static void matmul_3x3(const float a[9], const float b[9], float c[9]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i * 3 + j] = 0;
            for (int k = 0; k < 3; k++) {
                c[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
            }
        }
    }
}

static icp_point_t matpointmul(const float r[4], const icp_point_t p) {
    icp_point_t out = {
            .x = r[0] * p.x + r[1] * p.y,
            .y = r[2] * p.x + r[3] * p.y,
    };
    return out;
}

static void compute_normals(icp_points_t *points, float normals[]) {
    for (size_t i = 0; i < points->num; ++i) {
        icp_point_t cur_point = points->items[i];
        size_t prev_idx, next_idx;
        float min_dist = INFINITY, snd_min_dist = INFINITY;
        for (int j = 0; j < points->num; ++j) {
            if (i == j) continue;
            float dist = points_dist_squared(cur_point, points->items[j]);
            if (dist < min_dist) {
                prev_idx = next_idx;
                next_idx = j;
                snd_min_dist = min_dist;
                min_dist = dist;
            } else if (dist < snd_min_dist) {
                prev_idx = j;
                snd_min_dist = dist;
            }
        }
        icp_point_t next_point = points->items[next_idx];
        icp_point_t prev_point = points->items[prev_idx];
        normals[i] = atan2f(prev_point.y - next_point.y,
                            next_point.x - prev_point.x);
    }
}

static void
compute_correspondences(const icp_points_t *P, const icp_points_t *Q,
                        float error_bound, int16_t cor_P[], int16_t cor_Q[]) {
    ASSERT(P->num < INT16_MAX);
    ASSERT(Q->num < INT16_MAX);
    int16_t p_num = (int16_t) P->num;
    int16_t q_num = (int16_t) Q->num;
    memset(cor_Q, 0, Q->num * sizeof(int16_t));
    for (int16_t i = 0; i < p_num; ++i) {
        float min_dist = error_bound;
        int16_t chosen_idx = -1;
        icp_point_t cur_point = P->items[i];
        for (int16_t j = 0; j < q_num; ++j) {
            float dist = points_dist_squared(cur_point, Q->items[j]);
            if (dist < min_dist) {
                min_dist = dist;
                chosen_idx = j;
            }
        }
        cor_P[i] = chosen_idx;
        if (chosen_idx >= 0) cor_Q[chosen_idx]++;
    }
}

static void center_data_P(const icp_points_t *points, const int16_t cor[],
                          icp_point_t *center, icp_points_t *centered) {
    ASSERT(points->num == centered->num);
    float x = 0, y = 0;
    for (int i = 0; i < points->num; ++i) {
        if (cor[i] < 0) continue;
        x += points->items[i].x;
        y += points->items[i].y;
    }
    x /= (float) points->num;
    y /= (float) points->num;
    for (int i = 0; i < points->num; ++i) {
        if (cor[i] < 0) continue;
        centered->items[i].x = points->items[i].x - x;
        centered->items[i].y = points->items[i].y - y;
    }
    center->x = x;
    center->y = y;
}

static void center_data_Q(const icp_points_t *points, const int16_t cor[],
                          icp_point_t *center, icp_points_t *centered) {
    ASSERT(points->num == centered->num);
    float x = 0, y = 0;
    size_t num = 0;
    for (int i = 0; i < points->num; ++i) {
        if (!cor[i]) continue;
        float count = cor[i];
        x += points->items[i].x * count;
        y += points->items[i].y * count;
        num += cor[i];
    }
    x /= (float) num;
    y /= (float) num;
    for (int i = 0; i < points->num; ++i) {
        if (!cor[i]) continue;
        centered->items[i].x = points->items[i].x - x;
        centered->items[i].y = points->items[i].y - y;
    }
    center->x = x;
    center->y = y;
}

static void compute_cross_variance(icp_points_t *P, icp_points_t *Q,
                                   const int16_t cor[], float cov[4]) {
    ASSERT(sizeof(float[4]) == 4 * 4);
    memset(cov, 0, sizeof(float[4]));
    for (size_t i = 0; i < P->num; ++i) {
        int16_t j = cor[i];
        if (j < 0) continue;
        icp_point_t p = P->items[i];
        icp_point_t q = Q->items[j];
        cov[0] += p.x * q.x;
        cov[1] += p.y * q.x;
        cov[2] += p.x * q.y;
        cov[3] += p.y * q.y;
    }
}

void svd(const float in[4], float u[4], float sigma[4], float v[4]) {

    float a = in[0], b = in[1], c = in[2], d = in[3];

    float e = 0.5f * (a + d);
    float f = 0.5f * (a - d);
    float g = 0.5f * (b + c);
    float h = 0.5f * (c - b);

    if (sigma != NULL) {

        float q = sqrtf(e * e + h * h);
        float r = sqrtf(f * f + g * g);

        float sx = q + r;
        float sy = q - r;

        sigma[0] = sx;
        sigma[1] = 0;
        sigma[2] = 0;
        sigma[3] = sy;

    }

    float a1 = atan2f(g, f);
    float a2 = atan2f(h, e);

    float theta = 0.5f * (a2 - a1);
    float phi = 0.5f * (a2 + a1);

    float theta_sin = sinf(theta);
    float theta_cos = cosf(theta);
    float phi_sin = sinf(phi);
    float phi_cos = cosf(phi);

    u[0] = theta_cos;
    u[1] = -theta_sin;
    u[2] = theta_sin;
    u[3] = theta_cos;

    v[0] = phi_cos;
    v[1] = -phi_sin;
    v[2] = phi_sin;
    v[3] = phi_cos;

}

/***
 * Performs the iterative closest point algorithm.
 * @param Q Original point cloud to match against.
 * @param P Moved point cloud that will be matched.
 * @param max_iterations Maximum number of iterations.
 */
void icp(const icp_points_t *Q, const icp_points_t *P, int max_iterations,
         float error_bound, icp_point_t *translation, float *rotation) {

    // Allocate additional memory
    POINTS_STATIC_ALLOC(P_copy, P->num)
    points_copy(P_copy, P);
    int16_t cor_P[P_copy->num];
    int16_t cor_Q[Q->num];
    POINTS_STATIC_ALLOC(P_centered, P->num)
    POINTS_STATIC_ALLOC(Q_centered, Q->num)

    float t_a[9] = H_ZERO, t_b[9] = H_ZERO;
    float *next = t_a, *prev = t_b;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {

        // Compute correspondences
        compute_correspondences(P_copy, Q, error_bound, cor_P, cor_Q);

        // Center the data for P and Q (based on correspondences)
        icp_point_t center_of_P, center_of_Q;
        center_data_P(P_copy, cor_P, &center_of_P, P_centered);
        center_data_Q(Q, cor_Q, &center_of_Q, Q_centered);

        // Computer the cross variance on centered data
        float cov[4];
        compute_cross_variance(P_centered, Q_centered, cor_P, cov);

        // Execute SVD to obtain transformation
        float u[4], sigma[4], v[4], r[4];
        svd(cov, u, sigma, v);
        matmul_2x2(u, v, r);
        icp_point_t t = matpointmul(r, center_of_P);
        t.x = center_of_Q.x - t.x;
        t.y = center_of_Q.y - t.y;

        // Chain transformations using homogenous coordinates
        float transform[9] = {
                r[0], r[1], t.x,
                r[2], r[3], t.y,
                0, 0, 1,
        };
        matmul_3x3(transform, prev, next);
        float *tmp = next;
        next = prev;
        prev = tmp;

        // Apply transformation to point cloud
        for (size_t i = 0; i < P_copy->num; ++i) {
            icp_point_t p = matpointmul(r, P_copy->items[i]);
            p.x += t.x;
            p.y += t.y;
            P_copy->items[i] = p;
        }

    }

    // Write transform result
    translation->x = prev[2];
    translation->y = prev[5];
    *rotation = atan2f(prev[3], prev[0]);

}
