/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef SLAM_FIRMWARE_ICP_H
#define SLAM_FIRMWARE_ICP_H

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define POINTS_SIZEOF(n)    (sizeof(icp_points_t) + (n) * sizeof(icp_point_t))

#define POINTS_STATIC_ALLOC(name, n) \
uint8_t _##name##_buf[POINTS_SIZEOF(n)]; \
icp_points_t *name = (icp_points_t *) _##name##_buf; \
name->num = (n);

typedef struct {
    float x;
    float y;
} icp_point_t;

typedef struct {
    size_t num;
    icp_point_t items[];
} icp_points_t;

static inline icp_points_t *points_alloc(size_t n) {
    icp_points_t *out = malloc(sizeof(icp_points_t) + n * sizeof(icp_point_t));
    out->num = n;
    return out;
}

static inline void points_copy(icp_points_t *out, const icp_points_t *in) {
    memcpy(out, in, sizeof(icp_points_t) + in->num * sizeof(icp_point_t));
}

static inline float point_norm(icp_point_t p) {
    return sqrtf(p.x * p.x + p.y * p.y);
}

static inline void point_normalize(icp_point_t *p) {
    float norm = point_norm(*p);
    p->x /= norm;
    p->y /= norm;
}

static inline float points_dist_squared(icp_point_t a, icp_point_t b) {
    float x = a.x - b.x;
    float y = a.y - b.y;
    return x * x + y * y;
}

void svd(const float in[4], float u[4], float sigma[4], float v[4]);

void icp(const icp_points_t *Q, const icp_points_t *P, int max_iterations,
         float error_bound, icp_point_t *translation, float *rotation);

#endif //SLAM_FIRMWARE_ICP_H
