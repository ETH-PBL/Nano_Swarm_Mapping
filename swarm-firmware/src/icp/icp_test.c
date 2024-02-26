/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include <stdio.h>

#include "icp.h"

points_t *read_csv(char *filename) {
    size_t i;
    points_t *cloud = points_alloc(1024);
    FILE *f = fopen(filename, "r");
    for (i = 0; i < 1024; ++i) {
        float x, y;
        int num = fscanf(f, "%f,%f\n", &x, &y);
        if (num != 2) break;
        cloud->items[i].x = x;
        cloud->items[i].y = y;
        cloud->num = i + 1;
    }
    fclose(f);
    printf("%s: read %zu points\n", filename, i);
    return cloud;
}

void write_csv(char *filename, points_t *cloud) {
    FILE *f = fopen(filename, "w");
    for (size_t i = 0; i < cloud->num; ++i) {
        fprintf(f, "%.16f,%.16f\n", cloud->items[i].x, cloud->items[i].y);
    }
    fclose(f);
    printf("%s: wrote %zu points\n", filename, cloud->num);
}

int main(int argc, char *argv[]) {

    points_t *cloud1 = read_csv("cloud1.csv");
    points_t *cloud2 = read_csv("cloud2.csv");

    point_t translation;
    float rotation;
    points_t *cloud3 = icp(cloud1, cloud2, 30, &translation, &rotation);

    printf("Transform: %f m\t%f m\t%f deg\n", translation.x, translation.y,
           rotation / M_PI * 180);

    write_csv("output.csv", cloud3);

    free(cloud3);

}