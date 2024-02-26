/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Vlad Niculescu
 */

#include "deck.h"
#include "debug.h"
#include <string.h>
#include <math.h>
#include "system.h"
#include "FreeRTOS.h"
#include "static_mem.h"
#include "task.h"
#include "semphr.h"
#include "param.h"
#include "graph-based-slam.h"

void ls_slam_example() {
	// Synthetic trajectory data
   int16_t nr_of_poses = 100;
   float x_cmd, y_cmd, yaw_cmd;
   float edge_l = 0;
   float edge_w = 0;

   float x[3*nr_of_poses];
   memset(x, 0, sizeof(x));

   for (int16_t i=0; i<nr_of_poses; i++) {
      yaw_cmd = i*0.002f;
      if (i < nr_of_poses / 4.0) {
         x_cmd = edge_l / 100.0f;
         y_cmd = edge_w / 100.0f;
         edge_l += 1;
      } 
      else {
         if (i < nr_of_poses / 2.0) {
            x_cmd = edge_l / 100.0f;
            y_cmd = edge_w / 100.0f;
            edge_w += 1;
         }
         else {
            if (i < 3.0f * nr_of_poses / 4.0f) {
               x_cmd = edge_l / 100.0f;
               y_cmd = edge_w / 100.0f;
               edge_l -= 1.0f;
            }
            else {
               x_cmd = edge_l / 100.0f;
               y_cmd = edge_w / 100.0f;
               edge_w -= 1.0f;
            }
         }
      }
      x[3 * i] = 10 * x_cmd;
      x[3 * i + 1] = 10 *  y_cmd;
      x[3 * i + 2] = yaw_cmd;
   }

   // Declare the loop closure constraints
   constraint constraints[10];
   memset(&constraints, 0, 10*sizeof(constraint));

   constraints[0].fromIdx = nr_of_poses-1;
   constraints[0].toIdx = 0;
   constraints[0].z[0] = -0.108861f;
   constraints[0].z[1] = -0.107133f;
   constraints[0].z[2] = -0.008400f;


   // Run SLAM
   long int t0 = xTaskGetTickCount();
   ls_slam(x, constraints, nr_of_poses, 1);
   long int t1 = xTaskGetTickCount();
   DEBUG_PRINT("Total time: %ld us \n", t1 - t0);

   DEBUG_PRINT("\nSolution: \n");
   for (int16_t i=0; i<3*nr_of_poses; i++)
      DEBUG_PRINT("%d: %.6f \n", i, x[i]);
    DEBUG_PRINT("\n \n");
}