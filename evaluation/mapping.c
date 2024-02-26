/*
 * This code is adapted from the following work:
 * T. Polonelli, C. Feldmann, V. Niculescu, H. Müller, M. Magno and L. Benini,
 * "Towards Robust and Efficient On-board Mapping for Autonomous Miniaturized UAVs,"
 * 2023 9th International Workshop on Advances in Sensors and Interfaces (IWASI),
 * Monopoli (Bari), Italy, 2023, pp. 9-14, doi: 10.1109/IWASI58316.2023.10164476.
 * https://ieeexplore.ieee.org/abstract/document/10164476
 */

// Compile with: gcc -c -fpic mapping.c
// gcc -shared -o libmapping.so mapping.o

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "math.h"

typedef struct {
    float x_buf[32];
    float y_buf[32];
    int16_t num;
} xy_buffer_t;

typedef enum {
    DIR_FRONT = 0,
    DIR_BACK = 1,
    DIR_LEFT = 2,
    DIR_RIGHT = 3,
} direction_t;


// Returns absolute, redundant if std implementation exists
int16_t absolute(int16_t value){
  if(value < 0){
    return -value;
  }
  else{
    return value;
  }
}

// Mark hit cell
void mark_hit(int16_t ix, int16_t iy, int8_t* map, int16_t n_col) {
  int8_t val = *(map + n_col*ix + iy);
  if(val < INT8_MAX){
    *(map + n_col*ix + iy) += 1;
  }
}

// Mark free cell
void mark_free(int16_t ix, int16_t iy, int8_t* map, int16_t n_col) {
  int8_t val = *(map + n_col*ix + iy);
  if(val > INT8_MIN){
    *(map + n_col*ix + iy) -= 1;
  }
}

//Mark cells correctly through which the ToF laser passes
void mark_laser_bresenham(int16_t obs_ix, int16_t obs_iy, int16_t drone_ix, int16_t drone_iy, int8_t* map, int16_t n_col) {
  
  //setup initial conditions
  int16_t x1 = drone_ix;
  int16_t y1 = drone_iy;
  int16_t x2 = obs_ix;
  int16_t y2 = obs_iy;
  int16_t dx = x2 - x1;
  int16_t dy = y2 - y1;

  //Bresenham algorithm:

  bool is_steep = absolute(dy) > absolute(dx);  // determine how steep the line is
  if (is_steep){  // rotate line
    int16_t temp = x1;
    x1 = y1;
    y1 = temp;

    temp = x2;
    x2 = y2;
    y2 = temp;
  }
  //swap start and end points if necessary and store swap state
  if (x1 > x2){
    int16_t temp = x1;
    x1 = x2;
    x2 = temp;

    temp = y1;
    y1 = y2;
    y2 = temp;
  }
  dx = x2 - x1;  // recalculate differentials
  dy = y2 - y1;  // recalculate differentials
  int16_t error = (dx / 2.0f);  // calculate error

  int16_t y_step;
  if(y1 < y2){
    y_step = 1;
  }
  else{
    y_step = -1;
  }
  // iterate over bounding box generating points between start and end
  int16_t y = y1;

  for (int16_t x_iter = x1; x_iter < x2+1; x_iter++){
    if(is_steep){
      mark_free(y, x_iter, map, n_col);
    }
    else{
      mark_free(x_iter, y, map, n_col);
    }

    error -= absolute(dy);
    if(error < 0){
      y += y_step;
      error += dx;
    }
  }
}


void scan_extract_points(float* pos, int16_t tof[4][8], int16_t max_range, xy_buffer_t *buffer) {
    buffer->num = 0;

    const float offsets[4] = {
            0.02f, 0.02f, 0.025f, 0.025f,
    };
    const float step = -((float) M_PI_4) / 8;

    float frame_x = pos[0];
    float frame_y = pos[1];
    float frame_yaw = pos[2];

    for (int dir = 0; dir < 4; dir++) {

        float dir_yaw = frame_yaw;
        if (dir == DIR_BACK) dir_yaw += (float) M_PI;
        else if (dir == DIR_LEFT) dir_yaw += (float) M_PI_2;
        else if (dir == DIR_RIGHT) dir_yaw -= (float) M_PI_2;
        float angle = -4 * step + step / 2;
        for (int col = 0; col < 8; angle += step, col++) {

            // Check if measurement is valid
            int16_t measurement = tof[dir][col];
            if (measurement < 0) continue;
            if (measurement > max_range) continue;

            // Apply rotation and add point
            float dist_x = (float) measurement / 1000.0f;
            float dist_y = tanf(angle) * dist_x;
            dist_x += offsets[dir];

            buffer->x_buf[buffer->num] = frame_x + dist_x * cosf(dir_yaw) -
                                                   dist_y * sinf(dir_yaw);
            buffer->y_buf[buffer->num] = frame_y + dist_x * sinf(dir_yaw) +
                                                   dist_y * cosf(dir_yaw);
            buffer->num++;
        }
    }
    
}


void get_occ_map(float pos[][3], 
                 int16_t tof_mm[][32], 
                 int16_t n,
                 int16_t max_range_mm, 
                 float min_x, 
                 float min_y, 
                 float l_x, 
                 float l_y, 
                 float res, 
                 int16_t* dim,
                 int8_t* map) {

  float max_x = min_x + l_x;
  float max_y = min_y + l_y;
  xy_buffer_t xy_buf = {0};

  int16_t dim_x = (int16_t)(l_x / res);
  int16_t dim_y = (int16_t)(l_y / res);

  dim[0] = dim_x;
  dim[1] = dim_y;
  memset(map, 0, dim_x * dim_y);

  for (int16_t i=0; i<n; i++) {
    float drone_x = pos[i][0];
    float drone_y = pos[i][1];

    if (drone_x > max_x ||
        drone_y > max_y ||
        drone_x < min_x ||
        drone_y < min_y)
      continue;

    int16_t drone_ix = floor((drone_x - min_x) / res);
    int16_t drone_iy = floor((drone_y - min_y) / res);

    scan_extract_points(pos[i], tof_mm[i], max_range_mm, &xy_buf);

    for (int16_t k=0; k<xy_buf.num; k++) {
        float obs_x = xy_buf.x_buf[k];
        float obs_y = xy_buf.y_buf[k];
        if (obs_x > max_x ||
            obs_y > max_y ||
            obs_x < min_x ||
            obs_y < min_y)
          continue;

        int16_t obs_ix = floor((obs_x - min_x) / res);
        int16_t obs_iy = floor((obs_y - min_y) / res);

        // Update occupied cell
        mark_hit(obs_ix, obs_iy, map, dim_y);
        mark_hit(obs_ix+1, obs_iy, map, dim_y);
        mark_hit(obs_ix, obs_iy+1, map, dim_y);
        mark_hit(obs_ix+1, obs_iy+1, map, dim_y);
        
        // Update free cells
        mark_laser_bresenham(obs_ix, obs_iy, drone_ix, drone_iy, map, dim_y);
    }
  }
}

void get_occ_map2(float pos[][3], 
                  float scan_xy[][2],
                  int16_t n,
                  int16_t max_range_mm, 
                  float min_x, 
                  float min_y, 
                  float l_x, 
                  float l_y, 
                  float res, 
                  int16_t* dim,
                  int8_t* map) {

  float max_x = min_x + l_x;
  float max_y = min_y + l_y;
  xy_buffer_t xy_buf = {0};

  int16_t dim_x = (int16_t)(l_x / res);
  int16_t dim_y = (int16_t)(l_y / res);

  dim[0] = dim_x;
  dim[1] = dim_y;
  memset(map, 0, dim_x * dim_y);

  for (int16_t i=0; i<n; i++) {
    float drone_x = pos[i][0];
    float drone_y = pos[i][1];

    if (drone_x > max_x ||
        drone_y > max_y ||
        drone_x < min_x ||
        drone_y < min_y)
      continue;

    int16_t drone_ix = floor((drone_x - min_x) / res);
    int16_t drone_iy = floor((drone_y - min_y) / res);

    float obs_x = scan_xy[i][0];
    float obs_y = scan_xy[i][1];
    if (obs_x > max_x ||
        obs_y > max_y ||
        obs_x < min_x ||
        obs_y < min_y)
      continue;

    int16_t obs_ix = floor((obs_x - min_x) / res);
    int16_t obs_iy = floor((obs_y - min_y) / res);

    // Update occupied cell
    mark_hit(obs_ix, obs_iy, map, dim_y);
    mark_hit(obs_ix, obs_iy, map, dim_y);
    mark_hit(obs_ix, obs_iy, map, dim_y);
    mark_hit(obs_ix, obs_iy, map, dim_y);
    mark_hit(obs_ix, obs_iy, map, dim_y);
    mark_hit(obs_ix, obs_iy, map, dim_y);
    mark_hit(obs_ix, obs_iy, map, dim_y);
    mark_hit(obs_ix, obs_iy, map, dim_y);

    // Update free cells
    mark_laser_bresenham(obs_ix, obs_iy, drone_ix, drone_iy, map, dim_y);

  }
}


// int main() {
//   float pos[10][3];
//   int16_t tof_mm[10][32];
//   for (int16_t i=0; i<10; i++) {
//     pos[i][0] = (float) i / 4.0;
//     pos[i][1] = (float) i / 5.0;
//     pos[i][2] = (float) i / 10.0;

//     for (int16_t j=0; j<32; j++) {
//       tof_mm[i][j] = 10*j;
//     }
//   }

//   int8_t map[100000];
//   int16_t dim[2];
//   get_occ_map(pos, tof_mm, 10, 2000, 0, 0, 5, 5, 0.1, dim, map);
//   printf("%d, %d\n", dim[0], dim[1]);

//   return 0;
// }