#pragma once
//Based on Michael Madden's work
//https://coderdojoathenry.org/2019/02/24/hackers-how-to-control-a-robots-wheel-motors-based-on-joystick-movements/
#include <stdint.h>
#include <math.h>

struct direction_t{
    int16_t left;
    int16_t right;
};

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calculate_motor_directions(struct direction_t * directions, int32_t xaxis, int32_t yaxis){
    int16_t remapped_x = map(xaxis, -512, 512, -255, 255);
    int16_t remapped_y = map(yaxis, -512, 512, -255, 255);

    directions->left = remapped_y + remapped_x;
    directions->right= remapped_y - remapped_x;

    if (directions->left > 255){
        directions->left = 255;
    }else if (directions->left < -255){
        directions->left = -255;
    }

    if (directions->right > 255){
        directions->right = 255;
    }else if (directions->right < -255){
        directions->right = -255;
    }
}