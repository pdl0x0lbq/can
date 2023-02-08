#ifndef  _odometry_h_
#define  _odometry_h_

#include "stdint.h"
# define    PI   3.141592653


union floatdata
{
    float d ;
    uint8_t  data[4];
};

union IntFloat{
    int i;
    float f;
};

extern uint8_t speed_data[10];
extern union  floatdata  right_speed,left_speed,positon_x,positon_y,oriention,vel_linear,vel_angular;

void robot_base_control(int l_speed, int r_speed);
void speed_transformation_send(void);

float  get_speed(void);
#endif
