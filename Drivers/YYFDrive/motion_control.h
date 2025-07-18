// motion_control.h
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>

void Motion_Forward(int16_t speed);
void Motion_Backward(int16_t speed);
void Motion_right(int16_t speed);
void Motion_left(int16_t speed);

void Motion_Spin(int16_t speed);
void Motion_ArcTurn(float angle_deg, int16_t forward_speed);

void Motion_Stop(void);

void Motion_SpinTaskStart(float angle_deg, int16_t speed);
void Motion_SpinTaskUpdate(float now_angle);
void Motion_AbortSpin(void);

#endif