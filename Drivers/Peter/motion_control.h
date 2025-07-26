// motion_control.h
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>

void Motion_Stop_WithPID(void);
void Motion_Stop_WithoutPID(void);

void Motion_Forward(int16_t speed);
void Motion_Backward(int16_t speed);
void Motion_LeftTurnByRadius(int16_t speed, float radius);
void Motion_RightTurnByRadius(int16_t speed, float radius);
void Motion_LeftTurnByAngdps(int16_t speed, float angspeed_dps);
void Motion_RightTurnByAngdps(int16_t speed, float angspeed_dps);

void Motion_SetLR_Speed(int16_t left_speed, int16_t right_speed);

void Motion_Spin(int16_t speed);
void Motion_RotationTurn(float angle_deg, int16_t forward_speed);



#endif