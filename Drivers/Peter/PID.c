// PID.c
#include "PID.h"

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->last_error = 0;
    pid->integral = 0.0f;
    pid->enable_limit = 0;
    pid->enable_output_limit = 0;
    pid->I_max = 0;
    pid->I_min = 0;
    pid->out_max = 0;
    pid->out_min = 0;
}

int16_t PID_Compute(PID_Controller* pid, int16_t error) {
    int16_t diff = error - pid->last_error;
    pid->last_error = error;

    pid->integral += error;
    if (pid->enable_limit) {
        if (pid->integral > pid->I_max) pid->integral = pid->I_max;
        if (pid->integral < pid->I_min) pid->integral = pid->I_min;
    }

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * diff;

    if (pid->enable_output_limit) {
        if (output > pid->out_max) output = pid->out_max;
        if (output < output < pid->out_min) output = pid->out_min;
    }

    return (int16_t)output;
}