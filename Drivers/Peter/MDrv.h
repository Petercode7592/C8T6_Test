//
// Created by Petercode on 25-7-9.
//


#ifndef MDRV_H
#define MDRV_H

#include "main.h"

extern uint32_t mall[4];
extern uint16_t mtep[4];

extern float v[4];
extern int32_t vstd[4]; // 标准速度
extern float Voltage; // 电池电压

HAL_StatusTypeDef MDrv_Reset();
HAL_StatusTypeDef MDrv_SetMtype(uint8_t mtype);
HAL_StatusTypeDef MDrv_SetMline(uint8_t mline);
HAL_StatusTypeDef MDrv_SetMphase(uint8_t mphase);
HAL_StatusTypeDef MDrv_SetDeadzone(uint32_t deadzone);
HAL_StatusTypeDef MDrv_SetWdiameter(uint8_t wdiameter);
HAL_StatusTypeDef MDrv_SetPID(float p, float i, float d);
HAL_StatusTypeDef MDrv_SetSpeed(int32_t Vm1, int32_t Vm2, int32_t Vm3, int32_t Vm4);
HAL_StatusTypeDef MDrv_SetPWM(int32_t pwm1, int32_t pwm2, int32_t pwm3, int32_t pwm4);
HAL_StatusTypeDef MDrv_UploadData(uint8_t mall_en,  uint8_t mtep_en, uint8_t mspd_en);

void MDrv_ProcessMALL(char *data);
void MDrv_ProcessMTEP(char *data);
void MDrv_ProcessMSPD(char *data);
void MDrv_ProcessVol(char *data);

void MDrv_ReadVol();
void MDrv_Init(int mode);

uint32_t Mdrv_ReadMALL(int8_t MX);
uint16_t Mdrv_ReadMTEP(int8_t MX);
float Mdrv_Readspeed(int8_t MX);

#endif //MDRV_H
