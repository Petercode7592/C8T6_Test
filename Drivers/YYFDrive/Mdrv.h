//
// Created by Petercode on 25-7-9.
//


#ifndef MDRV_H
#define MDRV_H

#include "stm32f1xx_hal.h"

extern float mall[4];
extern int mtep[4];
extern float mspd[4];

void MDrv_ReadVol(char *charArray);

HAL_StatusTypeDef MDrv_Reset();
HAL_StatusTypeDef MDrv_SetMtype(uint8_t mtype);
HAL_StatusTypeDef Mdrv_SetMline(uint8_t mline);
HAL_StatusTypeDef Mdrv_SetMphase(uint8_t mphase);
HAL_StatusTypeDef MDrv_SetDeadzone(uint32_t deadzone);
HAL_StatusTypeDef MDrv_SetWdiameter(uint8_t wdiameter);
HAL_StatusTypeDef MDrv_SetPID(float p, float i, float d);
HAL_StatusTypeDef MDrv_SetSpeed(const int32_t Vm1, const int32_t Vm2, const int32_t Vm3, const int32_t Vm4);
HAL_StatusTypeDef MDrv_SetPWM(int16_t *pwm) ;
HAL_StatusTypeDef Mdrv_UploadData(uint8_t mall,  uint8_t mtep, uint8_t mspd);

void MDrv_ProcessMALL(char *data);
void MDrv_ProcessMTEP(char *data);
void MDrv_ProcessMSPD(char *data);

void MDrv_JudgeDataType(char *Udata);

void MDrv_GoSpeed(int8_t mode, int32_t Vm);

void MDrv_Init();

#endif //MDRV_H
