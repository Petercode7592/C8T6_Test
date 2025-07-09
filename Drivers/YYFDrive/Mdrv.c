//
// Created by Petercode on 25-7-9.
//
#include "stm32f1xx_hal.h"
#include "Mdrv.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"

void MDrv_ReadVol(char *charArray) {
    // 接收放在中断内
    HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) "$read_vol#", 10, Uart_MaxWait);
}

/** @brief  (Checked)MCU 驱动板复位
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_Reset() {
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) "$flash_reset#", 13, Uart_MaxWait);
}

/** @brief 配置电机类型
  * @param mtype 电机类型，1: 520电机 2：310电机 3: TT电机(带有编码器的) 4: TT电机(不带编码器)
  * @note 固件默认值为1 (520电机)
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetMtype(uint8_t mtype) {
    char cmd[25];
    snprintf(cmd, sizeof(cmd), "$mtype:%d#", mtype);
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 配置电机的相位线
  * @param mline 是霍尔编码器转一圈的相位，此值需要查商家电机的参数表可得
  * @note 带编码器电机来说:此值对于控制速度起主要作用，此值需要正确.配置电机霍尔编码器的相位为13线
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef Mdrv_SetMline(uint8_t mline) {
    char cmd[25];
    snprintf(cmd, sizeof(cmd), "$mline:%d#", mline); // 发送指令
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 配置电机的减速比
  * @param mphase 减速比，固件默认。此值需要查商家电机的参数表可得
  * @note 带编码器电机来说:此值对于控制速度起主要作用，此值需要正确.固件默认值30
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef Mdrv_SetMphase(uint8_t mphase) {
    char cmd[25];
    snprintf(cmd, sizeof(cmd), "$mphase:%d#", mphase); // 发送指令
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 配置电机的pwm脉冲死区
  * @param deadzone 死区值，范围0-3600。这个需要测量得出,可通过改变此值，消除电机最小震荡。死区值的范围(0-3600)
  * @note 固件默认值1600.这样控制pwm时会默认加上死区的值，使得电机不会出现震荡区域
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetDeadzone(uint32_t deadzone) {
    char cmd[25];
    if (deadzone <= 3600) {
        // 确保死区值在0到3600之间
        snprintf(cmd, sizeof(cmd), "$deadzone:%ld#", deadzone); // 发送指令
        return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
    }
    return HAL_ERROR; // 错误处理
}

/**
  * @brief (可选)配置车轮直径
  * @param wdiameter 车轮直径，单位为毫米
  * @note 带编码器电机来说：此值对于控制速度起主要作用，此值单位为毫米（mm）；如果此值不正确，只会影响速度数据不准确，不会影响编码器的数据
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetWdiameter(uint8_t wdiameter) {
    char cmd[25];
    snprintf(cmd, sizeof(cmd), "$wdiameter:%d#", wdiameter); // 发送指令
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}

/** @brief (bug)设置PID参数, 设置后芯片会重启，会把正在运动的电机停止
  * @param p 比例系数
  * @param i 积分系数
  * @param d 微分系数
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetPID(float p, float i, float d) {
    // 发送指令
    char cmd[50];
    snprintf(cmd, sizeof(cmd), "$MPID:%f,%f,%f#", p, i, d);
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}

/** * @brief 设置各个电机速度(单位mm/s)
  * @param Vm1 电机1速度
  * @param Vm2 电机2速度 (-1000到1000之间)
  * @param Vm3 电机3速度 (-1000到1000之间)
  * @param Vm4 电机4速度 (-1000到1000之间)
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef MDrv_SetSpeed(const int32_t Vm1, const int32_t Vm2, const int32_t Vm3, const int32_t Vm4) {
    char cmd[50];
    if (Vm1 < -1000 || Vm1 > 1000 || Vm2 < -1000 || Vm2 > 1000 || Vm3 < -1000 || Vm3 > 1000 || Vm4 < -1000 || Vm4 >
        1000) {
        return HAL_ERROR; // 错误处理，速度值超出范围
    }
    snprintf(cmd, sizeof(cmd), "$spd:%d,%d,%d,%d#", Vm1, Vm2, Vm3, Vm4);

    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 设置电机的PWM值
  * @param pwm 电机的PWM值数组，长度为4，范围为-3600到3600. 为负时表示反向转动，正时表示正向转动
  * @note 此函数会设置所有电机的PWM值，pwm[0]对应电机1，pwm[1]对应电机2，pwm[2]对应电机3，pwm[3]对应电机4
  */
HAL_StatusTypeDef MDrv_SetPWM(int16_t *pwm) {
    char cmd[50];
    snprintf(cmd, sizeof(cmd), "$pwm:%d,%d,%d,%d#", pwm[0], pwm[1], pwm[2], pwm[3]);
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 设置电机速度(单位mm/s)
  * @param mode 1: 正向速度, -1: 反向速度, 0: 停止
  * @param Vm 电机速度(单位mm/s),内部判断需为正。
  * @note 此函数会设置所有电机的速度，Vm为正时表示正向转动，负时表示反向转动，0时表示停止
  */
void MDrv_GoSpeed(int8_t mode, int32_t Vm) {
    if (Vm > 0) {
        if (mode == 1)
            MDrv_SetSpeed(Vm, Vm, Vm, Vm); // 设置所有电机速度
        else if (mode == -1)
            MDrv_SetSpeed(-1 * Vm, -1 * Vm, -1 * Vm, -1 * Vm); // 转置所有电机速度
        else if (mode == 0)
            MDrv_SetSpeed(0, 0, 0, 0); // 停止所有电机
    }
}

/**
  * @brief (bug)初始化驱动板
  * @retval HAL_StatusTypeDef 返回状态
  * @note 该函数会设置电机类型、相位线、车轮直径和速度等参数。为确保设置生效各命令延时90ms,收到HAL_OK只代表发出去不代表已经MD收到
  * @bug 1. 设置电机速度无论延迟多少都无法正确设置，可能要收到单片机回传的正确时间
  * @bug 2. 死区值需要测量得出，不能随意设置
  */
void MDrv_Init() {
    MDrv_SetPID(0.8, 0.06, 0.5);
    HAL_Delay(90);
    MDrv_SetMtype(2); // 设置为310电机
    HAL_Delay(90);
    Mdrv_SetMline(13); // 设置电机的相位线
    HAL_Delay(90);
    Mdrv_SetMphase(20); // 设置电机的减速比
    HAL_Delay(90);
    MDrv_SetWdiameter(48); // 设置车轮直径
    HAL_Delay(90);
    MDrv_SetDeadzone(1650); // 设置电机的pwm脉冲死区,需要测量得出
    HAL_Delay(90);
    MDrv_SetSpeed(0, 0, 0, 0); // 设置电机速度
    HAL_Delay(90);
}