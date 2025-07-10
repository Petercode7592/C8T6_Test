//
// Created by Petercode on 25-7-9.
//
#include "Mdrv.h"
#include "stdio.h"
#include "string.h" // strlen
#include "usart.h"
#include "math.h"   // fabs

float mall[4] = {0};
int mtep[4] = {0};
float mspd[4] = {0};

extern float V[4];

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
  * @note 注意使用 printf 搭配 %f 需要额外进行库的链接 ！
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
    snprintf(cmd, sizeof(cmd), "$spd:%ld,%ld,%ld,%ld#", Vm1, Vm2, Vm3, Vm4);
    V[0] = Vm1; // 更新全局速度数组
    V[1] = Vm2;
    V[2] = Vm3;
    V[3] = Vm4;
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
  * @brief 上传数据
  * @param mall 是否上传轮子转动的总编码器数据
  * @param mtep 是否上传10ms转动的实时转动编码器数据(实际值)
  * @param mspd 是否上传轮子转动的实时速度数据(计算值)
  * @note mall, mtep, mspd分别为0或1，1代表回传该部分的数据
  * @retval HAL_StatusTypeDef 返回状态
  */
HAL_StatusTypeDef Mdrv_UploadData(uint8_t mall,  uint8_t mtep, uint8_t mspd) {
    // 发送上传数据指令
    // 假设mall, mtep, mspd是需要上传的数据 相应变量赋值1代表回传该部分的数据
    char cmd[50];
    snprintf(cmd, sizeof(cmd), "$upload:%d,%d,%d#", mall, mtep, mspd);
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}


void MDrv_ProcessMALL(char *data) {
    // 处理 $MALL 数据 代表轮子转动的总编码器数据
    // 例如解析数据并存储到相应的变量中
    // 假设数据格式为 "$MAll:23427,23801,24543,24453#"
    sscanf(data, "$MAll:%f,%f,%f,%f", &mall[0], &mall[1], &mall[2], &mall[3]);
}

void MDrv_ProcessMTEP(char *data) {
    // 处理 $MTEP 数据 代表轮子转动的实时转动编码器数据 10ms
    // 例如解析数据并存储到相应的变量中
    // 假设数据格式为 "$MTEP:63,64,66,66#"·
    sscanf(data, "$MTEP:%d,%d,%d,%d", &mtep[0], &mtep[1], &mtep[2], &mtep[3]);
}

void MDrv_ProcessMSPD(char *data) {
    // 处理 $MSPD 数据 代表轮子转动的实时速度数据
    // 例如解析数据并存储到相应的变量中
    // 假设数据格式为 "$MSPD:608.99,618.65,637.98,637.98#"
    sscanf(data, "$MSPD:%f,%f,%f,%f", &mspd[0], &mspd[1], &mspd[2], &mspd[3]);
    int i = 0 , needUpdate = 0;
    for (i = 0; i < 4; i++) {
        // 将速度转换为绝对值
        if (fabs(mspd[i]-V[i]) > 2.0f && fabs(mspd[i]-V[i]) < 10.0f && V[i] != 0) {
            needUpdate = 1; // 如果速度变化超过0.1，标记需要更新
            V[i] = V[i] + 0.5 * (V[i] - mspd[i]); // 更新V数组
        }
    }
    // 如果速度变化超过阈值，更新V数组
      // memcpy(V, mspd, sizeof(V));
    if (needUpdate)
        MDrv_SetSpeed(V[0], V[1], V[2], V[3]); // 更新电机速度
}


void MDrv_JudgeDataType(char *Udata) {
    if (strncmp(Udata, "$MALL:", 6) == 0) {
        MDrv_ProcessMALL(Udata + 6);
    } else if (strncmp(Udata, "$MTEP:", 6) == 0) {
        MDrv_ProcessMTEP(Udata + 6);
    } else if (strncmp(Udata, "$MSPD:", 6) == 0) {
        MDrv_ProcessMSPD(Udata + 6);
    } else if (strncmp(Udata, "$Battery:", 9) == 0) {
        // Battery 电池电压数据
    }
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
    // MDrv_SetPID(0.8, 0.06, 0.5);
    // HAL_Delay(90);
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

// 可以使用DMA完成
// char* MDrv_ReadFlash(char *charArray) {
//     // 发送指令
//     HAL_StatusTypeDef status = HAL_UART_Transmit(&UMUART_Cfg, (uint8_t*)"$read_flash#", 12, Uart_MaxWait);
//     if (status != HAL_OK)
//       return NULL;
//
//     uint32_t index = 0;
//     char ch = 0;
//
//     while (index < 63)  // 预留1字节给 '\0'
//     {
//       status = UART_ReceiveOnebit(1, (uint8_t*)&ch);
//       if (status != HAL_OK)
//         return NULL;  // 接收失败或超时，返回NULL
//
//       charArray[index++] = ch;
//
//       if (ch == '#')  // 遇到结束符，结束接收
//         break;
//     }
//
//     charArray[index] = '\0';  // 字符串结束符
//     return charArray;
// }