//
// Created by Petercode on 25-7-9.
//
#include "Mdrv.h"
#include "stdio.h"
#include "string.h" // strlen
#include "usart.h"
#include <stdlib.h> // atof

/**
 * M1 -> 左上电机（小车的左前轮）
 * M2 -> 左下电机（小车的左后轮）
 * M3 -> 右上电机（小车的右前轮）
 * M4 -> 右下电机（小车的右后轮）
*/
float mall[4] = {0};
int mtep[4] = {0};

float V[4];
int32_t Vstd[4]; // 标准速度
float Voltage;

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
HAL_StatusTypeDef MDrv_SetSpeed(int32_t Vm1, int32_t Vm2, int32_t Vm3, int32_t Vm4) {
    char cmd[50];
    if (Vm1 < -1000 || Vm1 > 1000 || Vm2 < -1000 || Vm2 > 1000 || Vm3 < -1000 || Vm3 > 1000 || Vm4 < -1000 || Vm4 >
        1000) {
        return HAL_ERROR; // 错误处理，速度值超出范围
    }
    snprintf(cmd, sizeof(cmd), "$spd:%ld,%ld,%ld,%ld#", Vm1, Vm2, Vm3, Vm4);
    Vstd[0] = Vm1; // 更新全局速度数组
    Vstd[1] = Vm2;
    Vstd[2] = Vm3;
    Vstd[3] = Vm4;
    return HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 设置电机的PWM值
  * @param pwm 电机的PWM值数组，长度为4，范围为-3600到3600. 为负时表示反向转动，正时表示正向转动
  * @note 此函数会设置所有电机的PWM值，pwm[0]对应电机1，pwm[1]对应电机2，pwm[2]对应电机3，pwm[3]对应电机4
  */
HAL_StatusTypeDef MDrv_SetPWM(int32_t pwm1, int32_t pwm2, int32_t pwm3, int32_t pwm4) {
    char cmd[50];
    snprintf(cmd, sizeof(cmd), "$pwm:%d,%d,%d,%d#", pwm1, pwm2, pwm3, pwm4);
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
    sscanf(data, "%f,%f,%f,%f", &mall[0], &mall[1], &mall[2], &mall[3]);
}

void MDrv_ProcessMTEP(char *data) {
    // 处理 $MTEP 数据 代表轮子转动的实时转动编码器数据 10ms
    // 例如解析数据并存储到相应的变量中
    // 假设数据格式为 "63,64,66,66#"·
    sscanf(data, "%d,%d,%d,%d", &mtep[0], &mtep[1], &mtep[2], &mtep[3]);
}

void MDrv_ProcessMSPD(char *data) {
    // 处理 $MSPD 数据 代表轮子转动的实时速度数据
    // 例如解析数据并存储到相应的变量中
    // 假设数据格式为 "$MSPD:608.99,618.65,637.98,637.98#"

    // 大坑！
    // 只要前缀匹配并且浮点格式正确，后面的 #\r\n 会被忽略。
    // 直接 # 是不行的 应为后面有 \r\n
    // 格式字符串不要写死这些尾部符号，只用 %f 接浮点数据，后面多余字符就会被自动忽略
    // 可以通过 sscanf 的返回值来判断是否解析成功

    // 期望格式："608.99,618.65,637.98,637.98#\r\n"
    char *ptr = data;
    char temp[16];
    int i = 0;

    while (i < 4 && *ptr != '\0') {
        int j = 0;

        // 提取当前字段数字，直到逗号或结束符
        while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.' || *ptr == '-') {
            if (j < sizeof(temp) - 1)
                temp[j++] = *ptr;
            ptr++;
        }
        temp[j] = '\0';

        // 转换为 float 并存入 V[i]
        if (j > 0)
            V[i++] = atof(temp);

        // 跳过逗号（如果有）
        if (*ptr == ',') ptr++;
    }

    // 2. 准备一个数组来存放PID计算出的新速度
    // int32_t NewSpeed[4];

    // 3. 调用PID模块来计算新速度
    //    我们将期望值(Vstd)和实际值(V)传递给它，它会把结果填入NewSpeed
    // PID_Run_All_Motors(Vstd, V, NewSpeed);

    // 4. 使用计算出的新速度去设置电机，完成闭环
    // MDrv_SetSpeed(NewSpeed[0], NewSpeed[1], NewSpeed[2], NewSpeed[3]);

    // 这里的函数逻辑是错误的，不能通过比例的这种方式来修正速度，而是应该要引入闭环控制 PID ——6050
    // int i = 0 , needUpdate = 0;
    // for (i = 0; i < 4; i++) {
    //     // 将速度转换为绝对值
    //     if (Vstd[i] != 0 && fabsf((float)Vstd[i]-V[i]) > 5.0f && fabsf((float)Vstd[i]- V[i]) < 10.0f) {
    //         needUpdate = 1; // 如果速度变化超过，标记需要更新
    //         V[i] = (float)Vstd[i] * (float)Vstd[i] / V[i];
    //     }
    // }
    // // 如果速度变化超过阈值，更新V数组
    //   // memcpy(V, mspd, sizeof(V));
    // if (needUpdate)
    //     MDrv_SetSpeed(V[0], V[1], V[2], V[3]); // 更新电机速度
}

void MDrv_ProcessVol(char *data) {
    // data 示例："5.12V#\r\n"
    char temp[16] = {0};
    int i = 0;

    // 提取数字字符直到遇到非数字/小数点（如 'V' 或 '#'）
    while ((*data >= '0' && *data <= '9') || *data == '.' || *data == '-') {
        if (i >= sizeof(temp) - 1) break;  // 防止溢出
        temp[i++] = *data++;
    }
    temp[i] = '\0';

    // 转换为 float
    if (i > 0) {
        Voltage = atof(temp);
    }
}

/** @brief 读取电池电压
  * @note 发送指令 "$read_vol#" 仅发送不解析数据
  * @retval void
  */
void Mdrv_ReadVol() {
    char cmd[] = "$read_vol#"; // 发送指令
    HAL_UART_Transmit(&UMUART_Cfg, (uint8_t *) cmd, strlen(cmd), Uart_MaxWait);
}

/**
  * @brief 设置电机速度(单位mm/s)
  * @param mode 1: 正向速度, -1: 反向速度, 0: 停止
  * @param Vm 电机速度(单位mm/s),内部判断需为正。
  * @note 此函数会设置所有电机的速度，Vm为正时表示正向转动，负时表示反向转动，0时表示停止
  */
void MDrv_GoSpeed(int8_t mode, int32_t Vm) {
    if (Vm >= 0) {
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
    // P:1.9,I:0.2,D:0.8
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
    MDrv_SetPWM(0, 0, 0, 0); // 设置电机的PWM值为0，确保电机停止
    // MDrv_SetSpeed(0, 0, 0, 0); // 设置电机速度
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