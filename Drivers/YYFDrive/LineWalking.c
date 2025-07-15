//
// Created by Petercode on 25-7-10.
//
#include "LineWalking.h"
#include "main.h"
#include "Mdrv.h"
#include "math.h"
#include "app_motion.h"
#include "usart.h"
#include "string.h" // strlen

#define High GPIO_PIN_SET
#define Low GPIO_PIN_RESET

uint8_t GetLineWalking() {
    uint8_t state = 0;
    if (HAL_GPIO_ReadPin(Track_L2X1_GPIO_Port, Track_L2X1_Pin) == High) state |= (1 << 3); // L2
    if (HAL_GPIO_ReadPin(Track_L1X2_GPIO_Port, Track_L1X2_Pin) == High) state |= (1 << 2); // L1
    if (HAL_GPIO_ReadPin(Track_R1X3_GPIO_Port, Track_R1X3_Pin) == High) state |= (1 << 1); // R1
    if (HAL_GPIO_ReadPin(Track_R2X4_GPIO_Port, Track_R2X4_Pin) == High) state |= (1 << 0); // R2
    return state;
}

/* 车体几何参数（单位 mm） */
#define TRACK_WIDTH   135.0f   // 左右轮中心距离
#define WHEEL_BASE    145.0f   // 前后轴中心距离

/**
 * @brief  对四轮差动驱动做“曲线转弯”速度分配
 * @param  Vc   车体中心线速度（mm/s）
 * @param  R    转弯半径，相对于车体中心（mm），须 > WHEEL_BASE/2 = 72.5mm
 */
static void setCurveSpeed(float R)
{
    float halfTrack = TRACK_WIDTH * 0.5f;   // 半跨距
    float halfBase  = WHEEL_BASE  * 0.5f;   // 半轴距

    // 1. 从全局 Vstd 数组读取并计算中心线速度
    float Vc = (Vstd[0] + Vstd[1] + Vstd[2] + Vstd[3]) / 4.0f;

    // 2. 角速度 ω = Vc / R
    float omega = Vc / R;

    // 3. 计算四个车轮到 ICC 的距离
    float R_fl = sqrtf((R - halfTrack)*(R - halfTrack) + halfBase*halfBase);  // 左前
    float R_fr = sqrtf((R + halfTrack)*(R + halfTrack) + halfBase*halfBase);  // 右前
    float R_rl = R_fl;  // 左后对称
    float R_rr = R_fr;  // 右后对称

    // 4. 线速度 vi = ω × Ri
    int32_t v_fl = (int32_t)(omega * R_fl);
    int32_t v_fr = (int32_t)(omega * R_fr);
    int32_t v_rl = v_fl;
    int32_t v_rr = v_fr;


    /* 限幅到驱动板支持范围 */
    #define CLAMP(x)  do { if ((x)>1000) (x)=1000; if ((x)<-1000) (x)=-1000; } while(0)
    CLAMP(v_fl); CLAMP(v_fr); CLAMP(v_rl); CLAMP(v_rr);

    /* 下发给闭环驱动 */
    MDrv_SetSpeed(v_fl, v_rl, v_fr, v_rr);
}


void LineWalking(uint8_t state, int16_t speed) {
    // 首先 Low 代表黑线 High 代表白线
    // L2   L1   R1   R2
    // Notice: 已知1.7mm的间距黑胶不会有 0011 和 1100 的情况 行进路线放置
    char txBuffer[64];

    // 拆位：提取每一位的高低状态（1或0）
    uint8_t L2 = (state >> 3) & 0x01;
    uint8_t L1 = (state >> 2) & 0x01;
    uint8_t R1 = (state >> 1) & 0x01;
    uint8_t R2 = (state >> 0) & 0x01;

    // 拼接字符串（可用 snprintf 更安全）
    sprintf(txBuffer, "$LWK:L %d:%d R%d:%d#\r\n", L2, L1, R1, R2);
    // 通过蓝牙发送（这里用 huart1 举例）
    HAL_UART_Transmit(&Bluetooth_Cfg, (uint8_t *)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);

    switch (state)
    {
        case 0b0000:  // 全部低
            // 压到黑线了，全速185度
            Motion_SpinStart(185.0f, (int16_t)((float)speed * 1.2f));

            break;
        case 0b1111:
            // 冲出去自己调节
            Motion_Backward(speed);
            break;

        case 0b0111:
            Motion_SpinStart(-25.0f,  speed);// 左偏大，需向又拉回
            break;
        case 0b1110:
            Motion_SpinStart(25.0f,  speed); // 右偏大，需向左拉回
            break;

        case 0b0001:
            // 左直角-> 转弯进入 1000
            // L2 + L1，高偏左，事实只要2度调节
            Motion_SpinStart(15.0f, (int16_t)((float)speed * 0.75f));  // 左直角
            break;
        case 0b1000:  // 右直角
            // 右直角 -> 转弯进入 0001
            Motion_SpinStart(-15.0f,  speed);
            break;

        /* impossible
        case 0b0010:
            break;
        case 0b0100:
            break;
        */

        case 0b0011: // 标准
            // 只能是左直角 -> 转弯进入 0001
            // 事实上这个的分布是  L2         L1  R1      R2
            Motion_SpinStart(-5.0f, (int16_t)((float)speed * 0.75f)); // 右直角
            break;
        case 0b1100:
            // 只能是右直角 -> 转弯进入 1000
            //事实只要2度调节
            Motion_SpinStart(5.0f, (int16_t)((float)speed * 0.75f));  // 左直角
            break;

        /* impossible
        case 0b0101:
            // L1 + R2
            break;
        case 0b1010:
            // L2 + R1
            break;

        case 0b0110:
            break;
        */
        case 0b1001:
            //  I like it
            break;

        case 0b1011:
            // 右偏大，需向左拉回 -> 1001
            Motion_SpinStart(2.0f, (int16_t)((float)speed * 0.75f)); // 右偏大，需向左拉回
            break;
        case 0b1101:
            // 左偏大，需向右拉回 -> 1001
            Motion_SpinStart(-2.0f, (int16_t)((float)speed * 0.75f)); // 右偏大，需向左拉回
            break;

        default:
            break;
    }
}
