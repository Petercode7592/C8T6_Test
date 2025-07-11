//
// Created by Petercode on 25-7-10.
//
#include "LineWalking.h"
#include "main.h"

#define Left1 Track_L1X2_Pin
#define Left2 Track_L2X1_Pin
#define Right1 Track_R1X3_Pin
#define Right2 Track_R2X4_Pin

#define High GPIO_PIN_SET
#define Low GPIO_PIN_RESET

void GetLineWalking(GPIO_PinState *L1,GPIO_PinState *L2, GPIO_PinState *R1, GPIO_PinState *R2)
{
    *L1 = HAL_GPIO_ReadPin(Track_L1X2_GPIO_Port, Left1);
    *L2 = HAL_GPIO_ReadPin(Track_L2X1_GPIO_Port, Left2);
    *R1 = HAL_GPIO_ReadPin(Track_R1X3_GPIO_Port, Right1);
    *R2 = HAL_GPIO_ReadPin(Track_R2X4_GPIO_Port, Right2);
}

void LineWalking()
{
    GPIO_PinState L1, L2, R1, R2;
    GetLineWalking(&L1, &L2, &R1, &R2);
    if((L1 == Low || L2 == Low) && R2 == Low)        // 左大弯
    {
        // Car_SpinLeft(7000, 7000);
        HAL_Delay(80);
    }
    else if (L1 == Low && (R1 == Low || R2 == Low))  // 右大弯
    { 
        // Car_SpinRight(7000, 7000);
        HAL_Delay(80);
    }  
    else if(L1 == Low)                            // 左最外侧检测
    {  
        // Car_SpinLeft(6800, 6800);
        HAL_Delay(10);
    }
    else if (R2 == Low)                           // 右最外侧检测
    {  
        // Car_SpinRight(6800, 6800);
        HAL_Delay(10);
    }
    else if (L2 == Low && R1 == High)             // 中间黑线上传感器微调车左转
    {   
        // Car_Left(6500);
    }
    else if (L2 == High && R1 == Low)                // 中间黑线上传感器微调车右转
    {   
        // Car_Right(6500);
    }
    else if(L2 == Low && R1 == Low)                 // 都是黑色，加速前进
    {  
        // Car_Run(7200);
    }	

}