# STM32 八路巡线小车控制

基于 **STM32F103C8T6、STM32 HAL库 ** 的四轮小车控制工程。主控采集八路灰度传感器数据，通过 PD 控制计算左右轮速度，再通过串口向独立电机驱动板发送指令。OLED 用于显示轮速、电池电压和巡线状态。使用 STM32CubeMX 或者 CLion ，在配置好的 IDE 中进行下载与调试。


## 功能与当前状态

| 模块 | 内容 | 当前状态 |
| --- | --- | --- |
| 八路灰度循迹 | 通道切换、ADC 均值采样、黑白阈值判断、误差加权、左右轮差速 | 默认启用 |
| PD 控制 | 使用通用 PID 模块，当前积分系数为 0，带输出限幅 | 用于八路循迹 |
| 丢线处理 | 全白时根据上一次有效误差选择旋转找线方向 | 已有实现，首次全白存在边界问题 |
| 电机驱动板通信 | 四路速度/PWM 指令、驱动参数设置、轮速与电池电压解析 | 主流程使用 |
| OLED | 显示四轮速度、电压、八路黑白状态；可显示偏航角 | 默认启用 |
| K230 指令接口 | 解析串口文本，调用前进、后退、旋转和停车函数 | 有解析代码；不包含 K230 端视觉程序 |
| MPU6050 | 初始化、陀螺仪校准和偏航角读取 | 默认关闭 |
| 四路循迹 | 保留早期循迹模块 | 默认关闭，需补齐引脚和构建配置后使用 |

项目采用裸机主循环和中断标志调度，未使用 RTOS。电机速度指令发送给外部驱动板，本仓库不包含该驱动板的固件。

## 工程目录

```text
C8T6_Test/
├── C8T6_Test.ioc                 # STM32CubeMX 配置
├── CMakeLists.txt                # 应用源码和依赖配置
├── CMakePresets.json             # 构建预设
├── STM32F103XX_FLASH.ld          # GCC 链接脚本
├── startup_stm32f103xb.s         # 启动文件
├── DAP.cfg                      # OpenOCD 配置，当前实际选择 ST-Link
├── Core/
│   ├── Inc/                     # 外设与主程序头文件
│   └── Src/                     # main、GPIO、ADC、DMA、I²C、串口、TIM2
├── Drivers/
│   ├── CMSIS/                   # ARM 与芯片支持文件
│   ├── STM32F1xx_HAL_Driver/     # STM32 HAL 驱动
│   └── Peter/
│       ├── Eight_LineWalking.*   # 八路循迹误差与差速控制
│       ├── Four_LineWalking.*    # 早期四路循迹模块
│       ├── Ganv_Grayscale_Sensor.c
│       ├── Ganv_Grayscale_Sensor_Config.h
│       ├── PID.*                 # PID 计算、积分与输出限幅
│       ├── MDrv.*                # 电机驱动板串口协议
│       ├── motion_control.*      # 前进、后退、转弯、旋转和停车
│       ├── MPU6050.*             # 姿态传感器驱动
│       └── Display/              # OLED、字库与 LED 模块
├── cmake/                       # 工具链与 CubeMX 生成的构建配置
├── MDK-ARM/                     # Keil 工程及相关配置
└── STM32CubeIDE/                # CubeIDE 工程及调试配置
```

入口为 `Core/Src/main.c`

## 主控与硬件接口

当前配置的系统时钟为 **72 MHz**；GCC 链接脚本配置 **64 KB Flash、20 KB RAM**。

下表来自 `Core/Inc/main.h`、外设初始化文件及灰度传感器配置。引脚按当前代码列出，实际接线需与所用板卡核对。

| 用途 | STM32 引脚 / 外设 | 当前配置 |
| --- | --- | --- |
| 八路灰度传感器模拟输出 | PA0 / ADC1_IN0 | 12 位 ADC |
| 灰度通道地址 AD0 | PB0 | GPIO 输出 |
| 灰度通道地址 AD1 | PB1 | GPIO 输出 |
| 灰度通道地址 AD2 | PA7 | GPIO 输出 |
| 电机驱动板及 MCU 间通信 | PA2 TX、PA3 RX / USART2 | 115200，8N1，接收使用空闲事件与 DMA |
| 蓝牙 / 调试输出 | PA9 TX、PA10 RX / USART1 | 9600，8N1；当前主流程未开启该串口接收 |
| OLED | PB10 SCL、PB11 SDA / I²C2 | 100 kHz，7 位地址 `0x3C` |
| MPU6050 | PB6 SCL、PB7 SDA / I²C1 | 400 kHz，模块默认关闭 |
| MPU6050 中断 | PB9 | 外部中断 |
| 循迹调度时基 | TIM2 | 按当前时钟配置每 1 ms 中断一次 |

电机编号约定为：M1 左前、M2 左后、M3 右前、M4 右后。左右轮差速通过 `Motion_SetLR_Speed()` 转换为四路速度指令。

灰度采集按三位地址逐通道切换，每个通道采样 8 次后取均值。当前 `Direction=1`，采集结果按反向顺序存放；黑白校准值保存在 `Ganv_Grayscale_Sensor.c` 的 `ganv_white`、`ganv_black` 数组中，需要按传感器、安装高度和场地重新确认。

## 运行流程

1. 初始化 HAL、系统时钟、GPIO、DMA、USART1/2、I²C1/2、ADC1 和 TIM2。
2. 初始化灰度传感器，启动 TIM2 中断，配置电机驱动板。
3. 启动 USART2 接收，请求电池电压和轮速回传，初始化循迹控制器及 OLED。
4. 主循环刷新显示、处理串口数据，并在循迹标志就绪时执行传感器采样和循迹计算。

TIM2 回调每累计 10 次中断调用一次 `Task_tick()` 并置位循迹标志，因此当前名义调度间隔为 **10 ms**。灰度驱动注释中提到的 1 ms 时基与这个实际调用位置不同，应以主程序为准。显示与通信使用的阻塞调用也会影响主循环的实际执行间隔。

循迹数据中，位值 `0` 表示黑线，`1` 表示白色。控制器对检测到黑线的通道权重求平均，经 PD 计算后调整左右轮目标速度。当前权重为：

```c
{35, 25, 15, 5, -5, -15, -25, -35}
```

## 默认配置与调参位置

### 功能开关

开关位于 `Core/Src/main.c`：

| 宏 | 默认值 | 含义 |
| --- | --- | --- |
| `Tracking8_Switch` | `1` | 八路循迹 |
| `Tracking4_Switch` | `0` | 四路循迹 |
| `OLED_Switch` | `1` | OLED 初始化与部分显示逻辑 |
| `MPU6050_Switch` | `0` | MPU6050 |
| `Debug_Switch` | `0` | 将收到的串口文本转发到蓝牙串口 |
| `Protection_Switch` | `0` | 置 1 时在应用初始化阶段停留于空循环 |

这些宏并非全部都能独立关闭对应模块。例如主循环的 `OLED_Debug(1)` 目前没有受 `OLED_Switch` 保护，关闭 OLED 时需一并检查调用处。

### 循迹与电机参数

| 参数 | 当前值 | 位置 |
| --- | --- | --- |
| 循迹基础速度 | `150`，按电机协议约定为 mm/s | `main.c` 的 `LineWalking8_Track()` 调用 |
| PD 参数 | Kp = `12.0`、Ki = `0.0`、Kd = `5.8` | `Eight_LineWalking.c` |
| 控制输出限幅 | `-400` 至 `400` | `Eight_LineWalking.c` |
| 找线旋转速度 | `120` | `Eight_LineWalking.c` 的 `RotSpeed` |
| 电机类型 | 类型 `2`，代码注释为 310 电机 | `MDrv.c` 的 `MDrv_Init()` |
| 编码器线数 / 减速比 | `13` / `20` | `MDrv_Init()` |
| 轮径 / PWM 死区 | `48 mm` / `1200` | `MDrv_Init()` |

这些是当前程序参数，不是经过本次测量确认的性能指标。更换车体、驱动板或传感器后需重新核对。当前 PID 微分项使用相邻误差之差，没有按时间归一化，修改调度周期后也应重新调参。

## 串口协议入口

接收入口为 `HAL_UARTEx_RxEventCallback()`，主循环通过 `Uart_JudgeDataType()` 分发文本。当前识别的前缀区分大小写：

| 接收前缀 | 用途 |
| --- | --- |
| `$K230:` | 解析运动指令 |
| `$MAll:` | 四轮累计编码器数据 |
| `$MTEP:` | 四轮实时编码器数据 |
| `$MSPD:` | 四轮速度数据 |
| `$Battery:` | 电池电压数据 |

K230 文本格式为 `$K230:<类型>,<结果>,<分数>#`，例如：

```text
$K230:car,F,0.95#
```

当类型为 `car` 时，结果 `F`、`B`、`L`、`R`、`S`、`SP` 分别调用前进、后退、两种方向的旋转、PWM 归零停车、目标速度归零停车。当前分数仅被解析，没有用于阈值判断。方向以 `K230_ProcessData()` 与 `Motion_Spin()` 的实际映射为准，需结合电机接线核对。

电机驱动板发送协议封装在 `MDrv.c` 中，例如四轮速度指令为 `$spd:v1,v2,v3,v4#`。K230 命令和驱动板回传目前共用 USART2 接收入口；仓库未提供独立 K230 串口接收通道或完整外部连接方案。

默认八路循迹与 K230 运动命令之间没有模式仲裁，后续循迹计算可能覆盖串口下发的运动指令。因此现有接口不能直接视为独立的遥控模式。

## 开发笔记

- [C Language in STM32：模块化开发、浮点数处理与串口调试](https://my.feishu.cn/docx/N7z5djng1o4I1KxSmz8cj39GnJe)
- [FreeRTOS：源码组织与任务通信学习笔记](https://my.feishu.cn/docx/KkENd4dZJotfppx0L64cQAOwnue)
- [电机应用：电机驱动、编码器、PID 与 MPU6050](https://my.feishu.cn/docx/KC6ldtylao174bxIx8LcJshAnrb)
