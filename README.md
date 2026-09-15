# STM32 自平衡小车（平衡车开源固件）

基于 **STM32F103C8** 的两轮自平衡小车固件，使用 **MPU6050 + DMP** 做姿态解算，**双闭环 PID**（直立环 + 速度环）驱动两路直流减速电机，板载 **SSD1306 OLED** 实时显示姿态与速度。

![MCU](https://img.shields.io/badge/MCU-STM32F103C8-blue)
![IDE](https://img.shields.io/badge/IDE-Keil%20MDK5-green)
![Library](https://img.shields.io/badge/SPL-STM32F10x%20V3.5.0-orange)
![Status](https://img.shields.io/badge/status-仅平衡%20%2F%20无转向环-yellow)

---

## 目录

- [功能特性](#功能特性)
- [控制原理](#控制原理)
- [硬件平台](#硬件平台)
- [引脚连接](#引脚连接)
- [目录结构](#目录结构)
- [快速开始](#快速开始)
- [参数整定](#参数整定)
- [可调参数速查](#可调参数速查)
- [已知问题与注意事项](#已知问题与注意事项)
- [二次开发建议](#二次开发建议)
- [版权与致谢](#版权与致谢)

---

## 功能特性

- ✅ **MPU6050 + DMP 硬件姿态解算**：直接输出 roll / pitch / yaw 欧拉角（精度 0.1°），免去软件互补滤波调参
- ✅ **串级 PID 控制**：直立环（PD）+ 速度环（PI），输出叠加后限幅送给电机
- ✅ **中断驱动**：控制算法在 MPU6050 INT 引脚（PA12）外部中断中执行，控制周期稳定，不占用主循环
- ✅ **编码器测速**：TIM2 / TIM3 硬件编码器接口模式（TI12），四倍频计数
- ✅ **OLED 实时显示**：roll 角、yaw 角、速度值；初始化阶段显示 MPU6050 / DMP 自检状态与错误码
- ✅ **安全保护**：倾角超过 ±30° 判定摔倒，立即关闭电机并清零速度积分与编码器计数
- ✅ **软件 I2C 驱动**：MPU6050 与 OLED 各用一组软件 I2C，不占用硬件 I2C 外设，引脚可自由改动

> ⚠️ 本版本**只实现了直立环 + 速度环，没有转向环**，小车可以稳定直立、前后平衡，但不能遥控转向。需要转向功能请参考[二次开发建议](#二次开发建议)。

---

## 控制原理

控制算法位于 `HARDWARE/MPU6050/mpuexti.c` 的 `EXTI15_10_IRQHandler()` 中，由 MPU6050 的 INT 引脚触发（DMP 数据就绪，约 100 Hz，即控制周期 ≈ 10 ms）。

```
   ┌──────────────────────────── 控制周期 ≈ 10ms（MPU6050 INT → EXTI12）────────────────────────────┐
   │                                                                                                │
   │  设定值 0 ──►(－)──► 直立环 PD  (Kp=-420, Ki=0, Kd=-2000)  ──┐                                 │
   │              ▲                                               ├──►(+)──► 限幅 ±7000 ──► SETPWM ──┼──► 电机
   │   roll ──────┘                                               │                                 │
   │   (DMP 解算)         速度环 PI  (VKp=+190, VKi=0.95) ────────┘                                 │
   │                            ▲                                                                   │
   │                            └── 编码器 TIM2/TIM3 脉冲数                                          │
   │                                (一阶低通 α=0.3，积分限幅 ±3000)                                 │
   └────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**直立环（PD）** —— 让车身立起来

```c
err          = roll - zhongzhi;          // 误差 = 测量角度 - 机械平衡点
err_difference = err - last_err;         // 微分项
输出 = Kp*err + Ki*err_sum + Kd*err_difference;
```

**速度环（PI）** —— 抑制小车缓慢漂移

```c
filt_velocity = 0.3*velocity + 0.7*last_filt_velocity;  // 一阶低通滤波
velocity_sum += filt_velocity;                          // 积分累加
I_xianfu(3000);                                         // 积分限幅
输出 = VKp*filt_velocity + VKi*velocity_sum;
```

两环输出相加后经 `PWM_Xianfu(7000, &PWM)` 限幅，再由 `SETPWM()` 写入 TIM1 的 CH1 / CH4。

**关于 PWM 反相**：TIM1 是高级定时器，部分通道输出存在反相（示波器可观测），因此代码中左轮写 `7200-PWM`、右轮直接写 `PWM`，见 `HARDWARE/MOTOR/motor.c`。

**安全保护**（`USER/main.c` 主循环）：

```c
if(roll < -30 || roll > 30) {   // 判定摔倒
    motor_flag = 0;             // 关闭电机
    velocity_sum = 0;           // 速度积分清零
    TIM_SetCounter(TIM3, 0);    // 编码器计数清零
    TIM_SetCounter(TIM2, 0);
} else motor_flag = 1;
```

---

## 硬件平台

| 项目 | 规格 |
|---|---|
| 主控 | STM32F103C8（Cortex-M3，64 KB Flash / 20 KB SRAM） |
| 系统时钟 | 72 MHz（HSE 8 MHz × PLL9，`system_stm32f10x.c` 中已启用 `SYSCLK_FREQ_72MHz`） |
| 姿态传感器 | MPU6050（六轴，陀螺仪 ±2000 dps，加速度 ±2 g，片内 DMP） |
| 显示 | SSD1306 OLED 128×64，I2C 接口 |
| 电机 | 两路直流减速电机 + 双 H 桥驱动（如 TB6612 / L298N） |
| 反馈 | 两路增量式霍尔编码器（接定时器编码器接口） |
| 开发环境 | Keil MDK 5（ARM Compiler 5，工程未启用 AC6）+ STM32F10x 标准外设库 V3.5.0 |
| 下载调试 | ST-Link / J-Link（**必须使用 SWD**，见注意事项 5） |

---

## 引脚连接

| 功能 | 引脚 | 外设 / 模式 | 说明 |
|---|---|---|---|
| 左电机 PWM | PA8 | TIM1_CH1，复用推挽 | PWM 频率 10 kHz |
| 右电机 PWM | PA11 | TIM1_CH4，复用推挽 | PWM 频率 10 kHz |
| 右电机方向 | PB12 / PB13 | 推挽输出 | 正 / 反转 |
| 左电机方向 | PB14 / PB15 | 推挽输出 | 正 / 反转 |
| 左编码器 | PA6 / PA7 | TIM3 编码器模式 TI12 | 输入浮空，IC 滤波 10 |
| 右编码器 | PA0 / PA1 | TIM2 编码器模式 TI12 | 输入浮空，IC 滤波 10 |
| MPU6050 SCL | PB8 | 软件 I2C | 延时 2 µs |
| MPU6050 SDA | PB9 | 软件 I2C | 双向（动态切换输入/输出） |
| MPU6050 AD0 | PA15 | 推挽输出 | 代码拉低 → 器件地址 `0x68` |
| MPU6050 INT | PA12 | EXTI12，上拉输入，下降沿触发 | 抢占优先级 0（分组 2） |
| OLED SDA | PB4 | 软件 I2C | 需 `GPIO_Remap_SWJ_JTAGDisable` |
| OLED SCL | PB5 | 软件 I2C | 从机地址 `0x78` |
| 调试串口 | PA9 / PA10 | USART1 | **当前未使用** |

> **注意**：PA15 默认是 JTAG 的 JTDI，工程中已通过 `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)` 关闭 JTAG（保留 SWD），因此下载器必须走 SWD 接口。

---

## 目录结构

```
平衡车开源/
├── USER/                    应用层与工程文件
│   ├── main.c               主程序：初始化 + 安全保护 + 状态显示
│   ├── stm32f10x_it.c/.h    中断服务函数模板
│   ├── stm32f10x_conf.h     标准外设库裁剪配置
│   ├── 平衡车.uvprojx        Keil MDK 工程文件 ★ 打开这个
│   ├── 平衡车.uvoptx         Keil 调试器设置（含 J-Link 配置）
│   └── DebugConfig/          调试器配置
├── HARDWARE/                硬件驱动层
│   ├── MOTOR/               电机驱动
│   │   ├── motor.c/.h       PWM 输出、正反转控制、编码器读取
│   │   └── timer.c/.h       TIM1 PWM 初始化、TIM2/TIM3 编码器模式初始化
│   ├── MPU6050/             姿态传感器
│   │   ├── mpu6050.c/.h     寄存器读写、初始化、DMP 初始化与欧拉角读取
│   │   ├── mpuiic.c/.h      MPU6050 软件 I2C（PB8/PB9）
│   │   ├── mpuexti.c/.h     外部中断初始化 ★ 控制算法在此
│   │   └── eMPL/            InvenSense 官方 DMP 运动驱动库（移植到 STM32F1）
│   ├── OLED/                显示驱动
│   │   ├── oled.c/.h        显示字符串 / 数字 / 角度 / 速度
│   │   ├── oled_i2c.c/.h    OLED 软件 I2C（PB4/PB5）
│   │   └── codetab.h        ASCII 与汉字点阵字库
│   └── PID/                 控制算法
│       └── pid.c/.h         直立环 PD、速度环 PI、积分限幅
├── ELSE/                    通用工具层
│   ├── delay/               SysTick 微秒 / 毫秒延时
│   ├── sys/                 位带操作宏（PAout / PBin 等）
│   └── usart/               串口初始化（当前未使用）
├── STM32F10x_FWLib/         ST 标准外设库 V3.5.0（inc + src）
├── CORE/                    CMSIS Cortex-M3 内核文件与启动文件
├── OBJ/                     编译输出目录（平衡车.hex 生成于此）
└── keilkilll.bat            一键清理 Keil 中间文件
```

---

## 快速开始

```bash
git clone https://github.com/Luoyu132/STM32-BalanceCar.git
```

> 仓库仅包含源码与工程文件，**不含编译产物**（`OBJ/` 目录会在首次编译后自动生成）。

### 1. 准备环境

- **Keil MDK 5**，并安装 **ARM Compiler 5**（本工程未启用 AC6，需在 Keil 中装 Legacy 支持包）
- **STM32F1xx_DFP** 器件支持包（提供 `STM32F103C8` 器件定义）
- 下载器驱动（ST-Link 或 J-Link）

### 2. 编译

```
打开  USER/平衡车.uvprojx  →  Build (F7)
```

工程已配置好头文件搜索路径与宏定义，无需额外设置：

| 配置项 | 值 |
|---|---|
| Device | STM32F103C8 |
| Define | `STM32F10X_MD, USE_STDPERIPH_DRIVER` |
| 优化等级 | Level 1 |
| 输出目录 | `..\OBJ\` |
| 生成 HEX | 已开启 → `OBJ/平衡车.hex` |

### 3. 下载

使用 **SWD 接口**下载 `OBJ/平衡车.hex`（ST-Link Utility / J-Flash / Keil 直接下载均可）。

### 4. 上电自检

OLED 屏幕上会依次显示：

```
By: WangGuanNan   →   QQ: 1501451224   →   2021/7/11      （作者信息，各 1 秒）
MPU6050 OK!                                                （传感器通信正常）
DMP ing...  Attempts: n  Error: e                          （DMP 初始化，失败会重试）
DMP OK! __WGN
```

随后进入正常界面，实时刷新 `roll`、`yaw` 与速度值。若 DMP 一直重试，检查 I2C 接线（PB8/PB9）、AD0 电平与供电。

### 5. 清理中间文件

```
双击运行 keilkilll.bat
```

---

## 参数整定

所有 PID 参数集中在 `USER/main.c` 顶部：

```c
float Kp = -420, Ki = 0, Kd = -2000;   // 直立环 PD 参数（调完速度环后精调）
float VKp = +190, VKi = 0.95;          // 速度环 PI 参数
float zhongzhi = 0;                    // roll 理论值（小车平衡时的机械中值角度）
```

**推荐整定顺序：**

1. **先只开直立环**：把 `VKp = VKi = 0`，`Kp` 从 0 逐步加大，直到车身能大致立住并出现低频抖动
2. **加微分抑制抖动**：逐步加大 `Kd`（负值加大绝对值），抖动明显减弱即合适；Kd 过大会引入高频噪声颤振
3. **再加速度环**：恢复 `VKp`，从小值逐步加大以抑制小车缓慢漂移；再给少量 `VKi` 消除稳态误差
4. **精调**：速度环加完后，回头微调 `Kp` / `Kd` 使直立更硬朗
5. **标定机械中值**：手持小车找到它自然平衡的角度，把该角度写入 `zhongzhi`

**关于极性：** 代码中 Kp、Kd 为负值，是**由电机转向与传感器安装方向共同决定的**。如果你重新焊接电机线或改变 MPU6050 安装方向，极性会反过来，此时小车会一上电就朝一侧加速倒下——把 `Kp`、`Kd`（以及 `VKp`、`VKi`）的符号取反即可。

**关于直立环 Ki：** 直立环中 `Ki` 保持为 0（速度环已经承担了消除漂移的职责），代码中虽保留了积分项接口但不建议启用。

---

## 可调参数速查

| 参数 | 位置 | 当前值 | 说明 |
|---|---|---|---|
| 直立环 `Kp` / `Ki` / `Kd` | `USER/main.c` | −420 / 0 / −2000 | PD 直立控制 |
| 速度环 `VKp` / `VKi` | `USER/main.c` | +190 / 0.95 | PI 速度控制 |
| 平衡点 `zhongzhi` | `USER/main.c` | 0 | 机械中值角度 |
| 倒地判定角 | `USER/main.c` | ±30° | 超限即关闭电机 |
| 速度一阶滤波系数 `a` | `HARDWARE/PID/pid.c` | 0.3 | 越大越不滤波 |
| 速度积分限幅 | `HARDWARE/PID/pid.c` | ±3000 | `I_xianfu(3000)` |
| 输出 PWM 限幅 | `HARDWARE/MPU6050/mpuexti.c` | ±7000 | `PWM_Xianfu(7000, &PWM)` |
| PWM 频率 | `HARDWARE/MOTOR/motor.c` | 10 kHz | `TIM1_PWM_Init(7200-1, 1-1)` |
| 编码器定时器 | `HARDWARE/MOTOR/timer.c` | ARR = 65535，PSC = 0 | TI12 模式 |
| DMP / 采样率 | `HARDWARE/MPU6050/eMPL/inv_mpu.h` | 100 Hz | `DEFAULT_MPU_HZ`，决定控制周期 |
| 陀螺仪量程 | `HARDWARE/MPU6050/mpu6050.c` | ±2000 dps | `MPU_Set_Gyro_Fsr(3)` |
| 加速度量程 | `HARDWARE/MPU6050/mpu6050.c` | ±2 g | `MPU_Set_Accel_Fsr(0)` |
| OLED 对比度 | `HARDWARE/OLED/oled.c` | 0xFF | `0x81` 指令后的参数 |

---

## 已知问题与注意事项

1. **无转向环**：仅直立环 + 速度环，小车不能转向。转向环（yaw 环）需要在两轮上叠加差速输出，代码中未实现。
2. **串口未使用且缺少中断服务函数**：`usart1_init()` 使能了 USART1 接收中断，但 `main()` 从未调用它（链接后的 map 文件显示该函数已被链接器移除：`Removing usart.o(i.usart1_init), (168 bytes)`），且工程内没有用户实现的 `USART1_IRQHandler`——中断向量指向的是启动文件里的弱定义默认处理函数。若要接蓝牙或上位机调参，需要自行补上中断服务函数并调用初始化。
3. **`MPU_Init()` 中的时钟使能笔误**：配置 PA15（AD0）前使能的是 `RCC_APB2Periph_GPIOB` 时钟，却调用了 `GPIO_Init(GPIOA, ...)`。目前能正常工作仅仅是因为 `motor_init()` 在它之前已使能了 GPIOA 时钟，属于隐藏的顺序依赖，建议修正为 `RCC_APB2Periph_GPIOA`。
4. **未使用的声明**：`HARDWARE/MOTOR/motor.h` 中声明了 `motor_enable(float pitch)` 但未实现；`HARDWARE/PID/pid.h` 中 `I_xianfu()` 的注释写作「pwm 限幅」，实际限幅对象是速度环积分量。
5. **下载必须用 SWD**：PA15 被用作 MPU6050 的 AD0 控制脚，工程已关闭 JTAG。若用 JTAG 下载会失败。
6. **Keil 的 Xtal 设置与晶振不一致**：工程 Target 中 Xtal 字段为 12 MHz，而 `USER/stm32f10x.h` 中 `HSE_VALUE` 按 8 MHz（MD 系列默认值）配置、PLL ×9 得到 72 MHz。这不影响实际运行（时钟由代码配置），只影响 Keil 仿真/下载时的时间估算，建议按自己板上的晶振核对。
7. **延时精度依赖 `SystemCoreClock`**：`ELSE/delay/delay.c` 使用 SysTick 且以 `SystemCoreClock` 换算，若改用其他主频（如 24/48 MHz），需同步修改 `system_stm32f10x.c` 中的 `SYSCLK_FREQ_*` 宏。
8. **DMP 初始化会阻塞**：`DMP_Init()` 在初始化失败时以 `while` 死循环重试并刷新 OLED，上电时必须接好传感器，否则程序停在初始化阶段。
9. **仓库内的中间文件**：`HARDWARE/OLED/` 下有若干 Keil 遗留的 `codetab.h~RF*.TMP` 临时文件，`OBJ/` 内有编译产物（约 28 MB）。这些已写入 `.gitignore`，提交前建议用 `keilkilll.bat` 清理。

---

## 二次开发建议

- **加转向环**：读取 DMP 的 yaw 角或 z 轴角速度，做 PD 运算后以「左轮 + 输出 / 右轮 − 输出」的差速形式叠加到 `SETPWM()` 之前
- **加串口调参**：补全 `USART1_IRQHandler()`，用蓝牙模块接收上位机下发的 PID 参数，实现在线整定（代码中已预留 `usart1_init(u32 bound)`）
- **改进控制周期**：当前控制算法跑在 MPU6050 的外部中断里，周期受姿态更新率约束。可改为定时器固定周期中断，把姿态更新与控制解耦
- **改用硬件 I2C**：两路软件 I2C 占用 CPU 且延时敏感，可迁移到硬件 I2C 以提高可靠性
- **速度换算为真实值**：当前速度是编码器脉冲数，可用「脉冲数 × 轮周长 / 每圈脉冲数 / 周期」换算成 m/s 后再进 PID，便于跨车型移植参数
- **增加低电量保护与蜂鸣器提示**：读取电池电压并在低压时停机报警

---

## 版权与致谢

本仓库为个人学习项目，**仅供学习与交流使用**。

| 组成部分 | 来源 / 版权 |
|---|---|
| 应用层、电机、OLED、PID 等自写代码 | 原作者 **WangGuanNan**（QQ：1501451224），创建于 2021/7/11，版本 V1.0「仅平衡」 |
| `HARDWARE/MPU6050/`（含 `mpuiic`、`mpu6050` 的移植与引脚宏） | 参考 **正点原子 ALIENTEK**（www.openedv.com），版权归广州市星翼电子科技有限公司所有 |
| `HARDWARE/MPU6050/eMPL/` | **InvenSense** 官方 eMPL 运动驱动库，版权归 InvenSense 所有 |
| `STM32F10x_FWLib/`、`CORE/`、`USER/stm32f10x*.c/h` | **STMicroelectronics** 标准外设库 V3.5.0 与 CMSIS，遵循 ST 原始许可条款 |

原始代码文件头部的声明为「本程序只供学习使用，未经作者许可，不得用于其它任何用途」，请遵守上述各方的原始许可条款；**如需商用，请联系相应版权所有者获取授权**。

本仓库由 **Luoyu132**（https://github.com/Luoyu132）整理与维护。如果这个项目对你有帮助，欢迎点一个 ⭐ Star。
