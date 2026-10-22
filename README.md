# PID_ctrl

基于 **TI MSPM0G3507**（LP-MSPM0G3507 LaunchPad）的循迹小车 + 钢球位置串级控制工程。
裸机（无 RTOS）实现，使用 MSPM0 DriverLib + SysConfig 生成外设初始化代码，
开发环境为 TI Code Composer Studio（Theia 内核版本，`ticlang` 编译器）。

---

## 1. 功能概览

- **循迹小车**：8 路灰度传感器加权求误差 → 循迹 PID → 左右轮差速 → 速度闭环。
  支持弯道自适应降速、直角弯识别与原地转向、丢线保护、航向保持。
- **钢球位置控制**：视觉上位机通过 UART 发送钢球位置（mm），
  外环把位置误差映射成滑轨目标倾角，内环把倾角误差映射成步进电机转速，
  再由步进电机驱动滑轨倾斜，使钢球稳定到目标位置。
- **路线状态机**：起点 → 中间目标（61.5 mm）→ 终点目标（139 mm），
  每一步都有到位判据（位置容差 + 速度门限 + 稳定时间）。
- **车载交互**：SSD1306 OLED 显示运行状态/参数，4 个按键完成启停、
  测试菜单切换、在线 PID 调参、MPU 重新标定。
- **姿态与补偿**：MPU6050 陀螺仪积分航向角，并提供加速度前馈、
  坡度补偿、IMU 微调，用于抑制小车加减速时钢球的惯性滑动。

## 2. 硬件平台与外设

| 功能 | 外设 / 引脚 | 说明 |
| --- | --- | --- |
| MCU | MSPM0G3507 | 80 MHz Cortex-M0+，LP-MSPM0G3507 |
| 直流电机 PWM | `DC_MOTOR_PWM`（TIMG8，10 kHz） | C1 → 左轮，C0 → 右轮 |
| 直流电机方向 | `DC_MOTOR` GPIO | AIN1/AIN2、BIN1/BIN2 |
| 编码器 | `DC_ENCODER` GPIO | L_ENCA/L_ENCB、R_ENCA/R_ENCB 正交计数 |
| 步进电机 | `DCC_100_PWM1/2`（TIMG0 等） | 硬件 PWM 产生脉冲，频率即转速 |
| 步进方向 | `STEP_MOTOR` GPIO | DIR/DCY/SLP/RST × 2 路 |
| 舵机 | `SERVO`（TIMG7，50 Hz） | 上电后输出 1.5 ms 脉宽 |
| 视觉串口 | `vision_proto`（UART0，115200，PA10/PA11） | 接收上位机帧，回 ACK |
| 灰度传感器 | `GRAY_SENSOR` 8 路 GPIO | L4 L3 L2 L1 R1 R2 R3 R4（`1` = 黑线） |
| 六轴模块 | `I2C_MPU6050` | 航向角积分、加速度前馈 |
| OLED | `I2C_OLED` | SSD1306 128×64 |
| 按键 | `KEY` 4 路 GPIO | KEY1~KEY4，支持短按/长按 |

> 完整引脚映射见 [`main.syscfg`](main.syscfg)。

## 3. 软件架构

### 3.1 时基与任务调度

SysTick 1 ms 中断只做计数和置标志，耗时的 I2C/OLED 操作全部放在主循环，
避免阻塞 1 ms 节拍：

| 周期 | 任务 | 所在位置 |
| --- | --- | --- |
| 1 ms | 系统时基、任务标志 | `SysTick_Handler` |
| 5 ms | 串级内环（角度环） | `CascadeCtrl_Task5ms` / `BallRouteCtrl_Task5ms` |
| 10 ms | 速度环、循迹、按键扫描、MPU 积分、球位置外环 | `SpeedCtrl_Task10ms` / `Yaw_Tick` / `Track` |
| 主循环 | 视觉帧解析、OLED 刷新、按键菜单 | `main()` 的 `while (1)` |

### 3.2 控制链路

```
视觉上位机 ──UART0 115200──▶ vision_proto（帧解析 + 4 帧环形队列）
                                  │ 0x06: distance_mm, valid
                                  ▼
                        BallRouteCtrl（路线状态机，5 ms）
                                  │ 目标位置 mm
                                  ▼
                CascadeCtrl 外环（位置 → 滑轨目标角，5/10 ms）
                  死区 / 条件积分 / 加速度前馈 / 坡度补偿 / IMU 微调
                                  │ 目标角 °
                                  ▼
                CascadeCtrl 内环（角度 → 步进 RPM，5 ms）
                                  ▼
                     stepper_axis / step_motor（脉冲频率）
                                  ▼
                        步进电机 → 滑轨倾角 → 钢球

8 路灰度 ──▶ Track_ApplyTarget（10 ms，循迹 PID + 差速）
                                  │ 左右轮目标速度
                                  ▼
              SpeedCtrl 速度环 PID + S 曲线规划（10 ms）
                                  │ PWM
                                  ▼
                    dc_motor（TIMG8）──▶ 直流电机
                    ▲
              编码器 5 周期滑窗测速（10 ms）

MPU6050 ──▶ Yaw（Z 轴积分、温补、故障降频）──▶ 航向保持 / 前馈
OLED ◀── 主界面 / 测试菜单 / 调参界面
```

### 3.3 视觉串口协议

帧格式（大端）：

| 字节 | 内容 |
| --- | --- |
| 0 | 帧头 `0xAA` |
| 1 | 帧头 `0x55` |
| 2 | `cmd` |
| 3 | `len`（≤ 32） |
| 4 … 4+len-1 | `payload` |
| 4+len | 校验和 = `(cmd + len + Σpayload) & 0xFF` |

| cmd | 名称 | payload | 说明 |
| --- | --- | --- | --- |
| `0x01` | `MOVE` | x(2), y(2) | 目标坐标 |
| `0x02` | `STOP` | — | 仅回 ACK，启停实际由按键控制 |
| `0x03` | `ERR` | err_x(2, 有符号), err_y(2, 有符号) | 位置误差 |
| `0x06` | `DISTANCE_STATE` | ball_x(2), ball_y(2), distance_mm(2), valid(1) | 钢球到目标距离，串级控制用该字段 |
| `0x81` | `ACK` | acked_cmd(1) | 对每条命令的应答 |

接收缓冲为 4 帧环形队列，中断里只做逐字节状态机解析（`vp_input_byte`），
主循环每轮最多处理 4 帧，避免通信量突增时卡住控制任务。

## 4. 模块说明

| 文件 | 说明 |
| --- | --- |
| `main.c` | 初始化、SysTick 调度、视觉帧分发、按键总入口 |
| `vision_proto.c/h` | UART 帧解析/打包、ACK、校验和 |
| `uart.c/h` | UART0 发送与接收中断 |
| `ball_route_ctrl.c/h` | 钢球路线状态机与到位判据 |
| `cascade_ctrl.c/h` | 位置-角度串级控制，死区、条件积分、预测、前馈、坡度补偿 |
| `ball_step_ctrl.c/h` | 早期单环小球 PID（卡尔曼滤波 + 提前刹车），`MAIN_USE_CASCADE_CTRL=0` 时启用 |
| `step_motor.c/h` | 步进电机驱动（PWM 频率调速）、步进 2 伪角度环 |
| `stepper_axis.c/h` | 滑轨轴封装：RPM 限幅、脉冲角度估算、软限位 |
| `dc_motor.c/h` | 直流电机 PWM + 方向控制 |
| `dc_encoder.c/h` | 正交编码器计数、5 周期滑动窗口测速 |
| `speed_ctrl.c/h` | 速度环 PID + 加加速度受限的速度规划 |
| `gray_sensor.c/h` | 8 路灰度误差计算、循迹 PID、差速合成 |
| `track_pid_tune.c/h` | 按键在线调参（PID、速度、转弯参数、球参数） |
| `yaw.c/h` | MPU6050 航向积分（主循环执行）、温补、关键区让路 |
| `mpu6050.c/h` | I2C 访问、零偏/运动标定、温度补偿、死区 |
| `pid.c/h` | 通用 PID 结构（速度环/循迹环复用），支持 D 项一阶低通 |
| `pid_core.c/h` | 浮点 PID 内核（条件积分、输出限幅） |
| `oled.c/h`, `oledfont.h` | SSD1306 OLED 驱动与字模 |
| `dc_motor_display.c/h` | 主界面、测试菜单、视觉状态显示 |
| `Key.c/h` | 4 按键短按/长按事件 |
| `Delay.c/h` | 毫秒级阻塞延时 |
| `i2c_override.c`, `sysclock_override.c` | I2C 与系统时钟的定制/覆盖实现 |
| `main.syscfg` | SysConfig 外设配置（引脚、时钟、PWM、UART、I2C） |
| `targetConfigs/MSPM0G3507.ccxml` | XDS110 调试目标配置 |

## 5. 按键与调试菜单

| 按键 | 功能 |
| --- | --- |
| KEY2 短按 | 停车：关闭循迹、停速度环 |
| KEY3 短按 | 启动：使能循迹并按设定速度运行 |
| KEY1 短按/长按 | 进入/操作在线调参界面 |
| KEY4 短按/长按 | 进入/退出测试菜单 |

调参界面可在线修改：循迹 Kp/Ki/Kd、基础速度、直角弯 PWM/停车时间/锁定时间、
钢球目标位置与钢球 PID、钢球使能、滑轨零点标定。

测试菜单共 5 页：主状态页（编码器/MPU）、计时页、直线页、小球页、路线页；
MPU 标定失败后可在此页按 KEY2 重新标定。

## 6. 编译与烧录

1. 安装 **Code Composer Studio（Theia 内核版本）**、**MSPM0 SDK 2.10.00.04**
   与 **SysConfig 1.28.0**（版本号见 `.cproject` 的 `PRODUCTS` 字段）。
2. `File → Import → CCS Projects`，选择本目录导入工程 `PID_ctrl`。
3. `Build`：构建时会自动调用 SysConfig，由 `main.syscfg` 生成
   `ti_msp_dl_config.c/.h`（输出在 `Debug/` 下）。
4. 用板载 XDS110 连接 LaunchPad，`Debug` 烧录并运行。

> 仓库不包含 `Debug/`、`tmp/`、`.backup_*/`，这些是构建产物、临时调试资料和
> 历史备份，已写入 [`.gitignore`](.gitignore)。克隆后直接 Build 即可重新生成。

## 7. 上电与调试注意事项

- **上电后约 2 秒内保持车身静止**：主循环会等待 MPU 预热并做陀螺仪零偏标定，
  OLED 会提示 `CALIBRATING... KEEP CAR STILL`。标定失败不阻塞启动，
  可进测试菜单重标。
- **方向宏**：电机/编码器/角度方向不对时改
  `DC_MOTOR_DIR_L/R`、`DC_ENCODER_DIR_L/R`、`DC_ENCODER_SWAP_LR`、
  `CASCADE_ANGLE_DIRECTION_INVERT`、`TRACK_GYRO_SIGN`。
- **参数集中位置**：
  - 串级控制：`cascade_ctrl.h`
  - 循迹：`gray_sensor.h`
  - 速度环与速度规划：`speed_ctrl.h`
  - 单环小球 PID：`ball_step_ctrl.h`
  - 路线目标点：`ball_route_ctrl.h`
  - 步进轴/电机：`stepper_axis.h`、`step_motor.h`、`dc_motor.h`
- **视觉丢帧保护**：超过 `CASCADE_INPUT_TIMEOUT_MS`（400 ms）没有有效位置数据，
  控制器会停止输出，防止失控。
- **倾角安全限幅**：目标角最大 18°，且每周期限幅变化 0.5°；
  滑轨轴另有 `±8°` 软限位。

## 8. 目录结构

```
PID_ctrl/
├── main.c                 入口、任务调度、视觉/按键分发
├── main.syscfg            SysConfig 外设配置
├── .project / .cproject   CCS 工程文件
├── .ccsproject            CCS 工程属性
├── .settings/             Eclipse 工程首选项
├── targetConfigs/         XDS110 调试配置
├── *.c / *.h              控制与驱动模块（见第 4 节）
├── .gitignore
└── README.md
```

## 9. 说明

本仓库为个人竞赛/课程项目代码，未附加开源许可证；如需引用或二次分发，
请先与作者联系。
