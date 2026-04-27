# Code Architecture Guide

## 1. 这份工程的总体分层

这套工程当前可以按下面这条链理解：

`入口与总装层`
-> `调度层`
-> `硬件抽象层 / 驱动层`
-> `设备层`
-> `控制层`
-> `行为层`
-> `任务层`
-> `交互层`

你之前要求的企业级分层、非阻塞、参数集中管理，现在这套工程基本就是按这个方向落下来的。


## 2. 各层对应哪些文件

### A. 入口与总装层

这一层负责把整个系统拼起来。

核心文件：

- `app/ARCH/app_system.c`
- `app/ARCH/app_system.h`

作用：

- 初始化串口、设备中心、行为层、控制层、任务层、交互层、调度器
- 注册所有周期任务
- 作为整个应用总入口

你可以把它理解成：

“系统装配厂”


### B. 调度层

这一层不是 RTOS，而是轻量级软件定时调度器。

核心文件：

- `app/ARCH/scheduler/app_scheduler.c`
- `app/ARCH/scheduler/app_scheduler.h`
- `app/SYS/AppTick.c`
- `app/SYS/AppTick.h`

作用：

- 提供毫秒时间基准
- 注册周期任务
- 轮询检查是否到期
- 到期就调用对应回调

现在注册进去的主要任务有：

- `1ms` HAL 服务
- `10ms` 底盘服务
- `10ms` 任务层
- `10ms` 设备层
- `10ms` 控制层
- `50ms` 交互刷新
- `500ms` 串口调试输出


### C. 硬件抽象层 / 驱动层

这一层负责屏蔽底层芯片细节和外设寄存器差异。

核心文件：

- `app/ARCH/hal/hal_board.c`
- `app/ARCH/hal/hal_board.h`
- `app/ARCH/hal/hal_chassis.c`
- `app/ARCH/hal/hal_chassis.h`
- `app/ARCH/config/board_profile.h`
- `ti_msp_dl_config.c`
- `ti_msp_dl_config.h`

作用：

`board_profile.h`
- 集中描述板级资源映射
- 例如 PWM、GPIO、编码器、595、串口、I2C 口等

`hal_board.*`
- 提供更基础的 GPIO/外设初始化封装

`hal_chassis.*`
- 底盘直接相关驱动
- 编码器读取
- 电机 PWM 输出
- 74HC595 方向控制
- RGB 输出
- 电机输出斜坡

这一层的目标就是：

- 上层不要直接碰芯片寄存器
- 上层只拿统一接口


### D. 设备层

这一层负责把“硬件”整理成“设备”。

核心文件：

- `app/ARCH/device/device_center.c`
- `app/ARCH/device/device_center.h`
- `app/ARCH/device/device_gyro.c`
- `app/ARCH/device/device_gyro.h`
- `app/ARCH/device/device_track_sensor.c`
- `app/ARCH/device/device_track_sensor.h`

作用：

`device_gyro.*`
- 负责陀螺仪/IMU 数据采集与解析

`device_track_sensor.*`
- 负责循迹传感器数据

`device_center.*`
- 是设备层的中心汇聚点
- 把编码器、IMU、循迹、电机状态统一整理成一份快照
- 做速度换算、里程换算、左右速度、左右里程、整车里程、编码器航向等统一计算

你可以把 `device_center` 理解成：

“设备总线中心”


### E. 控制层

这一层负责闭环控制。

核心文件：

- `app/ARCH/control/motion_control.c`
- `app/ARCH/control/motion_control.h`
- `app/ARCH/control/pid.c`
- `app/ARCH/control/pid.h`

作用：

当前已经落下来的控制主要包括：

- 速度 PID
- 里程 PID
- 姿态 PID
- 转向 PID
- 巡线 PID

`motion_control.*` 负责把这些环串起来：

- 先拿设备层快照
- 再根据行为层指令决定开哪些环
- 最后输出成左右速度目标/四轮占空比

这层是整套小车“会不会稳”的核心。


### F. 行为层

这一层负责“动作语义封装”。

核心文件：

- `app/ARCH/behavior/behavior_motion.c`
- `app/ARCH/behavior/behavior_motion.h`

作用：

把控制目标封装成更高层动作，例如：

- `Stop`
- `OpenLoop`
- `SpeedHold`
- `Straight`
- `StraightDistance`
- `Turn`
- `TurnToHeading`
- `LineFollow`
- `GoToLine`

行为层不直接管底层 PWM，也不直接管菜单。

它只负责：

“我现在想让车做什么动作”


### G. 任务层

这一层负责状态机和动作队列。

核心文件：

- `app/ARCH/task/task_service.c`
- `app/ARCH/task/task_service.h`

作用：

- 管理任务队列
- 管理当前动作阶段
- 做简单状态机推进
- 决定什么时候切行为

当前里面已经有这些动作：

- `Run Mode A`
- `Run Mode B`
- `Straight Test`
- `M1 Speed CL`
- `M2 Spin OL`
- `Turn Left 90`
- `Turn Right 90`
- `Sensor Scan`

这一层不是直接控制电机，而是：

“安排下一步让行为层做什么”


### H. 交互层

这一层负责 OLED 页面和菜单触发任务。

核心文件：

- `app/ARCH/interaction/interaction_service.c`
- `app/ARCH/interaction/interaction_service.h`
- `app/MENU/menu_app.c`
- `app/MENU/menu_app.h`
- `app/MENU/menu_port.c`
- `app/MENU/menu_port.h`
- `app/MENU/menu.c`
- `app/MENU/menu.h`
- `app/OLED/OLED.c`
- `app/OLED/OLED.h`
- `app/OLED/OLED_Data.c`
- `app/OLED/OLED_Data.h`

作用：

`menu_*`
- 按键导航
- 菜单结构
- 页面进出

`interaction_service.*`
- 根据当前动作生成 OLED 显示内容
- 把菜单动作转成任务层动作

`OLED.*`
- 真正的显示驱动

这一层就是：

“人怎么跟车交互”


### I. 系统公共层

这部分虽然不属于你分层里的某一层，但它是工程公共基础能力。

核心文件：

- `app/SYS/Serial.c`
- `app/SYS/Serial.h`
- `app/SYS/Buzzer.c`
- `app/SYS/Buzzer.h`
- `app/SYS/Delay.c`
- `app/SYS/Delay.h`

作用：

- 串口调试
- 蜂鸣器模式
- 基础延时支持


## 3. 推荐的数据流理解方式

当前整套工程，最推荐你按下面这条数据流理解：

### 情况 1：菜单触发直行

`菜单按键`
-> `menu_port`
-> `menu_app`
-> `interaction_service`
-> `task_service`
-> `behavior_motion`
-> `motion_control`
-> `hal_chassis`
-> `电机`


### 情况 2：编码器/IMU 反馈回来

`编码器 / IMU / 循迹`
-> `hal_chassis / device_gyro / device_track_sensor`
-> `device_center`
-> `motion_control`
-> `新的控制输出`


### 情况 3：OLED 显示

`device_center / task_service / motion_control`
-> `interaction_service`
-> `menu_app`
-> `OLED`


## 4. 当前工程里最重要的几个“中心文件”

如果你只想先记住最关键的几个文件，优先记这几个：

`app/ARCH/config/app_config.h`
- 所有核心参数中心

`app/ARCH/config/board_profile.h`
- 所有板级硬件映射中心

`app/ARCH/device/device_center.c`
- 设备层数据汇聚中心

`app/ARCH/control/motion_control.c`
- 控制层核心

`app/ARCH/task/task_service.c`
- 任务编排中心

`app/ARCH/interaction/interaction_service.c`
- 显示与交互中心

`app/ARCH/app_system.c`
- 总装入口


## 5. 你后面改代码时的建议

推荐按下面的原则继续扩展：

### 加新硬件

例如：

- OLED 新接口
- 新按键
- 新传感器
- 新蜂鸣器模式

优先落在：

- `board_profile.h`
- `hal_*`
- `device_*`

不要直接从任务层或菜单层去操作寄存器。


### 加新控制

例如：

- 新速度环
- 新角度环
- 新巡线算法

优先落在：

- `motion_control.*`
- `pid.*`

不要把控制算法直接写到菜单里。


### 加新动作

例如：

- 走固定距离
- 原地转角
- 走到黑线

优先落在：

- `behavior_motion.*`


### 加新任务流程

例如：

- 比赛任务状态机
- 多段动作组合

优先落在：

- `task_service.*`


### 加新菜单页

例如：

- 新测试页
- 新参数观察页

优先落在：

- `interaction_service.*`
- `menu_app.*`


## 6. 现在这套分层的优势

当前这套结构的好处是：

- 参数集中
- 硬件映射集中
- 控制和 UI 解耦
- 任务和动作解耦
- 驱动和芯片细节被屏蔽
- 非阻塞，适合电赛现场快速改参数和查问题


## 7. 当前最适合你记住的一句话

如果你想快速记住整套工程，就记这句话：

`board_profile/app_config 定义资源和参数`

`hal/device 负责采集和执行`

`control 负责闭环`

`behavior 负责动作语义`

`task 负责状态机编排`

`interaction/menu 负责人机交互`

`app_system/scheduler 负责把整套系统跑起来`

