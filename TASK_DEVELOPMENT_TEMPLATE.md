# Task Development Template

## 1. 这份文档是干什么的

这份文档专门给你后面加“比赛任务”用。

目标是：

- 新任务尽量按分层加
- 不把菜单、控制、驱动混在一起
- 后面比赛前好改、好查、好关功能


## 2. 加一个新任务时，先想清楚 4 件事

先不要急着写代码，先把下面 4 个问题写清楚：

### 1. 这个任务最终想让车做什么

例如：

- 直行 500mm
- 左转 90 度
- 巡线到黑线停止
- 出库后直行，再转弯，再巡线


### 2. 这是“动作”还是“任务流程”

如果只是单一步骤：

- 直行
- 转向
- 巡线

它更像“行为层动作”。

如果是多步组合：

- 先直行
- 再转向
- 再巡线
- 再停下

它更像“任务层流程”。


### 3. 这个任务依赖哪些反馈

例如：

- 编码器里程
- IMU 航向
- 灰度循迹
- 按键触发


### 4. 这个任务需要用户怎么触发

例如：

- 菜单触发
- 上电自动运行
- 按键触发


## 3. 分层开发原则

以后你加新功能，优先按这个顺序判断放哪一层：

### 底层硬件相关

放：

- `board_profile.h`
- `hal_*`
- `device_*`

适合：

- 新 GPIO
- 新 PWM
- 新传感器
- 新串口/I2C/SPI 外设


### 新控制逻辑

放：

- `motion_control.*`
- `pid.*`

适合：

- 新闭环
- 新控制器
- 新转向算法


### 新动作语义

放：

- `behavior_motion.*`

适合：

- 直行固定距离
- 转到某个角度
- 跑到黑线


### 新任务编排

放：

- `task_service.*`

适合：

- 多段动作状态机
- 动作队列
- 比赛流程控制


### 新菜单入口和显示页

放：

- `interaction_service.*`
- `menu_app.*`

适合：

- 菜单入口
- OLED 调试页
- 参数观察页


## 4. 推荐开发顺序

推荐永远按这个顺序做：

1. 先把底层和设备层打通
2. 再确认控制层接口可用
3. 再封装行为层动作
4. 再到任务层做状态机
5. 最后才接菜单和 OLED

这样好处是：

- 出问题好定位
- 不会一开始就把菜单和控制搅在一起


## 5. 标准开发模板

下面是最推荐的加新任务模板。

### 第一步：先定义任务目标

例子：

```text
任务名：Go To Black Line
输入：目标速度
反馈：灰度传感器 + IMU
结束条件：检测到黑线
触发方式：菜单
```


### 第二步：如果缺新动作，先加行为层动作

比如你以后要加：

- `BehaviorMotion_GoToBlackLine()`
- `BehaviorMotion_GoToWhiteLine()`
- `BehaviorMotion_StraightDistance()`

优先改：

- `app/ARCH/behavior/behavior_motion.h`
- `app/ARCH/behavior/behavior_motion.c`

行为层只负责表达：

“我要做什么”

不要在这里写菜单逻辑。


### 第三步：如果是多步流程，再加任务层状态机

例如一个比赛动作：

```text
阶段1：直行 300mm
阶段2：左转 90 度
阶段3：巡线
阶段4：遇黑线停车
```

这种就应该落在：

- `app/ARCH/task/task_service.h`
- `app/ARCH/task/task_service.c`

你需要做的事情一般是：

- 新增 `TaskAction_e`
- 新增一个 `TaskService_RunXxx()` 函数
- 在 `TaskService_Tick10ms()` 里挂进去
- 在 `TaskService_GetActionName()` 里补显示名字


### 第四步：如果要菜单触发，再接交互层

一般改：

- `app/ARCH/interaction/interaction_service.h`
- `app/ARCH/interaction/interaction_service.c`
- `app/MENU/menu_app.c`

常见步骤：

- 新增 `InteractionAction_e`
- 在 `InteractionService_RequestAction()` 里转成任务层动作
- 在 `InteractionService_UpdateRuntimePage()` 里定义 OLED 页面内容
- 在 `menu_app.c` 菜单里加入口


### 第五步：最后补调试页

建议每个新任务都配一个最小调试页。

优先显示：

- 当前阶段
- 当前关键传感器
- 当前目标值
- 当前输出值

不要一上来显示太多数字。


## 6. 一个完整例子

假设你以后要加一个新任务：

`出库 -> 直行 -> 左转 -> 巡线`

推荐这样做：

### A. 行为层

确认已有这些动作可用：

- `StraightDistance`
- `TurnToHeading`
- `LineFollow`

如果没有，再先补。


### B. 任务层

在 `TaskAction_e` 里新增：

```c
TASK_ACTION_OUT_AND_LINE
```

然后写：

```c
static void TaskService_RunOutAndLine(void)
{
    // phase 1: 出库直行
    // phase 2: 左转
    // phase 3: 巡线
    // done
}
```

再挂进：

- `TaskService_Tick10ms()`
- `TaskService_GetActionName()`


### C. 交互层

新增：

```c
INTERACTION_ACTION_OUT_AND_LINE
```

然后：

- 在 `InteractionService_RequestAction()` 里入队
- 在 `InteractionService_UpdateRuntimePage()` 里显示


### D. 菜单层

在 `menu_app.c` 里加一项菜单：

```text
Out And Line
```


## 7. 新任务最容易犯的错误

### 错误 1：直接在菜单里写控制逻辑

不要这样做。

菜单层只负责：

- 触发
- 显示

不要在 `menu_app.c` 里直接写电机控制。


### 错误 2：在任务层直接碰底层驱动

不要这样做。

任务层应该调：

- 行为层
- 控制层接口

不要直接在任务层里改 GPIO/PWM。


### 错误 3：一个函数里面把感知、控制、任务、显示全写了

这样后面一定难改。

正确做法是拆开：

- 设备层拿数据
- 控制层算输出
- 任务层编排阶段
- 交互层显示


### 错误 4：没有调试页就上复杂任务

建议每个新任务都先做最小调试页。

哪怕只显示：

- phase
- heading
- distance
- output

也比盲调强很多。


## 8. 新任务推荐最少要显示什么

### 直行类任务

推荐显示：

- 航向
- 左右速度
- 左右输出


### 转向类任务

推荐显示：

- 当前角度
- 目标角度
- 当前角速度


### 巡线类任务

推荐显示：

- 线位置
- 左右输出
- 当前阶段


### 多段比赛任务

推荐显示：

- phase
- 当前动作名
- 当前关键误差


## 9. 以后你自己加任务时，优先改哪几个文件

最常用的是这几个：

- `app/ARCH/behavior/behavior_motion.h`
- `app/ARCH/behavior/behavior_motion.c`
- `app/ARCH/task/task_service.h`
- `app/ARCH/task/task_service.c`
- `app/ARCH/interaction/interaction_service.h`
- `app/ARCH/interaction/interaction_service.c`
- `app/MENU/menu_app.c`
- `app/ARCH/config/app_config.h`


## 10. 最推荐你以后照着走的套路

记这一条就够了：

`先把动作做成 behavior`

`再把流程做成 task`

`再把入口接到 interaction/menu`

`最后补 OLED 调试页`


## 11. 如果你准备进入比赛任务开发

建议下一阶段优先做这几个：

1. 固定距离直行任务
2. 固定角度转向任务
3. 巡线到黑线停止任务
4. 多段状态机比赛任务

这是最适合你当前工程继续往比赛推进的顺序。

