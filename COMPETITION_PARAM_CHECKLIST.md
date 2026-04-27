# Competition Param Checklist

## 1. 赛前优先确认的文件

主要看这个文件：

`app/ARCH/config/app_config.h`

大部分比赛前要改的核心参数都在这里。


## 2. 当前这台车建议先保留的参数

如果你现在这台车已经“基本直线”，建议先保留这组：

```c
#define APP_STRAIGHT_TEST_SPEED_MMPS            (220)
#define APP_STRAIGHT_SIDE_BIAS_MMPS             (-6)
```

这组是当前硬件状态下收出来的。


## 3. 底盘与编码器参数

这些参数必须和当前硬件一致：

```c
#define APP_CHASSIS_WHEEL_DIAMETER_MM           (65.0f)
#define APP_CHASSIS_WHEEL_BASE_MM               (120.0f)
#define APP_CHASSIS_GEAR_RATIO                  (28.0f)

#define APP_ENCODER_PAIR0_COUNT_PER_REV         (220.0f)
#define APP_ENCODER_PAIR1_COUNT_PER_REV         (224.0f)
#define APP_ENCODER_PAIR2_COUNT_PER_REV         (224.0f)
#define APP_ENCODER_PAIR3_COUNT_PER_REV         (223.0f)
```

赛前如果改了这些，下面几类量都会跟着变：

- 里程换算
- 速度换算
- 编码器角度换算
- 转向估算

所以不要随便改。


## 4. 直行类参数

最常用的是：

```c
#define APP_STRAIGHT_TEST_SPEED_MMPS            (220)
#define APP_STRAIGHT_SIDE_BIAS_MMPS             (-6)
#define APP_STRAIGHT_HEADING_DEADBAND_DEG10     (8)
#define APP_STRAIGHT_YAW_DEADBAND_MDPS          (25)
#define APP_STRAIGHT_DIFF_DEADBAND_MMPS         (8)
```

用途：

`APP_STRAIGHT_TEST_SPEED_MMPS`
- 直行测试速度。

`APP_STRAIGHT_SIDE_BIAS_MMPS`
- 左右补偿。
- 比赛前最常调的就是它。

`APP_STRAIGHT_HEADING_DEADBAND_DEG10`
- 小角度误差死区。

`APP_STRAIGHT_YAW_DEADBAND_MDPS`
- 小角速度死区。

`APP_STRAIGHT_DIFF_DEADBAND_MMPS`
- 小转向速度差死区。


## 5. 速度环参数

```c
#define APP_WHEEL_SPEED_PID_KP                 (0.28f)
#define APP_WHEEL_SPEED_PID_KI                 (0.02f)
#define APP_WHEEL_SPEED_PID_KD                 (0.00f)
#define APP_WHEEL_SPEED_PID_OUT_MIN            (-180.0f)
#define APP_WHEEL_SPEED_PID_OUT_MAX            (180.0f)
```

这组已经调到相对稳定。

赛前如果没有明显问题：
- 不建议乱动。

只有出现下面情况再考虑动：
- 速度跟随明显发软
- 加减速拖泥带水
- 闭环明显抖动


## 6. 姿态 / 转向参数

```c
#define APP_ATTITUDE_PID_KP                     (10.50f)
#define APP_ATTITUDE_PID_KI                     (0.02f)
#define APP_ATTITUDE_PID_KD                     (0.00f)

#define APP_TURN_PID_KP                         (0.12f)
#define APP_TURN_PID_KI                         (0.001f)
#define APP_TURN_PID_KD                         (0.00f)
```

用途：

`APP_ATTITUDE_PID_*`
- 负责航向锁定。
- 直行能不能锁住车头，和它有关。

`APP_TURN_PID_*`
- 负责把目标角速度变成左右速度差。
- 转向是否柔和、是否过冲，和它有关。

如果现在直行已经基本正常：
- 这组先不要动。


## 7. 电机输出相关参数

```c
#define APP_MOTOR_SPEED_TO_DUTY_GAIN            (2.8f)
#define APP_MOTOR_SPEED_FEEDFORWARD_GAIN        (APP_MOTOR_SPEED_TO_DUTY_GAIN)
#define APP_MOTOR_OUTPUT_SLEW_STEP              (120)
```

用途：

`APP_MOTOR_SPEED_TO_DUTY_GAIN`
- 速度目标转占空比的前馈。

`APP_MOTOR_OUTPUT_SLEW_STEP`
- 输出斜坡。
- 它太小会肉，太大可能更冲。

如果车现在起步和平顺性已经能接受：
- 先不动。


## 8. 四轮补偿参数

```c
#define APP_MOTOR_REAR_RIGHT_DUTY_SCALE         (1.00f)
#define APP_MOTOR_FRONT_RIGHT_DUTY_SCALE        (1.00f)
#define APP_MOTOR_REAR_LEFT_DUTY_SCALE          (0.99f)
#define APP_MOTOR_FRONT_LEFT_DUTY_SCALE         (1.01f)
```

这组是四轮本体差异补偿。

它和 `APP_STRAIGHT_SIDE_BIAS_MMPS` 的区别：

- 四轮补偿：修四个电机个体差异
- 直行补偿：修整车左右跑偏

如果不是四轮某一只特别明显快或慢：
- 先不要动这组。


## 9. 转向测试参数

```c
#define APP_TURN_TEST_ANGLE_90_DEG10            (900)
#define APP_TURN_DONE_HEADING_ERR_DEG10         (30)
#define APP_TURN_DONE_YAW_ABS_MDPS              (25)
#define APP_TURN_DONE_HOLD_CYCLES               (8U)
```

用途：

- 90 度转向测试
- 判断转向什么时候算完成

如果以后你要收 90 度转向：
- 主要会看这组。


## 10. 比赛前最常见的 4 种微调

直行右偏
- 把 `APP_STRAIGHT_SIDE_BIAS_MMPS` 调得更负一点。

直行左偏
- 把 `APP_STRAIGHT_SIDE_BIAS_MMPS` 往 `0` 方向回一点。

想更稳
- 先降 `APP_STRAIGHT_TEST_SPEED_MMPS`
- 不要先乱改 PID

想更快
- 先升 `APP_STRAIGHT_TEST_SPEED_MMPS`
- 再看会不会重新跑偏


## 11. 推荐赛前检查顺序

1. 先确认编码器计数和方向没问题。
2. 再确认 M1 闭环速度基本正常。
3. 再进入 Straight Test 调整直行补偿。
4. 直行稳定后，再测 90 度转向。
5. 最后再进任务层做完整动作联调。


## 12. 哪些情况必须重新标定

以下任意一项变化，都建议重新调一次：

- 换轮胎
- 换电机
- 改驱动接线
- 改轮子安装位置
- 改底盘重心
- 改电源
- 改机械结构


## 13. 你现在最该记住的参数

如果你只想先记最重要的几个，就记这几个：

```c
APP_STRAIGHT_TEST_SPEED_MMPS
APP_STRAIGHT_SIDE_BIAS_MMPS
APP_WHEEL_SPEED_PID_KP
APP_ATTITUDE_PID_KP
APP_TURN_PID_KP
```

其中比赛现场最常改的，优先还是：

`APP_STRAIGHT_SIDE_BIAS_MMPS`

