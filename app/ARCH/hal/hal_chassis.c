#include "ARCH/hal/hal_chassis.h"

#include "ARCH/config/app_config.h"
#include "ARCH/hal/hal_board.h"

static HalChassis_MotorState_t g_motor_state;
static HalChassis_EncoderState_t g_encoder_state;
static uint8_t g_shift595_shadow;
static uint8_t g_shift595_dirty;
static uint8_t g_prev_quad_state[APP_BOARD_WHEEL_COUNT];
static int16_t g_last_wheel_duty[APP_BOARD_WHEEL_COUNT];
static int16_t g_target_wheel_duty[APP_BOARD_WHEEL_COUNT];

typedef struct
{
    HalBoard_Pin_t pin;
    uint32_t pwm_func;
    GPTIMER_Regs *inst;
    DL_TIMER_CC_INDEX cc_index;
    uint8_t in1_bit;
    uint8_t in2_bit;
    uint8_t forward_in1_high;
} HalChassis_MotorHw_t;

static const HalChassis_MotorHw_t g_motor_hw[APP_BOARD_WHEEL_COUNT] = {
    [APP_BOARD_WHEEL_REAR_RIGHT_INDEX] = {
        {APP_BOARD_MOTOR0_PWM_PORT, APP_BOARD_MOTOR0_PWM_PIN, APP_BOARD_MOTOR0_PWM_IOMUX},
        APP_BOARD_MOTOR0_PWM_FUNC,
        APP_BOARD_MOTOR0_PWM_INST,
        APP_BOARD_MOTOR0_CC_INDEX,
        APP_BOARD_MOTOR0_IN1_BIT,
        APP_BOARD_MOTOR0_IN2_BIT,
        APP_BOARD_MOTOR0_FORWARD_IN1_HIGH,
    },
    [APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] = {
        {APP_BOARD_MOTOR1_PWM_PORT, APP_BOARD_MOTOR1_PWM_PIN, APP_BOARD_MOTOR1_PWM_IOMUX},
        APP_BOARD_MOTOR1_PWM_FUNC,
        APP_BOARD_MOTOR1_PWM_INST,
        APP_BOARD_MOTOR1_CC_INDEX,
        APP_BOARD_MOTOR1_IN1_BIT,
        APP_BOARD_MOTOR1_IN2_BIT,
        APP_BOARD_MOTOR1_FORWARD_IN1_HIGH,
    },
    [APP_BOARD_WHEEL_REAR_LEFT_INDEX] = {
        {APP_BOARD_MOTOR3_PWM_PORT, APP_BOARD_MOTOR3_PWM_PIN, APP_BOARD_MOTOR3_PWM_IOMUX},
        APP_BOARD_MOTOR3_PWM_FUNC,
        APP_BOARD_MOTOR3_PWM_INST,
        APP_BOARD_MOTOR3_CC_INDEX,
        APP_BOARD_MOTOR3_IN1_BIT,
        APP_BOARD_MOTOR3_IN2_BIT,
        APP_BOARD_MOTOR3_FORWARD_IN1_HIGH,
    },
    [APP_BOARD_WHEEL_FRONT_LEFT_INDEX] = {
        {APP_BOARD_MOTOR2_PWM_PORT, APP_BOARD_MOTOR2_PWM_PIN, APP_BOARD_MOTOR2_PWM_IOMUX},
        APP_BOARD_MOTOR2_PWM_FUNC,
        APP_BOARD_MOTOR2_PWM_INST,
        APP_BOARD_MOTOR2_CC_INDEX,
        APP_BOARD_MOTOR2_IN1_BIT,
        APP_BOARD_MOTOR2_IN2_BIT,
        APP_BOARD_MOTOR2_FORWARD_IN1_HIGH,
    },
};

static const HalBoard_Pin_t g_encoder_pins[APP_BOARD_ENCODER_RAW_COUNT] = {
    {APP_BOARD_ENCODER_0_PORT, APP_BOARD_ENCODER_0_PIN, APP_BOARD_ENCODER_0_IOMUX},
    {APP_BOARD_ENCODER_1_PORT, APP_BOARD_ENCODER_1_PIN, APP_BOARD_ENCODER_1_IOMUX},
    {APP_BOARD_ENCODER_2_PORT, APP_BOARD_ENCODER_2_PIN, APP_BOARD_ENCODER_2_IOMUX},
    {APP_BOARD_ENCODER_3_PORT, APP_BOARD_ENCODER_3_PIN, APP_BOARD_ENCODER_3_IOMUX},
    {APP_BOARD_ENCODER_4_PORT, APP_BOARD_ENCODER_4_PIN, APP_BOARD_ENCODER_4_IOMUX},
    {APP_BOARD_ENCODER_5_PORT, APP_BOARD_ENCODER_5_PIN, APP_BOARD_ENCODER_5_IOMUX},
    {APP_BOARD_ENCODER_6_PORT, APP_BOARD_ENCODER_6_PIN, APP_BOARD_ENCODER_6_IOMUX},
    {APP_BOARD_ENCODER_7_PORT, APP_BOARD_ENCODER_7_PIN, APP_BOARD_ENCODER_7_IOMUX},
};

static const HalBoard_Pin_t g_hc595_ds_pin   = {APP_BOARD_595_DS_PORT, APP_BOARD_595_DS_PIN, APP_BOARD_595_DS_IOMUX};
static const HalBoard_Pin_t g_hc595_shcp_pin = {APP_BOARD_595_SHCP_PORT, APP_BOARD_595_SHCP_PIN, APP_BOARD_595_SHCP_IOMUX};
static const HalBoard_Pin_t g_hc595_stcp_pin = {APP_BOARD_595_STCP_PORT, APP_BOARD_595_STCP_PIN, APP_BOARD_595_STCP_IOMUX};

static const DL_Timer_ClockConfig g_motor_pwm_clock_cfg = {
    APP_BOARD_MOTOR_PWM_CLOCK_SEL,
    APP_BOARD_MOTOR_PWM_CLOCK_DIVIDE,
    APP_BOARD_MOTOR_PWM_CLOCK_PRESCALE,
};

static const DL_Timer_PWMConfig g_motor_pwm_cfg = {
    APP_BOARD_MOTOR_PWM_PERIOD_COUNTS,
    APP_BOARD_MOTOR_PWM_MODE,
    false,
    DL_TIMER_START,
};

static const uint32_t g_encoder_gpiob_mask =
    APP_BOARD_ENCODER_0_PIN |
    APP_BOARD_ENCODER_1_PIN |
    APP_BOARD_ENCODER_2_PIN |
    APP_BOARD_ENCODER_3_PIN |
    APP_BOARD_ENCODER_4_PIN |
    APP_BOARD_ENCODER_5_PIN |
    APP_BOARD_ENCODER_6_PIN |
    APP_BOARD_ENCODER_7_PIN;

static void HalChassis_EnableEncoderInputFilter(const HalBoard_Pin_t *pin)
{
    if (pin == 0) {
        return;
    }

    if (pin->pin <= DL_GPIO_PIN_15) {
        switch (pin->pin) {
        case DL_GPIO_PIN_4:
            DL_GPIO_setLowerPinsInputFilter(pin->port, DL_GPIO_PIN_4_INPUT_FILTER_8_CYCLES);
            break;
        case DL_GPIO_PIN_5:
            DL_GPIO_setLowerPinsInputFilter(pin->port, DL_GPIO_PIN_5_INPUT_FILTER_8_CYCLES);
            break;
        case DL_GPIO_PIN_6:
            DL_GPIO_setLowerPinsInputFilter(pin->port, DL_GPIO_PIN_6_INPUT_FILTER_8_CYCLES);
            break;
        case DL_GPIO_PIN_7:
            DL_GPIO_setLowerPinsInputFilter(pin->port, DL_GPIO_PIN_7_INPUT_FILTER_8_CYCLES);
            break;
        case DL_GPIO_PIN_13:
            DL_GPIO_setLowerPinsInputFilter(pin->port, DL_GPIO_PIN_13_INPUT_FILTER_8_CYCLES);
            break;
        default:
            break;
        }
    } else {
        switch (pin->pin) {
        case DL_GPIO_PIN_18:
            DL_GPIO_setUpperPinsInputFilter(pin->port, DL_GPIO_PIN_18_INPUT_FILTER_8_CYCLES);
            break;
        case DL_GPIO_PIN_19:
            DL_GPIO_setUpperPinsInputFilter(pin->port, DL_GPIO_PIN_19_INPUT_FILTER_8_CYCLES);
            break;
        case DL_GPIO_PIN_23:
            DL_GPIO_setUpperPinsInputFilter(pin->port, DL_GPIO_PIN_23_INPUT_FILTER_8_CYCLES);
            break;
        default:
            break;
        }
    }
}

static int8_t HalChassis_GetWheelDirectionSign(uint8_t wheel_index)
{
    switch (wheel_index) {
    case APP_BOARD_WHEEL_FRONT_LEFT_INDEX:
    case APP_BOARD_WHEEL_REAR_LEFT_INDEX:
        return -1;
    case APP_BOARD_WHEEL_FRONT_RIGHT_INDEX:
    case APP_BOARD_WHEEL_REAR_RIGHT_INDEX:
    default:
        return 1;
    }
}

static void HalChassis_ConfigEncoderInterrupts(void)
{
    DL_GPIO_setLowerPinsPolarity(GPIOB,
        DL_GPIO_PIN_4_EDGE_RISE_FALL |
        DL_GPIO_PIN_5_EDGE_RISE_FALL |
        DL_GPIO_PIN_6_EDGE_RISE_FALL |
        DL_GPIO_PIN_7_EDGE_RISE_FALL |
        DL_GPIO_PIN_13_EDGE_RISE_FALL);

    DL_GPIO_setUpperPinsPolarity(GPIOB,
        DL_GPIO_PIN_18_EDGE_RISE_FALL |
        DL_GPIO_PIN_19_EDGE_RISE_FALL |
        DL_GPIO_PIN_23_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(GPIOB, g_encoder_gpiob_mask);
    DL_GPIO_enableInterrupt(GPIOB, g_encoder_gpiob_mask);
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

static void HalChassis_RefreshEncoderLevel(uint8_t raw_index)
{
    uint8_t level = HalBoard_ReadPin(&g_encoder_pins[raw_index]);

    if (level != g_encoder_state.raw_level[raw_index]) {
        g_encoder_state.raw_level[raw_index] = level;
        g_encoder_state.raw_edges[raw_index]++;
    }
}

static int16_t HalChassis_ClampDuty(int16_t duty)
{
    if (duty > APP_MOTOR_DUTY_LIMIT) {
        return APP_MOTOR_DUTY_LIMIT;
    }
    if (duty < -APP_MOTOR_DUTY_LIMIT) {
        return -APP_MOTOR_DUTY_LIMIT;
    }
    return duty;
}

static int16_t HalChassis_RoundFloatToInt16(float value)
{
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }

    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static float HalChassis_GetWheelDutyScale(uint8_t wheel_index)
{
    switch (wheel_index) {
    case APP_BOARD_WHEEL_REAR_RIGHT_INDEX:
        return APP_MOTOR_REAR_RIGHT_DUTY_SCALE;
    case APP_BOARD_WHEEL_FRONT_RIGHT_INDEX:
        return APP_MOTOR_FRONT_RIGHT_DUTY_SCALE;
    case APP_BOARD_WHEEL_REAR_LEFT_INDEX:
        return APP_MOTOR_REAR_LEFT_DUTY_SCALE;
    case APP_BOARD_WHEEL_FRONT_LEFT_INDEX:
    default:
        return APP_MOTOR_FRONT_LEFT_DUTY_SCALE;
    }
}

static int16_t HalChassis_GetWheelDutyTrim(uint8_t wheel_index)
{
    switch (wheel_index) {
    case APP_BOARD_WHEEL_REAR_RIGHT_INDEX:
        return APP_MOTOR_REAR_RIGHT_DUTY_TRIM;
    case APP_BOARD_WHEEL_FRONT_RIGHT_INDEX:
        return APP_MOTOR_FRONT_RIGHT_DUTY_TRIM;
    case APP_BOARD_WHEEL_REAR_LEFT_INDEX:
        return APP_MOTOR_REAR_LEFT_DUTY_TRIM;
    case APP_BOARD_WHEEL_FRONT_LEFT_INDEX:
    default:
        return APP_MOTOR_FRONT_LEFT_DUTY_TRIM;
    }
}

static int16_t HalChassis_GetWheelStartDuty(uint8_t wheel_index)
{
    switch (wheel_index) {
    case APP_BOARD_WHEEL_REAR_RIGHT_INDEX:
        return APP_MOTOR_REAR_RIGHT_START_DUTY;
    case APP_BOARD_WHEEL_FRONT_RIGHT_INDEX:
        return APP_MOTOR_FRONT_RIGHT_START_DUTY;
    case APP_BOARD_WHEEL_REAR_LEFT_INDEX:
        return APP_MOTOR_REAR_LEFT_START_DUTY;
    case APP_BOARD_WHEEL_FRONT_LEFT_INDEX:
    default:
        return APP_MOTOR_FRONT_LEFT_START_DUTY;
    }
}

static int16_t HalChassis_GetWheelCompensatedDuty(uint8_t wheel_index, int16_t side_duty)
{
    float scaled_duty;
    int16_t trimmed_duty;
    int16_t start_duty;

    if (side_duty == 0) {
        return 0;
    }

    scaled_duty = (float)side_duty * HalChassis_GetWheelDutyScale(wheel_index);
    trimmed_duty = HalChassis_RoundFloatToInt16(scaled_duty);

    if (trimmed_duty > 0) {
        trimmed_duty = (int16_t)(trimmed_duty + HalChassis_GetWheelDutyTrim(wheel_index));
    } else if (trimmed_duty < 0) {
        trimmed_duty = (int16_t)(trimmed_duty - HalChassis_GetWheelDutyTrim(wheel_index));
    }

    start_duty = HalChassis_GetWheelStartDuty(wheel_index);
    if ((start_duty > 0) && (trimmed_duty > 0) && (trimmed_duty < start_duty)) {
        trimmed_duty = start_duty;
    } else if ((start_duty > 0) && (trimmed_duty < 0) &&
        ((int16_t)(-trimmed_duty) < start_duty)) {
        trimmed_duty = (int16_t)(-start_duty);
    }

    return HalChassis_ClampDuty(trimmed_duty);
}

static int16_t HalChassis_SlewDuty(int16_t current, int16_t target)
{
    int16_t slew_step = APP_MOTOR_OUTPUT_SLEW_STEP;

    if (((current > 0) && (target < 0)) || ((current < 0) && (target > 0))) {
        slew_step = APP_MOTOR_OUTPUT_SLEW_REVERSE_STEP;
    }

    if (target > current) {
        int16_t next = (int16_t)(current + slew_step);
        return (next > target) ? target : next;
    }

    if (target < current) {
        int16_t next = (int16_t)(current - slew_step);
        return (next < target) ? target : next;
    }

    return target;
}

static int16_t HalChassis_AverageDuty(int16_t a, int16_t b)
{
    return (int16_t)((a + b) / 2);
}

static uint32_t HalChassis_GetPwmOffCompare(GPTIMER_Regs *inst)
{
    return DL_Timer_getLoadValue(inst);
}

static uint32_t HalChassis_AbsDutyToCompare(GPTIMER_Regs *inst, uint16_t duty_abs)
{
    uint32_t load_value;

    if (duty_abs >= (uint16_t)APP_MOTOR_DUTY_LIMIT) {
        return 0U;
    }

    load_value = DL_Timer_getLoadValue(inst);
    return ((uint32_t)(APP_MOTOR_DUTY_LIMIT - duty_abs) * load_value) /
        (uint32_t)APP_MOTOR_DUTY_LIMIT;
}

static void HalChassis_Set595Bit(uint8_t bit_index, uint8_t high)
{
    uint8_t bit_mask;

    if (bit_index >= 8U) {
        return;
    }

    bit_mask = (uint8_t)(1U << bit_index);
    if (high != 0U) {
        g_shift595_shadow |= bit_mask;
    } else {
        g_shift595_shadow &= (uint8_t)(~bit_mask);
    }
}

static void HalChassis_SetMotorDirectionBits(uint8_t wheel_index, int8_t direction)
{
    const HalChassis_MotorHw_t *motor;
    uint8_t forward_in1;

    if (wheel_index >= APP_BOARD_WHEEL_COUNT) {
        return;
    }

    motor = &g_motor_hw[wheel_index];
    forward_in1 = motor->forward_in1_high;

    if (direction > 0) {
        HalChassis_Set595Bit(motor->in1_bit, forward_in1);
        HalChassis_Set595Bit(motor->in2_bit, (uint8_t)(!forward_in1));
    } else if (direction < 0) {
        HalChassis_Set595Bit(motor->in1_bit, (uint8_t)(!forward_in1));
        HalChassis_Set595Bit(motor->in2_bit, forward_in1);
    } else {
        HalChassis_Set595Bit(motor->in1_bit, 0U);
        HalChassis_Set595Bit(motor->in2_bit, 0U);
    }
}

static void HalChassis_ConfigPwmTimer(GPTIMER_Regs *inst)
{
    if (inst == 0) {
        return;
    }

    DL_Timer_reset(inst);
    DL_Timer_enablePower(inst);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_Timer_setClockConfig(inst, &g_motor_pwm_clock_cfg);
    DL_Timer_setCCPDirection(inst, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_Timer_initPWMMode(inst, &g_motor_pwm_cfg);
    DL_Timer_setCaptureCompareValue(inst, HalChassis_GetPwmOffCompare(inst), DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(inst, HalChassis_GetPwmOffCompare(inst), DL_TIMER_CC_1_INDEX);
    DL_Timer_enableClock(inst);
    DL_Timer_startCounter(inst);
}

static void HalChassis_InitMotorPwm(void)
{
    uint8_t i;

    HalChassis_ConfigPwmTimer(APP_BOARD_MOTOR_LEFT_PWM_INST);
    HalChassis_ConfigPwmTimer(APP_BOARD_MOTOR_RIGHT_PWM_INST);

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        HalBoard_InitPeripheralOutput(&g_motor_hw[i].pin, g_motor_hw[i].pwm_func);
    }
}

static void HalChassis_ApplyWheelDuty(uint8_t wheel_index, int16_t duty)
{
    const HalChassis_MotorHw_t *motor;
    uint16_t duty_abs;
    uint32_t on_compare;
    uint32_t off_compare;

    if (wheel_index >= APP_BOARD_WHEEL_COUNT) {
        return;
    }

    if (g_last_wheel_duty[wheel_index] == duty) {
        return;
    }

    motor = &g_motor_hw[wheel_index];
    if (motor->inst == 0) {
        return;
    }

    duty_abs = (uint16_t)((duty >= 0) ? duty : -duty);
    on_compare = HalChassis_AbsDutyToCompare(motor->inst, duty_abs);
    off_compare = HalChassis_GetPwmOffCompare(motor->inst);

    HalChassis_SetMotorDirectionBits(wheel_index, (duty > 0) ? 1 : ((duty < 0) ? -1 : 0));
    DL_Timer_setCaptureCompareValue(motor->inst,
        (duty_abs > 0U) ? on_compare : off_compare,
        motor->cc_index);
    g_last_wheel_duty[wheel_index] = duty;
    g_shift595_dirty = 1U;
}

static uint8_t HalChassis_GetQuadState(uint8_t index_a, uint8_t index_b)
{
    return (uint8_t)((g_encoder_state.raw_level[index_a] << 1) | g_encoder_state.raw_level[index_b]);
}

static uint8_t HalChassis_GetWheelState(uint8_t wheel_index)
{
    switch (wheel_index) {
    case APP_BOARD_WHEEL_FRONT_RIGHT_INDEX:
        return HalChassis_GetQuadState(APP_BOARD_WHEEL0_ENCODER_A_INDEX, APP_BOARD_WHEEL0_ENCODER_B_INDEX);
    case APP_BOARD_WHEEL_REAR_RIGHT_INDEX:
        return HalChassis_GetQuadState(APP_BOARD_WHEEL1_ENCODER_A_INDEX, APP_BOARD_WHEEL1_ENCODER_B_INDEX);
    case APP_BOARD_WHEEL_FRONT_LEFT_INDEX:
        return HalChassis_GetQuadState(APP_BOARD_WHEEL2_ENCODER_A_INDEX, APP_BOARD_WHEEL2_ENCODER_B_INDEX);
    case APP_BOARD_WHEEL_REAR_LEFT_INDEX:
    default:
        return HalChassis_GetQuadState(APP_BOARD_WHEEL3_ENCODER_A_INDEX, APP_BOARD_WHEEL3_ENCODER_B_INDEX);
    }
}

static void HalChassis_UpdateWheelTicks(uint8_t wheel_index, uint8_t new_state)
{
    static const int8_t quad_table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
    uint8_t old_state = g_prev_quad_state[wheel_index];
    uint8_t table_index = (uint8_t)((old_state << 2) | new_state);
    int8_t step = quad_table[table_index];

    if (new_state == old_state) {
        return;
    }

    if (step != 0) {
        g_encoder_state.wheel_ticks[wheel_index] +=
            (int32_t)(step * HalChassis_GetWheelDirectionSign(wheel_index));
        g_prev_quad_state[wheel_index] = new_state;
    }
}

static void HalChassis_ShiftOut595(uint8_t value)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++) {
        uint8_t data_bit = (uint8_t)((value & (uint8_t)(0x80U >> bit)) != 0U);
        HalBoard_WritePin(&g_hc595_ds_pin, data_bit);
        HalBoard_WritePin(&g_hc595_shcp_pin, 1U);
        HalBoard_WritePin(&g_hc595_shcp_pin, 0U);
    }

    HalBoard_WritePin(&g_hc595_stcp_pin, 1U);
    HalBoard_WritePin(&g_hc595_stcp_pin, 0U);
}

void HalChassis_Init(void)
{
    uint8_t i;

    if (APP_BOARD_ENABLE_MOTOR_PWM_DIRECT) {
        HalChassis_InitMotorPwm();
    } else {
        for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
            HalBoard_InitOutput(&g_motor_hw[i].pin, 0U);
        }
    }

    HalBoard_InitOutput(&g_hc595_ds_pin, 0U);
    HalBoard_InitOutput(&g_hc595_shcp_pin, 0U);
    HalBoard_InitOutput(&g_hc595_stcp_pin, 0U);

    for (i = 0U; i < APP_BOARD_ENCODER_RAW_COUNT; i++) {
        HalBoard_InitInputPullup(&g_encoder_pins[i]);
        HalChassis_EnableEncoderInputFilter(&g_encoder_pins[i]);
        g_encoder_state.raw_level[i] = HalBoard_ReadPin(&g_encoder_pins[i]);
        g_encoder_state.raw_edges[i] = 0U;
    }

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        g_encoder_state.wheel_ticks[i] = 0;
        g_prev_quad_state[i] = HalChassis_GetWheelState(i);
        g_last_wheel_duty[i] = 0;
        g_target_wheel_duty[i] = 0;
        g_motor_state.wheel_duty[i] = 0;
    }

    g_motor_state.left_duty = 0;
    g_motor_state.right_duty = 0;
    g_motor_state.pwm_online = APP_BOARD_ENABLE_MOTOR_PWM_DIRECT;

    g_shift595_shadow = 0U;
    g_shift595_dirty = 1U;

    HalChassis_ConfigEncoderInterrupts();
    HalChassis_ShiftOut595(0U);
    g_shift595_dirty = 0U;
}

void HalChassis_Tick10ms(void)
{
    if (g_shift595_dirty) {
        HalChassis_ShiftOut595(g_shift595_shadow);
        g_shift595_dirty = 0U;
    }
}

void HalChassis_Service1ms(void)
{
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending_mask;
    uint32_t group_iidx;
    uint8_t i;

    group_iidx = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);
    if ((group_iidx != DL_INTERRUPT_GROUP1_IIDX_GPIOB) &&
        (group_iidx != DL_INTERRUPT_GROUP1_IIDX_GPIOA)) {
        return;
    }

    pending_mask = DL_GPIO_getEnabledInterruptStatus(GPIOB, g_encoder_gpiob_mask);
    if (pending_mask == 0U) {
        return;
    }

    DL_GPIO_clearInterruptStatus(GPIOB, pending_mask);

    for (i = 0U; i < APP_BOARD_ENCODER_RAW_COUNT; i++) {
        if ((pending_mask & g_encoder_pins[i].pin) != 0U) {
            HalChassis_RefreshEncoderLevel(i);
        }
    }

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        HalChassis_UpdateWheelTicks(i, HalChassis_GetWheelState(i));
    }
}

void HalChassis_SetMotorDuty(int16_t left_duty, int16_t right_duty)
{
    int16_t wheel_duty[APP_BOARD_WHEEL_COUNT];

    wheel_duty[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] = right_duty;
    wheel_duty[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] = right_duty;
    wheel_duty[APP_BOARD_WHEEL_REAR_LEFT_INDEX] = left_duty;
    wheel_duty[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] = left_duty;

    HalChassis_SetWheelDuty(wheel_duty);
}

void HalChassis_SetWheelDuty(const int16_t *wheel_duty)
{
    uint8_t i;
    int16_t applied_duty;

    if (wheel_duty == 0) {
        return;
    }

    g_motor_state.left_duty = 0;
    g_motor_state.right_duty = 0;

    if (APP_BOARD_ENABLE_MOTOR_PWM_DIRECT) {
        for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
            g_target_wheel_duty[i] =
                HalChassis_GetWheelCompensatedDuty(i, HalChassis_ClampDuty(wheel_duty[i]));
            applied_duty = HalChassis_SlewDuty(g_last_wheel_duty[i], g_target_wheel_duty[i]);
            HalChassis_ApplyWheelDuty(i, applied_duty);
            g_motor_state.wheel_duty[i] = applied_duty;
        }

        if (g_shift595_dirty) {
            HalChassis_ShiftOut595(g_shift595_shadow);
            g_shift595_dirty = 0U;
        }
    } else {
        for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
            HalBoard_WritePin(&g_motor_hw[i].pin, 0U);
            g_motor_state.wheel_duty[i] = 0;
        }
    }

    g_motor_state.right_duty = HalChassis_AverageDuty(
        g_motor_state.wheel_duty[APP_BOARD_WHEEL_REAR_RIGHT_INDEX],
        g_motor_state.wheel_duty[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX]);
    g_motor_state.left_duty = HalChassis_AverageDuty(
        g_motor_state.wheel_duty[APP_BOARD_WHEEL_REAR_LEFT_INDEX],
        g_motor_state.wheel_duty[APP_BOARD_WHEEL_FRONT_LEFT_INDEX]);
}

void HalChassis_GetMotorState(HalChassis_MotorState_t *state)
{
    if (state == 0) {
        return;
    }

    *state = g_motor_state;
}

void HalChassis_GetEncoderState(HalChassis_EncoderState_t *state)
{
    if (state == 0) {
        return;
    }

    *state = g_encoder_state;
}

void HalChassis_SetRgb(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t new_mask = 0U;

    if (!APP_BOARD_ENABLE_RGB_595) {
        return;
    }

    if (red > 0U) {
        new_mask |= (uint8_t)(1U << APP_BOARD_RGB_RED_BIT);
    }
    if (green > 0U) {
        new_mask |= (uint8_t)(1U << APP_BOARD_RGB_GREEN_BIT);
    }
    if (blue > 0U) {
        new_mask |= (uint8_t)(1U << APP_BOARD_RGB_BLUE_BIT);
    }

    if (new_mask != g_shift595_shadow) {
        g_shift595_shadow = new_mask;
        g_shift595_dirty = 1U;
    }
}
