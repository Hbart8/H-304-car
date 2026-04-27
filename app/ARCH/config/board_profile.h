#ifndef APP_ARCH_BOARD_PROFILE_H
#define APP_ARCH_BOARD_PROFILE_H

#include "ti_msp_dl_config.h"

/*
 * Board profile keeps all hardware resource assumptions in one place.
 * If the PCB or chip migration changes, we adjust this file first.
 */

#define APP_BOARD_ENABLE_MOTOR_PWM_DIRECT         (1U)
#define APP_BOARD_ENABLE_TRACK_ADC_DIRECT         (0U)
#define APP_BOARD_ENABLE_GYRO_I2C                 (1U)
#define APP_BOARD_ENABLE_RGB_595                  (0U)

#define APP_BOARD_ENCODER_RAW_COUNT               (8U)
#define APP_BOARD_WHEEL_COUNT                     (4U)
#define APP_BOARD_SIDE_COUNT                      (2U)

#define APP_BOARD_MOTOR0_PWM_PORT                 (GPIOA)
#define APP_BOARD_MOTOR0_PWM_PIN                  (DL_GPIO_PIN_0)
#define APP_BOARD_MOTOR0_PWM_IOMUX               (IOMUX_PINCM1)
#define APP_BOARD_MOTOR0_PWM_FUNC                (IOMUX_PINCM1_PF_TIMA0_CCP0)

#define APP_BOARD_MOTOR1_PWM_PORT                 (GPIOA)
#define APP_BOARD_MOTOR1_PWM_PIN                  (DL_GPIO_PIN_1)
#define APP_BOARD_MOTOR1_PWM_IOMUX               (IOMUX_PINCM2)
#define APP_BOARD_MOTOR1_PWM_FUNC                (IOMUX_PINCM2_PF_TIMA0_CCP1)

#define APP_BOARD_MOTOR2_PWM_PORT                 (GPIOA)
#define APP_BOARD_MOTOR2_PWM_PIN                  (DL_GPIO_PIN_7)
#define APP_BOARD_MOTOR2_PWM_IOMUX               (IOMUX_PINCM14)
#define APP_BOARD_MOTOR2_PWM_FUNC                (IOMUX_PINCM14_PF_TIMG8_CCP0)

#define APP_BOARD_MOTOR3_PWM_PORT                 (GPIOA)
#define APP_BOARD_MOTOR3_PWM_PIN                  (DL_GPIO_PIN_22)
#define APP_BOARD_MOTOR3_PWM_IOMUX               (IOMUX_PINCM47)
#define APP_BOARD_MOTOR3_PWM_FUNC                (IOMUX_PINCM47_PF_TIMG8_CCP1)

#define APP_BOARD_MOTOR_LEFT_PWM_INST             (TIMA0)
#define APP_BOARD_MOTOR_RIGHT_PWM_INST            (TIMG8)
#define APP_BOARD_MOTOR_PWM_CLOCK_SEL             (DL_TIMER_CLOCK_BUSCLK)
#define APP_BOARD_MOTOR_PWM_CLOCK_DIVIDE          (DL_TIMER_CLOCK_DIVIDE_8)
#define APP_BOARD_MOTOR_PWM_CLOCK_PRESCALE        (0U)
#define APP_BOARD_MOTOR_PWM_PERIOD_COUNTS         (3000U)
#define APP_BOARD_MOTOR_PWM_MODE                  (DL_TIMER_PWM_MODE_EDGE_ALIGN)

#define APP_BOARD_MOTOR0_CC_INDEX                 (DL_TIMER_CC_0_INDEX)
#define APP_BOARD_MOTOR1_CC_INDEX                 (DL_TIMER_CC_1_INDEX)
#define APP_BOARD_MOTOR2_CC_INDEX                 (DL_TIMER_CC_0_INDEX)
#define APP_BOARD_MOTOR3_CC_INDEX                 (DL_TIMER_CC_1_INDEX)

#define APP_BOARD_MOTOR0_PWM_INST                 (TIMA0)
#define APP_BOARD_MOTOR1_PWM_INST                 (TIMA0)
#define APP_BOARD_MOTOR2_PWM_INST                 (TIMG8)
#define APP_BOARD_MOTOR3_PWM_INST                 (TIMG8)

#define APP_BOARD_MOTOR0_IN1_BIT                  (0U)
#define APP_BOARD_MOTOR0_IN2_BIT                  (1U)
#define APP_BOARD_MOTOR1_IN1_BIT                  (2U)
#define APP_BOARD_MOTOR1_IN2_BIT                  (3U)
#define APP_BOARD_MOTOR2_IN1_BIT                  (4U)
#define APP_BOARD_MOTOR2_IN2_BIT                  (5U)
#define APP_BOARD_MOTOR3_IN1_BIT                  (6U)
#define APP_BOARD_MOTOR3_IN2_BIT                  (7U)

#define APP_BOARD_MOTOR0_FORWARD_IN1_HIGH         (1U)
#define APP_BOARD_MOTOR1_FORWARD_IN1_HIGH         (0U)
#define APP_BOARD_MOTOR2_FORWARD_IN1_HIGH         (0U)
#define APP_BOARD_MOTOR3_FORWARD_IN1_HIGH         (1U)

#define APP_BOARD_ENCODER_0_PORT                  (GPIOB)
#define APP_BOARD_ENCODER_0_PIN                   (DL_GPIO_PIN_4)
#define APP_BOARD_ENCODER_0_IOMUX                (IOMUX_PINCM17)

#define APP_BOARD_ENCODER_1_PORT                  (GPIOB)
#define APP_BOARD_ENCODER_1_PIN                   (DL_GPIO_PIN_5)
#define APP_BOARD_ENCODER_1_IOMUX                (IOMUX_PINCM18)

#define APP_BOARD_ENCODER_2_PORT                  (GPIOB)
#define APP_BOARD_ENCODER_2_PIN                   (DL_GPIO_PIN_6)
#define APP_BOARD_ENCODER_2_IOMUX                (IOMUX_PINCM23)

#define APP_BOARD_ENCODER_3_PORT                  (GPIOB)
#define APP_BOARD_ENCODER_3_PIN                   (DL_GPIO_PIN_7)
#define APP_BOARD_ENCODER_3_IOMUX                (IOMUX_PINCM24)

#define APP_BOARD_ENCODER_4_PORT                  (GPIOB)
#define APP_BOARD_ENCODER_4_PIN                   (DL_GPIO_PIN_19)
#define APP_BOARD_ENCODER_4_IOMUX                (IOMUX_PINCM45)

#define APP_BOARD_ENCODER_5_PORT                  (GPIOB)
#define APP_BOARD_ENCODER_5_PIN                   (DL_GPIO_PIN_18)
#define APP_BOARD_ENCODER_5_IOMUX                (IOMUX_PINCM44)

#define APP_BOARD_ENCODER_6_PORT                  (GPIOB)
#define APP_BOARD_ENCODER_6_PIN                   (DL_GPIO_PIN_23)
#define APP_BOARD_ENCODER_6_IOMUX                (IOMUX_PINCM51)

#define APP_BOARD_ENCODER_7_PORT                  (GPIOB)
#define APP_BOARD_ENCODER_7_PIN                   (DL_GPIO_PIN_13)
#define APP_BOARD_ENCODER_7_IOMUX                (IOMUX_PINCM30)

/*
 * Four-wheel mapping assumption based on current bring-up observation:
 * PB4/PB5   -> rear-right wheel
 * PB6/PB7   -> front-right wheel
 * PB19/PB18 -> rear-left wheel
 * PB23/PB13 -> front-left wheel
 */
#define APP_BOARD_WHEEL_REAR_RIGHT_INDEX          (0U)
#define APP_BOARD_WHEEL_FRONT_RIGHT_INDEX         (1U)
#define APP_BOARD_WHEEL_REAR_LEFT_INDEX           (2U)
#define APP_BOARD_WHEEL_FRONT_LEFT_INDEX          (3U)

#define APP_BOARD_SIDE_LEFT_INDEX                 (0U)
#define APP_BOARD_SIDE_RIGHT_INDEX                (1U)

#define APP_BOARD_WHEEL0_ENCODER_A_INDEX          (0U)
#define APP_BOARD_WHEEL0_ENCODER_B_INDEX          (1U)
#define APP_BOARD_WHEEL1_ENCODER_A_INDEX          (2U)
#define APP_BOARD_WHEEL1_ENCODER_B_INDEX          (3U)
#define APP_BOARD_WHEEL2_ENCODER_A_INDEX          (4U)
#define APP_BOARD_WHEEL2_ENCODER_B_INDEX          (5U)
#define APP_BOARD_WHEEL3_ENCODER_A_INDEX          (6U)
#define APP_BOARD_WHEEL3_ENCODER_B_INDEX          (7U)

#define APP_BOARD_595_DS_PORT                     (GPIOA)
#define APP_BOARD_595_DS_PIN                      (DL_GPIO_PIN_25)
#define APP_BOARD_595_DS_IOMUX                   (IOMUX_PINCM55)

#define APP_BOARD_595_SHCP_PORT                   (GPIOA)
#define APP_BOARD_595_SHCP_PIN                    (DL_GPIO_PIN_15)
#define APP_BOARD_595_SHCP_IOMUX                 (IOMUX_PINCM37)

#define APP_BOARD_595_STCP_PORT                   (GPIOA)
#define APP_BOARD_595_STCP_PIN                    (DL_GPIO_PIN_14)
#define APP_BOARD_595_STCP_IOMUX                 (IOMUX_PINCM36)

#define APP_BOARD_RGB_RED_BIT                     (0U)
#define APP_BOARD_RGB_GREEN_BIT                   (1U)
#define APP_BOARD_RGB_BLUE_BIT                    (2U)

#define APP_BOARD_TRACK_ADC_PORT                  (GPIOA)
#define APP_BOARD_TRACK_ADC_PIN                   (DL_GPIO_PIN_27)
#define APP_BOARD_TRACK_ADC_IOMUX                (IOMUX_PINCM60)

#define APP_BOARD_TRACK_SEL0_PORT                 (GPIOA)
#define APP_BOARD_TRACK_SEL0_PIN                  (DL_GPIO_PIN_24)
#define APP_BOARD_TRACK_SEL0_IOMUX               (IOMUX_PINCM54)

#define APP_BOARD_TRACK_SEL1_PORT                 (GPIOA)
#define APP_BOARD_TRACK_SEL1_PIN                  (DL_GPIO_PIN_30)
#define APP_BOARD_TRACK_SEL1_IOMUX               (IOMUX_PINCM5)

#define APP_BOARD_TRACK_SEL2_PORT                 (GPIOA)
#define APP_BOARD_TRACK_SEL2_PIN                  (DL_GPIO_PIN_29)
#define APP_BOARD_TRACK_SEL2_IOMUX               (IOMUX_PINCM4)

/*
 * Gyro software-I2C assumption from the current PCB notes:
 * SCL -> PA8, SDA -> PA26.
 * If field verification changes, adjust only this block.
 */
#define APP_BOARD_GYRO_I2C_SCL_PORT               (GPIOA)
#define APP_BOARD_GYRO_I2C_SCL_PIN                (DL_GPIO_PIN_8)
#define APP_BOARD_GYRO_I2C_SCL_IOMUX            (IOMUX_PINCM19)

#define APP_BOARD_GYRO_I2C_SDA_PORT               (GPIOA)
#define APP_BOARD_GYRO_I2C_SDA_PIN                (DL_GPIO_PIN_26)
#define APP_BOARD_GYRO_I2C_SDA_IOMUX            (IOMUX_PINCM59)

#define APP_BOARD_GYRO_I2C_DELAY_CYCLES           (48U)
#define APP_BOARD_GYRO_RETRY_PERIOD_MS            (200U)

#define APP_BOARD_DEBUG_UART_INST                 (UART3)
#define APP_BOARD_DEBUG_UART_TX_PORT              (GPIOB)
#define APP_BOARD_DEBUG_UART_TX_PIN               (DL_GPIO_PIN_2)
#define APP_BOARD_DEBUG_UART_TX_IOMUX            (IOMUX_PINCM15)
#define APP_BOARD_DEBUG_UART_TX_FUNC             (IOMUX_PINCM15_PF_UART3_TX)

#define APP_BOARD_DEBUG_UART_RX_PORT              (GPIOB)
#define APP_BOARD_DEBUG_UART_RX_PIN               (DL_GPIO_PIN_3)
#define APP_BOARD_DEBUG_UART_RX_IOMUX            (IOMUX_PINCM16)
#define APP_BOARD_DEBUG_UART_RX_FUNC             (IOMUX_PINCM16_PF_UART3_RX)

#define APP_BOARD_DEBUG_UART_BAUD                 (9600U)
#define APP_BOARD_DEBUG_UART_IBRD                 (208U)
#define APP_BOARD_DEBUG_UART_FBRD                 (21U)

#endif
