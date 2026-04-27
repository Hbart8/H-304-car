#ifndef APP_ARCH_HAL_BOARD_H
#define APP_ARCH_HAL_BOARD_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

typedef struct
{
    GPIO_Regs *port;
    uint32_t pin;
    IOMUX_PINCM iomux;
} HalBoard_Pin_t;

void HalBoard_InitOutput(const HalBoard_Pin_t *pin, uint8_t initial_high);
void HalBoard_InitInputPullup(const HalBoard_Pin_t *pin);
void HalBoard_InitPeripheralOutput(const HalBoard_Pin_t *pin, uint32_t function);
void HalBoard_InitPeripheralInput(const HalBoard_Pin_t *pin, uint32_t function);
void HalBoard_WritePin(const HalBoard_Pin_t *pin, uint8_t high);
uint8_t HalBoard_ReadPin(const HalBoard_Pin_t *pin);

#endif
