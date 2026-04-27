#include "ARCH/hal/hal_board.h"

void HalBoard_InitOutput(const HalBoard_Pin_t *pin, uint8_t initial_high)
{
    if (pin == 0) {
        return;
    }

    DL_GPIO_initDigitalOutput(pin->iomux);
    if (initial_high) {
        DL_GPIO_setPins(pin->port, pin->pin);
    } else {
        DL_GPIO_clearPins(pin->port, pin->pin);
    }
    DL_GPIO_enableOutput(pin->port, pin->pin);
}

void HalBoard_InitInputPullup(const HalBoard_Pin_t *pin)
{
    if (pin == 0) {
        return;
    }

    DL_GPIO_initDigitalInputFeatures(pin->iomux,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

void HalBoard_InitPeripheralOutput(const HalBoard_Pin_t *pin, uint32_t function)
{
    if (pin == 0) {
        return;
    }

    DL_GPIO_initPeripheralOutputFunction(pin->iomux, function);
}

void HalBoard_InitPeripheralInput(const HalBoard_Pin_t *pin, uint32_t function)
{
    if (pin == 0) {
        return;
    }

    DL_GPIO_initPeripheralInputFunction(pin->iomux, function);
}

void HalBoard_WritePin(const HalBoard_Pin_t *pin, uint8_t high)
{
    if (pin == 0) {
        return;
    }

    if (high) {
        DL_GPIO_setPins(pin->port, pin->pin);
    } else {
        DL_GPIO_clearPins(pin->port, pin->pin);
    }
}

uint8_t HalBoard_ReadPin(const HalBoard_Pin_t *pin)
{
    if (pin == 0) {
        return 0U;
    }

    return (DL_GPIO_readPins(pin->port, pin->pin) != 0U) ? 1U : 0U;
}
