#include "Serial.h"

#include <stdarg.h>

#include "ARCH/config/app_config.h"
#include "ARCH/hal/hal_board.h"

typedef struct
{
    uint8_t tx_buffer[APP_UART_TX_BUFFER_SIZE];
    uint16_t tx_head;
    uint16_t tx_tail;
    uint8_t initialized;
} Serial_Runtime_t;

static Serial_Runtime_t g_serial;
uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

static const HalBoard_Pin_t g_debug_uart_tx_pin = {
    APP_BOARD_DEBUG_UART_TX_PORT,
    APP_BOARD_DEBUG_UART_TX_PIN,
    APP_BOARD_DEBUG_UART_TX_IOMUX
};

static const HalBoard_Pin_t g_debug_uart_rx_pin = {
    APP_BOARD_DEBUG_UART_RX_PORT,
    APP_BOARD_DEBUG_UART_RX_PIN,
    APP_BOARD_DEBUG_UART_RX_IOMUX
};

static uint16_t Serial_GetNextIndex(uint16_t index)
{
    index++;
    if (index >= APP_UART_TX_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}

static uint8_t Serial_IsTxBufferFull(void)
{
    return (uint8_t)(Serial_GetNextIndex(g_serial.tx_head) == g_serial.tx_tail);
}

static void Serial_PushTxByte(uint8_t byte_data)
{
    uint16_t next_head;

    if (!g_serial.initialized) {
        return;
    }

    next_head = Serial_GetNextIndex(g_serial.tx_head);
    if (next_head == g_serial.tx_tail) {
        return;
    }

    g_serial.tx_buffer[g_serial.tx_head] = byte_data;
    g_serial.tx_head = next_head;
}

static uint8_t Serial_PopTxByte(uint8_t *byte_data)
{
    if ((byte_data == 0) || (g_serial.tx_head == g_serial.tx_tail)) {
        return 0U;
    }

    *byte_data = g_serial.tx_buffer[g_serial.tx_tail];
    g_serial.tx_tail = Serial_GetNextIndex(g_serial.tx_tail);
    return 1U;
}

static void Serial_PrintInt(int num)
{
    char buf[12];
    int i = 0;
    uint32_t n;

    if (num < 0)
    {
        Serial_SendByte('-');
        n = (uint32_t)(-(num + 1)) + 1U;
    }
    else
    {
        n = (uint32_t)num;
    }

    if (n == 0U)
    {
        Serial_SendByte('0');
        return;
    }

    while ((n > 0U) && (i < (int)sizeof(buf)))
    {
        buf[i++] = (char)('0' + (n % 10U));
        n /= 10U;
    }

    while (i > 0)
    {
        Serial_SendByte((uint8_t)buf[--i]);
    }
}

static void Serial_PrintHex(uint32_t num)
{
    char buf[9];
    int i = 0;
    uint8_t digit;

    if (num == 0U)
    {
        Serial_SendByte('0');
        return;
    }

    while ((num > 0U) && (i < (int)sizeof(buf)))
    {
        digit = (uint8_t)(num % 16U);
        if (digit < 10U) {
            buf[i++] = (char)('0' + digit);
        } else {
            buf[i++] = (char)('A' + digit - 10U);
        }
        num /= 16U;
    }

    while (i > 0)
    {
        Serial_SendByte((uint8_t)buf[--i]);
    }
}

void Serial_Init(void)
{
    static const DL_UART_Main_ClockConfig clock_config = {
        .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    };
    static const DL_UART_Main_Config uart_config = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity      = DL_UART_MAIN_PARITY_NONE,
        .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
    };

    g_serial.tx_head = 0U;
    g_serial.tx_tail = 0U;
    g_serial.initialized = 0U;
    Serial_RxData = 0U;
    Serial_RxFlag = 0U;

    HalBoard_InitPeripheralOutput(&g_debug_uart_tx_pin, APP_BOARD_DEBUG_UART_TX_FUNC);
    HalBoard_InitPeripheralInput(&g_debug_uart_rx_pin, APP_BOARD_DEBUG_UART_RX_FUNC);

    DL_UART_Main_reset(APP_BOARD_DEBUG_UART_INST);
    DL_UART_Main_enablePower(APP_BOARD_DEBUG_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_UART_Main_setClockConfig(APP_BOARD_DEBUG_UART_INST, (DL_UART_Main_ClockConfig *)&clock_config);
    DL_UART_Main_init(APP_BOARD_DEBUG_UART_INST, (DL_UART_Main_Config *)&uart_config);
    DL_UART_Main_setOversampling(APP_BOARD_DEBUG_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(APP_BOARD_DEBUG_UART_INST,
        APP_BOARD_DEBUG_UART_IBRD,
        APP_BOARD_DEBUG_UART_FBRD);
    DL_UART_Main_enable(APP_BOARD_DEBUG_UART_INST);

    g_serial.initialized = 1U;
}

void Serial_Tick1ms(void)
{
    uint8_t byte_data;

    if (!g_serial.initialized) {
        return;
    }

    while (!DL_UART_isTXFIFOFull(APP_BOARD_DEBUG_UART_INST) && Serial_PopTxByte(&byte_data)) {
        DL_UART_transmitData(APP_BOARD_DEBUG_UART_INST, byte_data);
    }

    while (!DL_UART_isRXFIFOEmpty(APP_BOARD_DEBUG_UART_INST)) {
        Serial_RxData = (uint8_t)DL_UART_receiveData(APP_BOARD_DEBUG_UART_INST);
        Serial_RxFlag = 1U;
    }
}

void Serial_SendByte(uint8_t Byte)
{
    Serial_PushTxByte(Byte);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;

    if (Array == 0) {
        return;
    }

    for (i = 0U; i < Length; i++)
    {
        if (Serial_IsTxBufferFull()) {
            break;
        }
        Serial_SendByte(Array[i]);
    }
}

void Serial_SendString(char *String)
{
    uint16_t i;

    if (String == 0) {
        return;
    }

    for (i = 0U; String[i] != '\0'; i++)
    {
        if (Serial_IsTxBufferFull()) {
            break;
        }
        Serial_SendByte((uint8_t)String[i]);
    }
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1U;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0U; i < Length; i++)
    {
        Serial_SendByte((uint8_t)(Number / Serial_Pow(10U, (uint32_t)Length - i - 1U) % 10U + '0'));
    }
}

void Serial_Printf(char *format, ...)
{
    va_list arg;
    va_start(arg, format);

    while ((format != 0) && (*format != '\0'))
    {
        if (*format == '%')
        {
            format++;
            if (*format == '\0') {
                break;
            }

            switch (*format)
            {
                case 'd':
                    Serial_PrintInt(va_arg(arg, int));
                    break;
                case 'u':
                    Serial_PrintInt((int)va_arg(arg, unsigned int));
                    break;
                case 's':
                    Serial_SendString(va_arg(arg, char *));
                    break;
                case 'c':
                    Serial_SendByte((uint8_t)va_arg(arg, int));
                    break;
                case 'x':
                case 'X':
                    Serial_PrintHex(va_arg(arg, uint32_t));
                    break;
                case '%':
                    Serial_SendByte('%');
                    break;
                default:
                    Serial_SendByte('%');
                    Serial_SendByte((uint8_t)*format);
                    break;
            }
        }
        else
        {
            Serial_SendByte((uint8_t)*format);
        }
        format++;
    }

    va_end(arg);
}

uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1U)
    {
        Serial_RxFlag = 0U;
        return 1U;
    }
    return 0U;
}

uint8_t Serial_GetRxData(void)
{
    return Serial_RxData;
}
