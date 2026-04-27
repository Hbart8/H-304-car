#include "Buzzer.h"

#include "ti_msp_dl_config.h"

#define BUZZER_PORT  (GPIOA)
#define BUZZER_PIN   (DL_GPIO_PIN_31)
#define BUZZER_IOMUX (IOMUX_PINCM6)

typedef struct
{
    uint16_t segments[6];
    uint8_t segment_count;
    uint8_t segment_index;
    uint16_t remain_ms;
    uint8_t active;
} BuzzerState_t;

static volatile BuzzerState_t g_buzzer = {0};

static void Buzzer_LoadSequence(const uint16_t *segments, uint8_t count)
{
    uint32_t primask;
    uint8_t i;

    if (count > (uint8_t)(sizeof(g_buzzer.segments) / sizeof(g_buzzer.segments[0]))) {
        count = (uint8_t)(sizeof(g_buzzer.segments) / sizeof(g_buzzer.segments[0]));
    }

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0U; i < count; i++) {
        g_buzzer.segments[i] = segments[i];
    }

    g_buzzer.segment_count = count;
    g_buzzer.segment_index = 0U;
    g_buzzer.remain_ms     = (count > 0U) ? g_buzzer.segments[0] : 0U;
    g_buzzer.active        = (count > 0U) ? 1U : 0U;

    if (g_buzzer.active) {
        Buzzer_On();
    } else {
        Buzzer_Off();
    }

    __set_PRIMASK(primask);
}

void Buzzer_Init(void)
{
    DL_GPIO_initDigitalOutput(BUZZER_IOMUX);
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
    DL_GPIO_enableOutput(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer_On(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer_Off(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer_Play(BuzzerPattern_e pattern)
{
    static const uint16_t click_pattern[]   = {20U};
    static const uint16_t confirm_pattern[] = {35U, 30U, 35U};
    static const uint16_t startup_pattern[] = {35U, 35U, 55U};

    switch (pattern) {
    case BUZZER_PATTERN_CLICK:
        Buzzer_LoadSequence(click_pattern,
                            (uint8_t)(sizeof(click_pattern) / sizeof(click_pattern[0])));
        break;
    case BUZZER_PATTERN_CONFIRM:
        Buzzer_LoadSequence(confirm_pattern,
                            (uint8_t)(sizeof(confirm_pattern) / sizeof(confirm_pattern[0])));
        break;
    case BUZZER_PATTERN_STARTUP:
    default:
        Buzzer_LoadSequence(startup_pattern,
                            (uint8_t)(sizeof(startup_pattern) / sizeof(startup_pattern[0])));
        break;
    }
}

void Buzzer_Tick1ms(void)
{
    if (!g_buzzer.active) {
        return;
    }

    if (g_buzzer.remain_ms > 0U) {
        g_buzzer.remain_ms--;
    }

    if (g_buzzer.remain_ms > 0U) {
        return;
    }

    g_buzzer.segment_index++;
    if (g_buzzer.segment_index >= g_buzzer.segment_count) {
        g_buzzer.active = 0U;
        Buzzer_Off();
        return;
    }

    g_buzzer.remain_ms = g_buzzer.segments[g_buzzer.segment_index];
    if ((g_buzzer.segment_index & 0x01U) == 0U) {
        Buzzer_On();
    } else {
        Buzzer_Off();
    }
}
