#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

typedef enum
{
    BUZZER_PATTERN_CLICK = 0,
    BUZZER_PATTERN_CONFIRM,
    BUZZER_PATTERN_STARTUP,
} BuzzerPattern_e;

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Play(BuzzerPattern_e pattern);
void Buzzer_Tick1ms(void);

#endif
