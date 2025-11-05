#ifndef _MAGNETIC_LEVITATION_H
#define _MAGNETIC_LEVITATION_H

#include "usrdef.h"
#include <board.h>


#define DUTY_CYCLE_LIMIT 0.5
#define ENABLE_ADC_VALUE 100.0
#define ADJUST_LIMIT 200.0
#define ADJUST_ERR_LIMIT 50.0
#define EX_DEFAULT 1203.0
#define EY_DEFAULT 1194.0
#define EZ_DEFAULT 1240.0

#define ADC_DEV "adc1"
#define HALL_X 3
#define HALL_Y 4
#define HALL_Z 5

#define PWM_DEV  "pwm3"
#define PWM_PERIOD  50000
#define PWM_A     2
#define PWM_B     1
#define PWM_C     3
#define PWM_D     4

#define TIMER_DEV  "timer4"

#define STBY1 GPIO_GET_PIN(B, 5)
#define STBY2 GPIO_GET_PIN(B, 12)

#define AIN1 GPIO_GET_PIN(B, 7)
#define AIN2 GPIO_GET_PIN(B, 6)

#define BIN1 GPIO_GET_PIN(B, 8)
#define BIN2 GPIO_GET_PIN(B, 9)

#define CIN1 GPIO_GET_PIN(A, 8)
#define CIN2 GPIO_GET_PIN(B, 15)

#define DIN1 GPIO_GET_PIN(B, 14)
#define DIN2 GPIO_GET_PIN(B, 13)


#define DEBUG_UART  "uart1"
#define CH_COUNT 10

typedef struct {
    float32_t data[CH_COUNT];
    uint8_t tail[4];
}vofa_just_float_t;

#endif
