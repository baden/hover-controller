#pragma once

#include <stdint.h>

// REMOTE_CONTROL_MODE:
// 0 - UART control mode (default)
// 1 - UART pins PB4(UART1_RX) used as + button, PB6(UART1_TX) used as - button
#define REMOTE_CONTROL_MODE_UART 0
#define REMOTE_CONTROL_MODE_UART_PINS 1

#define REMOTE_CONTROL_MODE REMOTE_CONTROL_MODE_UART_PINS

#if REMOTE_CONTROL_MODE == REMOTE_CONTROL_MODE_UART
#define UART_BAUD             115200                  // UART3 baud rate (short wired cable)
#endif

// =========================================================
//  POWER GPIO
// =========================================================
#define BUTTON_PORT GPIOB
#define BUTTON_PIN  5
#define OFF_PORT    GPIOB
#define OFF_PIN     2

// =========================================================
//  LED GPIO
// =========================================================

// PA11 - LEDG (DG)
#define LEDG_OFF()      {GPIOA->BRR = 1<<11;}
#define LEDG_ON()       {GPIOA->BSRR = 1<<11;}
#define LEDG_TOGGLE()  {(GPIOA->ODR & (1<<11)) ? (GPIOA->BRR= 1<<11) : (GPIOA->BSRR =1<<11);}

// PD3 - LEDY (DY)
#define LEDY_OFF()      {GPIOD->BRR= 1<<3;}
#define LEDY_ON()       {GPIOD->BSRR = 1<<3;}
#define LEDY_TOGGLE()  {(GPIOD->ODR & (1<<3)) ? (GPIOD->BRR= 1<<3) : (GPIOD->BSRR = 1<<3);}

// PD2 - LEDR (DR)
#define LEDR_OFF()     {GPIOD->BRR = 1<<2;}
#define LEDR_ON()      {GPIOD->BSRR = 1<<2;}
#define LEDR_TOGGLE() {(GPIOD->ODR & (1<<2)) ? (GPIOD->BRR=1<<2) : (GPIOD->BSRR = 1<<2);}

// PB10 - LEDU
#define LEDU_OFF()      {GPIOB->BRR = 1<<10;}
#define LEDU_ON()       {GPIOB->BSRR = 1<<10;}
#define LEDU_TOGGLE()  {(GPIOB->ODR & (1<<10)) ? (GPIOB->BRR=1<<10) : (GPIOB->BSRR = 1<<10);}


#define PWM_FREQ            12000     // PWM frequency in Hz / is also used for buzzer
#define PWM_RES   (72000000 / 2 / PWM_FREQ)          // = 2250
#define DEAD_TIME             30 // 48     // PWM deadtime
#define A2BIT_CONV             50     // A to bit for current conversion on ADC. Example: 1 A = 50, 2 A = 100, etc

// Limitation settings
#define I_MOT_MAX       15              // [A] Maximum single motor current limit
#define I_DC_MAX        17              // [A] Maximum stage2 DC Link current limit for Commutation and Sinusoidal types (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
#define N_MOT_MAX       1000            // [rpm] Maximum motor speed limit


#define CURR_DC_MAX (I_DC_MAX * A2BIT_CONV)

typedef struct {
    uint16_t dcr;
    uint16_t rrB;
    uint16_t rrC;
    uint16_t batt1;
    uint16_t temp;
} adc_buf_t;

#define ABS(a) (((a) < 0) ? -(a) : (a))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
