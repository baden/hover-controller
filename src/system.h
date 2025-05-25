#pragma once

#include <stdint.h>

void GPIO_Init(void);
void UART_Init(void);
void delay_Init(void);
void delay_ms(unsigned count);
void TIM1_Init(void);
void ADC1_Init(void);

extern volatile uint32_t millis;
