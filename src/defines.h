#pragma once

#define UART_BAUD             115200                  // UART3 baud rate (short wired cable)

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

// PA11 - LEDR
#define LEDR_OFF()      {GPIOA->BRR = 1<<11;}
#define LEDR_ON()       {GPIOA->BSRR = 1<<11;}
#define LEDR_TOGGLE()  {(GPIOA->ODR & (1<<11)) ? (GPIOA->BRR= 1<<11) : (GPIOA->BSRR =1<<11);}

// PD3 - LEDG
#define LEDG_OFF()      {GPIOD->BRR= 1<<3;}
#define LEDG_ON()       {GPIOD->BSRR = 1<<3;}
#define LEDG_TOGGLE()  {(GPIOD->ODR & (1<<3)) ? (GPIOD->BRR= 1<<3) : (GPIOD->BSRR = 1<<3);}

// PD2 - LEDB
#define LEDB_OFF()     {GPIOD->BRR = 1<<2;}
#define LEDB_ON()      {GPIOD->BSRR = 1<<2;}
#define LEDB_TOGGLE() {(GPIOD->ODR & (1<<2)) ? (GPIOD->BRR=1<<2) : (GPIOD->BSRR = 1<<2);}

// PB10 - LEDU
#define LEDU_OFF()      {GPIOB->BRR = 1<<10;}
#define LEDU_ON()       {GPIOB->BSRR = 1<<10;}
#define LEDU_TOGGLE()  {(GPIOB->ODR & (1<<10)) ? (GPIOB->BRR=1<<10) : (GPIOB->BSRR = 1<<10);}
