#include "mm32_device.h"
#include "defines.h"

extern uint32_t SystemCoreClock;

void GPIO_Init(void)
{
    // LEDs PA11, PD3, PD2, PB10
    RCC->AHBENR |= RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB | RCC_AHBENR_GPIOC| RCC_AHBENR_GPIOD;   

    GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_11_Pos);            // PA11
    GPIOA->CRH |= GPIO_CNF_MODE_OUT_PP << GPIO_CRH_CNF_MODE_11_Pos;
    GPIOA->BRR  = GPIO_BRR_BR11;                                                // PA11  output low

    GPIOD->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_3_Pos);             // PD3
    GPIOD->CRL |= GPIO_CNF_MODE_OUT_PP << GPIO_CRL_CNF_MODE_3_Pos;
    GPIOD->BRR  = GPIO_ODR_ODR3;                                                // PD3  output low	       

    GPIOD->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_2_Pos);             // PD2
    GPIOD->CRL |= GPIO_CNF_MODE_OUT_PP << GPIO_CRL_CNF_MODE_2_Pos;
    GPIOD->BRR  = GPIO_BRR_BR2;                                                 // PD2  output high	                                             

    GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_10_Pos);             // PB10
    GPIOB->CRH |= GPIO_CNF_MODE_OUT_PP << GPIO_CRH_CNF_MODE_10_Pos;
    GPIOB->BRR  = GPIO_BRR_BR10;                                                 // PB10  output low	       

    // power enable (latch) - PB2
    GPIOB->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_2_Pos);             // PB2
    GPIOB->CRL |= GPIO_CNF_MODE_OUT_PP << GPIO_CRL_CNF_MODE_2_Pos;
    GPIOB->BRR  = GPIO_BRR_BR2; 

    // button input - PB9
    GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_9_Pos);             // PB9
    GPIOB->CRH |= GPIO_CNF_MODE_FLOATING << GPIO_CRH_CNF_MODE_9_Pos;			

    // buzzer (тільки на slave)
    //   GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_9_Pos);             // PB9
    //   GPIOB->CRH |= GPIO_CNF_MODE_OUT_PP << GPIO_CRH_CNF_MODE_9_Pos;
    //   GPIOB->BRR  = GPIO_BRR_BR9;  	

    // hall sensors PC15,14,13
    GPIOC->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_15_Pos);             
    GPIOC->CRH |= GPIO_CNF_MODE_FLOATING << GPIO_CRH_CNF_MODE_15_Pos;			
    GPIOC->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_14_Pos);             
    GPIOC->CRH |= GPIO_CNF_MODE_FLOATING << GPIO_CRH_CNF_MODE_14_Pos;			
    GPIOC->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_14_Pos);             
    GPIOC->CRH |= GPIO_CNF_MODE_FLOATING << GPIO_CRH_CNF_MODE_14_Pos;			    	
}

void UART_Init(void)
{
	RCC->APB2ENR |=RCC_APB2ENR_UART1;
    RCC->AHBENR |= RCC_AHBENR_GPIOB;  

	// PB4 - UART1_RX, PB6 - UART1_TX
    GPIOB->AFRL	&= ~GPIO_AFRL_AFR6;              // AF0 UART_TX
	GPIOB->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_6_Pos);           
    GPIOB->CRL |= GPIO_CNF_MODE_AF_PP << GPIO_CRL_CNF_MODE_6_Pos;

    GPIOB->AFRL	&= ~GPIO_AFRL_AFR4;	
    GPIOB->AFRL	|= GPIO_AF_MODE3<<GPIO_AFRL_AFR4_Pos;	     // AF3 UART_RX 
	GPIOB->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_4_Pos);           
    GPIOB->CRL |= GPIO_CNF_MODE_FLOATING << GPIO_CRL_CNF_MODE_4_Pos;

	
    UART1->GCR = 0;
    UART1->CCR = UART_CCR_CHAR_8b;	
	UART1->GCR |= UART_GCR_TX | UART_GCR_RX | UART_GCR_UART;
	uint16_t uartdiv = SystemCoreClock / UART_BAUD;  
	UART1->BRR = uartdiv / 16;   // Mantissa
	UART1->FRA = uartdiv % 16;   // Fraction
	//UART1->IER = 	UART_IER_RX;
	
    NVIC_EnableIRQ(UART1_IRQn);  
}

// TODO: delay я звісно перероблю
static uint32_t DelayCnt;
uint32_t millis;

void delay_Init(void)
{
    millis = 0;
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

void SysTick_Handler(void)  // interruption
{
  if (DelayCnt != 0) DelayCnt--;
	millis++;
}

void delay_ms(unsigned count)
{
  DelayCnt = count;
  while(DelayCnt != 0);
}
