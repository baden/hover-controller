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


// TODO: delay я звісно перероблю
static uint32_t DelayCnt;
volatile uint32_t millis;

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


void TIM1_Init(void)
{
    RCC->APB2ENR |=  RCC_APB2ENR_TIM1EN;
 	RCC->AHBENR |= RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB;

    // Керування мосфетами: PA8,9,10 & PC13,14,15
    // alternative AF2 PWM pullpush output ports
    GPIOA->AFRH	&= ~GPIO_AFRH_AFR8;
    GPIOA->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR8_Pos;
    GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_8_Pos);
    GPIOA->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_8_Pos;

    GPIOA->AFRH	&= ~GPIO_AFRH_AFR9;
    GPIOA->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR9_Pos;
    GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_9_Pos);
    GPIOA->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_9_Pos;

    GPIOA->AFRH	&= ~GPIO_AFRH_AFR10;
    GPIOA->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR10_Pos;
    GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_10_Pos);
    GPIOA->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_10_Pos;

    GPIOB->AFRH	&= ~GPIO_AFRH_AFR13;
    GPIOB->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR13_Pos;
    GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_13_Pos);
    GPIOB->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_13_Pos;

    GPIOB->AFRH	&= ~GPIO_AFRH_AFR14;
    GPIOB->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR14_Pos;
    GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_14_Pos);
    GPIOB->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_14_Pos;

    GPIOB->AFRH	&= ~GPIO_AFRH_AFR15;
    GPIOB->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR15_Pos;
    GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_15_Pos);
    GPIOB->CRH |= GPIO_CNF_MODE_AF_PP <<GPIO_CRH_CNF_MODE_15_Pos;

	// -------------
	TIM1->SMCR = 0;
    TIM1->CR1 = TIM_CR1_CMS_CENTERALIGNED1;  // Center-aligned mode 1
    TIM1->PSC = 0;
    TIM1->ARR = PWM_RES;  // 2250
    TIM1->RCR = 1;
    // master/slave
    TIM1->CR2 = TIM_CR2_MMS_UPDATE;   // TRGO

    // ConfigChannel
    TIM1->CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_PWM1;

    // break
    TIM1->BDTR = TIM_BDTR_OSSR | TIM_BDTR_OSSI | DEAD_TIME;
    // enables
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE
	            | TIM_CCER_CC2E | TIM_CCER_CC2NE
				| TIM_CCER_CC3E | TIM_CCER_CC3NE;
    TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CR2 |= TIM_CR2_OIS1N | TIM_CR2_OIS2N | TIM_CR2_OIS3N;

    // start
    TIM1->CR1 |= TIM_CR1_CEN;
}


uint16_t adc_buffer_raw[32];

void ADC1_Init(void)
{
    RCC->APB2ENR |=  RCC_APB2ENR_ADC1EN;

 	RCC->AHBENR |= RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB;
    GPIOA->CRL  = 0;
    GPIOB->CRL &= 0xFFFFFF00;

    ADC1->CFGR = ADC_CFGR_ADEN |  ADC_CFGR_TEN | ADC_CFGR_VEN |  ADC_CFGR_SAMCTL_7_5;   //14.4MHz, temper & ref enable
    ADC1->CR = ADC_CR_SCAN | ADC_CR_T1_TRIG  | ADC_CR_ADIE;	  	// interruption

    ADC1->CHSR = ADC_CHSR_CH0 | ADC_CHSR_CH1   // phase b
                | ADC_CHSR_CH4 | ADC_CHSR_CH5   //phase c
                | ADC_CHSR_CH7                  // I common
	            | ADC_CHSR_CH9                  // voltage (PB1/ADC1_CH9)
                | ADC_CHSR_CHT;                 // temperature


    NVIC_SetPriority(ADC1_IRQn, 0);
    NVIC_EnableIRQ(ADC1_IRQn);
}
