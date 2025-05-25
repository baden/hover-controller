#include "system.h"
#include "mm32_device.h"
#include "defines.h"
#include "uart.h"
#include "tinyprintf.h"
// #include <stdio.h>

void UART_tx(void);

uint8_t enable = 0;        // initially motor is disabled for SAFETY

volatile adc_buf_t adc_buffer;
volatile int16_t cur_phaB, cur_phaC, cur_DC;

volatile int ur = 0;
volatile int vr = 0;
volatile int wr = 0;

// При напряжении питания 19.00V
//   0 =  9.52V
//  50 =  9.70V
// 100 =  9.99V
// 150 = 10.46V

int sinus_amplitude = 20; // Amplitude of sinusoidal signal for testing

#define SINUS_FREQ  1000 // Frequency of sinusoidal signal in Hz

#define SINUS_TABLE_SIZE  32 // Size of sinusoidal table

// Signed sinusoidal table values [-100, 100] representing a full cycle!
const int sinus_table[SINUS_TABLE_SIZE] = {
    0,   19,   38,   56,   71,   83,   92,   98,
  100,   98,   92,   83,   71,   56,   38,   19,
    0,  -19,  -38,  -56,  -71,  -83,  -92,  -98,
 -100,  -98,  -92,  -83,  -71,  -56,  -38,  -19
};

int vector_index = 0; // Index for sinusoidal vector

int main(void)
{
    RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;   // DMA1CLK_DISABLE();
    delay_Init();
    GPIO_Init();
    UART_Init();

    TIM1_Init();
    ADC1_Init();

    OFF_PORT->BSRR = 1<<OFF_PIN;   // Activate Latch

    // Start ADC conversion
    ADC1->CR |= ADC_CR_TRGEN;

    UART_tx_send("\r\nINFO: Hover board stepper motor test\r\n");

    for(;;) {
        // LEDR_ON();
        // delay_ms(100);
        LEDG_ON();
        // delay_ms(100);
        // LEDY_ON();
        delay_ms(500);
        // LEDR_OFF();
        // delay_ms(100);
        LEDG_OFF();
        // delay_ms(100);
        // LEDY_OFF();
        delay_ms(500);
        tfp_printf(
            "millis:%d"
            "ADC"
            " rrB:%d"
            " rrC:%d"
            " dcr:%d"
            " batt1:%d"
            " temp:%d"
            "  enable:%s"
            "  ur:%d"
            "  vr:%d"
            "  wr:%d"
            "\r\n",
            millis,
            adc_buffer.rrB,
            adc_buffer.rrC,
            adc_buffer.dcr,
            adc_buffer.batt1,
            adc_buffer.temp,
            enable ? "+" : "-",
            ur, vr, wr
        );
    }
}

void uart_data_cb(char v)
{
    if(v == '1') {
        ur++;
    } else if(v == '2') {
        vr++;
    } else if(v == '3') {
        wr++;
    } else if(v == 'q') {
        ur--;
    } else if(v == 'w') {
        vr--;
    } else if(v == 'e') {
        wr--;
    } else if(v == ' ') {
        enable = !enable;  // Toggle enable state
    }
}


// Якшо я нічого не плутаю, то значення повинно бути в діапазоні:
// (pwm_margin) ... (PWM_RES - pwm_margin)
// Середина = PWM_RES / 2

// This margin allows to have a window in the PWM signal
static int16_t pwm_margin = 0; // Для FOC контролера це 300, для інших контролерів це 0


void  ADC1_COMP_IRQHandler(void)
{
    //int16_t cur_phaB, cur_phaC, cur_DC;

    ADC1->SR |= ADC_SR_ADIF;
    //GPIOB->BSRR = GPIO_BRR_BR12;  // test

    adc_buffer.rrB = (ADC1->ADDR0 - ADC1->ADDR1) & 0xFFF;
    adc_buffer.rrC = (ADC1->ADDR4 - ADC1->ADDR5) & 0xFFF;
    adc_buffer.dcr = ADC1->ADDR7 & 0xFFF;
    adc_buffer.batt1 = ADC1->ADDR9 & 0xFFF;
    adc_buffer.temp =  ADC1->ADDR14 & 0xFFF;

    // offset calibrate
    static uint8_t DummyCnt = 100;
    if (DummyCnt > 0) { DummyCnt--; return; }

    static int32_t phaseBV = 0;
    static int32_t phaseCV = 0;
    static int32_t commonIV = 0;
    static int16_t offsetrrB = 0;
    static int16_t offsetrrC = 0;
    static int16_t offsetdcr = 0;

    static int16_t AvrCnt = 1024;
    if(AvrCnt > 0) {
        phaseBV  +=  adc_buffer.rrB;
        phaseCV  +=  adc_buffer.rrC;
        commonIV += adc_buffer.dcr;
        AvrCnt--;
        if(AvrCnt != 0) return;
        else {
            offsetrrB = (int16_t)(phaseBV/1024);
            offsetrrC = (int16_t)(phaseCV/1024);
            offsetdcr = (int16_t)(commonIV/1024);
        }
    }
    // offset calibrate end


    // Filter battery voltage at a slower sampling rate
    // 680 = 17.0V
    // 720 = 18.0V
    // 768 = 19.0V
    // Я так бачу шо якшо поділити на 40, то буде значення в вольтах
    #if 0
    if (buzzerTimer % 1000 == 0) {
        filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
        batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
    }
    #endif

    // Get motor currents
    cur_phaB = (int16_t)(adc_buffer.rrB - offsetrrB);
    cur_phaC = (int16_t)(adc_buffer.rrC - offsetrrC);
    cur_DC   = (int16_t)(adc_buffer.dcr - offsetdcr);
    //if (cur_phaB>max_cur_phaB) max_cur_phaB = cur_phaB;
    //if (cur_phaC>max_cur_phaC) max_cur_phaC = cur_phaC;
    //if (cur_DC>max_cur_DC) max_cur_DC = cur_DC;


    // Disable PWM when current limit is reached (current chopping)
    // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
    if((ABS(cur_DC)  > CURR_DC_MAX) || (enable == 0)) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    } else {
        TIM1->BDTR |= TIM_BDTR_MOE;
    }


    // Create square wave for buzzer
    #if 0
    buzzerTimer++;
    if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
        if (buzzerPrev == 0)
        {
        buzzerPrev = 1;  // pause 2 periods
        if (++buzzerIdx > (buzzerCount + 2)) buzzerIdx = 1;
        }
        if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0))
        {
        (BUZZER_PORT->ODR & (1<<BUZZER_PIN)) ? (BUZZER_PORT->BRR = 1<<BUZZER_PIN) : (BUZZER_PORT->BSRR = 1<<BUZZER_PIN);
        }
    } else {
        if (buzzerPrev) {
            BUZZER_PORT->BRR = 1<<BUZZER_PIN;
            buzzerPrev = 0;
        }
    }
    #endif

    // GPIOB->BSRR = GPIO_BRR_BR12;  // test

    // Adjust pwm_margin depending on the selected Control Type
    //if (rtP.z_ctrlTypSel == FOC_CTRL) pwm_margin = 110;
    #if 0
    if (rtP.z_ctrlTypSel == FOC_CTRL) pwm_margin = 300;
    else                              pwm_margin = 0;
    #endif

    // ############################### MOTOR CONTROL ###############################
    // int ur, vr, wr;
    #if 0
    static boolean_T OverrunFlag = false;

    // Check for overrun
    if (OverrunFlag) return;
    OverrunFlag = true;
    // Make sure to stop motor in case of an error
    enableFin = enable && !rtY.z_errCode;
    #endif

    // ========================= MOTOR ===========================
    // Get hall sensors values
    uint8_t hall_u = !(GPIOC->IDR & (1<<15));
    uint8_t hall_v = !(GPIOC->IDR & (1<<14));
    uint8_t hall_w = !(GPIOC->IDR & (1<<13));

    #if 0
    // Set motor inputs here
    rtU.b_motEna      = enableFin;
    rtU.z_ctrlModReq  = ctrlModReq;
    rtU.r_inpTgt      = pwmr;
    rtU.b_hallA       = hall_u;
    rtU.b_hallB       = hall_v;
    rtU.b_hallC       = hall_w;
    rtU.i_phaAB       = cur_phaB;
    rtU.i_phaBC       = cur_phaC;
    rtU.i_DCLink      = cur_DC;

    // Step the controller
    BLDC_controller_step(rtM);

    // Get motor outputs here
    ur  = rtY.DC_phaA;
    vr  = rtY.DC_phaB;
    wr  = rtY.DC_phaC;
    #endif

    // TODO: Use millis for choosing the sinusoidal vector_index
    ur = sinus_table[vector_index] * sinus_amplitude / 100; // Scale to amplitude
    vr = sinus_table[(vector_index + SINUS_TABLE_SIZE / 3) % SINUS_TABLE_SIZE] * sinus_amplitude / 100; // Phase shift by 120 degrees
    wr = sinus_table[(vector_index + 2 * SINUS_TABLE_SIZE / 3) % SINUS_TABLE_SIZE] * sinus_amplitude / 100; // Phase shift by 240 degrees
    vector_index = (vector_index + 1) % SINUS_TABLE_SIZE; // Increment index for next sinusoidal value

    // Apply commands (Ось шо ми повинні зробити в кінці кінців)
    #if 1
    TIM1->CCR1  = (uint16_t)CLAMP(ur + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
    TIM1->CCR2  = (uint16_t)CLAMP(vr + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
    TIM1->CCR3  = (uint16_t)CLAMP(wr + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
    #endif
    // =================================================================

    // Indicate task complete
    #if 0
    OverrunFlag = false;
    #endif
    // GPIOB->BRR=GPIO_BRR_BR12;  // test
}
