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

volatile int adc_irq_counter = 0;

volatile int ur = 0;
volatile int vr = 0;
volatile int wr = 0;

// При напряжении питания 19.00V
//   0 =  9.52V
//  50 =  9.70V
// 100 =  9.99V
// 150 = 10.46V

int sinus_amplitude = 100; // Amplitude of sinusoidal signal for testing
volatile int speed = 60;   // Швидкість в обертах на хвилину (rpm) для фази
#define direction (speed >= 0) // Напрямок обертання: true - вперед, false - назад
// bool direction = true; // Напрямок обертання: true - вперед, false - назад

volatile bool enablePID = false; // Enable motor PID control

#define SINUS_TABLE_SIZE  128 // Size of sinusoidal table (4x more)

// Signed sinusoidal table values [-100, 100] representing a full cycle!
// Generated with: for (int i = 0; i < 128; ++i) printf("%d, ", (int)(100.0 * sin(2*M_PI*i/128.0)));
const int sinus_table[SINUS_TABLE_SIZE] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 49, 54, 59, 64, 69, 74,
    78, 83, 87, 91, 95, 98, 100, 103, 105, 107, 109, 110, 111, 112, 113, 113,
    113, 113, 112, 111, 110, 109, 107, 105, 103, 100, 98, 95, 91, 87, 83, 78,
    74, 69, 64, 59, 54, 49, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0,
    -5, -10, -15, -20, -25, -30, -35, -40, -45, -49, -54, -59, -64, -69, -74, -78,
    -83, -87, -91, -95, -98, -100, -103, -105, -107, -109, -110, -111, -112, -113, -113, -113,
    -113, -112, -111, -110, -109, -107, -105, -103, -100, -98, -95, -91, -87, -83, -78, -74,
    -69, -64, -59, -54, -49, -45, -40, -35, -30, -25, -20, -15, -10, -5
};

int vector_index = 0; // Index for sinusoidal vector

// TODO: Hide later
// Фаза двигуна, виміряна через датчики Холла
volatile int angle_by_hall_cur = 0; // Angle, detected by hall sensors for current position (0..359 degrees)
volatile int turns_by_hall = 0; // Number of turns detected by hall sensors (can be negative)
volatile int sync_phase = 120; // Synchronization phase for hall sensors and motor
// Абсолютна фаза
#define phase_by_hall_abs (turns_by_hall * 360 + angle_by_hall_cur - sync_phase)
// phase_by_hall_abs calculates as turns_by_hall * 360 + angle_by_hall_cur
// volatile int phase_by_hall_abs = 0; // Current phase by hall sensors in absolute value (-2^31 .. 2^31-1)

// Фаза двигуна, яка встановлена в контролері
volatile int motor_phase_abs = 0; // Current motor phase absolute in degrees (-2^31..2^31 degrees)
// volatile int motor_turns = 0; // Current motor turns (can be negative)
#define motor_turns (motor_phase_abs / 360) // Calculate current motor turns based on motor_phase_abs
#define motor_phase (motor_phase_abs % 360) // Calculate current motor phase in degrees (0..359 degrees)

// Controller must try to keep motor_phase is equal to phase_by_hall_abs
// Use PID controller to adjust motor_phase_abs based on phase_by_hall_abs

// Фаза, яку ми хочемо встановити в контролері
volatile int phase_to_set_abs = 0; // Phase to set in controller (0..359 degrees)

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

        uint8_t hall_u = !(GPIOC->IDR & (1<<15));
        uint8_t hall_v = !(GPIOC->IDR & (1<<14));
        uint8_t hall_w = !(GPIOC->IDR & (1<<13));

        tfp_printf(
            // " ADC:%d"
            // " cur_phaB:%d"
            // " cur_phaC:%d"
            // " cur_DC:%d"
            // " batt1:%d"
            // " temp:%d"
            " en:%s"
            " speed:%d"
            " dir:%d"
            " sinus_U:%d"
            " turns_by_hall:%d"
            " angle_h:%d"
            " phase_h:%d"
            " phase_m:%d"
            " phase_s:%d"
            " u:%c v:%c w:%c"
            "\r\n",
            // adc_irq_counter,
            // cur_phaB,
            // cur_phaC,
            // cur_DC,
            // adc_buffer.batt1 * 1000 / 40, // Convert to volts (assuming 40 is the divisor for milivoltage)
            // adc_buffer.temp,
            enable ? "1" : "0",
            speed,
            direction,
            sinus_amplitude,
            turns_by_hall,
            angle_by_hall_cur,
            phase_by_hall_abs,
            motor_phase_abs,
            phase_to_set_abs,
            hall_u ? '1' : '0',
            hall_v ? '1' : '0',
            hall_w ? '1' : '0'
        );
        adc_irq_counter = 0; // Reset ADC IRQ counter
    }
}

void uart_data_cb(char v)
{
    if(v == '1') {
        #if 0
        // Decrease speed by 10%
        speed = speed * 90 / 100; // Decrease speed by 10%
        if(speed > 0) {
            speed--;
        }
        #endif
        motor_phase_abs -= 10; // Decrease motor phase by 10 degrees
    } else if(v == '2') {
        #if 0
        speed = speed * 110 / 100; // Increase speed by 10%
        speed++;
        #endif
        motor_phase_abs += 10; // Increase motor phase by 10 degrees
    } else if(v == 'q') {
        phase_to_set_abs -= 10; // Decrease phase to set by 10 degrees
    } else if(v == 'w') {
        phase_to_set_abs += 10; // Increase phase to set by 10 degrees
    } else if(v == 'a') {
        speed -= 5; // Decrease speed by 5 rpm
        if (speed < 0) speed = 0;
        tfp_printf("Speed decreased to: %d\r\n", speed);
    } else if(v == 's') {
        speed += 5; // Increase speed by 5 rpm
        if (speed > 300) speed = 300; // Limit max speed to 300 rpm
        tfp_printf("Speed increased to: %d\r\n", speed);
    } else if(v == '3') {
        // direction = !direction;  // Toggle direction
        // tfp_printf("Direction changed to: %s\r\n", direction ? "forward" : "backward");
    } else if(v == '4') {
        sinus_amplitude -= 5;  // Decrease amplitude
        if (sinus_amplitude < 0) sinus_amplitude = 0;
        tfp_printf("Sinus amplitude decreased to: %d\r\n", sinus_amplitude);
    } else if(v == '5') {
        sinus_amplitude += 5;  // Increase amplitude
        if (sinus_amplitude > 500) sinus_amplitude = 500;
        tfp_printf("Sinus amplitude increased to: %d\r\n", sinus_amplitude);
    } else if(v == '6') {
        enable = 1;  // Enable motor
        TIM1->BDTR |= TIM_BDTR_MOE; // Enable main output
        tfp_printf("Motor enabled\r\n");
    } else if(v == '7') {
        enable = 0;  // Disable motor
        TIM1->BDTR &= ~TIM_BDTR_MOE; // Disable main output
        tfp_printf("Motor disabled\r\n");
    } else if(v == '9') {
        enablePID = !enablePID;  // Toggle PID control
        tfp_printf("PID control %s\r\n", enablePID ? "enabled" : "disabled");
    } else if(v == '0') {
        // Sync phases motor and hall sensors
        // phase_by_hall_abs
        // (turns_by_hall * 360 + angle_by_hall_cur - sync_phase) needs to be equal to motor_phase_abs
        sync_phase = (turns_by_hall * 360 + angle_by_hall_cur) - motor_phase_abs;
        // sync_phase = phase_by_hall_abs - motor_phase_abs;
        tfp_printf("Sync phase set to: %d\r\n", sync_phase);
    } else if(v == ' ') {
        enable = !enable;  // Toggle enable state
    }
}

// Якшо я нічого не плутаю, то значення повинно бути в діапазоні:
// (pwm_margin) ... (PWM_RES - pwm_margin)
// Середина = PWM_RES / 2

// This margin allows to have a window in the PWM signal
static int16_t pwm_margin = 0; // Для FOC контролера це 300, для інших контролерів це 0

// Declare as always inline to optimize the function call
static void __inline__ update_hall_angle(void)
{
    // ========================= MOTOR ===========================
    // Get hall sensors values
    uint8_t hall_u = !(GPIOC->IDR & (1<<15));
    uint8_t hall_v = !(GPIOC->IDR & (1<<14));
    uint8_t hall_w = !(GPIOC->IDR & (1<<13));

    static int angle_by_hall_prev = 0; // Previous angle for hall sensors

    // Calculate new phase angle based on hall sensors to angle_by_hall variable

    // prev
    // u v h  angle
    // 0 0 1 ->   0
    // 0 1 1 ->  60
    // 0 1 0 -> 120
    // 1 1 0 -> 180
    // 1 0 0 -> 240
    // 1 0 1 -> 300

    if (!hall_u && !hall_v && hall_w) {
        angle_by_hall_cur = 0;   // 0 degrees
    } else if (!hall_u && hall_v && hall_w) {
        angle_by_hall_cur = 60;  // 60 degrees
    } else if (!hall_u && hall_v && !hall_w) {
        angle_by_hall_cur = 120; // 120 degrees
    } else if (hall_u && hall_v && !hall_w) {
        angle_by_hall_cur = 180; // 180 degrees
    } else if (hall_u && !hall_v && !hall_w) {
        angle_by_hall_cur = 240; // 240 degrees
    } else if (hall_u && !hall_v && hall_w) {
        angle_by_hall_cur = 300; // 300 degrees
    } else {
        // Заборонена комбінація датчиків..
        // Не оновлюємо кут
        return;
    }

    // Calculate absolute phase by hall sensors
    // based on difference between current and previous angle
    // prev cur diff
    // 0    60  +60
    // 60   120 +60
    // 120  180 +60
    // 180  240 +60
    // 240  300 +60
    // 300  0   +60
    // 0    300 -60
    // 300  240 -60
    // 240  180 -60
    // 180  120 -60
    // 120  60  -60
    // 60   0   -60
    // This is to ensure that the phase is continuous and does not jump
    // when the motor direction changes
    if(angle_by_hall_cur == angle_by_hall_prev) return;

    int delta_angle = angle_by_hall_cur - angle_by_hall_prev;
    if (delta_angle > 180) {
        delta_angle -= 360; // Adjust for wrap-around
    } else if (delta_angle < -180) {
        delta_angle += 360; // Adjust for wrap-around
    }
    // Оновлюємо turns_by_hall тільки при переході через межу
    if (angle_by_hall_prev >= 240 && angle_by_hall_cur <= 60) {
        turns_by_hall++;
    } else if (angle_by_hall_prev <= 60 && angle_by_hall_cur >= 240) {
        turns_by_hall--;
    }

    angle_by_hall_prev = angle_by_hall_cur; // Update previous angle
}

// Викликається 12 тисяч разів на секунду
void  ADC1_COMP_IRQHandler(void)
{
    //int16_t cur_phaB, cur_phaC, cur_DC;

    ADC1->SR |= ADC_SR_ADIF;
    //GPIOB->BSRR = GPIO_BRR_BR12;  // test

    adc_irq_counter++;

    adc_buffer.rrB = (ADC1->ADDR0 - ADC1->ADDR1) & 0xFFF;       // ADC1_CH0(PA0) - ADC1_CH1(PA1) phase b
    adc_buffer.rrC = (ADC1->ADDR4 - ADC1->ADDR5) & 0xFFF;       // ADC1_CH4(PA4) - ADC1_CH5(PA5) phase c
    adc_buffer.dcr = ADC1->ADDR7 & 0xFFF;                       // ADC1_CH7(PA7) DC Link current
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

    update_hall_angle();


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

#if 0
// --- Phase calculation based on millis, with phase continuity on speed/direction change ---
#define PHASE_FRAC_BITS 16
#define PHASE_FRAC_MASK ((1UL << PHASE_FRAC_BITS) - 1)

    static uint32_t phase_accum = 0;
    static uint32_t last_millis = 0;

    // Розрахунок phase_step: фазовий приріст за 1 мс (speed — в об/хв)
    uint32_t phase_step = ((uint64_t)ABS(speed) * (1UL << PHASE_FRAC_BITS) * SINUS_TABLE_SIZE) / 60 / 1000;

    uint32_t now = millis;
    uint32_t delta_ms = now - last_millis;
    last_millis = now;

    if (direction) {
        phase_accum += phase_step * delta_ms;
    } else {
        phase_accum -= phase_step * delta_ms;
    }
    vector_index = (phase_accum >> PHASE_FRAC_BITS) % SINUS_TABLE_SIZE;

    // Фази для трифазного двигуна (A, B, C):
    ur = sinus_table[vector_index] * sinus_amplitude / 100;
    vr = sinus_table[(vector_index + SINUS_TABLE_SIZE / 3) % SINUS_TABLE_SIZE] * sinus_amplitude / 100;
    wr = sinus_table[(vector_index + 2 * SINUS_TABLE_SIZE / 3) % SINUS_TABLE_SIZE] * sinus_amplitude / 100;
#endif

    if(enablePID) {
        // Намагаємося синхронізувати фазу phase_to_set_abs з фазою, визначеною датчиками Холла (phase_by_hall_abs)
        // Не намагатись зробити точніше ніж 60 градусів
        // Значення фази абсолютне, тому не треба враховувати перехід через 0 градусів
        // TODO: Задіяти PID контролер для синхронізації фаз
        int phase_error = phase_by_hall_abs - phase_to_set_abs; // Calculate phase error
        if(ABS(phase_error) > 60) {
            // Тимчасово спрощена процедура синхронізації
            if(phase_error < 0) {
                // motor_phase_abs += 10; // Increase motor phase by 10 degrees
                speed = ABS(phase_error) / 60; //1;
                if(speed > 10) speed = 10; // Limit speed to 100 rpm
            } else {
                // motor_phase_abs -= 10; // Decrease motor phase by 10 degrees
                speed = -ABS(phase_error) / 60; //-1;
                if(speed < -10) speed = -10; // Limit speed to -100 rpm
            }
        } else {
            // Якщо фаза близька до бажаної, то зупиняємося
            speed = 0; // Stop the motor
        }

        // Розрахуємо нове значення motor_phase_abs в залежності від speed
        // TODO: Треба обмежити значення швидкості
        // TODO: Треба обмежити значення прискорення
        static int debounce = 0;
        if(debounce < 30) { // 400 ітерацій в секунду
            debounce++;
        } else {
            debounce = 0;
            motor_phase_abs += speed;
        }
    }

    // Фази для трифазного двигуна (A, B, C):
    unsigned r_index = motor_phase * SINUS_TABLE_SIZE / 360; // Convert motor phase to index in sinus_table
    // Ensure index is within bounds
    r_index = r_index % SINUS_TABLE_SIZE; // Wrap around if necessary
    // Calculate motor phase based on speed and direction
    ur = sinus_table[r_index] * sinus_amplitude / 100;
    vr = sinus_table[(r_index + SINUS_TABLE_SIZE / 3) % SINUS_TABLE_SIZE] * sinus_amplitude / 100;
    wr = sinus_table[(r_index + 2 * SINUS_TABLE_SIZE / 3) % SINUS_TABLE_SIZE] * sinus_amplitude / 100;

    // Apply commands (Ось шо ми повинні зробити в кінці кінців)
    TIM1->CCR1  = (uint16_t)CLAMP(ur + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
    TIM1->CCR2  = (uint16_t)CLAMP(vr + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
    TIM1->CCR3  = (uint16_t)CLAMP(wr + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);

    // =================================================================

    // Indicate task complete
    #if 0
    OverrunFlag = false;
    #endif
    // GPIOB->BRR=GPIO_BRR_BR12;  // test
}
