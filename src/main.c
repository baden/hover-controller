#include "system.h"
#include "mm32_device.h"
#include "defines.h"
#include "uart.h"

void UART_tx(void);

int main(void)
{
    RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;   // DMA1CLK_DISABLE();	
    delay_Init();
    GPIO_Init();
    UART_Init();

    OFF_PORT->BSRR = 1<<OFF_PIN;   // Activate Latch

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
    }
}
