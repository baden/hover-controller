#include "system.h"
#include "mm32_device.h"
#include "defines.h"

void UART_tx(void);

int main(void)
{
    RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;   // DMA1CLK_DISABLE();	
    delay_Init();
    GPIO_Init();
    UART_Init();

    OFF_PORT->BSRR = 1<<OFF_PIN;   // Activate Latch
    for(;;) {
        LEDR_ON();
        delay_ms(1000);
        LEDR_OFF();
        delay_ms(1000);
        UART_tx();
    }
}

void UART_tx(void)
{
//   TXcouter = sizeof(Feedback);
//   TXponter = (uint8_t *)&Feedback; 
    UART1->ISR &= ~UART_ISR_TX_INTF;
    while (!(UART1->CSR & UART_CSR_TXEPT)) {};
    UART1->TDR = '+'; //*TXponter;
    // TXcouter--; 
    // TXponter++;
    UART1->IER |= UART_IER_TXIEN;
}

void UART1_IRQHandler(void)
{
    // receiver
    if(UART1->ISR & UART_ISR_RX_INTF) { // rx is not empty
        static uint8_t state = 0;
        static uint8_t* buffPointer;
        UART1->ICR |= UART_ICR_RXICLR;
        uint8_t data = UART1->RDR;
        // if (state==0) {
        //     if (data == (SERIAL_START_FRAME & 0xFF)) state++;
        // } else if (state==1) {
        //     if (data == (SERIAL_START_FRAME >> 8)) {
        //         state++;
        //         buffPointer = (uint8_t *)&command_raw + 2;
        //         command_raw.start = SERIAL_START_FRAME;
        //     } else state=0;
        // } else if(state<8) {
        //     *buffPointer = data;
        //     buffPointer++;
        //     if (state==7) {
        //         state = 0;  
        //         uint16_t checksum = (uint16_t)(command_raw.start ^ command_raw.steer ^ command_raw.speed);
        //         if (command_raw.checksum == checksum) {
        //             commandR = command_raw;
        //             timeoutFlgSerial_R = 0;         // Clear timeout flag
        //             timeoutCntSerial_R = 0;         // Reset timeout counter           
        //         }
        //     }
        //     else state++;
        // }
    }

    // transmitter
    if (UART1->ISR & UART_ISR_TX_INTF) { // tx buff null
        UART1->ICR |= UART_ICR_TXICLR;
        //TXcouter--;
        /*if (!(TXcouter>0))*/ UART1->IER &= ~UART_IER_TXIEN;    // disable interrupt
        UART1->TDR = '+'; //*TXponter;
        //TXponter++;
    }
}
