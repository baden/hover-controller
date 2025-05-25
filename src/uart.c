#include "uart.h"
#include "system.h"
#include "mm32_device.h"
#include "defines.h"


#define UART_TX_BUFFER_SIZE 128

char uart_tx_buffer[UART_TX_BUFFER_SIZE];
unsigned uart_tx_buffer_index = 0;  // index for the next byte to send
unsigned uart_tx_buffer_length = 0; // length of the data to send

// void UART_tx(void)
// {
// //   TXcouter = sizeof(Feedback);
// //   TXponter = (uint8_t *)&Feedback; 
//     UART1->ISR &= ~UART_ISR_TX_INTF;
//     while (!(UART1->CSR & UART_CSR_TXEPT)) {};
//     UART1->TDR = '+'; //*TXponter;
//     // TXcouter--; 
//     // TXponter++;
//     UART1->IER |= UART_IER_TXIEN;
// }

void UART1_IRQHandler(void)
{
    // receiver
    if(UART1->ISR & UART_ISR_RX_INTF) { // rx is not empty
        static uint8_t state = 0;
        static uint8_t* buffPointer;
        UART1->ICR |= UART_ICR_RXICLR;
        uint8_t data = UART1->RDR;
        LEDY_TOGGLE(); // Toggle LEDY on each received byte
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

        if (uart_tx_buffer_length > 0) {
            UART1->TDR = uart_tx_buffer[uart_tx_buffer_index++];
            if(uart_tx_buffer_index >= UART_TX_BUFFER_SIZE) {
                uart_tx_buffer_index = 0;
            }
            uart_tx_buffer_length--;
            if(uart_tx_buffer_length == 0) UART1->IER &= ~UART_IER_TXIEN;    // disable interrupt
        } else {
            UART1->IER &= ~UART_IER_TXIEN; // disable transmit interrupt if buffer is empty
        }
    }
}

void UART_tx_send_char(char c)
{
    // Disable transmit interrupt to avoid conflicts
    UART1->IER &= ~UART_IER_TXIEN;
    if(uart_tx_buffer_length < UART_TX_BUFFER_SIZE) {
        unsigned write_index = (uart_tx_buffer_index + uart_tx_buffer_length) % UART_TX_BUFFER_SIZE;
        uart_tx_buffer[write_index] = c; // add data to circular buffer
        uart_tx_buffer_length++;
    }
    UART1->IER |= UART_IER_TXIEN; // re-enable transmit interrupt
}

void UART_tx_send(const char* str)
{
    while (*str) {
        UART_tx_send_char(*str++);
    }
}