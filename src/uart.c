#include "uart.h"
#include "system.h"
#include "mm32_device.h"
#include "defines.h"
#include "tinyprintf.h"

extern uint32_t SystemCoreClock;

#if REMOTE_CONTROL_MODE == REMOTE_CONTROL_MODE_UART

void putc(void* p, char c);

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
	UART1->IER = 	UART_IER_RX;

    NVIC_EnableIRQ(UART1_IRQn);

    init_printf(&putc, putc);  // Initialize tinyprintf with UART putc function
}

#define UART_TX_BUFFER_SIZE 512

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

void uart_data_cb(char v);

void UART1_IRQHandler(void)
{
    // receiver
    if(UART1->ISR & UART_ISR_RX_INTF) { // rx is not empty
        static uint8_t state = 0;
        static uint8_t* buffPointer;
        UART1->ICR |= UART_ICR_RXICLR;
        uint8_t data = UART1->RDR;
        uart_data_cb(data); // Call the callback function with received data
        if(data == '1') {
            LEDY_TOGGLE(); // Toggle LEDY on each received byte
        } else if(data == '2') {
            LEDR_TOGGLE(); // Toggle LEDG on each received byte
        }
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

// // Реалізація для printf
// int __io_putchar(int ch) {
//     UART_tx_send_char((char)ch);
//     return ch;
// }

// #include <stdio.h>
// // Для сумісності з fputc (деякі бібліотеки використовують саме її)
// int fputc(int ch, FILE *f) {
//     UART_tx_send_char((char)ch);
//     return ch;
// }

// Реалізація putc для tinyprintf
void putc(void* p, char c)
{
    UART_tx_send_char(c);
}

// #include <sys/unistd.h> // Для визначення ssize_t

// // Реалізація _write для newlib (printf/fputc)
// int _write(int file, char *ptr, int len) {
//     for (int i = 0; i < len; i++) {
//         UART_tx_send_char(ptr[i]);
//     }
//     return len;
// }

// // Заглушки для інших функцій (якщо потрібно)
// int _isatty(int fd) { return 1; }
// int _close(int fd) { return -1; }
// int _lseek(int fd, int ptr, int dir) { return 0; }
// int _read(int fd, char *ptr, int len) { return 0; }
// int _fstat(int fd, void *st) { return 0; }
#endif
