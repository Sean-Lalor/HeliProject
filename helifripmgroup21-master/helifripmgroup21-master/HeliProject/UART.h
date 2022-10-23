//*****************************************************************************
//
// UART.h - sends info about helicopter through UART
//
// Author:  Ben Empson, Sean Lalor and Sam Heustice.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#ifndef UART_H_
#define UART_H_

enum messageName{DESIRED_YAW = 0, CURRENT_YAW, DESIRED_ALT, CURRENT_ALT, MAIN_PWM, TAIL_PWM, MODE};
#define NUM_OF_STATS 7
#define MAX_STR_LEN 75
//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

char statusStr[MAX_STR_LEN + 1];



//********************************************************
// initialiseUSB_UART - 8 bits, 1 stop bit, no parity
//********************************************************
void initiUSB_UART (void);

//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void UARTSend (char *pucBuffer);

//**********************************************************************
// Transmits message about heli
//**********************************************************************
void UARTSendCheck(int32_t targetYaw, int32_t currentYaw, uint32_t targetAlt, uint32_t currentAlt, uint32_t mainRotDuty, uint32_t tailRotDuty, bool *slowTick, bool engineOff);

#endif /* UART_H_ */
