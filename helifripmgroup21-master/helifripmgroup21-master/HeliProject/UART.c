//*****************************************************************************
//
// UART.c - sends heli info through UART
//
// Author:  Ben Empson, Sean Lalor and Sam Heustice.
//
//*****************************************************************************



#include "UART.h"
#include <stdlib.h>
#include <stdint.h>
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "utils/ustdlib.h"

uint8_t message = 0;

//********************************************************
// initialiseUSB_UART - 8 bits, 1 stop bit, no parity
//********************************************************
void
initiUSB_UART(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}


//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void
UARTSend(char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}

//**********************************************************************
// Transmits message about heli
//**********************************************************************

void
UARTSendCheck(int32_t targetYaw, int32_t currentYaw, uint32_t targetAlt, uint32_t currentAlt, uint32_t mainRotDuty, uint32_t tailRotDuty, bool *slowTick, bool engineOff)
{
    // Is it time to send a message?
    if(*slowTick)
    {
        *slowTick = false;

        //Invert Ready to Fly
        char mode[4];


        usnprintf(mode, sizeof(mode), (engineOff) ? "OFF" : "ON" );


        int currentYawDisplay = currentYaw%360;

        if (currentYawDisplay < 0) currentYawDisplay = 360 - abs(currentYawDisplay);

        int targetYawDisplay = targetYaw%360;

        if (targetYawDisplay < 0) targetYawDisplay = 360 - abs(targetYawDisplay);

        //Form and send a status message to the console

        switch(message)
        {
        case DESIRED_YAW:
            usprintf(statusStr, "Desired_Yaw: %3d° | ", targetYawDisplay);
            break;

        case CURRENT_YAW:
            usprintf(statusStr, "Current_Yaw: %3d° | ", currentYawDisplay);
            break;

        case DESIRED_ALT:
            usprintf(statusStr, "Desired_Alt: %3d%% | ", targetAlt);
            break;

        case CURRENT_ALT:
            usprintf(statusStr, "Current_Alt: %3d%% | ", currentAlt);
            break;

        case MAIN_PWM:
            usprintf(statusStr, "Main_PWM: %3d%% | ", mainRotDuty);
            break;

        case TAIL_PWM:
            usprintf(statusStr, "Tail_PWM: %3d%% | ", tailRotDuty);
            break;

        case MODE:
            usprintf(statusStr, "Mode: %3s\r\n", mode);
            break;
        }

        message = (message+1) % NUM_OF_STATS;

        UARTSend(statusStr);
    }
}
