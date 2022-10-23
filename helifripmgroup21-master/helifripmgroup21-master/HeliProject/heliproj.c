//*****************************************************************************
//
// heliproj.c - Program to make a helicopter go up, down and around.
//
// Author:  Ben Empson, Sean Lalor and Sam Heustice.
//
//*****************************************************************************

//*****************************************************************************
// Display definitions.
//
//*****************************************************************************
#define TITLE "HeliProj"
#define VERSION "0.9.9"
#define TITLE_BLOCK (TITLE " " VERSION)
#define MEAN_HEAD "Mean ADC = "
#define BLANK_LINE "                "

//*****************************************************************************
// Debug mode 0 = CH9 for monitoring the helicopter.
// Debug mode 1 = CH0 for monitoring the Orbit dial.
//*****************************************************************************
#define DEBUG_MODE 0

//*****************************************************************************
// Includes.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "utils/ustdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "buttons4.h"
#include "driverlib/pin_map.h"

#include "UART.h"
#include "PWMconfig.h"
#include <PID_Heli.h>

//*****************************************************************************
// Constants.
//
//*****************************************************************************
#define BUF_SIZE 50
#define SAMPLE_RATE_HZ 5000
#define INITIAL_DELAY_HZ 3
#define REFRESH_RATE_HZ 300
#define ALTITUDE_RANGE 1221

#define PHASE_A_PIN GPIO_PIN_0
#define PHASE_B_PIN GPIO_PIN_1
#define PHASE_AB_PORT GPIO_PORTB_BASE
#define PHASE_AB_PERIPH  SYSCTL_PERIPH_GPIOB

#define ZERO_YAW_PERIPH SYSCTL_PERIPH_GPIOC
#define ZERO_YAW_PORT GPIO_PORTC_BASE
#define ZERO_YAW_PIN GPIO_PIN_4

// PID GAINS.
// Alt gains.
#define P_GAIN_ALT 10
#define I_GAIN_ALT 1
#define D_GAIN_ALT 1
#define ZERO_VELO_ALT 50

// Yaw gains.
#define P_GAIN_YAW 10
#define I_GAIN_YAW 1
#define D_GAIN_YAW 1
#define ZERO_VELO_YAW 40

// Duty cycle for yaw calibration.
#define YAW_CALI_DUTY (ZERO_VELO_YAW + 20)

// Converts Yaw from notch count to degrees (Angle).
#define YAW_DEG (currentYawNotchCount*DEGRESS_IN_CIRCLE)/NOTCHES_ON_ENCODER

#define DUTY_MAX 98
#define DUTY_MAIN_MIN 40
#define DUTY_TAIL_MAX 80
#define DUTY_TAIL_MIN 0

#define SWITCH_ON (GPIOPinRead(SW1_PORT_BASE, SW1_PIN) == SW1_PIN)
#define SWITCH_OFF (GPIOPinRead(SW1_PORT_BASE, SW1_PIN) != SW1_PIN)

#define SLOWTICK_RATE_HZ 2

#define STARTING_HEIGHT_PER 5

// Step size for button pushes.
#define YAW_STEP 15
#define ALT_STEP 10

#define DEGRESS_IN_CIRCLE 360
#define NOTCHES_ON_ENCODER 448
#define MAX_ALTITUDE 100

//*****************************************************************************
// Global variables.
//
//*****************************************************************************
static circBuf_t g_inBuffer;
static uint32_t g_ulSampCnt;
static int32_t currentYawNotchCount = 0;
static int32_t targetYaw = 0;
static uint32_t mainRotDuty = 0;
static uint32_t tailRotDuty = 0;
static uint32_t currentAlt = 0;
static uint32_t targetAlt = 0;
static uint8_t B_prev = 0;
static uint8_t A_current = 0;
static uint8_t B_current = 0;

//FLAGS
static bool landing = false;
static bool readyToFly = false;
static bool engineOff = true;
static bool slowTick = false;
static bool calibrated = false;
static bool landed = true;

//*****************************************************************************
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    // Initiate a conversion.
    ADCProcessorTrigger(ADC0_BASE, 3); 
    g_ulSampCnt++;

    static uint8_t tickCount = 0;
    const uint16_t ticksPerSlow = REFRESH_RATE_HZ / SLOWTICK_RATE_HZ;
    if (++tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        tickCount = 0;
        slowTick = true;
    }
}

//*****************************************************************************
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    // Get the single sample from ADC0.  ADC_BASE is defined in inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);

    // Place it in the circular buffer (advancing write index).
    writeCircBuf (&g_inBuffer, ulValue);

    // Clean up, clearing the interrupt.
    ADCIntClear(ADC0_BASE, 3);

}

//*****************************************************************************
// Initializes ADC interrupts.
//
//*****************************************************************************
void
initADC(void)
{
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //Chooses a channel depending on debug mode.
    if (DEBUG_MODE)
    {
        //CH0 for monitoring the Orbit dial.
        ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    } else {
        //CH9 for monitoring the helicopter.
        ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    }

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    // Register the interrupt handler.
    ADCIntRegister(ADC0_BASE, 3, ADCIntHandler);


    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts).
    ADCIntEnable(ADC0_BASE, 3);
}

//*****************************************************************************
// The handler for pin change of the Phase A and B pin.
// Increases/decreases yaw notch count based on direction encoder is turning.
//*****************************************************************************

void
yawIntHandler(void)
{

    A_current = (GPIOPinRead(PHASE_AB_PORT, PHASE_A_PIN) == PHASE_A_PIN);
    B_current = (GPIOPinRead(PHASE_AB_PORT, PHASE_B_PIN) == PHASE_B_PIN);


    GPIOIntClear(PHASE_AB_PORT, PHASE_A_PIN|PHASE_B_PIN);

    (A_current ^ B_prev) ? currentYawNotchCount-- : currentYawNotchCount++;

    B_prev = B_current;
}

//*****************************************************************************
// Initializes yaw counting interrupts.
//
//*****************************************************************************
void
initYaw(void)
{
    SysCtlPeripheralEnable (PHASE_AB_PERIPH);
    GPIOPinTypeGPIOInput (PHASE_AB_PORT, PHASE_A_PIN|PHASE_B_PIN);
    GPIOIntTypeSet(PHASE_AB_PORT, PHASE_A_PIN|PHASE_B_PIN, GPIO_BOTH_EDGES);
    GPIOIntRegister(PHASE_AB_PORT, yawIntHandler);
    GPIOIntEnable(PHASE_AB_PORT, GPIO_INT_PIN_0|GPIO_INT_PIN_1);
}

//*****************************************************************************
// Initialization functions for the clock (incl. SysTick), ADC, display.
//
//*****************************************************************************
void
initClock(void)
{
    // Set the clock rate to 20 MHz.
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);

    // Register the interrupt handler.
    SysTickIntRegister(SysTickIntHandler);

    // Enable interrupt and device.
    SysTickIntEnable();
    SysTickEnable();
}

//*****************************************************************************
// The handler for the Switch 1 pin change interrupt.
// Turns on/off motor and tail rotor and turns on flags.
//*****************************************************************************
void
SW1IntHandler(void)
{
    // Initialization is complete, so turn on the output.
    if (SWITCH_ON && readyToFly) {
        targetAlt = STARTING_HEIGHT_PER;

        PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
        PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);

        readyToFly = false;
        landing = false;
        engineOff = false;

    } else if (SWITCH_OFF) {

        currentYawNotchCount = YAW_DEG%DEGRESS_IN_CIRCLE;
        landing = true;

        targetYaw = 0;
        targetAlt = 0;
    }

    GPIOIntClear(SW1_PORT_BASE, SW1_PIN);
}

//*****************************************************************************
// Initializes Switch 1 interrupts.
//
//*****************************************************************************
void
initSwitch(void)
{
    // Initialise SW1 to turn on/ off Helicopter
    // GPIO PA7
    // Switch 1 button (active HIGH)
    SysCtlPeripheralEnable (SW1_PERIPH);
    GPIOPinTypeGPIOInput (SW1_PORT_BASE, SW1_PIN);
    GPIOIntTypeSet(SW1_PORT_BASE, SW1_PIN, GPIO_BOTH_EDGES);


    GPIOIntRegister(SW1_PORT_BASE, SW1IntHandler);
    GPIOIntEnable(SW1_PORT_BASE, GPIO_INT_PIN_7);
}

//*****************************************************************************
// Yaw Calibration interrupt.
//
//*****************************************************************************
void yawCaliHandler(void)
{
    currentYawNotchCount = 0;
    calibrated = true;
    GPIOIntClear(ZERO_YAW_PORT, ZERO_YAW_PIN);
}

//*****************************************************************************
// Initializes Yaw Calibration interrupts.
//
//*****************************************************************************
void
initYawCali(void)
{
    // Initialise Yaw Calibration Switch.
    // GPIO PC4
    // YAW Cali button (active HIGH)
    SysCtlPeripheralEnable (ZERO_YAW_PERIPH);
    GPIOPinTypeGPIOInput (ZERO_YAW_PORT, ZERO_YAW_PIN);
    GPIOIntTypeSet(ZERO_YAW_PORT, ZERO_YAW_PIN, GPIO_BOTH_EDGES);
    GPIOIntRegister(ZERO_YAW_PORT, yawCaliHandler);
    GPIOIntEnable(ZERO_YAW_PORT, GPIO_INT_PIN_4);
}

//*****************************************************************************
// Function to convert a given altitude to its percentage of total height.
//
//*****************************************************************************
void
toPercentage(uint32_t input, uint32_t base)
{
    // Converts mean ADC value to percentage.
    double percentValue;
    percentValue = (base - (double)input) / (ALTITUDE_RANGE) * MAX_ALTITUDE;
    currentAlt = (int32_t)percentValue;

}

//*****************************************************************************
// Function to display the mean ADC value (10-bit value) and sample count.
//
//*****************************************************************************
void
display(void)
{
    char string[17];  // 16 characters across the display.

    // Displays relevant information according it's setting: percent, raw, off.
    int yawDisplay = YAW_DEG%DEGRESS_IN_CIRCLE;

    usnprintf(string, sizeof(string), "Height    = %3d%%", currentAlt);
    OLEDStringDraw (string, 0, 0);

    if (yawDisplay < 0) yawDisplay = DEGRESS_IN_CIRCLE - abs(yawDisplay);


    usnprintf(string, sizeof(string), "Yaw(deg)  =  %3d", yawDisplay%DEGRESS_IN_CIRCLE);
    OLEDStringDraw (string, 0, 1);

    usnprintf(string, sizeof(string), "Main Duty = %3d%%", mainRotDuty);
    OLEDStringDraw (string, 0, 2);

    usnprintf(string, sizeof(string), "Tail Duty = %3d%%", tailRotDuty);
    OLEDStringDraw (string, 0, 3);
}



//*****************************************************************************
// Checks the buttons pushes and adjusts Target Alt and Target Yaw
// depending on the button pushed.
//*****************************************************************************
void
checkButtons(void)
{
    uint8_t butState = 0;

    butState = checkButton(LEFT);
    if(butState == PUSHED) targetYaw -= YAW_STEP;

    butState = checkButton(RIGHT);
    if(butState == PUSHED) targetYaw += YAW_STEP;

    // if UP button is pressed, add 10 to targetAlt to a maximum of 100
    butState = checkButton(UP);
    if(butState == PUSHED) targetAlt = (targetAlt > MAX_ALTITUDE - ALT_STEP) ? MAX_ALTITUDE : targetAlt + ALT_STEP;

    // if DOWN button is pressed, subtract 10 to targetAlt to a minimum of 0
    butState = checkButton(DOWN);
    if(butState == PUSHED) targetAlt = (targetAlt < ALT_STEP) ? 0 : targetAlt - ALT_STEP;


}

//*****************************************************************************
// Spins helicopter until the Yaw zero pin is triggered then
// resets the Yaw counter
//*****************************************************************************
void
yawCalibrate(void)
{
    //Make the helicopter hover and slowly turn
    tailRotDuty = YAW_CALI_DUTY;
    setPWMTail(PWM_START_RATE_HZ, tailRotDuty);
}


//*****************************************************************************
// Main function for doing helicopter stuff.
//
//*****************************************************************************
int
main(void)
{
    // Initialize local variables.
    int32_t ADCSum = 0;
    uint32_t i = 0;
    uint32_t landedValue = 0;
    uint32_t meanADC = 0;


    // Initialize peripherals.
    initClock();
    initADC();
    OLEDInitialise();
    initButtons();
    initYaw();
    initCircBuf(&g_inBuffer, BUF_SIZE);
    initPWM();
    initSwitch();
    initYawCali();
    initiUSB_UART();


    // creates the PID control systems for the Yaw and Alt.
    PIDValues AltPID = initPID(P_GAIN_ALT, I_GAIN_ALT, D_GAIN_ALT, ZERO_VELO_ALT, DUTY_MAX, DUTY_MAIN_MIN);
    PIDValues YawPID = initPID(P_GAIN_YAW, I_GAIN_YAW, D_GAIN_YAW, ZERO_VELO_YAW, DUTY_TAIL_MAX, DUTY_TAIL_MIN);

    // Enable interrupts to the processor.
    IntMasterEnable();

    // Display a loading screen.
    OLEDStringDraw (TITLE_BLOCK, 0, 0);
    OLEDStringDraw ("Loading...", 0, 2);
    // Waits to initialize buffer.
    SysCtlDelay (SysCtlClockGet() / INITIAL_DELAY_HZ);

    while (1)
    {
        // Calculate the (approximate) mean of the values in the circular buffer.
        ADCSum = 0;
        for (i = 0; i < BUF_SIZE; i++)
            ADCSum = ADCSum + readCircBuf (&g_inBuffer);
        meanADC = (2 * ADCSum + BUF_SIZE) / 2 / BUF_SIZE;

        // Sets the landed value if this is the first time running the program
        // or the left button is pushed.
        if (landed)
        {
            landedValue = meanADC;
            landed = 0;
        }

        toPercentage(meanADC, landedValue);
        display();

        if (!engineOff)
        {
            IntMasterDisable();
            mainRotDuty = UpdatePID(&AltPID, targetAlt, currentAlt);
            tailRotDuty = UpdatePID(&YawPID, targetYaw, YAW_DEG);
            IntMasterEnable();

            if (!landing && calibrated)
            {
                updateButtons();
                checkButtons();
            }

        } else {
            mainRotDuty = 0;
            tailRotDuty = 0;
        }

        if (currentAlt == 0)
        {
            if (YAW_DEG == 0 && landing)
            {
                //Switch off engine
                engineOff = true;
                PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
                PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
            }
            if (SWITCH_OFF && engineOff) readyToFly = true;
        }

        setPWMMain(PWM_START_RATE_HZ, mainRotDuty);

        if (calibrated) {
            setPWMTail(PWM_START_RATE_HZ, tailRotDuty);
        }

        if (!calibrated && !engineOff) {
            yawCalibrate(); // If not calibrated then calibrate
        }

        UARTSendCheck(targetYaw, YAW_DEG, targetAlt, currentAlt, mainRotDuty, tailRotDuty, &slowTick, engineOff);

        IntMasterDisable();
        SysCtlDelay (SysCtlClockGet() / REFRESH_RATE_HZ);  // Update display at ~ 300 Hz.
        IntMasterEnable();
    }
}

