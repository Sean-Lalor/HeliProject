//*****************************************************************************
//
// PWMconfig.h - Configures PWM for helicopter
//
// Author:  Ben Empson, Sean Lalor and Sam Heustice.
//
//*****************************************************************************


#ifndef PWM_H_
#define PWM_H_
#endif /* PWM_H_ */


// PWM configuration
#define PWM_START_RATE_HZ  200
#define PWM_START_DUTY     10
#define PWM_STEP_DUTY      5
#define PWM_MIN_DUTY       5
#define PWM_MAX_DUTY       95
#define PWM_RATE_STEP_HZ   50
#define PWM_RATE_MIN_HZ    150
#define PWM_RATE_MAX_HZ    300
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        4

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  PWM Hardware Details M1PWM5 (gen 2)
//  ---Tail Rotor PWM: PF1, J3-10

#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

#define MAIN_ROT_DUTY_DEFAULT 0
#define TAIL_ROT_DUTY_DEFAULT 0

//setPWMMAIN: Sets the Pulse width modulation for the main motor that controls the altitude.
void
setPWMMain (uint32_t ui32Freq, uint32_t ui32Duty);

//setPWMTail: Sets the Pulse width modulation for the tail motor that controls the rotation (yaw).
void
setPWMTail (uint32_t ui32Freq, uint32_t ui32Duty);

//initPWM: inits the pulse width modulation for both tail and main motors
void
initPWM (void);



