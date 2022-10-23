//*****************************************************************************
//
// PID_Heli.h - Program to make a helicopter go up, down and around.
//
// Author:  Ben Empson, Sean Lalor and Sam Heustice.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#ifndef PID_HELI_H_
#define PID_HELI_H_

//*****************************************************************************
// PIDValues holds information about PID control system.
//
//*****************************************************************************
typedef struct {
    uint32_t pGain;
    uint32_t iGain;
    uint32_t dGain;
    int32_t totalError;
    int32_t prevError;
    uint32_t bias;
    int32_t dutyMax;
    int32_t dutyMin;
} PIDValues;

//*****************************************************************************
//  Creates a PID system using PID values.
//
//*****************************************************************************
PIDValues initPID(uint32_t p_gain, uint32_t i_gain, uint32_t d_gain, uint32_t bias, int32_t max, int32_t min);

//*****************************************************************************
// UpdatePID sends next duty value dictated by the control system.
//
//*****************************************************************************
int32_t UpdatePID(PIDValues* Name, uint32_t target, uint32_t current);

#endif /* PI_HELI_H_ */
