//*****************************************************************************
//
// PID_Heli.c - creates and uses a PID control system
//
// Author:  Ben Empson, Sean Lalor and Sam Heustice.
//
//*****************************************************************************

#include <PID_Heli.h>
#include <stdlib.h>
#include <stdint.h>

int32_t Limit = 20;

//*****************************************************************************
//
//  PIDValues holds information about PID control system
//
//
//
//*****************************************************************************
PIDValues
initPID(uint32_t p_gain, uint32_t i_gain, uint32_t d_gain, uint32_t bias, int32_t max, int32_t min)
{
    PIDValues pid;
    pid.pGain =  p_gain;
    pid.iGain =  i_gain;
    pid.dGain =  d_gain;
    pid.bias =  bias;
    pid.dutyMax = max;
    pid.dutyMin = min;
    pid.totalError = 0;
    pid.prevError = 0;
    return pid;
}

//*****************************************************************************
//
//  UpdatePID sends next duty value dictated by the control system
//
//
//
//*****************************************************************************
int32_t
UpdatePID(PIDValues* Name, uint32_t target, uint32_t current)
{
    int32_t error = target - current;
    //Set Limiters on totalError section of the function to prevent overshoot
    //Change Made here
    Name->totalError += error;

    //Setting a limiter on the totalError Will it prevent overshoot???
    if (Name->totalError >= Limit) {
        Name->totalError = Limit;
    } else if (Name->totalError <= -1*Limit) {
        Name->totalError = -1*Limit;
    }

    int32_t ctrlDuty = (Name->pGain * error) + (Name->iGain * Name->totalError) + (Name->dGain * (error - Name->prevError)) + Name->bias;

    if (ctrlDuty < Name->dutyMin) {
        ctrlDuty = Name->dutyMin;
    }
    if(ctrlDuty > Name->dutyMax){
        ctrlDuty = Name->dutyMax;
    }
    return ctrlDuty;
}
