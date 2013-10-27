#ifndef __ROBFUNC_H__
#define __ROBFUNC_H__

#include "RobSock.h"

const int RUN  = 1;
const int STOP = 2;
const int WAIT = 3;
const int RETURN = 4;
const int FINISHED = 5;



const int BYPASSIN_LEFT  = 1;
const int BYPASSIN_RIGTH = 2;
const int RUNNING = 3;


void DetermineAction(int beaconToFollow, float *lPow, float *rPow, int *state);

#endif
