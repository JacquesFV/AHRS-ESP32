#ifndef _SENSORFUSION_H_
#define _SENSORFUSION_H_

#include "matrixMath.h"
#include "moreMath.h"


void integrateClosedLoopAccel(Matrix *, double [SIZE][SIZE], double [SIZE], double);
void integrateClosedLoop(Matrix *, double [SIZE][SIZE], double [SIZE], double);

#endif
