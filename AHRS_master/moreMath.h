#ifndef _MOREMATH_H_
#define _MOREMATH_H_

#include <Arduino.h>
#include "matrixMath.h"

void extractAngles(Matrix *, double [SIZE][SIZE]);
double deg2Rad(double);
double rad2Deg(double);
double norm(double[SIZE]);
void normalizeVector(double[3], double[3]);
void projectVectorOntoPlane(const double[3],const double[3], double[3]) ;
void normalizeDCM(double [3][3]);


#endif