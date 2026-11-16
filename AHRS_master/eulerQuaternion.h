#ifndef _EULER_QUATERNION_H_
#define _EULER_QUATERNION_H_

#include <Arduino.h>
struct Quaternion {
  float w, x, y, z;
};

Quaternion quatFromDCM(const double [3][3]);

Quaternion quatConjugate(const Quaternion &);

Quaternion quatMultiply(const Quaternion &, const Quaternion &);

void normalizeQuat(Quaternion &);

void eulerFromQuat(const Quaternion &, float &, float &, float &);


#endif