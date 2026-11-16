#include "eulerQuaternion.h"


Quaternion quatFromDCM(const double R[3][3]) {
  Quaternion q;
  float trace = R[0][0] + R[1][1] + R[2][2];

  if (trace > 0.0f) {
    float s = 0.5f / sqrtf(trace + 1.0f);
    q.w = 0.25f / s;
    q.x = (R[2][1] - R[1][2]) * s;
    q.y = (R[0][2] - R[2][0]) * s;
    q.z = (R[1][0] - R[0][1]) * s;
  } else {
    if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
      float s = 2.0f * sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]);
      q.w = (R[2][1] - R[1][2]) / s;
      q.x = 0.25f * s;
      q.y = (R[0][1] + R[1][0]) / s;
      q.z = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
      float s = 2.0f * sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]);
      q.w = (R[0][2] - R[2][0]) / s;
      q.x = (R[0][1] + R[1][0]) / s;
      q.y = 0.25f * s;
      q.z = (R[1][2] + R[2][1]) / s;
    } else {
      float s = 2.0f * sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]);
      q.w = (R[1][0] - R[0][1]) / s;
      q.x = (R[0][2] + R[2][0]) / s;
      q.y = (R[1][2] + R[2][1]) / s;
      q.z = 0.25f * s;
    }
  }
  return q;
}

Quaternion quatConjugate(const Quaternion &q) {
  Quaternion qc;

  qc.w = q.w;
  qc.x = -q.x;
  qc.y = -q.y;
  qc.z = -q.z;

  return qc;
}


Quaternion quatMultiply(const Quaternion &a, const Quaternion &b) {
  Quaternion r;

  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;

  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;

  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;

  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

  return r;
}

void normalizeQuat(Quaternion &q) {
  double n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

  if (n < 1e-12) {
    return;
  }

  q.w /= n;
  q.x /= n;
  q.y /= n;
  q.z /= n;
}


void eulerFromQuat(const Quaternion &q, float &roll, float &pitch, float &yaw) {
  // roll (X axis)
  float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  roll = atan2f(sinr_cosp, cosr_cosp);

  // pitch (Y axis)
  float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  if (fabsf(sinp) >= 1.0f)
    pitch = copysignf(M_PI / 2.0f, sinp);  // clamp
  else
    pitch = asinf(sinp);

  // yaw (Z axis)
  float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  yaw = atan2f(siny_cosp, cosy_cosp);
}
