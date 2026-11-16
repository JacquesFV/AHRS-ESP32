
#include "moreMath.h"
//euler angle extraction from dcm
void extractAngles(Matrix *m, double R[SIZE][SIZE])
{
    double roll =
        atan2(R[1][2],
              R[2][2]);

    double pitch =
        -atan2(-R[0][2],
              sqrt(R[1][2]*R[1][2] +
                   R[2][2]*R[2][2]));

    double yaw =
        atan2(R[1][0],
              R[0][0]);


    m->roll = roll;
    m->pitch = pitch;
    m->yaw = yaw;
}



//convert radians into degrees
double rad2Deg(double r) {
  return r * (180.0 / PI);
}

//convert degrees into radians
double deg2Rad(double r) {
  return r * (PI / 180.0);
}


// Projects vector v onto the plane orthogonal to normal n.
// Both v and n are 3‑element vectors.
// Result is the horizontal component of v (with respect to n).
void projectVectorOntoPlane(const double v[3],
                            const double n[3],
                            double out[3]) {
  // n must be normalized. If not, normalize it.
  double n_norm = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  double nx = n[0] / n_norm;
  double ny = n[1] / n_norm;
  double nz = n[2] / n_norm;

  // Dot product v·n
  double dot = v[0] * nx + v[1] * ny + v[2] * nz;

  // Projection of v onto n: (v·n) * n
  double proj[3] = { dot * nx, dot * ny, dot * nz };

  // Subtract vertical component → horizontal component
  out[0] = v[0] - proj[0];
  out[1] = v[1] - proj[1];
  out[2] = v[2] - proj[2];
}

//normalizes a 3x3 matrix
void normalizeDCM(double R[3][3]) {
  // Extract columns
  double X[3] = { R[0][0], R[1][0], R[2][0] };  // North
  double Y[3] = { R[0][1], R[1][1], R[2][1] };  // East
  double Z[3] = { R[0][2], R[1][2], R[2][2] };  // Down

  // Step 1: Make X and Y orthogonal
  double error = X[0] * Y[0] + X[1] * Y[1] + X[2] * Y[2];  // dot(X,Y)
  double halfErr = 0.5 * error;

  // Correct X and Y
  X[0] -= halfErr * Y[0];
  X[1] -= halfErr * Y[1];
  X[2] -= halfErr * Y[2];

  Y[0] -= halfErr * X[0];
  Y[1] -= halfErr * X[1];
  Y[2] -= halfErr * X[2];

  // Step 2: Renormalize X and Y
  double invX = 1.0 / sqrt(X[0] * X[0] + X[1] * X[1] + X[2] * X[2]);
  double invY = 1.0 / sqrt(Y[0] * Y[0] + Y[1] * Y[1] + Y[2] * Y[2]);

  X[0] *= invX;
  X[1] *= invX;
  X[2] *= invX;
  Y[0] *= invY;
  Y[1] *= invY;
  Y[2] *= invY;

  // Step 3: Recompute Z = X × Y
  Z[0] = X[1] * Y[2] - X[2] * Y[1];
  Z[1] = X[2] * Y[0] - X[0] * Y[2];
  Z[2] = X[0] * Y[1] - X[1] * Y[0];

  // Step 4: Write back into DCM
  R[0][0] = X[0];
  R[1][0] = X[1];
  R[2][0] = X[2];
  R[0][1] = Y[0];
  R[1][1] = Y[1];
  R[2][1] = Y[2];
  R[0][2] = Z[0];
  R[1][2] = Z[1];
  R[2][2] = Z[2];
}



//returns the magnitude of a 1x3 vector
double norm(double n[SIZE]) {
  double sum = 0.0;
  for (int i = 0; i < SIZE; i++) {
    sum += n[i] * n[i];
  }
  return sqrt(sum);
}

//normalizes each element of a 1x3 vector with its magnitude
void normalizeVector(double v[3], double normalized[3]) {
  double n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n < 1e-12) {// avoid divide-by-zero
    normalized[0] = 0;
    normalized[1] = 0;
    normalized[2] = 0;
    return;
  }
  normalized[0] = v[0] / n;
  normalized[1] = v[1] / n;
  normalized[2] = v[2] / n;
}

