#ifndef _MATRIXMATH_H_
#define _MATRIXMATH_H_

#include <Arduino.h>




#define SIZE 3

typedef struct Matrix{
  double pitch;
  double yaw;
  double roll;
  
  double DCM[SIZE][SIZE];

  double rcross[SIZE][SIZE];
  double rexp[SIZE][SIZE];
  double rplus[SIZE][SIZE];
  double bplus[SIZE];
  double hardIron[SIZE];
  double softIron[SIZE][SIZE];
  

  double gyros[SIZE];
  double mag[SIZE];
  double accels[SIZE];
  double magInertial[SIZE];
  double accelInertial[SIZE];
  double magInertialUnitVector[SIZE];
  double accelInertialUnitVector[SIZE];
  
  double DCMAlign[SIZE][SIZE];
  

  double accelM[SIZE][SIZE];
  double accelb[SIZE];

  double magAlign[SIZE][SIZE];

  double gyroNullShift[SIZE];


}Matrix;


Matrix initializeMatrix(void);
void printMatrix(double [SIZE][SIZE]);
void clearMatrix(double [SIZE][SIZE]);


void multiplyMatrix(double[SIZE][SIZE], double[SIZE][SIZE], double[SIZE][SIZE]);
void addMatrix(double [SIZE][SIZE], double [SIZE][SIZE],double [SIZE][SIZE]);
void transposeMatrix(double [SIZE][SIZE], double [SIZE][SIZE]);
void storeValueMatrix(double [SIZE][SIZE], int, int, double);
void copyMatrix(double [SIZE][SIZE], double [SIZE][SIZE]);

void rExpMatrix(double [SIZE], double [SIZE][SIZE]);
void rcrossMatrix(double [SIZE][SIZE], double [SIZE]);
void eye(double [SIZE][SIZE]);

void scalerMultiplyMatrix(double [SIZE][SIZE], double [SIZE][SIZE], double);
void multiplyMatrixVector(double [SIZE][SIZE], double [SIZE], double [SIZE]);
void scalerMultiplyVector(double [SIZE], double [SIZE], double);
void scalerDivideVector(double [SIZE], double [SIZE], double);

void printVector(double [SIZE]);
void clearVector(double [SIZE]);
void addVector(double [SIZE], double [SIZE], double [SIZE]);
void subtractVector(double [SIZE], double [SIZE], double [SIZE]);
void copyVector(double [SIZE], double [SIZE]);

double dotProduct(const double *, const double *);


#endif