#include "matrixMath.h"
#include "moreMath.h"

//initialize vectors and matrices as well as gravity and magnetic reference
Matrix initializeMatrix(void) {
  Matrix m;

  //euler angles set to 0 (for extractAngles() function)
  m.pitch = 0;
  m.roll = 0;
  m.yaw = 0;

  //clear matrices and vectors
  clearMatrix(m.rcross);
  clearMatrix(m.rexp);
  clearMatrix(m.rplus);
  clearVector(m.bplus);
  clearVector(m.gyros);
  clearVector(m.accels);
  clearVector(m.mag);

  //set matrices to identity
  eye(m.rplus);
  eye(m.DCM);


  //inertial frame accelerometer (aerospace standard)
  //NOTE:NED (North = 0; EAST= 0; DOWN = 1)
  m.accelInertial[0] = 0;
  m.accelInertial[1] = 0;
  m.accelInertial[2] = 1;

  scalerDivideVector(m.accelInertial, m.accelInertialUnitVector, norm(m.accelInertial));

  //inertial-frame magnetic reference in San Leandro, California [23.4, 5.4, 41.6]uT
  //unit vector: magInertial_unitVector = magInertial/norm(magInertial)
  //norm magnitude = sqrt((23.4^2) + (-5.4^2)+(41.6^2)) = 48
  //normalize: magInertial = 1/48 [23.4, 5.4, 41.6]
  m.magInertial[0] = 22.3848;  //horizontal intensity
  m.magInertial[1] = 0;        //5.1223;
  m.magInertial[2] = 41.4702;  //vertical component


  //gyro null shift calculated by placing sensor still and collecting 10 sec of data & averaged
  //note: scale factor was negligible for this sensor
  //matlab calculation code: gyros_nullShift_scaleFactor.m
  m.gyroNullShift[0] = 0.0192;
  m.gyroNullShift[1] = -0.5934;
  m.gyroNullShift[2] = 0.1332;


  //mag bias calculated by taking tumble data and running through matlab magcal
  //matlab calculation code: magTumble.m
  double softIronTemp[SIZE][SIZE] = {
    { 1.0029, 0.0053, -0.0024 },
    { 0.0053, 0.9744, 0.0065 },
    { -0.0024, 0.0065, 1.0235 }
  };
  memcpy(m.softIron, softIronTemp, sizeof(softIronTemp));

  double hardIronTemp[SIZE] = { 13.8700, 5.4403, 3.8453 };
  memcpy(m.hardIron, hardIronTemp, sizeof(hardIronTemp));

  /*m.softIron[0][0] = 0.9991; 
  m.softIron[0][1] = 0.0078; 
  m.softIron[0][2] = 0.0015;
  m.softIron[1][0] = 0.0078;
  m.softIron[1][1] = 0.9713; 
  m.softIron[1][2] = 0.0276;
  m.softIron[2][0] = 0.0015; 
  m.softIron[2][1] = 0.0276; 
  m.softIron[2][2] = 1.0314;  
  //hard iron bias(uT)
  m.hardIron[0] = 21.6759;   
  m.hardIron[1] = 39.7449; 
  m.hardIron[2] = 14.6147;*/



  //accel bias:
  //1) collect six static poses 'raw' (+x_up, -x_up, +y_up, -y_up, +z_up,-z_up) for ~200 samplea and average the mean vector;
  //2) accelb is the average fo the opposite faces : b_x = ((+x_avg) + (-x_avg)/2; b_y = ((+y_avg) + (-y_avg))/2; b_z = ((+z_avg)+ (-z_avg))/2, etc...
  //3) computer the scale factors: s_x = 2/((+x_avg)-(-x_avg)); s_y = 2/((+y_avg)-(-y_avg)); s_z = 2/((+z_avg)-(-z_avg)), etc...
  //   build diagonal scale matrix: S = diag([sx,sy,sz]);
  //4) compute misalignment matrix. we want the calibrated vectors to match the ideal axes:
  //+X -> [1,0,0]
  //-X ->[-1,0,0]
  //+Y ->[0,1,0]
  //-Y ->[0,-1,0]
  //+Z ->[0,0,1]
  //-Z ->[0,0,-1]
  //vx = ((+x_avg) - (-x_avg))/2; vy = ((+y_avg) - (-y_avg))/2; vz = ((+z_avg)-(-z_avg))/2
  //build matrices V = [vx -accelb, vy - accelb, vz -accelb]
  //               I = [1 0 0; 0 1 0; 0 0 1]
  //apply scale Vs = S*V
  //solve the full calibration matrix using Wahba's problem: accelM = whabaSVD(Vs,I)*S
  //matlab code: accel_calibration.m

  m.accelM[0][0] = 1.0000;
  m.accelM[0][1] = -0.0030;
  m.accelM[0][2] = -0.0025;
  m.accelM[1][0] = 0.0030;
  m.accelM[1][1] = 1.0050;
  m.accelM[1][2] = -0.0076;
  m.accelM[2][0] = 0.0025;
  m.accelM[2][1] = 0.0076;
  m.accelM[2][2] = 1.0050;

  m.accelb[0] = -0.0100;
  m.accelb[1] = -0.0150;
  m.accelb[2] = 0.0250;


  //matrix for aligning calibrated accel and gyro axes to the runtime fusion frame;
  //calibrated accel data was collected by posing the board under its 6 axes (flat, upside down,
  //right side down, left side down, nose down, and nose up). Alignment matrix and vectors are generated using
  //measured data and the ideal gravity direction in board frame through matlab's svd()
  //matlab code: DCMAlignAccel.m
  m.DCMAlign[0][0] = 1.0000;
  m.DCMAlign[0][1] = -0.0025;
  m.DCMAlign[0][2] = -0.0050;

  m.DCMAlign[1][0] = 0.0025;
  m.DCMAlign[1][1] = 1.0000;
  m.DCMAlign[1][2] = -0.0075;

  m.DCMAlign[2][0] = 0.0050;
  m.DCMAlign[2][1] = 0.0075;
  m.DCMAlign[2][2] = 1.0000;



  // Magnetometer axis alignment.
  // Determined experimentally by comparing magnetometer heading
  // to the calibrated accelerometer/gyro body frame.

  double magAlignTemp[SIZE][SIZE] = {
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, -1 }
  };
  memcpy(m.magAlign, magAlignTemp, sizeof(magAlignTemp));

  double magInertialUnitVector_initial[SIZE];

  // 1. Normalize measured horizontal magnetic field
  scalerDivideVector(m.magInertial, m.magInertialUnitVector, norm(m.magInertial));

  // 2. Normalize again (CRITICAL)
  normalizeVector(m.magInertialUnitVector, magInertialUnitVector_initial);

  // 3. Store final inertial magnetic reference
  copyVector(magInertialUnitVector_initial, m.magInertialUnitVector);


  return m;
}

//sets each element of 3x3 matrix to 0
void clearMatrix(double a[SIZE][SIZE]) {
  int row, col;

  for (row = 0; row < SIZE; row++) {
    for (col = 0; col < SIZE; col++) {
      a[row][col] = 0;
    }
  }
}

//prints out a 3x3 matrix
void printMatrix(double c[SIZE][SIZE]) {
  int row, col;

  for (int row = 0; row < SIZE; row++) {
    for (int col = 0; col < SIZE; col++) {
      Serial.print(c[row][col], 11);
      Serial.print(" ");  // space between numbers
    }
    Serial.println();  // new line after each row
  }
}

//multiplies two 3x3 matrices; outputs a 3x3 product
void multiplyMatrix(double m1[SIZE][SIZE], double m2[SIZE][SIZE], double product[SIZE][SIZE]) {

  double value;
  for (int i = 0; i < SIZE; i++) {
    for (int j = 0; j < SIZE; j++) {
      value = 0;
      for (int k = 0; k < SIZE; k++) {
        value += m1[i][k] * m2[k][j];
      }
      product[i][j] = value;
    }
  }
  //printMatrix(product);
}

//adds two 3x3 matrices; outputs a 3x3 sum
void addMatrix(double m1[SIZE][SIZE], double m2[SIZE][SIZE], double sum[SIZE][SIZE]) {
  int indexRow = 0;
  int indexCol = 0;

  for (indexRow = 0; indexRow < SIZE; indexRow++) {
    for (indexCol = 0; indexCol < SIZE; indexCol++) {
      sum[indexRow][indexCol] = m1[indexRow][indexCol] + m2[indexRow][indexCol];
    }
  }
  //printMatrix(sum);
}

//transposes a 3x3 matrix into another matrix
void transposeMatrix(double m[SIZE][SIZE], double transpose[SIZE][SIZE]) {
  int indexRow = 0;
  int indexCol = 0;

  for (indexRow = 0; indexRow < SIZE; indexRow++) {
    for (indexCol = 0; indexCol < SIZE; indexCol++) {
      transpose[indexCol][indexRow] = m[indexRow][indexCol];
    }
  }
  //printMatrix(transpose);
}

//creates the skew-symmetric cross-product 3x3 matrix of a 1x3 vector
void rcrossMatrix(double rcross[SIZE][SIZE], double r[SIZE]) {
  rcross[0][0] = 0;
  rcross[0][1] = -r[2];
  rcross[0][2] = r[1];
  rcross[1][0] = r[2];
  rcross[1][1] = 0;
  rcross[1][2] = -r[0];
  rcross[2][0] = -r[1];
  rcross[2][1] = r[0];
  rcross[2][2] = 0;

  //printMatrix(rcross);
}

//calculates the rotation matrix expononential; updates the attitude using gyro measurements
void rExpMatrix(double w[3], double R[3][3]) {

  double theta = norm(w);

  double wx[3][3];
  rcrossMatrix(wx, w);

  double I[3][3];
  eye(I);

  if (theta < 1e-9) {
    // small-angle: R ≈ I + [w]×
    double wxdt[3][3];
    scalerMultiplyMatrix(wx, wxdt, 1.0);
    addMatrix(I, wxdt, R);
    return;
  }

  // MATLAB: s = sinc(theta/2/pi)
  // sinc(x) in MATLAB = sin(pi*x)/(pi*x)
  // So:
  // s = sin(theta/2) / (theta/2)
  double s = sin(theta / 2.0) / (theta / 2.0);
  double c = cos(theta / 2.0);

  double term1[3][3];
  double term2[3][3];
  double wx2[3][3];

  // s*c * wx
  scalerMultiplyMatrix(wx, term1, s * c);

  // s*s/2 * wx^2
  multiplyMatrix(wx, wx, wx2);
  scalerMultiplyMatrix(wx2, term2, (s * s) / 2.0);

  double temp[3][3];
  addMatrix(I, term1, temp);
  addMatrix(temp, term2, R);
}

//sets a value to a specific rol and col of 3x3 matrix
void storeValueMatrix(double m[SIZE][SIZE], int row, int col, double value) {
  m[row][col] = value;
}

//copies 3x3 matrix into another 3x3 matrix
void copyMatrix(double m1[SIZE][SIZE], double m2[SIZE][SIZE]) {
  int indexRow = 0;
  int indexCol = 0;

  for (indexRow = 0; indexRow < SIZE; indexRow++) {
    for (indexCol = 0; indexCol < SIZE; indexCol++) {
      m2[indexRow][indexCol] = m1[indexRow][indexCol];
    }
  }
  //printMatrix(m2);
}

//sets a 3x3 matrix into an identity matrix
void eye(double m[SIZE][SIZE]) {
  int indexRow = 0;
  int indexCol = 0;

  for (indexRow = 0; indexRow < SIZE; indexRow++) {
    for (indexCol = 0; indexCol < SIZE; indexCol++) {
      if ((indexRow == indexCol)) {
        m[indexRow][indexCol] = 1;
      } else {
        m[indexRow][indexCol] = 0;
      }
    }
  }
  //printMatrix(m);
}

//multiplies each element of 3x3 matrix with a scaler; outputs a 3x3 product
void scalerMultiplyMatrix(double m[SIZE][SIZE], double product[SIZE][SIZE], double value) {
  int indexRow = 0;
  int indexCol = 0;

  for (indexRow = 0; indexRow < SIZE; indexRow++) {
    for (indexCol = 0; indexCol < SIZE; indexCol++) {
      product[indexRow][indexCol] = m[indexRow][indexCol] * value;
    }
  }
  //printMatrix(product);
}

//multiplies two 3x3 matrices; outputs a 3x3 matrix
void multiplyMatrixVector(double m[SIZE][SIZE], double v[SIZE], double product[SIZE]) {
  product[0] = (m[0][0] * v[0]) + (m[0][1] * v[1]) + (m[0][2] * v[2]);
  product[1] = (m[1][0] * v[0]) + (m[1][1] * v[1]) + (m[1][2] * v[2]);
  product[2] = (m[2][0] * v[0]) + (m[2][1] * v[1]) + (m[2][2] * v[2]);

  //printVector(product);
}

//multiplies each element of 1x3 vector with a scalar; outputs a 1x3 product
void scalerMultiplyVector(double v[SIZE], double product[SIZE], double value) {
  int index = 0;
  for (index = 0; index < SIZE; index++) {
    product[index] = v[index] * value;
  }
  //printVector(product);
}

//divides each element of a 1x3 vector with a scalar; outputs a 1x3 quotient
void scalerDivideVector(double v[SIZE], double quotient[SIZE], double value) {
  int index = 0;


  if (value == 0) {
    Serial.println("ERROR (scalerDivideVector()): Value cannot be 0.");
    while (true) {
      // freeze
    }
  } else {
    for (index = 0; index < SIZE; index++) {
      quotient[index] = v[index] / value;
    }
    //printVector(quotient);
  }
}

//prints out each elements of a 1x3 vector
void printVector(double v[SIZE]) {
  int index = 0;
  for (index = 0; index < SIZE; index++) {
    Serial.print(v[index], 5);
    Serial.print(" ");
    Serial.print("\t");
  }
  Serial.println();
}


//sets 1x3 vector elements to 0
void clearVector(double v[SIZE]) {
  int index = 0;
  for (index = 0; index < SIZE; index++) {
    v[index] = 0;
  }
  // printVector(v);
}

//copy 1x3 vector to another 1x3 vector
void copyVector(double v1[SIZE], double v2[SIZE]) {
  int index = 0;
  for (index = 0; index < SIZE; index++) {
    v2[index] = v1[index];
  }
  // printVector(sum);
}

//add two 1x3 vector; outputs a 1x3 sum
void addVector(double v1[SIZE], double v2[SIZE], double sum[SIZE]) {
  int index = 0;
  for (index = 0; index < SIZE; index++) {
    sum[index] = v1[index] + v2[index];
  }
  // printVector(sum);
}


//subtract two 1x3 vectors; outputs a 1x3 difference
void subtractVector(double v1[SIZE], double v2[SIZE], double difference[SIZE]) {
  int index = 0;

  for (index = 0; index < SIZE; index++) {
    difference[index] = v1[index] - v2[index];
  }
  //printVector(difference);
}


// Function to compute the dot product of two 1x3 vectors; outputs 1x3 product
double dotProduct(const double *vec1, const double *vec2) {
  double result = 0.0;
  for (int i = 0; i < SIZE; i++) {
    result += vec1[i] * vec2[i];
  }
  return result;
}
