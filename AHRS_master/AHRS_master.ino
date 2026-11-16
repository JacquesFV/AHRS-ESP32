/*Attitude & Heading Estimation System
Outputs: Quaternions: w,i,j,k & Euler angles roll, pitch, yaw
Sensor: SparkFun 9DoF IMU Breakout - ISM330DHCX, MMC5983MA (Qwiic)
What it does: corrects gyroscopic drift by using the accelerometer and magnetometer in a PID filter 
*/

#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <SparkFun_ISM330DHCX.h>
#include <Wire.h>


#include "matrixMath.h"
#include "moreMath.h"
#include "sensorFusion.h"
#include "eulerQuaternion.h"


SFE_MMC5983MA myMag;
SparkFun_ISM330DHCX myISM;

//structs for x,y,z data
sfe_ism_data_t accelData;
sfe_ism_data_t gyroData;

Matrix m = initializeMatrix();
unsigned long previousTime = 0;
unsigned long lastTime = micros();


//use triad method at setup to generate initial DCM from accelerometer and magnetometer only
void buildInitialDCM_TRIAD(Matrix *m, double accelBody[SIZE], double magBody[SIZE]) {

  //normalize accel body vector
  double zb[SIZE];
  copyVector(accelBody, zb);
  normalizeVector(zb, zb);

  //take magnetometer vector and remove gravity component, then normalize; ensures that yaw does not change with pitch and roll
  //note: in California, the magnetic field points significantly downward; we only need
  //the magnetic north projected onto the horizontal plane
  double xb[SIZE];
  projectVectorOntoPlane(magBody, zb, xb);
  normalizeVector(xb, xb);

  double crossMat[SIZE][SIZE];
  double yb[SIZE];

  rcrossMatrix(crossMat, zb);
  multiplyMatrixVector(crossMat, xb, yb);
  normalizeVector(yb, yb);

  double B[SIZE][SIZE];

  B[0][0] = xb[0];
  B[1][0] = xb[1];
  B[2][0] = xb[2];

  B[0][1] = yb[0];
  B[1][1] = yb[1];
  B[2][1] = yb[2];

  B[0][2] = zb[0];
  B[1][2] = zb[1];
  B[2][2] = zb[2];

  /*inertial triad*/

  //normalize accel reference
  double zi[SIZE];
  copyVector(m->accelInertialUnitVector, zi);

  //take mag reference and remove gravity component
  double xi[SIZE];
  projectVectorOntoPlane(m->magInertialUnitVector, zi, xi);
  normalizeVector(xi, xi);

  double yi[SIZE];

  rcrossMatrix(crossMat, zi);
  multiplyMatrixVector(crossMat, xi, yi);
  normalizeVector(yi, yi);

  double I[SIZE][SIZE];

  I[0][0] = xi[0];
  I[1][0] = xi[1];
  I[2][0] = xi[2];

  I[0][1] = yi[0];
  I[1][1] = yi[1];
  I[2][1] = yi[2];

  I[0][2] = zi[0];
  I[1][2] = zi[1];
  I[2][2] = zi[2];

  /*DCM = B *I^T*/
  double IT[SIZE][SIZE];
  transposeMatrix(I, IT);
  multiplyMatrix(B, IT, m->DCM);

  copyMatrix(m->DCM, m->rplus);
}

//gather, calibrate, and aligned data for triad
bool computeInitialAccelMagAligned(Matrix *m, double accelAligned_initial[SIZE], double magAligned_initial[SIZE]) {


  if (!myISM.checkStatus()){
    //no new imu sample yet
    return false;
  }
  //read accel & apply sensitivity
  myISM.getAccel(&accelData);

  double ax_s = accelData.xData / 1000.0;
  double ay_s = accelData.yData / 1000.0;
  double az_s = accelData.zData / 1000.0;

  //apply accel calibration
  double tmp[SIZE] = {
    ax_s - m->accelb[0],
    ay_s - m->accelb[1],
    az_s - m->accelb[2],
  };

  double accelCal[SIZE];
  multiplyMatrixVector(m->accelM, tmp, accelCal);

  //apply accel Alignment matrix (CRITICAL)
  //this makes accel match the runtime fusion frame

  double accelAligned[SIZE];
  multiplyMatrixVector(m->DCMAlign, accelCal, accelAligned);

  //makes sure magnitude lies within gravity magnitude
  double accelNorm = norm(accelAligned);
  if (accelNorm < 0.5 || accelNorm > 1.5){
    //Serial.println("ACCEL GATE");
    return false;
  }

  normalizeVector(accelAligned, accelAligned_initial);  //runtime accel vector


  uint32_t currentX = 0;
  uint32_t currentY = 0;
  uint32_t currentZ = 0;

  //read mag & apply sensitivity and check for trash data
  if (!myMag.readFieldsXYZ(&currentX, &currentY, &currentZ)){
    Serial.println("MAG FAIL");
    return false;
  }


  double scale = 100.0 / 16384.0;  // µT/count

  double mx_s = ((double)currentX - 131072.0) * scale;
  double my_s = ((double)currentY - 131072.0) * scale;
  double mz_s = ((double)currentZ - 131072.0) * scale;

  double magXYZ_initial[SIZE] = { mx_s, my_s, mz_s };


  double magH_initial[SIZE];
  double magCal_initial[SIZE];


  //apply mag calibration
  subtractVector(magXYZ_initial, m->hardIron, magH_initial);
  multiplyMatrixVector(m->softIron, magH_initial, magCal_initial);


  //align mag into fusion frame
  multiplyMatrixVector(m->magAlign, magCal_initial, magAligned_initial);


  //make sure magnitude lies within magnetic field range of locatoin
  double magNorm = norm(magAligned_initial);
  if (magNorm < 20 || magNorm > 80) {
    return false;
  }

  normalizeVector(magAligned_initial, magAligned_initial);

  return true;
}


void testPrint(Matrix *m, double accelAligned[SIZE], double magAligned[SIZE]) {

  double RT[SIZE][SIZE];
  transposeMatrix(m->DCM, RT);

  double mag_nav[SIZE];
  multiplyMatrixVector(RT, magAligned, mag_nav);
  normalizeVector(mag_nav, mag_nav);

  double g_nav[SIZE];
  multiplyMatrixVector(RT, accelAligned, g_nav);
  normalizeVector(g_nav, g_nav);

  double mag_nav1[SIZE];
  multiplyMatrixVector(m->DCM, magAligned, mag_nav1);
  normalizeVector(mag_nav1, mag_nav1);

  double mag_nav2[SIZE];
  multiplyMatrixVector(RT, magAligned, mag_nav2);
  normalizeVector(mag_nav2, mag_nav2);


  double mag_nav_h[SIZE];
  projectVectorOntoPlane(mag_nav, m->accelInertialUnitVector, mag_nav_h);
  normalizeVector(mag_nav_h, mag_nav_h);


  double magRefHorizontal[SIZE];

  projectVectorOntoPlane(m->magInertialUnitVector, m->accelInertialUnitVector, magRefHorizontal);
  normalizeVector(magRefHorizontal, magRefHorizontal);


  double dot_h = mag_nav_h[0] * magRefHorizontal[0] + mag_nav_h[1] * magRefHorizontal[1] + mag_nav_h[2] * magRefHorizontal[2];
  dot_h = constrain(dot_h, -1.0, 1.0);
  double headingErr = rad2Deg(acos(dot_h));

  //Serial.println("mag_hav_h");
  //printVector(mag_nav_h);
  //printVector(accelAligned);
  //printVector(magAligned);
  //printVector(mag_nav1);
  //printVector(mag_nav2);
  //printMatrix(m->DCM);
  //Serial.print("g_nav: ");

  // printVector(g_nav);
  //printVector(m->magInertialUnitVector);
  //Serial.print("mag_nav: ");
  //printVector(mag_nav);
  //printVector(mag_nav_h);
  //printVector(magRefHorizontal);

  //printVector(m->accelInertialUnitVector);
  //Serial.print("err: ");
  Serial.println(headingErr);
  Serial.println();
}


void setup() {
  Serial.begin(115200);
  Wire.begin();

  /*gyroscope and accelerometer setup*/
  if (!myISM.begin()) {
    Serial.println("Did not begin");
    while (true)
      ;
  }

  //reset the device to default setting; useful if you're doing multiple uploads testing different settings
  myISM.deviceReset();

  //wait for it to finish resetting
  while (!myISM.getDeviceReset()) {
    delay(1);
  }

  Serial.println("Reset");
  Serial.println("Applying settings");
  delay(100);

  //set the output data rate and precision of the accelerometer
  myISM.setAccelDataRate(ISM_XL_ODR_416Hz);
  myISM.setAccelFullScale(ISM_4g);

  //set the output data rate and precision of the gyroscope
  myISM.setGyroDataRate(ISM_GY_ODR_416Hz);
  myISM.setGyroFullScale(ISM_500dps);

  //turn on the accelerometer's filter and apply settings
  myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

  //turn on the gyroscope's filter and apply settings
  myISM.setGyroFilterLP1();
  myISM.setGyroLP1Bandwidth(ISM_MEDIUM);

  //protect data integrity
  myISM.setBlockDataUpdate();

  //run device config to lock in and wrap up all configurations
  myISM.setDeviceConfig();


  /*magnetometer setup*/

  if (!myMag.begin()) {
    Serial.println("MMC5983MA did not respond - check your wiring. Freezing");
    while (true)
      ;
  }

  //1. reset and give the internal charge pump a moment to stabilize
  myMag.softReset();
  delay(20);

  //2.configure the data rates and bandwidth first
  myMag.setFilterBandwidth(100);          //100 hz
  myMag.setContinuousModeFrequency(100);  //100 hz


  //3. Kick off continuous mode so the internal clock manager is running
  myMag.enableContinuousMode();
  delay(10);


  //4. enable magnetic SET/RESET once per boot. THIS IS NECESSARY TO MAGNETIZE THE SENSOR!!!
  myMag.enableAutomaticSetReset();


  //5. Clear out hte initial stabilazation readings
  delay(500);  //flush out trash data

  Serial.println("MMC5983MA connected");


  /*generate initial dcm using triad with initial aligned accel and mag*/
  double magAligned_initial[SIZE];
  double accelAligned_initial[SIZE];

  // average and initialize
  double magSum[SIZE] = { 0 };
  double accelSum[SIZE] = { 0 };


  int validSamples = 0;
  while (validSamples < 50) {

    if (computeInitialAccelMagAligned(&m, accelAligned_initial, magAligned_initial)) {

      accelSum[0] += accelAligned_initial[0];
      accelSum[1] += accelAligned_initial[1];
      accelSum[2] += accelAligned_initial[2];

      magSum[0] += magAligned_initial[0];
      magSum[1] += magAligned_initial[1];
      magSum[2] += magAligned_initial[2];
      validSamples++;
    }

    delay(10);
  }


  magAligned_initial[0] = magSum[0] / validSamples;
  magAligned_initial[1] = magSum[1] / validSamples;
  magAligned_initial[2] = magSum[2] / validSamples;

  accelAligned_initial[0] = accelSum[0] / validSamples;
  accelAligned_initial[1] = accelSum[1] / validSamples;
  accelAligned_initial[2] = accelSum[2] / validSamples;

  normalizeVector(accelAligned_initial, accelAligned_initial);
  normalizeVector(magAligned_initial, magAligned_initial);


  //build initial dcm using triad
  buildInitialDCM_TRIAD(&m, accelAligned_initial, magAligned_initial);


  //testPrint(&m, accelAligned_initial, magAligned_initial);
}


void collect(double x, double y, double z) {
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.println();
}

void loop() {
//test
static uint32_t heartbeat = 0;

if (millis() - heartbeat > 1000)
{
    heartbeat = millis();
  Serial.println("ALIVE");
}
//

  //set timing
  unsigned long now = micros();
  unsigned long currentTime = millis();
  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;

  double ax_s = 0;
  double ay_s = 0;
  double az_s = 0;

  double gx_s = 0;
  double gy_s = 0;
  double gz_s = 0;

  //check for trash data
  if (!myISM.checkStatus()) {
     //no new imu sample yet

    return;
  }

  //collect accel and gyro data + sensitivity; convert (mg) to (g); convert (mdps) to (dps)
  myISM.getAccel(&accelData);
  myISM.getGyro(&gyroData);

  ax_s = accelData.xData / 1000.0;
  ay_s = accelData.yData / 1000.0;
  az_s = accelData.zData / 1000.0;

  gx_s = gyroData.xData / 1000.0;
  gy_s = gyroData.yData / 1000.0;
  gz_s = gyroData.zData / 1000.0;

  //calibrate accel
  double tmp[SIZE] = { ax_s - m.accelb[0], ay_s - m.accelb[1], az_s - m.accelb[2] };
  double accelCal[SIZE];
  multiplyMatrixVector(m.accelM, tmp, accelCal);

  //align to fusion runtime frame
  double accelAligned[SIZE];
  multiplyMatrixVector(m.DCMAlign, accelCal, accelAligned);

  //gravity magnitude gate
  double accelNorm = norm(accelAligned);

  if (accelNorm < 0.5 || accelNorm > 1.5) {
   // Serial.println("ACCEL GATE");
    return;
  }

  //normalize accel
  normalizeVector(accelAligned, m.accels);

  //calibrate gyro
  double tmp1[SIZE] = { deg2Rad(gx_s - m.gyroNullShift[0]), deg2Rad(gy_s - m.gyroNullShift[1]), deg2Rad(gz_s - m.gyroNullShift[2]) };

  //align to fusion runtime frame
  double gyroAligned[SIZE];
  multiplyMatrixVector(m.DCMAlign, tmp1, gyroAligned);
  copyVector(gyroAligned, m.gyros);


  uint32_t currentX = 0;
  uint32_t currentY = 0;
  uint32_t currentZ = 0;

  //collect mag data
  //check for trash data
  if (!myMag.readFieldsXYZ(&currentX, &currentY, &currentZ)) {
    Serial.println("READ FAIL");
    return;
  }

  double scale = 100.0 / 16384.0;  // µT/count
  double magXYZ[SIZE] = { ((double)currentX - 131072.0) * scale, ((double)currentY - 131072.0) * scale, ((double)currentZ - 131072.0) * scale };


  //logging raw mag
  //
/*
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    double heading = rad2Deg(atan2(magXYZ[1], magXYZ[0]));
    collect(magXYZ[0], magXYZ[1], magXYZ[2]);
    
    //Serial.println(heading);
  }*/




  //calibrate mag
  double magH[SIZE] = { 0 };
  double magCal[SIZE] = { 0 };

  subtractVector(magXYZ, m.hardIron, magH);

  multiplyMatrixVector(m.softIron, magH, magCal);

  //align to fusion runtime frame
  double magAligned[SIZE];
  multiplyMatrixVector(m.magAlign, magCal, magAligned);



  //magnetic field gate
  double magNorm = norm(magAligned);
  //makes sure the norm is around the magnetic field magnitude of location
  bool magValid = (magNorm > 20.0) && (magNorm < 80.0);
  if (!magValid) {
    Serial.print("MAG NORM= ");
    Serial.println(magNorm);
    Serial.print("MAG ALIGNED= ");
    Serial.print(magAligned[0]);
    Serial.print(" ");
    Serial.print(magAligned[1]);
    Serial.print(" ");
    Serial.println(magAligned[2]);


    return;
  }

  normalizeVector(magAligned, m.mag);

  //fusion
  integrateClosedLoop(&m, m.DCM, m.bplus, dt);

  //extract quaternion & euler
  Quaternion quat = quatFromDCM(m.DCM);

  extractAngles(&m, m.DCM);






  //test
  /**double heading_raw = atan2(magAligned[1], magAligned[0]);
  double roll_acc =
    atan2(m.accels[1], m.accels[2]);
  double pitch_acc =
    atan2(-m.accels[0],
          sqrt(m.accels[1] * m.accels[1] + m.accels[2] * m.accels[2]));
  double gpred[3];
  multiplyMatrixVector(
    m.DCM,
    m.accelInertialUnitVector,
    gpred);
  double roll_g =
    atan2(m.DCM[1][2],
          m.DCM[2][2]);
  double pitch_g =
    atan2(-m.DCM[0][2],
          sqrt(
            m.DCM[1][2] * m.DCM[1][2] + m.DCM[2][2] * m.DCM[2][2]));*/
  //


  if ((currentTime - previousTime) >= 20) {
    previousTime = currentTime;

    Serial.print("roll: ");
    Serial.print(rad2Deg(m.roll));
    Serial.print(" ");
    Serial.print("pitch: ");
    Serial.print(rad2Deg(m.pitch));
    Serial.print(" ");
    Serial.print("yaw: ");
    Serial.println(rad2Deg(m.yaw));


    /*Serial.print(quat.w);
    Serial.print(" ");
    Serial.print(quat.x);
    Serial.print(" ");
    Serial.print(quat.y);
    Serial.print(" ");
    Serial.print(quat.z);
    Serial.println();*/
  }
}