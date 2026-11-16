#include "sensorFusion.h"
//pid controller function thata integrates gyro with accel and mag correction
void integrateClosedLoop(Matrix *m,
                         double rMinus[SIZE][SIZE],
                         double bMinus[SIZE],
                         double dt) {
  // Gains (now in rad/s units)
  const double kpA = 3;          // accel proportional gain
  const double kiA = kpA / 200;  // accel integral gain
  const double kpM = 4.0;        // mag proportional gain (yaw only)
  const double kiM = kpM / 500;  // mag integral gain

  // --- Scratch variables ---
  double gyroInputBias[SIZE] = { 0 };
  double gyroInputAccelFeedback[SIZE] = { 0 };
  double gyroInputFeedback[SIZE] = { 0 };
  double gyroInputFeedbackRad[SIZE] = { 0 };

  double a_pred[SIZE] = { 0 };
  double wMeasA[SIZE] = { 0 };

  double mag_meas_h[SIZE] = { 0 };
  double mag_pred_h[SIZE] = { 0 };
  double m_pred[SIZE] = { 0 };
  double g_pred[SIZE] = { 0 };

  double wMeasM[SIZE] = { 0 };

  double tempA[SIZE] = { 0 };
  double tempM[SIZE] = { 0 };
  double bdot[SIZE] = { 0 };
  double bdot_dt[SIZE] = { 0 };

  // --- 1) Gyro minus bias (rad/s) ---
  subtractVector(m->gyros, bMinus, gyroInputBias);

  // Predicted gravity in body frame
  multiplyMatrixVector(rMinus, m->accelInertialUnitVector, a_pred);

  // Body-frame attitude error from gravity: wMeasA = a_pred x a_measured
  rcrossMatrix(m->rcross, a_pred);
  multiplyMatrixVector(m->rcross, m->accels, wMeasA);

  // --- 2) Yaw-only magnetometer correction (rad/s) ---

  // Predicted gravity again (explicit)
  multiplyMatrixVector(rMinus, m->accelInertialUnitVector, g_pred);

  // Project measured mag onto horizontal plane
  projectVectorOntoPlane(m->mag, m->accels, mag_meas_h);

  // Predicted mag from inertial reference
  multiplyMatrixVector(rMinus, m->magInertialUnitVector, m_pred);
  //takes predicted mag and removes gravity/vertical component
  projectVectorOntoPlane(m_pred, g_pred, mag_pred_h);

  // Normalize both
  //prevent amplifying a nearly-zero horizontal field
  double yawError = 0;
  double hmag = norm(mag_meas_h);
  if (hmag >= 0.1) {
    normalizeVector(mag_meas_h, mag_meas_h);
    normalizeVector(mag_pred_h, mag_pred_h);

    //2) magnetometer attitude correction (body-frame error)
    //mag_pred_h x mag_meas_h
    double magError[SIZE];
    rcrossMatrix(m->rcross, mag_pred_h);
    multiplyMatrixVector(m->rcross, mag_meas_h, magError);

    yawError = magError[0] * g_pred[0] + magError[1] * g_pred[1] + magError[2] * g_pred[2];

    //proportional gain
    wMeasM[0] = yawError * g_pred[0] * kpM;
    wMeasM[1] = yawError * g_pred[1] * kpM;
    wMeasM[2] = yawError * g_pred[2] * kpM;
  } else {
    clearVector(wMeasM);
  }

  ///////////////////////////////////////////
  // --- 3) Gyro + accel + mag proportional feedback (rad/s) ---

  // Accel proportional correction
  scalerMultiplyVector(wMeasA, tempA, kpA);

  // Combine gyro − bias + accel
  addVector(gyroInputBias, tempA, gyroInputAccelFeedback);

  // Add mag yaw correction
  addVector(gyroInputAccelFeedback, wMeasM, gyroInputFeedback);

  // Convert to angular rate (rad/s) into rotation vector(rad)
  scalerMultiplyVector(gyroInputFeedback, gyroInputFeedbackRad, dt);

  // Exponential map rotation update
  rExpMatrix(gyroInputFeedbackRad, m->rexp);

  // Update DCM: R+ = exp([w]× dt) * R-
  double Rtemp[SIZE][SIZE];
  multiplyMatrix(m->rexp, rMinus, Rtemp);
  copyMatrix(Rtemp, m->DCM);

  // Normalize DCM
  normalizeDCM(m->DCM);

  // --- 4) Bias integral correction (rad/s) ---

  // Accel integral term
  scalerMultiplyVector(wMeasA, tempA, -kiA);


  tempM[0] = -kiM * yawError * g_pred[0];
  tempM[1] = -kiM * yawError * g_pred[1];
  tempM[2] = -kiM * yawError * g_pred[2];

  // Combine
  addVector(tempA, tempM, bdot);

  // Integrate bias: b += bdot * dt
  scalerMultiplyVector(bdot, bdot_dt, dt);
  addVector(bMinus, bdot_dt, m->bplus);

  // Update bias state
  copyVector(m->bplus, bMinus);
}






//accel correction ONLY for pitch and roll
void integrateClosedLoopAccel(Matrix *m, double rMinus[SIZE][SIZE], double bMinus[SIZE], double dt) {

  double kpA = 1;  // keep this strong for now
  double kiA = kpA / 200;

  double wMeasA[SIZE] = { 0 };
  double rMinusTransposed[SIZE][SIZE] = { 0 };
  double a_pred[SIZE] = { 0 };
  double gyroInputFeedback[SIZE] = { 0 };
  double gyroInputFeedbackRadians[SIZE] = { 0 };


  // R- is previous DCM
  transposeMatrix(rMinus, rMinusTransposed);

  // predicted accel in body frame
  multiplyMatrixVector(rMinusTransposed, m->accelInertialUnitVector, a_pred);

  // wMeasA = a_meas × a_pred
  rcrossMatrix(m->rcross, m->accels);  // [a_meas]_x
  multiplyMatrixVector(m->rcross, a_pred, wMeasA);

  // proportional correction only
  // 1) gyro minus bias
  double gyroInputBias[SIZE] = { 0 };
  subtractVector(m->gyros, bMinus, gyroInputBias);

  // 2) gyro + accel correction + mag correction
  double temp[SIZE] = { 0 };
  scalerMultiplyVector(wMeasA, temp, kpA);
  addVector(gyroInputBias, temp, gyroInputFeedback);

  // integrate: R+ = exp([w]× dt) * R-
  scalerMultiplyVector(gyroInputFeedback, gyroInputFeedbackRadians, dt);
  rExpMatrix(gyroInputFeedbackRadians, m->rexp);
  multiplyMatrix(rMinus, m->rexp, m->rplus);
  normalizeDCM(m->rplus);
  copyMatrix(m->rplus, m->DCM);


  //GATING for bias integration
  // accel magnitude check (near 1 g)
  double ax = m->accels[0];
  double ay = m->accels[1];
  double az = m->accels[2];
  double anorm = sqrt(ax * ax + ay * ay + az * az);
  bool accel_ok = fabs(anorm - 1.0) < 0.25;  // tune threshold

  // low-dynamics check (small correction)
  double th = 0.15;  // rad/s, tune this
  bool low_dyn =
    fabs(wMeasA[0]) < th && fabs(wMeasA[1]) < th && fabs(wMeasA[2]) < th;
  //


  double bdot[SIZE] = { 0 };
  if (accel_ok && low_dyn) {
    //3)bias correction
    scalerMultiplyVector(wMeasA, bdot, kiA * -1);


  } else {
    clearVector(bdot);
  }


  double bdot_dt[SIZE] = { 0 };
  scalerMultiplyVector(bdot, bdot_dt, dt);
  addVector(bMinus, bdot_dt, m->bplus);
}
