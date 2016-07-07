#include "kalman.h"
float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
//
    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix

float getAngle(float newAngle, float newRate, float dt) {
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};



void setAngle(float angle) { angle = angle; }; // Used to set angle, this should be set as the starting angle
float getRate() { return rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(float Q_angle) { Q_angle = Q_angle; };
void setQbias(float Q_bias) { Q_bias = Q_bias; };
void setRmeasure(float R_measure) { R_measure = R_measure; };

float getQangle() { return Q_angle; };
float getQbias() { return Q_bias; };
float getRmeasure() { return R_measure; };