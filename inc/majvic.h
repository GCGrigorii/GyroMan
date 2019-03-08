// #pragma once
// #ifndef MAJVIC_h
// #define MAJVIC_h
// //code
// #include <math.h>
// // System constants
// #define deltat 0.05f // sampling period in seconds (shown as 50 ms)
// #define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
// #define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
// // Global system variables
// float a_x, a_y, a_z; // accelerometer measurements
// float w_x, w_y, w_z; // gyroscope measurements in rad/s
// float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions
//
// void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);
//
// #endif //MAJVIC_h
