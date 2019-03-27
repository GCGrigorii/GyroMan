#ifndef main_h
#define main_h

// includes
#include "tm_stm32f4_mpu6050.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "pid.h"
#include "../inc/initial.h"
#include "MadgwickAHRS.h"
// #include "MahonyAHRS.h"

// defines
#define TIMER_DEF 5
#define RX_BUF_SIZE 80
#define FK 0.05 // komplemetray coefficient
#define TO_DEG 57.29577951308232087679815481410517033f
// variables
volatile char RX_FLAG_END_LINE = 0;
volatile char RXi;
volatile char RXc;
volatile char RX_BUF[RX_BUF_SIZE] = {'\0'};
volatile char str[120];
volatile char I2C_FLAG_READ = 0;
volatile char state = 0;
TM_MPU6050_t MPU6050_Raw;
TM_MPU6050_t_data MPU6050_Raw_factor;
TM_MPU6050_t_data MPU6050_Data;
volatile uint8_t timer = 0;
float roll, pitch, yaw;
//komplementar filter
float angle_ax, angle_gx, angle_cpl;
int dt = 0;
long int t_next, p_next;
//PID
double K_P = 0;
double K_I = 0;
double K_D = 0;
volatile int16_t referenceValue, measurementValue, PID_out;

// functions
void cycle();
void USART1_IRQHandler(void);
void TIM4_IRQHandler(void);
void USARTSendDMA(char*);
void DMA1_Channel4_IRQHandler(void);
void clear_RXBuffer(void);
void toEulerAngle();
void pwmREG(uint8_t, uint8_t);
float clamp(float, float, float);
#endif //main_h
