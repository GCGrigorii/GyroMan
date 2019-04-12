#ifndef main_h
#define main_h

// includes
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../inc/tm_stm32f4_mpu6050.h"
#include "../inc/pid.h"
#include "../inc/initial.h"
#include "../inc/usart_cmd.h"
// #include "../inc/IMU.h"
// #include "MadgwickAHRS.h"
// #include "MahonyAHRS.h"

// defines
#define TIMER_DEF 10 // usart print delay TIM4irq \ TIMER_DEF

//USART/IIC
#define RX_BUF_SIZE 80
volatile char RX_FLAG_END_LINE = 0;
volatile char RX_FLAG_READ_END = 0;
volatile char RXi, RXi2;
volatile char RXc, RXc2;
volatile char RX_BUF[RX_BUF_SIZE] = {'\0'};
volatile char RX_BUF2[RX_BUF_SIZE] = {'\0'};
volatile char buffer_str[120];
volatile char buffer_str_t[80];
volatile char I2C_FLAG_READ = 0;
volatile char state = 0;
volatile uint8_t timer = 0; // usart print timer
// CMD
usart_cmd_ UCMD;

// MPU6050
// TM_MPU6050_t MPU6050_Raw;
// TM_MPU6050_t_data MPU6050_Raw_factor;
// TM_MPU6050_t_data MPU6050_Data;
// double roll, pitch, yaw;
double qw, qx, qy, qz;
uint8_t MPtrig = 0;
uint16_t MPcounter;

// komplementar filter
#define FK 0.1 // komplemetray coefficient
#define TO_DEG 57.29577951308232087679815481410517033f
float angle_ax, angle_cpl, last_ay;
double angle_gx;
int dt = 0;
long int t_next, p_next;

//PID
double K_P = 1;
double K_I = 0;
double K_D = 0;
volatile double referenceValue, measurementValue, PID_out;

// functions
void cycle();
void USART1_IRQHandler(void);
void TIM4_IRQHandler(void);
void USARTSendDMA(char*);
void DMA1_Channel4_IRQHandler(void);
void clear_RXBuffer(void);
void clear_RXBuffer2(void);
void toEulerAngle();
float clamp(float, float, float);
double strtd(char* str, int len, char* symbin, int len2,  char symbout);
#endif //main_h
