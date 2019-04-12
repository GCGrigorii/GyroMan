/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/



#include "main.h"

//min func
int main(void) {

	uint8_t sensor1 = 0;
	init_();

	for(int i = 0; i < 1000000; i++);
	USARTSendDMA("INIT_START\r\n");
	sensor1 = TM_MPU6050_Init(&MPU6050_Raw, TM_MPU6050_Device_0,
		TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_500s);
	for(int i = 0; i < 1000000; i++);
	if (sensor1 == TM_MPU6050_Result_Ok) {
	        /* Display message to user */
		USARTSendDMA("MPU6050_INITED\r\n");

	        /* Sensor 1 OK */
	        sensor1 = 1;
	    }
	else
	{
		strcmp(buffer, itoa(sensor1));
		strcmp(buffer, " - MPU6050_INIT_ERROR\r\n");
		USARTSendDMA(buffer);
		while(1);
	}
	for(int i = 0; i < 1000000; i++);

	for (int i = 0; i < 1000; i++ )
	{
		TM_MPU6050_ReadAll(&MPU6050_Raw);
		MPU6050_Raw_factor.Accelerometer_X += (float)MPU6050_Raw.Accelerometer_X;
		MPU6050_Raw_factor.Accelerometer_Y += (float)MPU6050_Raw.Accelerometer_Y;
		MPU6050_Raw_factor.Accelerometer_Z += (float)MPU6050_Raw.Accelerometer_Z;
		MPU6050_Raw_factor.Gyroscope_X += (float)MPU6050_Raw.Gyroscope_X;
		MPU6050_Raw_factor.Gyroscope_Y += (float)MPU6050_Raw.Gyroscope_Y;
		MPU6050_Raw_factor.Gyroscope_Z += (float)MPU6050_Raw.Gyroscope_Z;
	}
	MPU6050_Raw_factor.Accelerometer_X = MPU6050_Raw_factor.Accelerometer_X / 1000;
	MPU6050_Raw_factor.Accelerometer_Y = MPU6050_Raw_factor.Accelerometer_Y / 1000;
	MPU6050_Raw_factor.Accelerometer_Z = MPU6050_Raw_factor.Accelerometer_Z / 1000;
	MPU6050_Raw_factor.Gyroscope_X = MPU6050_Raw_factor.Gyroscope_X / 1000;
	MPU6050_Raw_factor.Gyroscope_Y = MPU6050_Raw_factor.Gyroscope_Y / 1000;
	MPU6050_Raw_factor.Gyroscope_Z = MPU6050_Raw_factor.Gyroscope_Z / 1000;

	pid_Init(K_P, K_I, K_D);

	for(int i = 0; i < 1000000; i++);
    USARTSendDMA("INIT_SUCCESS\r\n");
    for(int i = 0; i < 1000000; i++);
    state = 1;
    cycle();

	while(1)
	{
		GPIOB->ODR ^= GPIO_Pin_12;
		for(int i = 0; i < 1000000; i++);
	}
}

void cycle() {
	while (1)
	{
		if (RX_FLAG_END_LINE == 1) {
						char* search;
						int outS;
		    		// Reset END_LINE Flag
		    		RX_FLAG_END_LINE = 0;
						// CMD
						sprintf(buffer_str, "\r\n");
						if (UCMD.state == 0xff) {
							switch (UCMD.fcn) {
								case 1: GPIOB->ODR ^= GPIO_Pin_12; break;
								case 5: pid_setParams(K_P, K_I, K_D); break;
								case 6: sprintf(buffer_str, "Kp %f Ki %f Kd %f\r\n",
								K_P, K_I, K_D); break;
								case 7: K_P = 0; K_I = 0; K_D = 0; break;
								default: break;
							}
						} else if (UCMD.state == 0x0f) {
							switch (UCMD.fcn) {
								case 2: K_P = atof(RX_BUF); break;
								case 3: K_I = atof(RX_BUF); break;
								case 4: K_D = atof(RX_BUF); break;
								default: break;
							}
							strcat(buffer_str, buffer_str_t);
							UCMD.state = 0xff;
						}
						USARTSendDMA(buffer_str);
						usart_cmd(RX_BUF, RX_BUF_SIZE, &UCMD);
		    		clear_RXBuffer();
						// CMD end
		    	}
		if (I2C_FLAG_READ == 1)
		{
			//PID


			I2C_FLAG_READ = 0;
			timer++;
			if ((timer >= TIMER_DEF) & (!UCMD.state))
			{
				GPIOB->ODR ^= GPIO_Pin_12;
				//toEulerAngle();
				sprintf(buffer_str, "AGX %f PWM1 %u PWM2 %u mesv %f P %f\r\n", angle_gx,
				PWM1, PWM2, measurementValue, PID_out);
				// sprintf(buffer_str, "Y %f P %f R %f q0 %f q1 %f q2 %f q3 %f gx %f gy %f gz %f\r\n",
				// yaw, pitch, roll, q0, q1, q2, q3, MPU6050_Data.Gyroscope_X, MPU6050_Data.Gyroscope_Y, MPU6050_Data.Gyroscope_Z );
				USARTSendDMA(buffer_str);
				//GPIOB->ODR ^= GPIO_Pin_12;
				timer = 0;
			}
		}


	}
}


//other func
void USART1_IRQHandler(void) {
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
	{
    		RXc = USART_ReceiveData(USART1);
    		RX_BUF[RXi] = RXc;
    		RXi++;

    		if (RXc != 13) {
    			if (RXi > RX_BUF_SIZE-1) {
    				clear_RXBuffer();
    			}
    		}
    		else {
    			RX_FLAG_END_LINE = 1;
    		}

			//Echo
    		USART_SendData(USART1, RXc);
	}
}

void TIM4_IRQHandler(void) {
        if ((TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET))
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
						if (state == 1)
						{
							double ay, az, ax, gx;
            	I2C_FLAG_READ = 1;
							TM_MPU6050_ReadAll(&MPU6050_Raw);
							MPU6050_Data.Accelerometer_X = ((float)MPU6050_Raw.Accelerometer_X
							- MPU6050_Raw_factor.Accelerometer_X) / 8192;
							MPU6050_Data.Accelerometer_Y  = ((float)MPU6050_Raw.Accelerometer_Y
							- MPU6050_Raw_factor.Accelerometer_Y) / 8192;
							MPU6050_Data.Accelerometer_Z = ((float)MPU6050_Raw.Accelerometer_Z
							- MPU6050_Raw_factor.Accelerometer_Z) / 8192;
							gx = ((float)MPU6050_Raw.Gyroscope_X
							- MPU6050_Raw_factor.Gyroscope_X) / 131; // TIIME_FACTOR;
							MPU6050_Data.Gyroscope_Y = ((float)MPU6050_Raw.Gyroscope_Y
							- MPU6050_Raw_factor.Gyroscope_Y) / 131; // TIIME_FACTOR;
							MPU6050_Data.Gyroscope_Z = ((float)MPU6050_Raw.Gyroscope_Z
							- MPU6050_Raw_factor.Gyroscope_Z) / 131; // TIIME_FACTOR;
							// IMUupdate(MPU6050_Data.Gyroscope_X, MPU6050_Data.Gyroscope_Y,
							// MPU6050_Data.Gyroscope_Z, MPU6050_Data.Accelerometer_X,
							// MPU6050_Data.Accelerometer_Y, MPU6050_Data.Accelerometer_Z);

							// toEulerAngle();
							ay = clamp(MPU6050_Data.Accelerometer_Y, -1.0, 1.0);
							az = clamp(MPU6050_Data.Accelerometer_Z, -1.0, 1.0);
							ax = clamp(MPU6050_Data.Accelerometer_X, -1.0, 1.0);
							angle_ax = 90 - TO_DEG*atan(ay/sqrt(ax * ax + az * az)) / 2;
							angle_gx = angle_gx + gx * 0.01; // 10 - every 10 ms calculate
							angle_gx = angle_gx *(1-FK) + angle_ax * FK;
							last_ay = ay;
							//angle_gx = MPU6050_Data.Gyroscope_Z;
							if (angle_gx > 0) {
								measurementValue = angle_gx;
								PID_out = pid_Controller(0, measurementValue);
								PWM2 = 0;
								PWM1 = PID_out;
							} else {
								measurementValue = angle_gx * (-1);
								PID_out = pid_Controller(0, measurementValue);
								PWM1 = 0;
								PWM2 = PID_out;
							}
							TIM3->CCR1 = PWM1;
							TIM3->CCR2 = PWM2;
            }
        }
}

void USARTSendDMA(char *pucBuffer) {
	strcpy(buffer, pucBuffer);

	/* Restart DMA Channel*/
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA1_Channel4->CNDTR = strlen(pucBuffer);
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

void DMA1_Channel4_IRQHandler(void) {
	DMA_ClearITPendingBit(DMA1_IT_TC4);
	DMA_Cmd(DMA1_Channel4, DISABLE);
}
void clear_RXBuffer(void) {
	for (RXi=0; RXi<RX_BUF_SIZE; RXi++)
		RX_BUF[RXi] = '\0';
	RXi = 0;
}

void toEulerAngle(qw, qx, qy, qz)
{
	yaw = pow(tan((2 * ((q1 * q2) + (q0 * q3)))/(q0*q0 - q3*q3 - q2*q2 - q1*q1)), -1) * TO_DEG;
	pitch = pow(sin(-2*(q1*q3 - q2*q0)), -1) * TO_DEG;
	roll = pow(tan((2*(q2*q3+q1*q0))/(q0*q0 + q3*q3 + q2*q2 +q1*q1)), -1) * TO_DEG;
}

float clamp(float v, float minv, float maxv){
    if( v>maxv )
        return maxv;
    else if( v<minv )
        return minv;
    return v;
}
