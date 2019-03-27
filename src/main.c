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
		    		// Reset END_LINE Flag
		    		RX_FLAG_END_LINE = 0;

		    		/* !!! This lines is not have effect. Just a last command USARTSendDMA(":\r\n"); !!!! */
		    		//USARTSendDMA("\r\nI has received a line:\r\n"); // no effect
		    		//USARTSendDMA(RX_BUF); // no effect
		    		//USARTSendDMA(":\r\n"); // This command does not wait for the finish of the sending of buffer. It just write to buffer new information and restart sending via DMA.

		    		if (strncmp(strupr(RX_BUF), "W", 1) == 0) {
		    			PWM1 = PWM1 + 10;
		    			PWM2 = 0;
		    			TIM3->CCR1 = PWM1;
		    			TIM3->CCR2 = PWM2;
		    		}

		    		if (strncmp(strupr(RX_BUF), "S", 1) == 0) {
		    			PWM2 = PWM2 + 10;
		    			PWM1 = 0;
		    			TIM3->CCR1 = PWM1;
		    			TIM3->CCR2 = PWM2;
		    		}
						if (strncmp(strupr(RX_BUF), "Q", 1) == 0) {
		    			PWM1 = PWM1 - 10;
		    			PWM2 = 0;
		    			TIM3->CCR1 = PWM1;
		    			TIM3->CCR2 = PWM2;
		    		}

		    		if (strncmp(strupr(RX_BUF), "A", 1) == 0) {
		    			PWM2 = PWM2 - 10;
		    			PWM1 = 0;
		    			TIM3->CCR1 = PWM1;
		    			TIM3->CCR2 = PWM2;
		    		}
		    		if (strncmp(strupr(RX_BUF), "C", 1) == 0) {
		    			PWM1 = 0;
		    			PWM2 = 0;
		    			TIM3->CCR1 = PWM1;
		    			TIM3->CCR2 = PWM2;
		    		}

		    		if (strncmp(strupr(RX_BUF), "DATA\r", 5) == 0)
		    		{
		    			USARTSendDMA("THIS IS A COMMAND \"DATA\"!!!\r\n");
		    			 sprintf(str, "Accelerometer X:%f Y:%f Z:%f Gyroscope X:%f Y:%f Z:%f\r\n",
		    			                    MPU6050_Data.Accelerometer_X,
		    			                    MPU6050_Data.Accelerometer_Y,
		    			                    MPU6050_Data.Accelerometer_Z,
		    			                    MPU6050_Data.Gyroscope_X,
		    			                    MPU6050_Data.Gyroscope_Y,
		    			                    MPU6050_Data.Gyroscope_Z
		    			                );
		    			 USARTSendDMA(str);
		    		}

		    		clear_RXBuffer();
		    	}
		if (I2C_FLAG_READ == 1)
		{
			float ay,gx;
			TM_MPU6050_ReadAll(&MPU6050_Raw);
			MPU6050_Data.Accelerometer_X = ((float)MPU6050_Raw.Accelerometer_X
			- MPU6050_Raw_factor.Accelerometer_X) / 16384;
			ay = ((float)MPU6050_Raw.Accelerometer_Y
			- MPU6050_Raw_factor.Accelerometer_Y) / 16384;
			MPU6050_Data.Accelerometer_Z = ((float)MPU6050_Raw.Accelerometer_Z
			- MPU6050_Raw_factor.Accelerometer_Z) / 16384;
			gx = ((float)MPU6050_Raw.Gyroscope_X
			- MPU6050_Raw_factor.Gyroscope_X) / 131; // TIIME_FACTOR;
			MPU6050_Data.Gyroscope_Y = ((float)MPU6050_Raw.Gyroscope_Y
			- MPU6050_Raw_factor.Gyroscope_Y) / 131; // TIIME_FACTOR;
			MPU6050_Data.Gyroscope_Z = ((float)MPU6050_Raw.Gyroscope_Z
			- MPU6050_Raw_factor.Gyroscope_Z) / 131; // TIIME_FACTOR;
			// MadgwickAHRSupdateIMU(MPU6050_Data.Gyroscope_X, MPU6050_Data.Gyroscope_Y,
			// MPU6050_Data.Gyroscope_Z, MPU6050_Data.Accelerometer_X,
			// MPU6050_Data.Accelerometer_Y, MPU6050_Data.Accelerometer_Z);
			// toEulerAngle();
			ay = clamp(ay, -1.0, 1.0);
			angle_ax = 90 - TO_DEG*acos(ay);
			angle_gx = angle_gx + gx*10/1000.0; // 10 - every 10 ms calculate
			angle_gx = angle_gx*(1-FK) + angle_ax * FK;
			//PID
			if (angle_gx > 0) {
				measurementValue = 300 - angle_gx * 10;
				PID_out = pid_Controller(300, measurementValue);
				PWM2 = 0;
				PWM1 = PID_out / 10;
			} else {
				measurementValue = 300 - angle_gx * (-10);
				PID_out = pid_Controller(300, measurementValue);
				PWM1 = 0;
				PWM2 = PID_out / 10;
			}
			TIM3->CCR1 = PWM1;
			TIM3->CCR2 = PWM2;

			I2C_FLAG_READ = 0;
			timer++;
			if (timer >= TIMER_DEF)
			{
				GPIOB->ODR ^= GPIO_Pin_12;
				//toEulerAngle();
				sprintf(str, "AGX %f PWM1 %u PWM2 %u mesv %i\r\n", angle_gx, PWM1, PWM2, measurementValue);
				USARTSendDMA(str);
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
//            if (state)
//            {
            	I2C_FLAG_READ = 1;
//            }
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

void toEulerAngle() {
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q0 * q1 + q2 * q3);
	double cosr_cosp = +1.0 - 2.0 * (q1 * q1 + q2 * q2);
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q0 * q2 - q3 * q1);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q0 * q3 + q1 * q2);
	double cosy_cosp = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
	yaw = atan2(siny_cosp, cosy_cosp);
}

float clamp(float v, float minv, float maxv){
    if( v>maxv )
        return maxv;
    else if( v<minv )
        return minv;
    return v;
}

void pwmREG(uint8_t step, uint8_t direction) {

	if (direction > 0) {
		if (PWM1 < 0xffff)
			PWM1 = PWM1 + step;
		if (PWM2 > 0)
			PWM2 = PWM2 - step;
	}
	else {
		if (PWM2 < 0xffff)
			PWM2 = PWM2 + step;
		if (PWM1 > 0)
			PWM1 = PWM1 - step;
	}
}
