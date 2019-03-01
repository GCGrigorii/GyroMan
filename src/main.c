/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_tim.h"
#include "main.h"
#include "tm_stm32f4_mpu6050.h"
#include <string.h>
#include <stdio.h>
///
#define RX_BUF_SIZE 80
volatile char RX_FLAG_END_LINE = 0;
volatile char RXi;
volatile char RXc;
volatile char RX_BUF[RX_BUF_SIZE] = {'\0'};
volatile char buffer[80] = {'\0'};
volatile char str[120];
volatile char I2C_FLAG_READ = 0;
TM_MPU6050_t MPU6050_Raw;
TM_MPU6050_t_data MPU6050_Raw_factor;
TM_MPU6050_t_data MPU6050_Data;
volatile uint8_t timer = 0;

void gpio_init(void);
char init_();
void rcc_init();
void i2c_init();
void tim4_init();
void uart_init();
void USART1_IRQHandler(void);
void TIM4_IRQHandler(void);
void USARTSendDMA(char*);
void DMA1_Channel4_IRQHandler(void);
void clear_RXBuffer(void);
void cycle();
void I2C_StartTxRx (I2C_TypeDef*, uint8_t, uint8_t);
void I2C_Tx (I2C_TypeDef*, uint8_t);
uint8_t I2C_Rx (I2C_TypeDef*);

//mpu addr 0x68 if ad0 low
// don't forget I2C_Generate STOP(I2Cx, ENABLE).
int main(void)
{

	uint8_t sensor1 = 0;
	init_();

	for(int i = 0; i < 1000000; i++);

	sensor1 = TM_MPU6050_Init(&MPU6050_Raw, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_2G, TM_MPU6050_Gyroscope_250s);
	if (sensor1 == TM_MPU6050_Result_Ok) {
	        /* Display message to user */
		USARTSendDMA("MPU6050 sensor 0 is ready to use!\r\n");

	        /* Sensor 1 OK */
	        sensor1 = 1;
	    }
	else
	{
		strcmp(buffer, itoa(sensor1));
		strcmp(buffer, " - MPU6050 sensor start error\r\n");
		USARTSendDMA(buffer);
		while(1);
	}
	for(int i = 0; i < 1000000; i++);

	for (int i = 0; i < 10; i++ )
	{
		TM_MPU6050_ReadAll(&MPU6050_Raw);
		MPU6050_Raw_factor.Accelerometer_X += (float)MPU6050_Raw.Accelerometer_X;
		MPU6050_Raw_factor.Accelerometer_Y += (float)MPU6050_Raw.Accelerometer_Y;
		MPU6050_Raw_factor.Accelerometer_Z += (float)MPU6050_Raw.Accelerometer_Z;
		MPU6050_Raw_factor.Gyroscope_X += (float)MPU6050_Raw.Gyroscope_X;
		MPU6050_Raw_factor.Gyroscope_Y += (float)MPU6050_Raw.Gyroscope_Y;
		MPU6050_Raw_factor.Gyroscope_Z += (float)MPU6050_Raw.Gyroscope_Z;
	}
	MPU6050_Raw_factor.Accelerometer_X = MPU6050_Raw_factor.Accelerometer_X / 10;
	MPU6050_Raw_factor.Accelerometer_Y = MPU6050_Raw_factor.Accelerometer_Y / 10;
	MPU6050_Raw_factor.Accelerometer_Z = MPU6050_Raw_factor.Accelerometer_Z / 10;
	MPU6050_Raw_factor.Gyroscope_X = MPU6050_Raw_factor.Gyroscope_X / 10;
	MPU6050_Raw_factor.Gyroscope_Y = MPU6050_Raw_factor.Gyroscope_Y / 10;
	MPU6050_Raw_factor.Gyroscope_Z = MPU6050_Raw_factor.Gyroscope_Z / 10;

	for(int i = 0; i < 1000000; i++);
    USARTSendDMA("Hello.\r\nUSART1 is ready.\r\n");
    for(int i = 0; i < 1000000; i++);
    cycle();
	while(1)
	{
		GPIOB->ODR ^= GPIO_Pin_12;
		for(int i = 0; i < 1000000; i++);
	}
}
void cycle()
{
	while (1)
	{
		if (RX_FLAG_END_LINE == 1) {
		    		// Reset END_LINE Flag
		    		RX_FLAG_END_LINE = 0;

		    		/* !!! This lines is not have effect. Just a last command USARTSendDMA(":\r\n"); !!!! */
		    		USARTSendDMA("\r\nI has received a line:\r\n"); // no effect
		    		USARTSendDMA(RX_BUF); // no effect
		    		USARTSendDMA(":\r\n"); // This command does not wait for the finish of the sending of buffer. It just write to buffer new information and restart sending via DMA.

		    		if (strncmp(strupr(RX_BUF), "ON\r", 3) == 0) {
		    			USARTSendDMA("THIS IS A COMMAND \"ON\"!!!\r\n");
		    			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		    		}

		    		if (strncmp(strupr(RX_BUF), "OFF\r", 4) == 0) {
		    			USARTSendDMA("THIS IS A COMMAND \"OFF\"!!!\r\n");
		    			GPIO_SetBits(GPIOB, GPIO_Pin_12);
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
			 TM_MPU6050_ReadAll(&MPU6050_Raw);
			 MPU6050_Data.Accelerometer_X = ((float)MPU6050_Raw.Accelerometer_X - MPU6050_Raw_factor.Accelerometer_X) / 16384;
			 MPU6050_Data.Accelerometer_Y = ((float)MPU6050_Raw.Accelerometer_Y - MPU6050_Raw_factor.Accelerometer_Y) / 16384;
			 MPU6050_Data.Accelerometer_Z = ((float)MPU6050_Raw.Accelerometer_Z - MPU6050_Raw_factor.Accelerometer_Z) / 16384;
			 MPU6050_Data.Gyroscope_X += ((float)MPU6050_Raw.Gyroscope_X - MPU6050_Raw_factor.Gyroscope_X) / 131 / 10 ;
			 MPU6050_Data.Gyroscope_Y += ((float)MPU6050_Raw.Gyroscope_Y - MPU6050_Raw_factor.Gyroscope_Y) / 131 / 10;
			 MPU6050_Data.Gyroscope_Z += ((float)MPU6050_Raw.Gyroscope_Z - MPU6050_Raw_factor.Gyroscope_Z) / 131 / 10;
			 I2C_FLAG_READ = 0;
			 GPIOB->ODR ^= GPIO_Pin_12;
			 sprintf(str, "A\G X:%f Y:%f \r\n",
					    			                    MPU6050_Data.Accelerometer_X,
					    			                    MPU6050_Data.Accelerometer_Y,
					    			                    MPU6050_Data.Accelerometer_Z,
					    			                    MPU6050_Data.Gyroscope_X,
					    			                    MPU6050_Data.Gyroscope_Y,
					    			                    MPU6050_Data.Gyroscope_Z
					    			                );
					    			 USARTSendDMA(str);
		}

	}
}


char init_()
{
	rcc_init();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |
				RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
				RCC_APB2Periph_USART1 , ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	uart_init();
	//i2c_init();
	tim4_init();
	gpio_init();

	return '/0';
}

/*GPIO_Init
 * Init a pin's / function
 * C13 - out_od
 * */
void gpio_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void rcc_init()
{
	ErrorStatus HSEStartUpStatus;
	RCC_DeInit();
	RCC_HSEConfig( RCC_HSE_ON);
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if (HSEStartUpStatus == SUCCESS)
	    {

	        FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

	       FLASH_SetLatency( FLASH_Latency_2);

	        RCC_HCLKConfig( RCC_SYSCLK_Div1);

	        RCC_PCLK2Config( RCC_HCLK_Div1);

	        RCC_PCLK1Config( RCC_HCLK_Div2);

	        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

	        RCC_PLLCmd( ENABLE);

	        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	        {
	        }

	        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

	        while (RCC_GetSYSCLKSource() != 0x08)
	        {
	        }
	    }
	    else
	    {
	        while (1)
	        {
	        }
	    }
}


 void uart_init()
{
	/* DMA */
		DMA_InitTypeDef DMA_InitStruct;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
		DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&buffer[0];
		DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStruct.DMA_BufferSize = sizeof(buffer);
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
		DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(DMA1_Channel4, &DMA_InitStruct);

		/* NVIC Configuration */
		NVIC_InitTypeDef NVIC_InitStructure;
		/* Enable the USARTx Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Configure the GPIOs */
		GPIO_InitTypeDef GPIO_InitStructure;

		/* Configure USART1 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Configure USART1 Rx (PA.10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Configure the USART1 */
		USART_InitTypeDef USART_InitStructure;

		/* USART1 configuration ------------------------------------------------------*/
		/* USART1 configured as follow:
			- BaudRate = 115200 baud
			- Word Length = 8 Bits
			- One Stop Bit
			- No parity
			- Hardware flow control disabled (RTS and CTS signals)
			- Receive and transmit enabled
			- USART Clock disabled
			- USART CPOL: Clock is active low
			- USART CPHA: Data is captured on the middle
			- USART LastBit: The clock pulse of the last data bit is not output to
				the SCLK pin
		 */
		USART_InitStructure.USART_BaudRate = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		USART_Init(USART1, &USART_InitStructure);

		/* Enable USART1 */
		USART_Cmd(USART1, ENABLE);

		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		//DMA_Cmd(DMA1_Channel4, ENABLE);

		DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
		NVIC_EnableIRQ(DMA1_Channel4_IRQn);


		/* Enable the USART1 Receive interrupt: this interrupt is generated when the
		USART1 receive data register is not empty */
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}


void i2c_init()
{
	 I2C_InitTypeDef  I2C_InitStructure;
	    GPIO_InitTypeDef  GPIO_InitStructure;

	    /* Configure I2C_EE pins: SCL and SDA */
	    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

	    /* I2C configuration */
	    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	    I2C_InitStructure.I2C_OwnAddress1 = 0x38;
	    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	    I2C_InitStructure.I2C_ClockSpeed = 100000;

	    /* I2C Peripheral Enable */
	    I2C_Cmd(I2C1, ENABLE);
	    /* Apply I2C configuration after enabling it */
	    I2C_Init(I2C1, &I2C_InitStructure);
}


void tim4_init()
{
	 TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	 TIM_TimeBaseStructInit(&TIMER_InitStructure);
	 TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIMER_InitStructure.TIM_Prescaler = 7200;
	 TIMER_InitStructure.TIM_Period = 1000;
	 TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
	 TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	 TIM_Cmd(TIM4, ENABLE);

	 /* NVIC Configuration */
     /* Enable the TIM4_IRQn Interrupt */
	 NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler(void)
{
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

void TIM4_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
            I2C_FLAG_READ = 1;
        }
}

void USARTSendDMA(char *pucBuffer)
{
	strcpy(buffer, pucBuffer);

	/* Restart DMA Channel*/
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA1_Channel4->CNDTR = strlen(pucBuffer);
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

void DMA1_Channel4_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC4);
	DMA_Cmd(DMA1_Channel4, DISABLE);
}

void clear_RXBuffer(void) {
	for (RXi=0; RXi<RX_BUF_SIZE; RXi++)
		RX_BUF[RXi] = '\0';
	RXi = 0;
}

void I2C_StartTxRx (I2C_TypeDef* I2Cx, uint8_t transmissionDirection, uint8_t slaveAddress)
{

	(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2Cx, ENABLE);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, slaveAddress, transmissionDirection);

	if(transmissionDirection== I2C_Direction_Transmitter)
	{while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));}

	if(transmissionDirection== I2C_Direction_Receiver)
	{while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));}
}

void I2C_Tx (I2C_TypeDef* I2Cx, uint8_t data)
{

	I2C_SendData(I2Cx, data);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t I2C_Rx (I2C_TypeDef* I2Cx)
{
	uint8_t data;
		while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	data = I2C_ReceiveData(I2Cx);
	return data;
}


