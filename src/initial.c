#include "../inc/initial.h"

volatile char buffer[80] = {'\0'};

char init_() {
	rcc_init();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 |
			RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |
				RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
				RCC_APB2Periph_USART1 , ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	uart_init();
	//i2c_init();
	tim4_init();
	gpio_init();
	tim3_init();
	return '/0';
}

void gpio_init() {
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void rcc_init() {
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

void uart_init() {
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
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		/* Enable the USARTx Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
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

		/* Configure USART2 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* Configure USART2 Rx (PA.10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
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

		USART_InitTypeDef USART_InitStructure2;
		/* Configure the USART2 */
		USART_InitStructure2.USART_BaudRate = 115200;
		USART_InitStructure2.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure2.USART_StopBits = USART_StopBits_1;
		USART_InitStructure2.USART_Parity = USART_Parity_No;
		USART_InitStructure2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure2.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART2, &USART_InitStructure2);

		/* Enable USART1 */
		USART_Cmd(USART1, ENABLE);
		/* Enable USART1 */
		USART_Cmd(USART2, ENABLE);

		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		//DMA_Cmd(DMA1_Channel4, ENABLE);

		DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
		NVIC_EnableIRQ(DMA1_Channel4_IRQn);


		/* Enable the USART1 Receive interrupt: this interrupt is generated when the
		USART1 receive data register is not empty */
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		/* Enable the USART2 Receive interrupt: this interrupt is generated when the
		USART1 receive data register is not empty */
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void i2c_init() {
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

void tim4_init() {
	 TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	 TIM_TimeBaseStructInit(&TIMER_InitStructure);
	 TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIMER_InitStructure.TIM_Prescaler = 7200;
	 TIMER_InitStructure.TIM_Period = 10000 / TIIME_FACTOR;
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

void tim3_init() {
	 TIM_TimeBaseInitTypeDef TIMER_InitStructure;

	 TIM_TimeBaseStructInit(&TIMER_InitStructure);
	 TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIMER_InitStructure.TIM_Prescaler = 0;
	 //TIMER_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	 TIMER_InitStructure.TIM_Period = 255;
	 TIMER_InitStructure.TIM_RepetitionCounter = 0;
	 TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
	 //PWM CHANEL 1
	 TIM_OCStructInit(&timerPWM);
	 timerPWM.TIM_Pulse = 0; //0x1000;
	 timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
	 timerPWM.TIM_OutputState = TIM_OutputState_Enable;
	 timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
	 // TIM_OC1Init(TIM3, &timerPWM);
	 //PWM CHANEL 2
	 TIM_OC2Init(TIM3, &timerPWM);
	 //PWM CHANEL 3
	 TIM_OC3Init(TIM3, &timerPWM);
	 // //PWM CHANEL 4
	 // TIM_OC4Init(TIM3, &timerPWM);

	 TIM_Cmd(TIM3, ENABLE);
	 PWM1 = timerPWM.TIM_Pulse;
	 PWM2 = timerPWM.TIM_Pulse;
	 PWM3 = timerPWM.TIM_Pulse;
	 PWM4 = timerPWM.TIM_Pulse;

}
