/*
 * mainCPP.cpp
 *
 *  Created on: May 14, 2023
 *      Author: HuczAs
 */
#include "Hexapod_PWM_Driver.hpp"
#include "mainCPP.hpp"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_i2c.h"
#include "main.h"
#include "stdio.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif


I2C_HandleTypeDef i2c_handle;

UART_HandleTypeDef com_protocol2;

static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_GPIO_Init(void);
void mainCPP_LOOP( Hexapod_PWM_Driver controller );
void mainCPP_INIT();
/*________________________DECLARATIONS____________________________________*/
/*____________________KINEMATICS FUNCTIONS________________________________*/
void mov_Forward(Hexapod_PWM_Driver controller){
	for(int l = 0; l<5;l++){
		for(float x = 60.0; x<120.0; x+=0.15){
			controller.MoveHorizonal(8, x);
			controller.MoveVertical(9 , 140);
		}

		for(float x = 120.0; x>60.0; x-=0.15){
			controller.MoveHorizonal(8, x);
			controller.MoveVertical(9 , 110);
		}
	}
}

void mov_Backward(Hexapod_PWM_Driver controller){
	for(int l = 0; l<5;l++){
		for(float x = 60.0; x<120.0; x+=0.15){
			controller.MoveHorizonal(8, x);
			controller.MoveVertical(9 , 110);
		}

		for(float x = 120.0; x>60.0; x-=0.15){
			controller.MoveHorizonal(8, x);
			controller.MoveVertical(9 , 140);
		}
	}
}
/*________________________PUMP UP ON Three LEG____________________________*/
void pump_Up(Hexapod_PWM_Driver controller){
	int legs[6] = {1, 5, 9, 3, 7, 11};

	int y = 0;
		for(float x = 60.0; x< 140.0; x+=0.15){
			controller.MoveVertical(legs[(1+y)-1], x);
			controller.MoveVertical(legs[(2+y)-1], x);
			controller.MoveVertical(legs[(3+y)-1], x);
		}

		y = 3;

		for(float x = 60.0; x< 140.0; x+=0.15){
			controller.MoveVertical(legs[(1+y)-1], x);
			controller.MoveVertical(legs[(2+y)-1], x);
			controller.MoveVertical(legs[(3+y)-1], x);
		}

		for(float x = 140.0; x> 60.0; x-=0.15){
			controller.MoveVertical(legs[(1+y)-1], x);
			controller.MoveVertical(legs[(2+y)-1], x);
			controller.MoveVertical(legs[(3+y)-1], x);
		}

		y = 0;

		for(float x = 140.0; x> 60.0; x-=0.15){
			controller.MoveVertical(legs[(1+y)-1], x);
			controller.MoveVertical(legs[(2+y)-1], x);
			controller.MoveVertical(legs[(3+y)-1], x);
		}
//------------------------------------------
		y = 3;
		for(float x = 60.0; x< 140.0; x+=0.15){
			controller.MoveVertical(legs[(1+y)-1], x);
			controller.MoveVertical(legs[(2+y)-1], x);
			controller.MoveVertical(legs[(3+y)-1], x);
		}

		y = 0;

		for(float x = 60.0; x< 140.0; x+=0.15){
			controller.MoveVertical(legs[(1+y)-1], x);
			controller.MoveVertical(legs[(2+y)-1], x);
			controller.MoveVertical(legs[(3+y)-1], x);
		}

		for(float x = 140.0; x> 60.0; x-=0.15){
			controller.MoveVertical(legs[(1+y)-1], x);
			controller.MoveVertical(legs[(2+y)-1], x);
			controller.MoveVertical(legs[(3+y)-1], x);
		}

		y = 3;

		for(float x = 140.0; x> 60.0; x-=0.15){
			controller.MoveVertical(legs[(1+y)-1], x);
			controller.MoveVertical(legs[(2+y)-1], x);
			controller.MoveVertical(legs[(3+y)-1], x);
		}


}
/*________________________________________________________________________*/
void mainCPP_INIT(){
	MX_I2C1_Init();//			It Initializing all configs for I2C Transmition
	MX_GPIO_Init();//			init GPIO Ports & declarations
	MX_USART2_UART_Init();// 	Init configs for HUART Comunications
	Hexapod_PWM_Driver controller = Hexapod_PWM_Driver(&i2c_handle, 0x80);



	//controller.MoveVertical(0, 90); //						45	- 140	VERTICAL (95)
	//controller.MoveHorizonal(1, 90); //							120 - 60 	HORIZONTAL (60)
	controller.MoveServo(0, 90);
	controller.MoveServo(1, 90);
	controller.MoveServo(2, 90);
	controller.MoveServo(3, 90);
	controller.MoveServo(4, 90);
	controller.MoveServo(5, 90);
	controller.MoveServo(6, 90);
	controller.MoveServo(7, 90);
	controller.MoveServo(8, 90);
	controller.MoveServo(9, 90);
	controller.MoveServo(10, 90);
	controller.MoveServo(11, 90);
	HAL_Delay(1000);

	mainCPP_LOOP(controller);	//		main loop function written below
}



void mainCPP_LOOP( Hexapod_PWM_Driver controller ){


	while(1){		// software main loop

		//mov_Forward(controller);
		//HAL_Delay(1500);
		//mov_Backward(controller);
		//HAL_Delay(1500);

		pump_Up(controller);
		HAL_Delay(800);

	}
}



#ifdef __cplusplus
}
#endif


static void MX_I2C1_Init(void){
	i2c_handle.Instance = I2C1;
	i2c_handle.Init.Timing = 0x00707CBB;
	i2c_handle.Init.OwnAddress1 = 0;
	i2c_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c_handle.Init.OwnAddress2 = 0;
	i2c_handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	i2c_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&i2c_handle) != HAL_OK)
  {
    Error_Handler();
  }


  if (HAL_I2CEx_ConfigAnalogFilter(&i2c_handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }


  if (HAL_I2CEx_ConfigDigitalFilter(&i2c_handle, 0) != HAL_OK)
  {
    Error_Handler();
  }


}

static void MX_USART2_UART_Init(void){
	com_protocol2.Instance = USART2;
	com_protocol2.Init.BaudRate = 115200;
	com_protocol2.Init.WordLength = UART_WORDLENGTH_8B;
	com_protocol2.Init.StopBits = UART_STOPBITS_1;
	com_protocol2.Init.Parity = UART_PARITY_NONE;
	com_protocol2.Init.Mode = UART_MODE_TX_RX;
	com_protocol2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	com_protocol2.Init.OverSampling = UART_OVERSAMPLING_16;
	com_protocol2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	com_protocol2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&com_protocol2) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}



