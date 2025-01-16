///*
// * mainCPP.cpp
// *
// *  Created on: May 14, 2023
// *      Author: HuczAs
// */
//#include "Hexapod_PWM_Driver.hpp"
//#include "mainCPP.hpp"
//#include "stm32l0xx_hal.h"
//#include "stm32l0xx_hal_i2c.h"
//#include "main.h"
//#include "stdio.h"
//#include "stdint.h"
//#include "math.h"
//
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//
//
//I2C_HandleTypeDef i2c_handle;
//
//UART_HandleTypeDef com_protocol2;
//
//void mainCPP_LOOP( Hexapod_PWM_Driver controller, Hexapod_Leg HexapodLeg);
//
//void mainCPP_INIT(){
//	MX_I2C1_Init();//			It Initializing all configs for I2C Transmition
//	MX_GPIO_Init();//			init GPIO Ports & declarations
//	MX_USART2_UART_Init();// 	Init configs for HUART Comunications
//	Hexapod_PWM_Driver controller = Hexapod_PWM_Driver(&i2c_handle, 0x80); //pca9685 address
//	vector<Hexapod_Leg> HexapodLeg;
//	for(int i = 0; i<10; i+=2 ){
//		HexapodLeg.emplace_back(i);
//	}
//
//	// User Program INIT
//
//	controller.BasePosition();
//	HAL_Delay(2000);
//
//	mainCPP_LOOP(controller, HexapodLeg);	//		main loop function written below
//}
//
//
//
//void mainCPP_LOOP( Hexapod_PWM_Driver controller, Hexapod_Leg HexapodLeg ){
//	float speed = 0.05; 		// moving speed
//	while(1){					// software main loop
/*_________Cartesian moving range___________*/

//			X-Axis:		< -29 ; 29 >
//			Z-Axis:		< -63 ; -29>
//			Y-Axis: 	The y-axis is based on the
//						remaining coordinates, and due to the
//						construction fact, it is not required
//						for correct movement.
/*__________________________________________*/
// ruch na przemian:	0, 2, 4, 6, 8, 10
//			jedna seria:
//				0, 4, 8
//			druga seria:
//				2, 6, 10
/*__________________________________________*/
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
//
//
//
//
//		/*
//		for(int i = 0 ; i<5 ; i++){
//			controller.turnover((-1), speed);
//		}
//
//		for(int i = 0 ; i<5 ; i++){
//			controller.turnover((1), speed);
//		}
//
//
//
//
//		for(int leg = 0; leg<=2; leg+=2){
//
//			for (float z = -29.0f, x = 15.0f; z > -63.0f && x > -15.0f; z -= speed, x -= speed) {
//				controller.cartesianPosition(0+leg, x, z);
//				controller.cartesianPosition(4+leg, x, z);
//				controller.cartesianPosition(8+leg, x, z);
//
//			}
//
//			for(float z = -63.0f, x = -15.0f; z < -29.0f && x<15.0f ; z += speed, x += speed) {
//				controller.cartesianPosition(0+leg, x, z);
//				controller.cartesianPosition(4+leg, x, z);
//				controller.cartesianPosition(8+leg, x, z);
//			}
//
//			controller.cartesianPosition(0+leg, 0, -29);
//			controller.cartesianPosition(4+leg, 0, -29);
//			controller.cartesianPosition(8+leg, 0, -29);
//
//		}
//*/
//
//	}//while bracket
//}//fun bracket
//
//
//
//#ifdef __cplusplus
//}
//#endif
//
//
//static void MX_I2C1_Init(void){
//	i2c_handle.Instance = I2C1;
//	i2c_handle.Init.Timing = 0x00707CBB;
//	i2c_handle.Init.OwnAddress1 = 0;
//	i2c_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//	i2c_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//	i2c_handle.Init.OwnAddress2 = 0;
//	i2c_handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
//	i2c_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//	i2c_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&i2c_handle) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//
//  if (HAL_I2CEx_ConfigAnalogFilter(&i2c_handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//
//  if (HAL_I2CEx_ConfigDigitalFilter(&i2c_handle, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//
//}
//
//static void MX_USART2_UART_Init(void){
//	com_protocol2.Instance = USART2;
//	com_protocol2.Init.BaudRate = 115200;
//	com_protocol2.Init.WordLength = UART_WORDLENGTH_8B;
//	com_protocol2.Init.StopBits = UART_STOPBITS_1;
//	com_protocol2.Init.Parity = UART_PARITY_NONE;
//	com_protocol2.Init.Mode = UART_MODE_TX_RX;
//	com_protocol2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//	com_protocol2.Init.OverSampling = UART_OVERSAMPLING_16;
//	com_protocol2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//	com_protocol2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&com_protocol2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//}
//
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
//
//  /*Configure GPIO pins : PB1 PB5 */
//  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PA15 */
//  GPIO_InitStruct.Pin = GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//}
//
//
//
