/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<inttypes.h>
#include "pca9685.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rxbuff[16];
int fast=1000;
int Buff1=20;
int Buff2=-20;
int BuffP=50;
int BuffN=-50;
int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1, ll2, rr2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void PWM (uint16_t CCR1LoadValue)
//{
//	uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
//	uint32_t *pAFRLPortAReg   =   (uint32_t*)0x40020020; // address of alternate function register of port A
//    uint32_t *pPortAModeReg   =   (uint32_t*)0x40020000; // address of port A mode register
//    uint32_t *pAPB1ClkCtrlReg =   (uint32_t*)0x40023840; // address of APB1 clock control register
//	uint32_t *pTimer2PsclrReg =   (uint32_t*)0x40000028; // address of timer 2 pre-scalar register
//	uint32_t *pTimer2ArrReg   =   (uint32_t*)0x4000002C; // address of timer 2 auto reload register
//    uint32_t *pTimer2CntReg   =   (uint32_t*)0x40000024; // address of timer 2 counter register
//    uint32_t *pTimer2CCMR1Reg =   (uint32_t*)0x40000018; // address of timer 2 capture/compare register
//	uint32_t *pTimer2CcerReg  =   (uint32_t*)0x40000020; // address of timer 2 capture/compare enable register
//	uint32_t *pTimer2CCR1Reg  =   (uint32_t*)0x40000034; // address of timer 2 capture/compare register
//	uint32_t *pTimer2CR1Reg   =   (uint32_t*)0x40000000; // address of timer 2 control register 1
//
//
//			*pAHB1ClkCtrlReg |= 0x1;        // port A clock enable
//		    *pAFRLPortAReg   |= 0x00100000; // alternate function of Timer 2 enabled
//		    *pPortAModeReg   &= 0xFFFFF3FF;
//		    *pPortAModeReg   |= 0x00000800; // port A Pin 5 configured for 'Alternate function'
//
//
//		    *pAPB1ClkCtrlReg |= 0x1;           // timer 2 clock enable
//		    *pTimer2PsclrReg = 180 - 1;         // pre-scalar value
//		    *pTimer2ArrReg   = 100 - 1 ;     // calculated auto reload value (60Hz PWM frequency)
//		  //  *pTimer2ArrReg   = 320000 - 1 ;     // calculated auto reload value (60Hz PWM frequency)
//		    *pTimer2CntReg   = 0;              // counter initialized to 0
//		    *pTimer2CCMR1Reg = 0x0060;         // output Compare mode 1 enabled in Timer 2 Channel 1
//		    *pTimer2CcerReg  = 1;              // configured as active low in Output compare mode
//		    *pTimer2CCR1Reg  = CCR1LoadValue;  // duty cycle of PWM signal
//		    *pTimer2CR1Reg   = 1;              // counter enable
//}

void PWM (uint16_t CCR1LoadValue)
{
	uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
	uint32_t *pAFRLPortAReg   =   (uint32_t*)0x40020020; // address of alternate function register of port A
    uint32_t *pPortAModeReg   =   (uint32_t*)0x40020000; // address of port A mode register
    uint32_t *pAPB1ClkCtrlReg =   (uint32_t*)0x40023840; // address of APB1 clock control register
	uint32_t *pTimer2PsclrReg =   (uint32_t*)0x40000C28; // address of timer 5 pre-scalar register
	uint32_t *pTimer2ArrReg   =   (uint32_t*)0x40000C2C; // address of timer 5 auto reload register
    uint32_t *pTimer2CntReg   =   (uint32_t*)0x40000C24; // address of timer 5 counter register
    uint32_t *pTimer2CCMR1Reg =   (uint32_t*)0x40000C18; // address of timer 5 capture/compare register
	uint32_t *pTimer2CcerReg  =   (uint32_t*)0x40000C20; // address of timer 5 capture/compare enable register
	uint32_t *pTimer2CCR1Reg  =   (uint32_t*)0x40000C34; // address of timer 5 capture/compare register
	uint32_t *pTimer2CR1Reg   =   (uint32_t*)0x40000C00; // address of timer 5 control register 1


			*pAHB1ClkCtrlReg |= 0x1;        // port A clock enable
		    *pAFRLPortAReg   |= 0x00000002; // alternate function of Timer 5 enabled
		    *pPortAModeReg   &= 0xFFFFFFFC;
		    *pPortAModeReg   |= 0x00000002; // port A Pin 0 configured for 'Alternate function'


		    *pAPB1ClkCtrlReg |= 0x8;           // timer 5 clock enable
		    *pTimer2PsclrReg = 180 - 1;         // pre-scalar value
		    *pTimer2ArrReg   = 100 - 1 ;     // calculated auto reload value (60Hz PWM frequency)
		    *pTimer2CntReg   = 0;              // counter initialized to 0
		    *pTimer2CCMR1Reg = 0x0060;         // output Compare mode 1 enabled in Timer 5 Channel 1
		    *pTimer2CcerReg  = 1;              // configured as active low in Output compare mode
		    *pTimer2CCR1Reg  = CCR1LoadValue;  // duty cycle of PWM signal
		    *pTimer2CR1Reg   = 1;              // counter enable

}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive(&huart4, rxbuff, 64,1000);
  PCA9685_Init(&hi2c2);
  PCA9685_SetServoAngle(0, 0);
  PCA9685_SetServoAngle(1, 0);
  PCA9685_SetServoAngle(2, 0);
  PCA9685_SetServoAngle(15, 0);
  PCA9685_SetServoAngle(8, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
  	  HAL_StatusTypeDef status;
  	 	 	 	  	 	  status=HAL_UART_Receive(&huart4, rxbuff,64,1000);

  	 	 	 	  	 	  if (status == HAL_OK)
  	 	 	 	  	 	    {
  	 	 	 	  	 	        // Convert the received bytes to signed integers
//	  	 	 	 	  	 	        ry = (rxbuff[1] & 0x80) ? (int32_t)rxbuff[1] - 256 : (int32_t)rxbuff[1];
//	  	 	 	 	  	 	        lx = (rxbuff[2] & 0x80) ? (int32_t)rxbuff[2] - 256 : (int32_t)rxbuff[2];
//	  	 	 	 	  	 	        ly = (rxbuff[3] & 0x80) ? (int32_t)rxbuff[3] - 256 : (int32_t)rxbuff[3];
//	  	 	 	 	  	 	        rx = (rxbuff[4] & 0x80) ? (int32_t)rxbuff[4] - 256 : (int32_t)rxbuff[4];
  	 	 	 	  	 		  	lx = (rxbuff[1] & 0x80) ? (int32_t)rxbuff[1] - 256 : (int32_t)rxbuff[1];
  	 	 	 	  	 		  	ly = (rxbuff[2] & 0x80) ? (int32_t)rxbuff[2] - 256 : (int32_t)rxbuff[2];
  	 	 	 	  	 		  	rx = (rxbuff[3] & 0x80) ? (int32_t)rxbuff[3] - 256 : (int32_t)rxbuff[3];
  	 	 	 	  	 		  	ry = (rxbuff[4] & 0x80) ? (int32_t)rxbuff[4] - 256 : (int32_t)rxbuff[4];
//  	 	 	 	  	 	        cro= (rxbuff[5] & 0x80) ? (int32_t)rxbuff[5] - 256 : (int32_t)rxbuff[5];
//  	 	 	 	  	 	        squ= (rxbuff[6] & 0x80) ? (int32_t)rxbuff[6] - 256 : (int32_t)rxbuff[6];
//  	 	 	 	  	 	        tri= (rxbuff[7] & 0x80) ? (int32_t)rxbuff[7] - 256 : (int32_t)rxbuff[7];
//  	 	 	 	  	 	        cir= (rxbuff[8] & 0x80) ? (int32_t)rxbuff[8] - 256 : (int32_t)rxbuff[8];
//  	 	 	 	  	            up= (rxbuff[9] & 0x80) ? (int32_t)rxbuff[9] - 256 : (int32_t)rxbuff[9];
//  	 	 	 	                down= (rxbuff[10] & 0x80) ? (int32_t)rxbuff[10] - 256 : (int32_t)rxbuff[10];
//  	 	 	 	                left= (rxbuff[11] & 0x80) ? (int32_t)rxbuff[11] - 256 : (int32_t)rxbuff[11];
//  	 	 	 	             	right=(rxbuff[12] & 0x80) ? (int32_t)rxbuff[12] - 256 : (int32_t)rxbuff[12];
//  	 	 	 	             	ll1= (rxbuff[13] & 0x80) ? (int32_t)rxbuff[13] - 256 : (int32_t)rxbuff[13];
//  	 	 	 	             	ll2= (rxbuff[14] & 0x80) ? (int32_t)rxbuff[14] - 256 : (int32_t)rxbuff[14];
//  	 	 	 	             	rr1= (rxbuff[15] & 0x80) ?    (int32_t)rxbuff[15] - 256 : (int32_t)rxbuff[15];
//  	 	 	 	             	rr2= (rxbuff[16] & 0x80) ? (int32_t)rxbuff[16] - 256 : (int32_t)rxbuff[16];
//  	 	 	 	  	 	lx = rxbuff[1];
//  	 	 	 	  	  	 	 	 	  	 		  	ly = rxbuff[2] ;
//  	 	 	 	  	  	 	 	 	  	 		  	rx = rxbuff[3];
//  	 	 	 	  	  	 	 	 	  	 		  	ry = rxbuff[4];
  	 	 	 	  	  	 	 	 	  	 	        cro= (rxbuff[5] & 0x80) ? (int32_t)rxbuff[5] - 256 : (int32_t)rxbuff[5];
  	 	 	 	  	  	 	 	 	  	 	        squ= (rxbuff[6] & 0x80) ? (int32_t)rxbuff[6] - 256 : (int32_t)rxbuff[6];
  	 	 	 	  	  	 	 	 	  	 	        tri= (rxbuff[7] & 0x80) ? (int32_t)rxbuff[7] - 256 : (int32_t)rxbuff[7];
  	 	 	 	  	  	 	 	 	  	 	        cir= (rxbuff[8] & 0x80) ? (int32_t)rxbuff[8] - 256 : (int32_t)rxbuff[8];
  	 	 	 	  	  	 	 	 	  	            up= (rxbuff[9] & 0x80) ? (int32_t)rxbuff[9] - 256 : (int32_t)rxbuff[9];
  	 	 	 	  	  	 	 	 	                down= (rxbuff[10] & 0x80) ? (int32_t)rxbuff[10] - 256 : (int32_t)rxbuff[10];
  	 	 	 	  	  	 	 	 	                left= (rxbuff[11] & 0x80) ? (int32_t)rxbuff[11] - 256 : (int32_t)rxbuff[11];
  	 	 	 	  	  	 	 	 	             	right=(rxbuff[12] & 0x80) ? (int32_t)rxbuff[12] - 256 : (int32_t)rxbuff[12];
  	 	 	 	  	  	 	 	 	             	ll1= (rxbuff[13] & 0x80) ? (int32_t)rxbuff[13] - 256 : (int32_t)rxbuff[13];
  	 	 	 	  	  	 	 	 	             	ll2= (rxbuff[14] & 0x80) ? (int32_t)rxbuff[14] - 256 : (int32_t)rxbuff[14];
  	 	 	 	  	  	 	 	 	             	rr1= (rxbuff[15] & 0x80) ?    (int32_t)rxbuff[15] - 256 : (int32_t)rxbuff[15];
  	 	 	 	  	  	 	 	 	             	rr2= (rxbuff[16] & 0x80) ? (int32_t)rxbuff[16] - 256 : (int32_t)rxbuff[16];

  	 	 	 	  	 	        // Print the received values
  	 	 	 	  	 	        printf("Received Integers:\n");
  	 	 	 	  	 	        printf("lx: %ld\n", lx);
  	 	 	 	  	 	        printf("ly: %ld\n", ly);
  	 	 	 	  	 	        printf("rx: %ld\n", rx);
  	 	 	 	  	 	        printf("ry: %ld\n", ry);

  	 	 	 	  	 	        printf("cro: %ld\n", cro);
  	 	 	 	  	 	        printf("squ: %ld\n", squ);
  	 	 	 	  	 	        printf("tri: %ld\n", tri);
  	 	 	 	  	 	        printf("cir: %ld\n", cir);

  	 	 	 	  	 	        printf("up: %ld\n", up);
  	 	 	 	  	 	        printf("down: %ld\n", down);
  	 	 	 	  	 	        printf("left: %ld\n", left);
  	 	 	 	  	 	        printf("right: %ld\n", right);

  	 	 	 	  	 	        printf("ll1: %ld\n", ll1);
  	 	 	 	  	 	        printf("ll2: %ld\n", ll2);
  	 	 	 	  	 	        printf("rr1: %ld\n", rr1);
  	 	 	 	  	 	        printf("rr2: %ld\n", rr2);

  	 	 	 	  	 	    }
  	 	 	 	  	 	  else{
  	 	 	 	  	 		            ry = 0;
  	 	 	 	  	 		  	        rx = 0;
  	 	 	 	  	 		  	        lx = 0;
  	 	 	 	  	 		  	        ly = 0;
  	 	 	 	  	 		            cro = 0;
  	 	 	 	  	 		  	        squ = 0;
  	 	 	 	  	 		  	        tri = 0;
  	 	 	 	  	 		  	        cir = 0;
  	 	 	 	  	 		            up = 0;
  	 	 	 	  	 		  	        down = 0;
  	 	 	 	  	 		  	        left = 0;
  	 	 	 	  	 		  	        right = 0;
  	 	 	 	  	 		  	        ll1=0;
  	 	 	 	  	 		  	        ll2=0;
  	 	 	 	  	 		  	        rr1=0;
  	 	 	 	  	 		  	        rr2=0;


  	 	 	 	  	 	  }

  	 	 	 	  	 	  uint16_t dutycycle;
	 		 	 	  	  dutycycle=0;

	 		 	 	  	  //chassis

	 		 	 	  	  //motors stop
	 		 	 	  	  if(ly>=Buff2 && ly<=Buff1 && lx>=Buff2 && lx<=Buff1){
	 	 	 	  	  		//  TIM2->CCR1 = 0;
	 		 	 	  		 dutycycle=0;
	 		 	 	  		PWM(dutycycle);
	 	 	 	  	  		// HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);

	 		 	 	  	  }
	 		 	 	  	//  else{
	 		 	 	  	  //forward
	 		 	 	  	  else if( ly>=Buff1 && (lx<=BuffP && lx>=BuffN) )
	 		 	 	  	  	   {
	 		 	 	  		  	 dutycycle=map(ly,Buff1,127,0,fast);


//	 		 	 	  	  		  TIM2->CCR1 = dutycycle;
//
//	 		 	 	  	  		  	 	 		 	 	 	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	 		 	 	  		PWM(dutycycle);

//  	 	 	 	  	  	PWM(1000);
  	 	 	 	  		  				 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
  	 	 	 	  		  				 	 	  	  				  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
  	 	 	 	  		  				 	 	  	  				  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
  	 	 	 	  		  				 	 	  	  				  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);

  	 	 	 	  		  				 	 	  	  				  	 	 		 	  //	 HAL_Delay(1000);
  	 	 	 	  		  			  	   }
  	 	 	 	  		  			  	 //  backward
  	 	 	 	  		  				 		 	 	  	  	 else if(ly<=Buff2 && (lx<=BuffP && lx>=BuffN)  )
  	 	 	 	  		  				 		 	 	  	  	   {
  	 	 	 	  		  				 		 	 	  	  		dutycycle=map(ly,-128,Buff2,fast,0);

//  	 	 	 	  		  				 	 	  	  		 TIM2->CCR1 =dutycycle;
//  	 	 	 	  		  				 	 	  	  				  	 	 		 	 	 	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  	 	 	 	  		  			 PWM(dutycycle);
  	 	 	 	  		  				 	 	  	  				  	 	 		// PWM(1000);

  	 	 	 	  		  			  				  	 	 		 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
  	 	 	 	  		  			  				  	 	 		 		 	 	  	  		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
  	 	 	 	  		  			  				  	 	 		 		 	 	  	  		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
  	 	 	 	  		  			  				  	 	 		 		 	 	  	  		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
  	 	 	 	  		  			  				  	 	 		 		 	 	  	  		  	// HAL_Delay(1000);


  	 	 	 	  		  			  	  }
  	 	 	 	  		  			  //	   right
  	 	 	 	  		  				 		 	 	  	  	  else if(lx>=Buff1 && (ly<=BuffP && ly>=BuffN))
  	 	 	 	  		  				 		 	 	  	  	   {
  	 	 	 	  		  				 		 	  	  		dutycycle=map(lx,Buff1,127,0,fast);

//  	 	 	 	  		  				 	 	  	  		  TIM2->CCR1 = dutycycle;
//  	 	 	 	  		  				 	 	  	  		 		  	 	 		 	 	 	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  	 	 	 	  		  			 PWM(dutycycle);
//  	 	 	 	  		  			  	  PWM(1000);
  	 	 	 	  		  			  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
  	 	 	 	  		  			  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
  	 	 	 	  		  			  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
  	 	 	 	  		  			  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
  	 	 	 	  		  			  		 		  	 	 		// HAL_Delay(1000);

  	 	 	 	  		  			  	   }
  	 	 	 	  		  			  	//    left
  	 	 	 	  		  				 		 	 	  	  	  else if(lx<=Buff2 && (ly<=BuffP && ly>=BuffN)  )
  	 	 	 	  		  				 		 	 	  	  	   {
  	 	 	 	  		  				 		 	 	  	  		dutycycle=map(lx,-128,Buff2,fast,0);

//  	 	 	 	  		  				 	 	  	  		  TIM2->CCR1 = dutycycle;
//  	 	 	 	  		  				 	 	  	  		 		  	 	 		 	 	 	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  	 	 	 	  		  			  PWM(dutycycle);
//  	 	 	 	  		  		//	PWM(1000);
  	 	 	 	  		  			  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
  	 	 	 	  		  			  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
  	 	 	 	  		  			  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
  	 	 	 	  		  			  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
  	 	 	 	  		  			  		 		  	 	 		//HAL_Delay(1000);
  	 	 	 	  		  				 		 	 	  	  	   }
  	 	 	 	  		  			  		 		  	 	 	//clockwise
  	 	 	 	  	 		 	 	  			 	 		 	 	  	  if(ll1==1){
  	 	 	 	  	 		 	 	  			 	 		 	 	  	 PWM(500);


  	 	 	 	  	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
  	 	 	 	  	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
  	 	 	 	  	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
  	 	 	 	  	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);

  	 	 	 	  	 		 	 	  			 	 		 	 	  	  }
  	 	 	 	  //	 		 	 	  			 	 		 	 	  	  else if(ll2==1){
  	 	 	 	  //	 		 	 	  			 	 		 	 	  PWM(500);
  	 	 	 	  //	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
  	 	 	 	  //	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
  	 	 	 	  //	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
  	 	 	 	  //	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
  	 	 	 	  //	 		 	 	  			 	 		 	 	  	  }
  	 	 	 	  	 		 	 	  			 	 		 	 	  	  //anticlockwise
  	 	 	 	  	 		 	 	  			 	 		 	 	  	  else if(rr1==1){
  	 	 	 	  	 		 	 	  			 	 		 	 	  	PWM(500);
  	 	 	 	  	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
  	 	 	 	  	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
  	 	 	 	  	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
  	 	 	 	  	 		 	 	  			 	 			 	 		 	 	  	  		 		  	 	 		 	 	 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);

  	 	 	 	  	 		 	 	  			 	 		 	 	  	  }

  	 	 	 	  	 		 	 	  			 	 		 	 	  	  //ball picking

  	 	 	 	  		  			  		 		  	 	 	  if(up==1){
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 	 	TIM11->CCR1 = 1000;
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 	 			 HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 			 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 		 	 	  	  }
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 		//	HAL_Delay(1000);
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 else if(down==1){
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 	 	TIM11->CCR1 = 1000;
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 	 			 HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 			 	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 		 	 		  }
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 			//HAL_Delay(1000);
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 else
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	 	{
  	 	 	 	  		  			  		 		  	 		 		 	 	  				 	 	TIM11->CCR1 = 0;
  	 	 	 	  		  			  		 		  	 		 		 	 	  				 	 	 		HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
  	 	 	 	  		  			  		 		  	 		 		 	 	  			 	    }
//  	 	 	 	  		  			  		 		  	 		 		 	 	  				 		 HAL_Delay(1000);


  	 	 	 	  		  			  		 		  	 	 	  //bldc

  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  if(cro==1)
  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  {
  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  TIM10->CCR1=1000;
  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 //	 HAL_Delay(3000);
  	 	 	 	  		  			  		 		  	 			 	 		 	 	  	 	  }
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  else if(squ==1)
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  {
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  TIM10->CCR1=500;
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	 HAL_Delay(3000);
  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 //	  }
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  else if(tri==1)
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  {
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  TIM10->CCR1=100;
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
//  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	 HAL_Delay(3000);
  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 //	  }
  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  else
  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  {
  	 	 	 	  		  			  		 		  	 	 	 		 	 		 	 	  	 	  TIM10->CCR2=0;
  	 	 	 	  		  			  		 		  	 	 	 		 	 		 	 	  	 	  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
  	 	 	 	  		  			  		 		  	 	 	 		 	 		 	 	 //HAL_Delay(3000);
  	 	 	 	  		  			  		 		  	 		 		 	 		 	 	  	 	  }


  	 	 	 	  		  			  		 		  	 		 		 	 	//seedling picking and planting
  	 	 	 	  		  			  		 		  	 		 				 	  if(left==1)
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 		  {
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  for (uint8_t Angle = 0; Angle < 180; Angle++) {
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		  PCA9685_SetServoAngle(0, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		PCA9685_SetServoAngle(1, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		PCA9685_SetServoAngle(2, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		PCA9685_SetServoAngle(15, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		PCA9685_SetServoAngle(8, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  	    }
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  	  HAL_Delay(500);


  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 		  }
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 		  else if(right==1){
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  for (uint16_t Angle = 180; Angle > 0; Angle--) {
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		  		PCA9685_SetServoAngle(0, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		  				  		PCA9685_SetServoAngle(1, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		  				  		PCA9685_SetServoAngle(2, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		  				  		PCA9685_SetServoAngle(15, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		  				  		PCA9685_SetServoAngle(8, Angle);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		  	  }
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			  		  	  HAL_Delay(500);
  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 		  }
//  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 		  else
//  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 		  {
//  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 		uint16_t Angle = 0;
//
//  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 		PCA9685_SetServoAngle(0, Angle);
//  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			 		 	 	  					 	 			  		  				  		PCA9685_SetServoAngle(1, Angle);
//  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			 		 	 	  					 	 			  		  				  		PCA9685_SetServoAngle(2, Angle);
//  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			 		 	 	  					 	 			  		  				  		PCA9685_SetServoAngle(15, Angle);
//  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 			 		 	 	  					 	 			  		  				  		PCA9685_SetServoAngle(8, Angle);
//  	 	 	 	  		  			  		 		  	 		 		 		 	 	  					 	 		  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 180-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 99;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 180-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 99;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
