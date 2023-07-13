/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_it.h"
#include "struct_typedef.h"
#include "pid.h"
//#include "oled.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint8_t len1=0;
uint8_t len2=0;
char strtemp[512];
double LEN=0;
double SPEED=0;

volatile int count[2]={0};
volatile int total[2]={0};
volatile double circle[2]={0};
volatile double speed[2]={0};
int PPR=500;
int ratio=30;
int t=0;
int mode=0;
int biaozhi=0;
pid_type_def motor_pid[2];

fp32 PID1[3]={600,25,150};
fp32 PID2[3]={600,25,150};
fp32 set_speed[2]={0,0};//0为左，1为右
fp32 set_circle=0;
extern int stop;
int FLAG=0;
int oled_flag=0;


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern int mode;
extern int biaozhi;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void xunji(void)
{
	if(str[0]=='0')
	{
		set_speed[0]=0.15;
		set_speed[1]=0.15;
		Chassis_Speed();
	
	}
	else if(str[0]=='1')
	{
		set_speed[0]=-0.15;
		set_speed[1]=0.15;
			Chassis_Speed();
	}
	else if(str[0]=='2')
	{
		set_speed[0]=0.15;
		set_speed[1]=-0.15;
			Chassis_Speed();
		
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int cishu=0;
	cishu++;
	
		if(cishu==10)
		{	
			cishu=0;
					if(biaozhi==1)
					{
						
						if(htim->Instance==TIM1)
						{
							char s[2]={0};
							s[0]='a';
							HAL_UART_Transmit(&huart2,(uint8_t*)&s,2,10);
						}
					}
		}
	
    if(htim->Instance==TIM2)
	{
		t++;
	}
	if(t==10)
	{
			
//		OLED_Clear();
		oled_flag =1;
		
		count[0]=__HAL_TIM_GET_COUNTER(&htim3); 
        if(count[0]<=32800)
	    {
	    	count[0]=count[0]-0;
	    	__HAL_TIM_SET_COUNTER(&htim3,0); 
	    }
	    else
	    {
	        count[0]=count[0]-65535;
	        __HAL_TIM_SET_COUNTER(&htim3,65535);
	    }
		total[0]+=count[0];
		circle[0]=(double)total[0]/(4*PPR*ratio);
	    speed[0]=(double)count[0]*3000/(4*PPR*ratio);
		
		count[1]=__HAL_TIM_GET_COUNTER(&htim4); 
        if(count[1]<=32800)
	    {
	    	count[1]=count[1]-0;
	    	__HAL_TIM_SET_COUNTER(&htim4,0); 
	    }
	    else
	    {
	        count[1]=count[1]-65535;
	        __HAL_TIM_SET_COUNTER(&htim4,65535);
	    }
		total[1]+=count[1];
		circle[1]=(double)total[1]/(4*PPR*ratio);
	   speed[1]=(double)count[1]*3000/(4*PPR*ratio);
		t=0;
			
//		sprintf (strtemp  ,"%.3f,%.3f%.3f\r\n",roll,pitch,yaw);
//		HAL_UART_Transmit (&huart2,(uint8_t*)strtemp,strlen (strtemp ),0xffff);
	}
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//void oled_t(void)
//{
//	if (1)
//	{
//					OLED_Clear();   
//				if(speed[1]>speed[0])
//				  SPEED=speed[1];
//				else
//				  SPEED=speed[0];
//				if(circle[1]>circle[0])
//				  LEN=circle[1];
//				else
//					LEN=circle[0];
//						  OLED_ShowString(0,0,"Speed:",16,1);
//							 OLED_Showdecimal(75,0,SPEED,2,3,16,0);
//							
//						  OLED_ShowString(0,2,"Distance:",16,1);
//						  OLED_Showdecimal(75,2,LEN,2,2,16,0);
//							
//							 OLED_ShowString(0,4,"MODE:",16,1);
//							OLED_ShowNum (75,4,mode,3,16,1);
//							
//							OLED_ShowString (0,6,"biaozhi:",16,1);
//							OLED_ShowNum (75,6,biaozhi ,3,16,1);	
//				oled_flag =0;
//			}				
//}	


void USART1_IRQHandler(void) //串口1 接受来自OPENMV的信息进行循迹（空闲中断可能无法进行）
{
  if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)
  {
   __HAL_UART_CLEAR_IDLEFLAG(&huart1);
   HAL_UART_DMAStop(&huart1);
   len1=ARRLEN-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
	
		
			gy901_decode();  									//预认为回调代码 

			memset(arr,0,len1);
	   HAL_UART_Receive_DMA(&huart1,arr,ARRLEN);
  
  }
}

void USART2_IRQHandler(void)
	
{

  if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)!=RESET)
  {
   __HAL_UART_CLEAR_IDLEFLAG(&huart2);
   HAL_UART_DMAStop(&huart2);
   len2=STRLEN-__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		
		
		if(biaozhi ==1)
		{ 
			xunji();
		
		}
		
			memset(str,0,len2);
   HAL_UART_Receive_DMA(&huart2,str,STRLEN);
  }
			}

			
			

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
void Chassis_Init(void);
void Chassis_Speed(void);
void Chassis_Circle(void);
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}



/* USER CODE BEGIN 1 */
void gy901_decode(void)
{
	
	if(arr[0]=='Q')
		{
	
		mode=1;
			
	
		}
	else if(arr[0]=='W')
		{
		mode =2;
		}
	
											
	else if(arr[0]=='E')
		{
		mode=3;			
		}
	
		
	else if(arr[0]=='R')
		{
		mode =4;
		}

}

void Chassis_Init(void)//运动代码 初始化
	
{	
	

	HAL_GPIO_WritePin (GPIOA ,GPIO_PIN_12,GPIO_PIN_RESET );
  HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_RESET);
  PID_init(&motor_pid[0],PID_POSITION,PID1,900,300);
  PID_init(&motor_pid[1],PID_POSITION,PID2,900,300);
}

void Chassis_Speed(void)
{
    PID_calc(&motor_pid[0],speed[0],set_speed[0]);
    PID_calc(&motor_pid[1],speed[1],set_speed[1]);
 if(set_speed[0]>0)
 {
	  HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);
	  if(motor_pid[0].out>0)
	  {
		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,motor_pid[0].out);
	  }
	  else if(motor_pid[0].out<=0)
	  {
		  motor_pid[0].out=0;
		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,motor_pid[0].out);
	  }
  }
  else if(set_speed[0]<0)
  {
	  HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_SET);
	  if(motor_pid[0].out<0)
	  {
		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-motor_pid[0].out);
	  }
	  else if(motor_pid[0].out>=0)
	  {
		  motor_pid[0].out=0;
	      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-motor_pid[0].out);
	  }
   }
  else if(set_speed[0]==0)
  {
	  HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_SET);
  }
  if(set_speed[1]>0)
  {
	  HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_RESET);
	  if(motor_pid[1].out>0)
	  {
		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,motor_pid[1].out);
	  }
	  else if(motor_pid[1].out<=0)
	  {
		  motor_pid[1].out=0;
		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,motor_pid[1].out);
	  }
  }
  else if(set_speed[1]<0)
  {
	  HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_SET);
	  if(motor_pid[1].out<0)
	  {
		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,-motor_pid[1].out);
	  }
	  else if(motor_pid[1].out>=0)
	  {
		  motor_pid[1].out=0;
	      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,-motor_pid[1].out);
	  }
   }
  else if(set_speed[1]==0)
  {
	  HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_SET);
  }
}

void Chassis_Circle(void)
{
	if(set_circle>0)
	{
	    if(circle[0]>=set_circle)
	    {
		  set_speed[0]=0;
	    }
	    if(circle[1]>=set_circle)
	    {
		  set_speed[1]=0;
	    }
	}
	else if(set_circle<0)
	{
	    if(circle[0]<=set_circle)
	    {
		  set_speed[0]=0;
	    }
	    if(circle[1]<=set_circle)
	    {
		  set_speed[1]=0;
	    }
	}
}
void buzzer(void)
{
	int j;
	for(j=0;j<3;j++)
	{
	HAL_GPIO_WritePin (GPIOA,GPIO_PIN_12 , GPIO_PIN_SET);
	HAL_Delay (500);
	HAL_GPIO_WritePin (GPIOA,GPIO_PIN_12 ,GPIO_PIN_RESET);
	HAL_Delay (500);
	}

	}
/* USER CODE END 1 */
