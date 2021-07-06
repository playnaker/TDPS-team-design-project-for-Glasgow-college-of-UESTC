/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  *  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HMC5883L.h"			//罗盘
#include <math.h>			//罗盘
#include "string.h"
#include "stdio.h"
#include "power.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);

HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);

RTC_TimeTypeDef GetTime;   //get time structure, to close line-tracing
RTC_TimeTypeDef GetTime0;
RTC_TimeTypeDef GetTime1;

RTC_TimeTypeDef GetTime2;//force the rover to turn
RTC_TimeTypeDef GetTime3;

RTC_DateTypeDef GetData;  //get date structure
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define basicpwm 0.30//basic speed
//#define pwmperiod 0.001 //0.001ms

#define given_dis2 300 //300mm,front
#define given_dis6 220 //220mm, right

#define printf1(...) HAL_UART_Transmit(&huart1,(uint8_t *)u_buf,sprintf((char*)u_buf,__VA_ARGS__),0xffff);		//串口通信初始�????????????????????
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int start=1;		//switch, 1 for on, 0 for off
uint8_t aTxStartMessages[] = "\r\n******UART communication using IT******\r\nPlease enter 10 characters:\r\n";	//串口通信初始�????????????????????
uint8_t i[2];

extern float angle;
extern Vector mag;
extern float head;//variable inside power.c
float s_angle;
int c=0;

//for ultrasonic
uint8_t receive[3];//received distance
double volatile dis=600;//存储计算得到的distance
uint8_t sign=0x55;//触发信号
double dis1,dis2,dis3,dis0;
uint8_t u_buffer[1];//receive angle from openmv

int tasknum = 1;//control number

double time_difference;
double time_difference1;
double given_time1=199;//time to start the ultrasonic
double given_time2=25 ;
double given_time3=30;
//int j=0;
float turning_angle1=73;
float turning_angle2=188;
float straight_angle1=69;
float straight_angle2=108;

uint8_t u_buf[256];

/*int ang_init;//recieve angle from gyroscope
//double angles[3];
float pwm_out1 = 0.0;//right
float pwm_out2 = 0.0;//left*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/*void pwm_init(void);//initialize configuration of pwm
void pid_all(void);//use openmv to follow the line
void pidinit(void);//initialize pid model
void serialcom(void);//communicate wuth uart ，between openmv and STM32

void turn(float tangle);//turn with specific angle
void get_angle(void);//with gyroscope
void pid_compute(void);//output 2 pwm
void overbridge(void);

void get_distance(void); */

void get_distance2(void);
void get_distance6(void);
void stop(void);//pwm=0

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //pwm_init();
  //pidinit();

  HMC5883L_setRange(HMC5883L_RANGE_1_3GA);
  HMC5883L_setMeasurementMode(HMC5883L_CONTINOUS);
  HMC5883L_setDataRate(HMC5883L_DATARATE_15HZ);
  HMC5883L_setSamples(HMC5883L_SAMPLES_1);
  HMC5883L_setOffset(380, 232);		//initialization of compass

  //HMC5883L_setOffset(332.5, 229.5);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);				//右轮
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);				//左轮

  //HAL_TIM_Base_Start_IT(&htim2);//interrupt, not clear

  //alternating interrupt
  /*HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);					//右轮
  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);				//左轮
  			HAL_TIM_Base_Start_IT(&htim2);          // 使能定时timer2 interrupt */
  			//start=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (tasknum) {

	              case 0://to test the ultrosonic module
	            	  printf1("case number is 0 for test\r\n");

	            	  while (c<=20){
	            	  HAL_UART_Transmit_IT(&huart6,&sign,1);
	            	  	HAL_UART_Receive_IT(&huart6,receive,3);
	            	  HAL_Delay(50);
	                 //printf1("%0.2f f\r\n", dis);
	            	 dis1=dis;
	            	HAL_UART_Transmit_IT(&huart6,&sign,1);
	            	 HAL_UART_Receive_IT(&huart6,receive,3);
	                 HAL_Delay(50);
	                  //printf1("%0.2f f\r\n", dis);
	            	 dis2=dis;
	            	 HAL_UART_Transmit_IT(&huart6,&sign,1);
	            	 HAL_UART_Receive_IT(&huart6,receive,3);
	            	 HAL_Delay(50);
	            	 dis3=dis;
	            	 dis0=(dis1+dis2+dis3)/3;
	                 printf1("%0.2f for test\r\n", dis0);
	                 c++;
	            	 if(dis0<=given_dis2&&dis0>=270){
	            	  	             	   //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,SET);
	            	  	             	   //HAL_Delay(5000);
	            	  	             	  //printf1( "meet beacon1\r\n");
	            	  	                   //printf1( "meet beacon1\r\n");
	            	 dis=500;//reset
	            	 tasknum=0;

	            	  	     break;}
	            	  }
	            	  if (c>9){
	            	  tasknum=1;}

	            	  break;

	              case 1://openmv line_tracing
	            	  printf1("begin to get time and go\r\n");
	            	  HAL_RTC_GetTime(&hrtc, &GetTime2, RTC_FORMAT_BIN);
	            	  HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);

                      while (1){

                    	  //HAL_UART_Receive_IT(&huart4,(uint8_t*)i,1);//接收openmv数据, position not clear

                    	  HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
                    	  HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);

                    	 // printf1("%02d:%02d:%02d\r\n",GetTime.Hours, GetTime.Minutes, GetTime.Seconds);
                          //HAL_Delay(1000);

                    	if ((GetTime.Minutes*60+GetTime.Seconds)>given_time1)
                    	{
                    		HAL_TIM_Base_Stop_IT(&htim2);



                    		tasknum = 5;

                    		break;
                    	                    }

                      }


	                  break;

	              case 5://close line_tracking, begin to get distance
	            	  printf1("case number is 5,get distance\r\n");

	             	            	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 28);
	             	            	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 28);
	                                    while(1){
	             	            	         HAL_UART_Transmit_IT(&huart6,&sign,1);
	             	            	         HAL_UART_Receive_IT(&huart6,receive,2);
	             	            	         HAL_Delay(50);
	             	            	        printf1("%0.2f f\r\n", dis);
	             	            	         dis1=dis;
	             	            	         HAL_UART_Transmit_IT(&huart6,&sign,1);
	             	            	         HAL_UART_Receive_IT(&huart6,receive,2);
	             	            	         HAL_Delay(50);
	             	            	         printf1("%0.2f f\r\n", dis);
	             	            	         dis2=dis;
	             	            	         HAL_UART_Transmit_IT(&huart6,&sign,1);
	             	            	         HAL_UART_Receive_IT(&huart6,receive,2);
	             	            	         HAL_Delay(50);
	             	            	         dis3=dis;
	             	            	         dis0=(dis1+dis2+dis3)/3;
	             	            	         printf1("%0.2f f\r\n", dis0);
	             	            	        HAL_RTC_GetTime(&hrtc, &GetTime3, RTC_FORMAT_BIN);
	             	            	        HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);
	             	            	       if((GetTime3.Minutes*60+GetTime3.Seconds)-(GetTime2.Minutes*60+GetTime2.Seconds)>=220){
	             	            	      	      HAL_TIM_Base_Stop_IT(&htim2);

	             	            	      	            	      tasknum=2;//get in to case 2
	             	            	      	            	      break;           }

	             	            	         if(dis0<=given_dis2&&dis0>=270){
	             	            	                      		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,SET);
	             	            	                      		//HAL_Delay(5000);
	             	            	                      		 //printf1( "meet beacon1\r\n");
	             	            	                      		 //printf1( "meet beacon1\r\n");
	             	            	            dis=500;//reset
	             	            	            tasknum=2;
	             	            	            break;
	             	            	         }}break;

	              case 2://turn right
	            	  //HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
	            	  //HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);

	            	  printf1("get in case 2 , tasknum is %d\r\n", tasknum);

	            	  HAL_TIM_Base_Stop_IT(&htim2);
	            	  //HAL_Delay(2000);
	            	  printf1("begin to turn\r\n");
	                  turning(turning_angle1);
	                  printf1("after %f\r\n",head);
	                  if (turning_angle1>head-3&&turning_angle1<head+3){
	                  printf1("finish turn\r\n");
	                  tasknum = 3;}

	                  break;

	              case 3://cross bridge
	            	  HAL_TIM_Base_Stop_IT(&htim2);
	            	  printf1("get in case 3\r\n");
	            	  //HAL_Delay(2000);

	            	  printf1("begin to straight\r\n");
	            	  //HAL_Delay(2000);
	            	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 62);
	            	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
	            	    HAL_Delay(5000);

	            	    printf1("begin to travel with same duty\r\n");
	            	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40);
	            	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30);
	            	    HAL_Delay(3000);
	            	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
	            	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30);
	            	    HAL_Delay(3000);
	            	    s_angle=straight_angle2;
	            	    HAL_TIM_Base_Start_IT(&htim3);
	            	   // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
	            	   // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30);
	            	   	 HAL_Delay(3000);

	            	   	HAL_TIM_Base_Stop_IT(&htim3);

	            	   	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
	            	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30);

            while(1){
	            	      HAL_UART_Transmit_IT(&huart6,&sign,1);
	            	       HAL_UART_Receive_IT(&huart6,receive,2);
	            	      HAL_Delay(50);
	            	                        	//printf1("%0.2f f\r\n", dis);
	            	       dis1=dis;
	            	     HAL_UART_Transmit_IT(&huart6,&sign,1);
	            	     HAL_UART_Receive_IT(&huart6,receive,2);
	            	      HAL_Delay(50);
	            	                        	//printf1("%0.2f f\r\n", dis);
	            	    dis2=dis;
	            	 HAL_UART_Transmit_IT(&huart6,&sign,1);
	            	HAL_UART_Receive_IT(&huart6,receive,2);
	            	  HAL_Delay(50);
	            	      dis3=dis;
	            	     dis0=(dis1+dis2+dis3)/3;
	            	 printf1("%0.2f f\r\n", dis0);
	            	                        	//
	            	 if(dis0<=given_dis2&&dis0>=250){
	            	                        		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,SET);
	            	                        		//HAL_Delay(5000);
	            	                        		 //printf1( "meet beacon1\r\n");
	            	                        		 //printf1( "meet beacon1\r\n");
	            	   dis=500;//reset
	            	   printf1("get distance2\r\n");
	                   tasknum = 4;

	            	      break;
	            	    }
            }

	            	 break;


	              case 4://after crossing the bridge, turn right
	            	    printf1("bigin to turn right\r\n");//右转
	            	    turning(22);

	            	  //  if (turning_angle2>head-10&&turning_angle2<head+10){
	            	    	   printf1("finish turn\r\n");
	            	    	     tasknum = 7;

	            	 //   }
	            	//  s_angle=straight_angle1;
	            	//  HAL_TIM_Base_Start_IT(&htim3);

	            	    break;

	              case 7://line tracking

	            	  	 printf1("begin to line tracking\r\n");

	            	  	     //HAL_Delay(2000);
	            	  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	            	  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);					//右轮
	            	  				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	            	  				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);				//左轮
	            	  				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
	            	  				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30);

	            	  	  HAL_TIM_Base_Start_IT(&htim2);
	            	  	HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
	            	    HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);
	            	  	  while(1){

	            	  	HAL_RTC_GetTime(&hrtc, &GetTime1, RTC_FORMAT_BIN);
	            	    HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);
	            	    //printf1("begin to get time1\r\n");

	            	    time_difference1=(GetTime1.Minutes*60+GetTime1.Seconds)-(GetTime.Minutes*60+GetTime.Seconds);
	            	    printf1("time1 is %f\r\n",time_difference1);
	            	    if((GetTime1.Minutes*60+GetTime1.Seconds)-(GetTime.Minutes*60+GetTime.Seconds)>=given_time2){
	            	    	HAL_TIM_Base_Stop_IT(&htim2);
	            	    	stop();
	            	  	printf1("finish stop\r\n");
	            	      tasknum=6;//for test
	            	      break;           }}


	              /*      HAL_UART_Transmit_IT(&huart6,&sign,1);
	            	    HAL_UART_Receive_IT(&huart6,receive,3);
	            	    HAL_Delay(50);
	            	  printf1("is %f suck my dick\r\n", dis);
	            	  //HAL_Delay(2000);

	            	  if(dis<=given_dis6&&dis>=200){
	            		  printf1("finish cross bridge\r\n");
	            		  //HAL_TIM_Base_Stop_IT(&htim3);
	            		  tasknum = 4;
	            		  //HAL_Delay(2000);
	            	                	}
	                  break;

	              case 4://turn left

	            	  printf1("get in case 4\r\n");

	            	  //HAL_Delay(2000);
	            	  printf1("begin to turn\r\n");
	                  turning(turning_angle2);
	                  printf1("after %f\r\n",head);
	                  if (turning_angle2>head-0.5&&turning_angle2<head+0.5){//tolerance is 2

	                	  printf1("finish turn\r\n");
	                  tasknum = 5;
	                  }
	                  break;

	              case 5://cross the arc
	              	            	  printf1("get in case 5 begin arc\r\n");
	              	            	  HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
	              	            	  HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);

	              	            	//HAL_TIM_Base_Start_IT(&htim2);
	              	            	  while (1){

	              	            	  s_angle=straight_angle2;
	              	            	   //HAL_TIM_Base_Start_IT(&htim3);

	              	             	 //printf1("begin to get distance6\r\n");
	              	             	 //get_distance6();
	              	            	   HAL_RTC_GetTime(&hrtc, &GetTime0, RTC_FORMAT_BIN);
	              	            	   HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);
	              	            	   //printf1("begin to get time\r\n");

	              	            	   time_difference=(GetTime0.Minutes*60+GetTime0.Seconds)-(GetTime.Minutes*60+GetTime.Seconds);
	              	            	   printf1("time %f\r\n",time_difference);

	              	             	 if((GetTime0.Minutes*60+GetTime0.Seconds)-(GetTime.Minutes*60+GetTime.Seconds)>=given_time3){
	              	             		printf1("finish cross arc\r\n");
	              	             		//HAL_Delay(2000);
	              	             		//HAL_TIM_Base_Stop_IT(&htim2);
	              	             		 stop();
	              	             		printf1("finish stop\r\n");
	              	             		tasknum=6;//for test
	              	             		break;           }
	              	             	  }

	              	            	  break; */

	              default:
	                  stop();
	                  break;
	          }
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//switch on&off
{

	if(GPIO_Pin == GPIO_PIN_13)
	{
		tasknum=1;//changed
		if(start==1){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);					//右轮
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);				//左轮
			HAL_TIM_Base_Start_IT(&htim2);          // 使能定时timer2 interrupt
			start=0;
		}
		else if(start==0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);			//右轮
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);			//左轮
			 HAL_TIM_Base_Stop_IT(&htim2);          // 关闭定时timer2 interrupt
			start=1;
		}
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)		//tim2中断回调函数
{
	if (htim == (&htim2)){
		static char c=0;
		c++;
	     //	  	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  line_tracking(i);		//【openmv巡线】根据openmv返回的数据i，straigt or turn
		if(c>100)
		{
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			c=0;
		}			//LED灯闪烁判断是否进入interrupt
	}
	if (htim == (&htim3)){
		  straight(s_angle);		//【openmv巡线】根据openmv返回的数据i，straigt or turn
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	dis=((double)receive[0]*256+(double)receive[1]);

}


void get_distance2(void){//get distance with

	HAL_UART_Transmit_IT(&huart2,&sign,1);
    HAL_UART_Receive_IT(&huart2,receive,3);
	HAL_Delay(50);

}


void get_distance6(void){//get distance with

	HAL_UART_Transmit_IT(&huart6,&sign,1);
    HAL_UART_Receive_IT(&huart6,receive,3);
    HAL_Delay(50);

}

void stop(void) {//pwm output is 0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);//右轮
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);//左轮
}



/*
void overbridge(void) {
    while (1) {
        //t.start();
    	TIM_CHANNEL_1 = 0.4;//speed up
    	TIM_CHANNEL_4 = 0.4;
        if (srf.read() = srfdis) //find the beacon1? move a constant distance
        {
            stop();
            HAL_Delay(1500);//has passed the bridge
            if (srf.read() = srfdis) //find the beacon2
            {
                //t.stop();
                //t.reset();
                tasknum = 4;  break;
            }
        }
    }
}

void pid_all(void) {
    while (1) {
    	pwm1 = pwm_out1;
    	pwm2 = pwm_out2;
    	pid_compute();
        if (tasknum != 1) {
            stop();
            HAL_Delay(1500);
            break;
          }

        get_distance(void)

        if (srf.read() == srfdis) {
        	tasknum = 2; break; }

    }
}

void pid_compute(void) {
    serialcom();
    pid.setProcessValue(openmv_angle);//transmit angle to pid,openmv_angle has sign
    pwm_out1 = pid.compute1();//输出矫正后的左pwm
    pwm_out2 = pid.compute2();//输出矫正后的右pwm
}


void serialcom(void) {//receive angle

    while (1) {
    	HAL_UART_Receive_IT(&huart1,(uint8_t *)&u_buffer,1);

        if (u_buffer[0]!=0) {
            if (u_buffer[0] > 5) { openmv_angle = u_buffer[0] - 10; }
            else { openmv_angle = u_buffer[0]; }

           // if (u_buffer[0] > 127) { openmv_angle += (u_buffer[0] - 256) * 0.01; }
           // else { openmv_angle += u_buffer[0] * 0.01; }
            //openmv_order = u_buffer[0];
            break;
        }
    }
    return;
}

void turn(int tangle) {//turn with given angle, need compass
    stop();

    HAL_Delay(200);
    get_angle();
    //float ang_init = angles[2];
    if (tangle>0) {
        pwm1 = basicpwm;
        pwm2 = basicpwm;
    }

    if (tangle<0) {

        pwm1 = basicpwm;
        pwm2 = basicpwm;

    }

    while (1) {
        get_angle();
        if (abs(ang_init) == abs(tangle)) {
            stop();
            break;
        }
    }
}

void get_angle(void) {//unknown
    while (1) {
    	ang_init = com.getc();
    }
}

void pwm_init(void) {
    com.baud(9600);
    pwm1.period(pwmperiod);
    pwm2.period(pwmperiod);
    stop();
} */


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
