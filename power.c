/*
 * straight.c
 *
 *  Created on: May 31, 2021
 *      Author: jackyzhang
 */
#include "tim.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "main.h"
#include "HMC5883L.h"
#include <math.h>

Vector mag;
float head;				//定义

/*void turning(float target)		//转弯函数，target为具体转弯角度
{
  float target_new;				//目标角度值
  mag = HMC5883L_readNormalize();
  head = atan2f(mag.YAxis,mag.XAxis)*180.00/M_PI;//-head_0;   //head=当前朝向角度
  target_new=head+target;			//根据转弯角度,获取目标角度值

  if(target<0)	//左转
    {
    //printf1("head1=%f\n",head);
  	if(target_new<-180){
  		target_new+=360;//左转没超过-180
  		}
    	while(!(head<target_new+10 && head>target_new-10))
    	{
    		HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin, GPIO_PIN_SET);
      		HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
      		HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_RESET);
      		HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_SET);
    		mag = HMC5883L_readNormalize();
    		head = atan2f(mag.YAxis,mag.XAxis)*180.00/M_PI;//-head_0;   //head=当前朝向角度
    		// up date the angle
    //      printf1("head2=%f\n tar=%f",head,target_new);
    	}
    		HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET); //恢复直行
  	}
  if(target>0)	//右转
  {
    //printf1("head1=%f\n",head);
  	if(target_new>180){
			target_new=target_new-360;
  		}//右转超过180
    	while(!(head<target_new+10 && head>target_new-10))
    	{
    		HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin, GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
    		mag = HMC5883L_readNormalize();
    		head = atan2f(mag.YAxis,mag.XAxis)*180.00/M_PI;//-head_0;   //head=当前朝向角度
    		// up date the angle
    //      printf1("head2=%f\n tar=%f",head,target_new);
    	}
    		HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET); //恢复直行
  	}
}
*/
void line_tracking(uint8_t *i)					//openmv循线
    	{
	 HAL_UART_Receive_IT(&huart4,(uint8_t*)i,1);//接收openmv数据, position not clear

	    	if(*i=='0'){							//直行
	    			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	    			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	    			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	        		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40);
	        		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 40);
	    		}
    		else if(*i=='1'){
        		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 15);
        		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 50);		// 	左转
    		}
	    		else if(*i=='2'){
	        		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 45);
	        		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 15);		// 右转
	    	}
    	}
void turning(float angle){						//电子罗盘控制直行，angel为直行方向的罗盘角度

	  mag = HMC5883L_readNormalize();
	  head = atan2f(mag.YAxis,mag.XAxis)*180.00/M_PI;//-head_0;   //head=当前朝向角度

	  while(!(angle>head-2&&angle<head+2))	//首先转到相应的直行角度，torlerance changed
	      	{
	      			HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin, GPIO_PIN_SET);
	        		HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
	        		HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_RESET);
	        		HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_SET);
	         		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 35);
	         		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 35);
	      		mag = HMC5883L_readNormalize();
	      		head = atan2f(mag.YAxis,mag.XAxis)*180.00/M_PI;//-head_0;   //head=当前朝向角度
	      		// up date the angle
	      //      printf1("head2=%f\n tar=%f",head,target_new);
	      	}
	  	  	  	HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin, GPIO_PIN_SET);
	      		HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
	      		HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
	      		HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET); //恢复直行
	}


void straight(float angle){						//电子罗盘控制直行，angel为直行方向的罗盘角度
	  mag = HMC5883L_readNormalize();
	  head = atan2f(mag.YAxis,mag.XAxis)*180.00/M_PI;//-head_0;   //head=当前朝向角度
	  if(angle<-170&&angle>-180){					//特殊情况,angle在(-180,-170)
		  if(head>0||(head<angle&&head>-180)){		//偏左，右转
				HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 35);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 20);
	       		 //printf1("case1\n");
		  }
		  else if(head<0&&head>angle){		//偏右，左转
			  HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
			  HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
			  	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
			  	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 35);
			  	//printf1("case2\n");
		  }
	  }
	  else if(angle>170&&angle<180){					//特殊情况,angle在(170,180)
	  		  if(head>0&&head<angle){		//偏左，右转
	  				HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
	  				HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
	  				HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
	  				HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
	  	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 35);
	  	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 20);
	  	       	//printf1("case3\n");
	  		  }
	  		  else if(head<0||(head>angle&&head<180)){		//偏右，左转
	  			  HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
	  			  HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
	  			  HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
	  			  HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
	  			  	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
	  			  	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 35);
	  			  	//printf1("case4\n");
	  		  }
	  	  }
	  else if(head>angle-0.5&&head<angle+0.5){							//直行
  			HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
  			HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
  			HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
      		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60);
      		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
      		//printf1("case5\n");
  		}
	  else if(angle+0.5<head){							//车偏右，左转
			HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 45);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
	       		//printf1("case6\n");
	   		}
	  else if(angle-0.5>head){							//车偏左，右转
			HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 45);
	       		//printf1("case7\n");
	   		}
         }

/*void straight(float angle){						//电子罗盘控制直行，angel为直行方向的罗盘角度
	  mag = HMC5883L_readNormalize();
	  head = atan2f(mag.YAxis,mag.XAxis)*180.00/M_PI;//-head_0;   //head=当前朝向角度

	  if(angle>head-2&&angle<head+2){							//直行
  			HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
  			HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
  			HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
      		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
      		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 33);
  		}
	  else if(angle+2<head){							//车偏右，左转
			HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 45);
	   		}
	  else if(angle-2>head){							//车偏左，右转
			HAL_GPIO_WritePin(R_high_GPIO_Port, R_high_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(R_low_GPIO_Port, R_low_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_high_GPIO_Port, L_high_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(L_low_GPIO_Port, L_low_Pin, GPIO_PIN_RESET);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 45);
	       		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30);
	   		}
         }*/

