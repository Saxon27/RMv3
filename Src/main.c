/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "test_beep.h"
#include "stdio.h"
/* USER CODE BEGIN Includes */
#include "test_imu.h"
#include "test_app.h"
#include "test_can.h"
#include "test_uart.h"
//ң��������
int ch0=0 , ch1=0 , ch2=0 , ch3=0 , s1=0 , s2=0  ; //ң��������
int16_t MouseX = 0 , MouseY = 0 , MouseZ = 0 ;    //���
uint8_t MouseLeft = 0 , MouseRight = 0 ;  //������Ҽ�
uint16_t key_All = 0 , key_W = 0 , key_S = 0 , key_A = 0 , 
		key_D = 0 , key_Q = 0 , key_E = 0 , key_Shift = 0 , key_Ctrl = 0 ;    //����
int Dbus = 0 ; 
int NUM1=0 , NUM2=0 , NUM3=0 , NUM4  = 0  , NUM5 = 0 , NUM6 = 0 ;    //����ֵ
int Speed1=0 , Speed2=0 , Speed3=0 , Speed4=0 ;      //Ŀ���ٶ�
int16_t ecode1 = 0 , ecode2 = 0 , ecode3 = 0 , ecode4 = 0 ,ecode5 = 0 , ecode6 = 0 , ecode7 = 0 , ecode8 = 0 ;  //����ֵ
int16_t Pitch_x_init=5000; 								//��������̨Pitch��ʼλ��ֵ
int16_t Pitch_x_control;           
int16_t Pitch_x_Max=5400;											//��������̨Pitch���Ƕ�
int16_t Pitch_x_Min=4200;											//��������̨Pitch��С�Ƕ�
int Pitch_I_Max = 4900 ;             				 	//Pitch�����������ֵ
int Pitch_I_Min =-4900 ;               				//Pitch����������Сֵ


/****************Ӣ�۳���̨����*****************/
int16_t Pitch_x_init_Hero = 3720 ;
int16_t Pitch_x_Max_Hero = 4270 ;
int16_t Pitch_x_Min_Hero = 3070 ;  
             

/*****************��̨PID����(����)**********************/
float Pitch_v_erro[2]={0,0},Pitch_v_feedback[2]={0,0},Pitch_v_except=0,Pitch_v_real=0;//x:�Ƕ�λ��,v:�Ƕ��ٶ�
float Pitch_x_erro[2]={0,0},Pitch_x_feedback[2]={0,0},Pitch_x_except=0,Pitch_x_real=0;
float Pitch_I_except=0,Pitch_I_lost=0;
int 	Pitch_I_control=0;
float Pitch_v_P=5,Pitch_v_I=0,Pitch_v_D=0,Pitch_v_integral=0,Pitch_v_diff;
float Pitch_x_P=5,Pitch_x_I=0,Pitch_x_D=0,Pitch_x_integral=0,Pitch_x_diff;
int Pitch_cont_T;

/*****************����PID����***********************/
static float Speed_Last_Least[4] = {0 , 0 , 0 , 0};//�ĸ����������

/*****************����������******************/
uint8_t mpu_buff[14];    
IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

/******************************************/
#define BUFFER_SIZE 18
uint8_t rx_len=0;
uint8_t recv_end_flag=0;
uint8_t rx_buffer[18];
//extern uint8_t aRxMessage[50];
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void tim122_pwm_setvalue(uint16_t value);//����Ħ����
void tim121_pwm_setvalue(uint16_t value);
void ReadControl(void);//��ң����
void ReadEncode(void);//��������
extern void Control(void);//����
int PitchPID(void);//��̨PID
int SpeedPID(int encoder , int except , int electric , int which);  // ����PID
float KalmanFitter(float);
float KalmanFitter2(float);
void OutPut_Data(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  MPU6500_Init();
  //IST8310_Init();
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
  
  HAL_UART_Receive_IT(&huart3, uart3_rx_buff, 1);
  HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 1);
  HAL_UART_Receive_IT(&huart6, uart1_rx_buff, 1);
  
  if(MPU_id != 0)sTestResult.imuTest = 0x01;
  /* USER CODE END 2 */

  /* Infinite loop */
  //Ħ���ֳ�ʼ��
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  tim121_pwm_setvalue(833);
  tim122_pwm_setvalue(833);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);    //ʹ�ܿ����ж�
  
/********************��ʼ����̨����************************/
	Pitch_x_control =Pitch_x_init; 
//Pitch_x_control = Pitch_x_init_Hero ; 
//Pitch_x_Max = Pitch_x_Max_Hero ;
//Pitch_x_Min = Pitch_x_Min_Hero ;

	//������ʱ��6
  HAL_TIM_Base_Start_IT(&htim6);
  while (1)
  {
      /* USER CODE BEGIN 3 */
 //     IMU_Get_Data();
//	  printf("IMU: %d %d %d %d %d %d %d %d %d %d\n" , imu_data.ax , imu_data.ay , imu_data.az 
//											, imu_data.gx , imu_data.gy , imu_data.gz 
//											, imu_data.mx , imu_data.my , imu_data.mz , imu_data.temp);
//	  ReadControl();Ϊ��ʵʱ�ɼ����ƶ���TIM6�ж�
//	  ReadEncode();//Ϊ��ʵʱ�ɼ����ƶ���TIM6�ж�
	  Control();
//	  HAL_Delay(1) ;
//	  HAL_CAN_WakeUp(&hcan1);
  }
  /* USER CODE END 3 */

}




//��ȡң��������
void ReadControl(){
	if(recv_end_flag ==1){  
		//printf("rx_len=%d\r\n",rx_len);  		
		ch0 = (rx_buffer[0]| (rx_buffer[1] << 8)) & 0x07ff;
		ch1 = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff;
		ch2 = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff;
		ch3 = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff; 
		s1 = ((rx_buffer[5] >> 4)& 0x000C) >> 2; 
		s2 = ((rx_buffer[5] >> 4)& 0x0003); 
		MouseX = rx_buffer[6] | (rx_buffer[7] << 8); 
		MouseY = rx_buffer[8] | (rx_buffer[9] << 8); 
		MouseZ = rx_buffer[10] | (rx_buffer[11] << 8); 
		MouseLeft = rx_buffer[12]; 
		MouseRight = rx_buffer[13];
		key_All = rx_buffer[14] | (rx_buffer[15] << 8); 
		key_W = key_All & 1 ;
		key_S = (key_All & (1 << 1 ))>>1;
		key_A = (key_All & (1 << 2 ))>>2;
		key_D = (key_All & (1 << 3 ))>>3;
		key_Shift = (key_All & (1 << 4 ))>>4;
		key_Ctrl = (key_All & (1 << 5 ))>>5;
		key_Q = (key_All & (1 << 6 ))>>6;
		key_E = (key_All & (1 << 7 ))>>7;
		//printf("%d %d %d %d %d %d \n" , ch0 , ch1 , ch2 , ch3 , s1 , s2);
		//printf("%d %d %d %d %d %d \n" , MouseX , MouseY , MouseZ , MouseLeft , MouseRight , key_All);
		//printf("%d %d %d %d %d %d %d %d \n" , key_W , key_S , key_A , key_D , key_Shift , key_Ctrl ,  key_Q , key_E );
		rx_len=0;
		recv_end_flag=0;
		//���ң�����Ƿ���
		if((ch0 != 0 ) && (ch1 != 0 )&&(ch2 != 0 ) && (ch3 != 0 )){
			Dbus = 1 ;
		}else{
			Dbus = 0 ;
		}
	}
	HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);
}
//����������ֵ(����)
void ReadEncode(){
	ecode1 = (can1_rx_data_buf[0][2]<<8) | can1_rx_data_buf[0][3] ;
	ecode2 = (can1_rx_data_buf[1][2]<<8) | can1_rx_data_buf[1][3] ;
	ecode3 = (can1_rx_data_buf[2][2]<<8) | can1_rx_data_buf[2][3] ;
	ecode4 = (can1_rx_data_buf[3][2]<<8) | can1_rx_data_buf[3][3] ;
	ecode5 = (can1_rx_data_buf[4][0]<<8) | can1_rx_data_buf[4][1] ;
  ecode6 = (can1_rx_data_buf[5][0]<<8) | can1_rx_data_buf[5][1] ;
//  ecode7 = (can1_rx_data_buf[5][2]<<8) | can1_rx_data_buf[5][3] ;
//  ecode8 = (can1_rx_data_buf[5][4]<<8) | can1_rx_data_buf[5][5] ;
	if(ecode1 == -3) ecode1 = 0 ;
	if(ecode2 == -3) ecode2 = 0 ;
	if(ecode3 == -3) ecode3 = 0 ;
	if(ecode4 == -3) ecode4 = 0 ;
//	printf("ecode : %d  %d  %d  %d  %d  %d \n" , ecode1 , ecode2 , ecode3 , ecode4 , ecode5 , ecode6);
	printf("ecode : %d\n",ecode5);
}
void Control(){
	if(Dbus == 1){
		if (s2 == 3){
			//���Կ���
			Speed1= ((1024 - ch1) + -(MouseX*2) + -(1024 - ch0))*12 ;
			Speed2 = -((1024 - ch1) + (MouseX*2) + (1024 - ch0))*12 ;
			Speed3 = ((1024 - ch1) + -(MouseX*2) + (1024 - ch0))*12 ;
			Speed4 = -((1024 - ch1) + (MouseX*2) + -(1024 - ch0))*12 ;
			Pitch_x_control = Pitch_x_control + MouseY/10;
			//���س�ԭ����ת����
			//Speed1 = 0 ; Speed2 = 0  ; 
			//Speed3 = 0 ; Speed4 = 0  ;
		}else if(s2 == 1){
			//ң��������
			if(s1 == 1){
				tim121_pwm_setvalue(1666);
				tim122_pwm_setvalue(1666);
				HAL_GPIO_WritePin(GPIOF, Motor_Bullet_Pin , GPIO_PIN_SET);
			}
			if(s1 == 3){
				tim121_pwm_setvalue(833);
				tim122_pwm_setvalue(833);
				HAL_GPIO_WritePin(GPIOF, Motor_Bullet_Pin , GPIO_PIN_RESET);
			}
			Speed1= ((1024 - ch1) + -(1024 - ch2) + -(1024 - ch0))*12 ;
			Speed2 = -((1024 - ch1) + (1024 - ch2) + (1024 - ch0))*12 ;
			Speed3 = ((1024 - ch1) + -(1024 - ch2) + (1024 - ch0))*12 ;
			Speed4 = -((1024 - ch1) + (1024 - ch2) + -(1024 - ch0))*12 ;
//			Pitch_x_control = Pitch_x_control + (int16_t)(( 1024 - ch3 )/200.0f);
			Pitch_x_control = (int16_t)((1024-ch3)*(Pitch_x_Max-Pitch_x_Min)/(1684-364)+Pitch_x_init);

			//���س�ԭ����ת����
			//Speed1 = 3000 ; Speed2 = 3000  ; 
			//Speed3 = 3000 ; Speed4 = 3000  ;
		}
		if(Pitch_x_control > Pitch_x_Max) Pitch_x_control	= Pitch_x_Max ;    //����Pitch
		if(Pitch_x_control < Pitch_x_Min) Pitch_x_control = Pitch_x_Min ;
//		printf("%d    %d\n",ch3,Pitch_x_control);
		
//PID�����ƶ���TIM6�ж�
//		NUM1 = SpeedPID(ecode1 , Speed1 , NUM1 , 0);
//		NUM2 = SpeedPID(ecode2 , Speed2 , NUM2 , 1);
//		NUM3 = SpeedPID(ecode3 , Speed3 , NUM3 , 2);
//		NUM4 = SpeedPID(ecode4 , Speed4 , NUM4 , 3);
//		//printf("%d %d %d %d \n" , NUM1 , NUM2 , NUM3 , NUM4);
//		NUM5 = 0 ;

//		NUM6 = PitchPID();
	}
	else{
		NUM1 = 0 ; NUM2 = 0 ; NUM3 = 0 ; 
		NUM4 = 0 ; NUM5 = 0 ; NUM6 = 0 ;
	}
//	ecode7=KalmanFitter(ecode7);
//	printf("x:%d   cont_T:%d     real_v:%f \nexcept_I:%f   except_v:%f   meas_v:%f\n\n" , ecode6 , Pitch_cont_T , Pitch_v_real, Pitch_I_except, Pitch_v_except, Pitch_v_feedback[1]);	
//	printf("%d\n" ,(int)Pitch_v_feedback[1]);	
//	OutPut_Data();//����ʾ����

}
//��̨Pitch PID
//int PitchPID(int yecode , int yexcept , int ynum){
//	float ynum_temp,ynum_lost=0;
//	float PID_P = 5  ,PID_I = 0, PID_D = 50 ;
//	least = yexcept - yecode ;              //��ƫ��
//	Integral_least += least ;
//	if(1){ PID_I = 0 , Integral_least=0 ,PID_P = 3 ,PID_D = 50;}	//���ַ���,������
//	else{ PID_I = 0.01, PID_P = 4 , PID_D = 100 ;}
//	ynum_temp = least*PID_P  + (float)PID_I*Integral_least + (float)PID_D*(least - Last_least);
//	
//	if(1)ynum_lost=0.8*(ynum+ynum_lost)+0.2*ynum_temp;
//	else ynum_lost=0.7*(ynum+ynum_lost)+0.3*ynum_temp;
//	
//	Last_least = least ;
//	ynum=ynum_lost;
//	ynum_lost-=ynum;
//	
//	if(ynum > 3500) ynum = 3500 ;                 //����������ֵ
//	if(ynum < -3500) ynum = -3500 ;               //���������Сֵ
//	//printf("%d\n" , ynum);
//	return ynum ;
//}



//��̨Pitch PID
int PitchPID(){
	
	Pitch_v_P=200,Pitch_v_I=1,Pitch_v_D=10;
	
	Pitch_x_P=0.015,Pitch_x_I=0,Pitch_x_D=0;
	
		
	/*λ�û�*/
	Pitch_x_feedback[0]=Pitch_x_feedback[1];
	Pitch_x_feedback[1]=ecode5;   
//	Pitch_x_feedback[1]=KalmanFitter2(Pitch_x_feedback[1]);	//�������˲�
	Pitch_x_except=Pitch_x_control;
	
//	Pitch_x[1]=0.0f*Pitch_x[1]+1.0f*ecode6;   //һ���˲�
//	Pitch_x_except=0.0f*Pitch_x_except+1.0f*Pitch_x_control;
	
	
	Pitch_x_erro[0]=Pitch_x_erro[1];
	Pitch_x_erro[1]=Pitch_x_except-Pitch_x_feedback[1];
	
	Pitch_x_integral+=Pitch_x_erro[1];
	
	Pitch_x_diff=Pitch_x_erro[1]-Pitch_x_erro[0];
	
	Pitch_v_except=Pitch_x_P*Pitch_x_erro[1]+Pitch_x_I*Pitch_x_integral+Pitch_x_D*Pitch_x_diff;
			
/*************�����ٶȻ�*************/
//	Pitch_v_except=( 1024 - ch3 )/10.0f;
/************************************/

	/*�ٶȻ�*/
	Pitch_v_feedback[0]=Pitch_v_feedback[1];
	Pitch_v_feedback[1]=(Pitch_x_feedback[1]-Pitch_x_feedback[0]);
	if(Pitch_v_feedback[1]<-3000||Pitch_v_feedback[1]>3000)Pitch_v_feedback[1]=Pitch_v_feedback[0];
	Pitch_v_real=Pitch_v_feedback[1];
	Pitch_v_feedback[1]=KalmanFitter(Pitch_v_feedback[1]);//�������˲�
	
	Pitch_v_erro[0]=Pitch_v_erro[1];
	Pitch_v_erro[1]=Pitch_v_except-Pitch_v_feedback[1];
	
	Pitch_v_integral+=Pitch_v_erro[1];
	
	Pitch_v_diff=Pitch_v_erro[1]-Pitch_v_erro[0];
	
	Pitch_I_except=Pitch_v_P*Pitch_v_erro[1]+Pitch_v_I*Pitch_v_integral+Pitch_v_D*Pitch_v_diff+Pitch_I_lost;
	Pitch_I_control=(int)Pitch_I_except;
	Pitch_I_lost=Pitch_I_except-Pitch_I_control;
	

	if(Pitch_I_except > Pitch_I_Max) Pitch_I_except = Pitch_I_Max ; //���Ƶ���������ֵ
	if(Pitch_I_except < Pitch_I_Min) Pitch_I_except = Pitch_I_Min ;

	if(++Pitch_cont_T>10000)Pitch_cont_T=0;//����������,���ڵ��Թ۲�
	
//	Pitch_v[1]=KalmanFitter(Pitch_v[1]);
//	printf("x:%d   cont_T:%d     real_v:%f \nexcept_I:%f   except_v:%f   meas_v:%f\n\n" , ecode6 , Pitch_cont_T , Pitch_v_real, Pitch_I_except, Pitch_v_except, Pitch_v[1]_feedback);	
	
//	Pitch_I_control=KalmanFitter2(Pitch_I_control);//��������˲�
	return Pitch_I_control;		
}


//����PID
int SpeedPID(int encoder , int except , int electric , int which){
	float PID_P = 0.02   , PID_D = 1 ;
	float least = 0  , new_electric = 0 ;
	least = except - encoder ;
	new_electric = electric + least*PID_P + PID_D*(least - Speed_Last_Least[which]) ;
	Speed_Last_Least[which] = least ;
	if(new_electric > 3500) new_electric = 3500 ;
	if(new_electric < -3500) new_electric = -3500 ;
	return new_electric;
}

//�������˲�
#define SysNoiseCoVar         (0.0123)							//ϵͳ����Э����
#define MeasNoiseCoVar        (16.7415926)          //��������Э����
//SysNoiseCoVarԽ�󣬾���Խ�ͣ��ٶ�Խ��MeasNoiseCovarԽ�󣬾���Խ�ߣ��ٶ�Խ��
float KalmanFitter(float MeasVar)
{
    static double	AdjustVar1=0;
    static double PreOptimalVar=1;                      //��ǰ����ֵ
    static double CurMeasVar=0,CurEstimateVar=0;        //��ǰ����ֵ����ǰԤ��ֵ
    static double CurOptimalVar=0;                      //��ǰ����ֵ
    static double CurSysCoVar=0,PreSysCoVar=30;         //��ǰϵͳЭ�����ǰϵͳЭ����
    static double KalmanGain=0;                         //����������
    CurMeasVar=MeasVar;                         				//��ǰϵͳ����ֵ
    //��ǰϵͳ����ֵ����ǰϵͳЭ������Ҫ���÷����ʼֵ��������
   
    /*��ǰ����ֵ=��ǰ����ֵ+����ֵ*/
    CurEstimateVar=PreOptimalVar+AdjustVar1;
    AdjustVar1=0;                                //�趨����ֵ��ԭ���Ǽӿ�����ٶ�
  
    /*��ǰϵͳЭ����=��ǰϵͳЭ����+ϵͳ����Э����*/
    CurSysCoVar=PreSysCoVar+SysNoiseCoVar;
   
    /*����������=��ǰϵͳЭ����/(��ǰϵͳЭ����+��������Э����)*/
    KalmanGain=CurSysCoVar/(CurSysCoVar+MeasNoiseCoVar);
   
    /*��ǰϵͳ����ֵ=��ǰϵͳ����ֵ+����������*������ֵ-��ǰϵͳ����ֵ��*/
    CurOptimalVar=CurEstimateVar+KalmanGain*(CurMeasVar-CurEstimateVar);
   
    /*��ǰϵͳ����Э����=��1-���������棩x��ǰϵͳЭ����*/
    PreSysCoVar =(1-KalmanGain)*CurSysCoVar;
   
		/*��ǰϵͳ����ֵ=��ǰϵͳ����ֵ*/
    PreOptimalVar=CurOptimalVar;		
    
    /*��������*/
    return (float)CurOptimalVar;
}

#define SysNoiseCoVar2         (0.0123)
#define MeasNoiseCoVar2        (0.01415926) 		//ϵͳ����Э�����������Э����
//SysNoiseCoVarԽ�󣬾���Խ�ͣ��ٶ�Խ��MeasNoiseCovarԽ�󣬾���Խ�ߣ��ٶ�Խ��
float KalmanFitter2(float MeasVar)
{
    static double	AdjustVar1=0;
    static double PreOptimalVar=1;                      //��ǰ����ֵ
    static double CurMeasVar=0,CurEstimateVar=0;        //��ǰ����ֵ����ǰԤ��ֵ
    static double CurOptimalVar=0;                      //��ǰ����ֵ
    static double CurSysCoVar=0,PreSysCoVar=30;         //��ǰϵͳЭ�����ǰϵͳЭ����
    static double KalmanGain=0;                         //����������
    CurMeasVar=MeasVar;                         				//��ǰϵͳ����ֵ
    //��ǰϵͳ����ֵ����ǰϵͳЭ������Ҫ���÷����ʼֵ��������
   
    /*��ǰ����ֵ=��ǰ����ֵ+����ֵ*/
    CurEstimateVar=PreOptimalVar+AdjustVar1;
    AdjustVar1=0;                                //�趨����ֵ��ԭ���Ǽӿ�����ٶ�
  
    /*��ǰϵͳЭ����=��ǰϵͳЭ����+ϵͳ����Э����*/
    CurSysCoVar=PreSysCoVar+SysNoiseCoVar2;
   
    /*����������=��ǰϵͳЭ����/(��ǰϵͳЭ����+��������Э����)*/
    KalmanGain=CurSysCoVar/(CurSysCoVar+MeasNoiseCoVar2);
   
    /*��ǰϵͳ����ֵ=��ǰϵͳ����ֵ+����������*������ֵ-��ǰϵͳ����ֵ��*/
    CurOptimalVar=CurEstimateVar+KalmanGain*(CurMeasVar-CurEstimateVar);
   
    /*��ǰϵͳ����Э����=��1-���������棩x��ǰϵͳЭ����*/
    PreSysCoVar =(1-KalmanGain)*CurSysCoVar;
   
		/*��ǰϵͳ����ֵ=��ǰϵͳ����ֵ*/
    PreOptimalVar=CurOptimalVar;		
    
    /*��������*/
    return (float)CurOptimalVar;
}


/****����ʾ����***/
//float OutData[4]; 
//unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
//{
//    unsigned short CRC_Temp;
//    unsigned char i,j;
//    CRC_Temp = 0xffff;

//    for (i=0;i<CRC_CNT; i++){      
//        CRC_Temp ^= Buf[i];
//        for (j=0;j<8;j++) {
//            if (CRC_Temp & 0x01)
//                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
//            else
//                CRC_Temp = CRC_Temp >> 1;
//        }
//    }
//    return(CRC_Temp);
//}
//void OutPut_Data()
//{
//  int temp[4] = {0};
//  unsigned int temp1[4] = {0};
//  unsigned char databuf[10] = {0};
//  unsigned char i;
//  unsigned short CRC16 = 0;
//	temp[0]=(int)Pitch_v_feedback[1];
//	temp1[0]=(unsigned int)Pitch_v_feedback[1];
//  for(i=0;i<4;i++)
//   {
//    
//    temp[i]  = (short)OutData[i];
//    temp1[i] = (unsigned short)temp[i];
//    
//   }
//   
//  for(i=0;i<4;i++) 
//  {
//    databuf[i*2]   = (unsigned char)(temp1[i]%256);
//    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
//  }
//  
//  CRC16 = CRC_CHECK(databuf,8);
//  databuf[8] = CRC16%256;
//  databuf[9] = CRC16/256;
//  
//  for(i=0;i<10;i++)
//  printf("%c",(char)databuf[i]); 
//}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
 
}

/* USER CODE BEGIN 4 */
void tim121_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  
}
void tim122_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);  
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
