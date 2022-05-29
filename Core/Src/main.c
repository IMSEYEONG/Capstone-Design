/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "dataType.h"  // pc와의 통신을 위한 구조체 데이터형 저장되어있음
#include "track_data.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

  #define pi 3.141592653589793238462643383279502884197169399375105820974944
  #define Ra  0.218                           // [ohm]          motor resister
  #define La 0.000000001526            // 1.526[nH ]   motor inductance
  #define Kt  0.05311583                     // [Nm/A]        motor torq constant
  #define Ke  0.05311583                     // [V/rad/s]    motor back EMF constant
  #define RX_BUFFER_SIZE 50
  #define TX_BUFFER_SIZE 100
  #define dt_curr 0.001                     //     1ms        current control period 
  #define dt_vel 0.005                         //      5ms        speed control period
  #define dt_pos 0.05                      //     50ms        system deg control period
  #define RAD2DEG 57.295779
  #define DEG2RAD 0.0174532

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

  volatile double voltage = 0;    // test variable    

  volatile uint8_t mode = 0;        // system working mode

  int fcc = 1000; //1ms, 1k[Hz]  

  volatile uint32_t cnt_ctrl = 1; // to make each control period;
  volatile uint32_t cnt_start = 0;
  volatile int button_cnt = -1;
  volatile int flag = 0;
  volatile double ctrl_ref =0;

 
  
  /////current sensor variable/////
  volatile uint16_t ADC_curr1[2];
  double V2A_value1=15.7;
  volatile double ADC_curr1_filtered=0;
  volatile double ADC_curr1_pre=0;
  volatile double curr_cur_bp=0;
  volatile double curr_cur_real_bp=0;
  
  volatile uint16_t ADC_curr2[2];
  double V2A_value2=15.7;
  volatile double ADC_curr2_filtered=0;
  volatile double ADC_curr2_pre=0;
  volatile double curr_cur_mp=0;
  
  /////current control/////
  double curr_limit = 12;      // [A]
  double wcc = 6283.1853 ;        //2 * pi * fcc;
  //bp
  volatile double curr_ref_bp = 0;
  volatile double curr_err_bp = 0;
  volatile double curr_err_sum_bp = 0;
  volatile double curr_ctrl_out_bp = 0;
  volatile double curr_estimated_bp = 0;
  
 // volatile double Kpc_bp = 0;
 // volatile double Kic_bp = 0;
  
  volatile double Kpc_bp = 0.3;
  volatile double Kic_bp = 17;
  //mp    
  
  double curr_limit_mp_up = 0.5;      // [A]
  double curr_limit_mp_down = 4;      // [A]
  volatile double curr_ref_mp = 0;
  volatile double curr_err_mp = 0;
  volatile double curr_err_sum_mp = 0;
  volatile double curr_ctrl_out_mp = 0;
  
  volatile double Kpc_mp = 0.3;
  volatile double Kic_mp = 17;
  
 
  /////encoder variable & speed control////
    
  volatile uint16_t Motor_ccr_bp=0;
  volatile uint16_t Motor_ccr_mp=0;
  
  volatile uint16_t encoder_cnt_bp = 0;
  volatile uint16_t encoder_cnt_pre_bp = 0;
  volatile int32_t encoder_err_bp =0;
  volatile double angle_bp = 0;
  volatile double angle_pre_bp = 0;
  volatile double omega_bp = 0;
  
  volatile uint16_t encoder_cnt_mp = 0;
  volatile uint16_t encoder_cnt_pre_mp = 0;
  volatile int32_t encoder_err_mp =0;
  volatile double angle_mp = 0;
  volatile double angle_pre_mp = 0;
  volatile double omega_mp = 0;
  
  /////speed control/////
  double vel_limit = 15.70796;        //[rad/s]
  volatile double vel_ref_bp = 0;
  volatile double vel_cur_bp = 0;
  volatile double vel_err_bp = 0;
  volatile double vel_err_sum_bp = 0;
  volatile double vel_ctrl_out_bp = 0;
  volatile double Kps_bp = 3;
  volatile double Kis_bp = 0.87;
  
  volatile double vel_ref_mp = 0;
  volatile double vel_cur_mp = 0;
  volatile double vel_err_mp = 0;
  volatile double vel_err_sum_mp = 0;
  volatile double vel_ctrl_out_mp = 0;
  volatile double Kps_mp = 15;
  volatile double Kis_mp = 0.1;
  
  /////Distuebance Observer speed control/////
  volatile double Jm = 0.01;                          // nominal inertia of bp motor
  volatile double tau_DOB = 0.001;                   //  time constant of Q-filter          // 기본적으로0.001
  volatile double w_cur_DOB = 0;                      //  w = predicted input+predicted disturb
  volatile double w_pre_DOB = 0;
  volatile double y_cur_DOB = 0;                       // output (omega) 
  volatile double y_pre_DOB = 0;
  volatile double u_cur_DOB = 0;                       // input (current)
  volatile double u_filtered_cur_DOB =0;
  volatile double u_filtered_pre_DOB = 0;
  volatile double d_DOB = 0;                              // predicted disturbance    d = w-u
  
    /////position control/////
  volatile double pos_ref_bp = -4.42;
  volatile double pos_cur_bp = 0;
  volatile double pos_err_cur_bp = 0;
  volatile double pos_err_pre_bp = 0;
  volatile double pos_err_dif_bp = 0;
  volatile double pos_ctrl_out_bp = 0;
  volatile double Kpp_bp = 2.5;
  volatile double Kdp_bp = 0.1;
  
  /////IMU variable/////
  char buffer[50];
  uint8_t tx_buffer[] = "*";
  uint8_t rx_buffer[RX_BUFFER_SIZE];
  uint8_t tx_len;
  char imu_buffer[50];
  float DegPitch;
  int checkdeg=0;
    
  /////variable of communication with PC/////
  volatile int32_t track_point = 0;
  volatile double track_vel = 0;
  volatile double track_deg = 0;
  
  volatile Packet_t packet;
  uint8_t tx_buffer_uart3[TX_BUFFER_SIZE];
  uint8_t tx_len_uart3;
  uint8_t tx_buffer_enter[] = "          ";
  
  /*
  ///// IIR LPF Parameter filter order=2, Fc=5/////
  volatile double b_iir[3]={0.067455273889072,0.134910547778144,0.067455273889072};                             // iir b계수
  volatile double a_iir[3]={1.0,-1.142980502539901,0.412801598096189};                  // iir a계수
  volatile double IIR_input[3]={0,};                                                    // 이전adc값 저장
  volatile double IIR[3]={0,};                                                          // 출력값 저장
  volatile uint32_t ADC_curr1_IIR = 0; 
  */
  
  ///////////////MAF////////////
  //bp
  volatile double MAF_sample_bp[20]={0,};      // MAF 계산을 위해 센서값을 저장할 샘플 배열변수 선언: Sampling 갯수 10개
  volatile int m_bp=0;                     // MAF Sampling 센서값 index 변수 (최대 10번 counting)
  volatile double MAF_bp=0;                  // sampling 한 센서값들을 누산하여 저장하는 변수
  volatile double MAF_filtered_bp = 0; 
  
  //mp
  volatile double MAF_sample_mp[20]={0,};      // MAF 계산을 위해 센서값을 저장할 샘플 배열변수 선언: Sampling 갯수 10개
  volatile int m_mp=0;                     // MAF Sampling 센서값 index 변수 (최대 10번 counting)
  volatile double MAF_mp=0;                  // sampling 한 센서값들을 누산하여 저장하는 변수
  volatile double MAF_filtered_mp = 0; 
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
double IIR_LPF(double data)
{
   double iir_save=0;  // 이전adc값 저장
   int j=0;
   int k=0;
   IIR_input[0] = data;
   IIR[0] = 0;
   
   for(j=0;j<3;j++) IIR[0] += b_iir[j]*IIR_input[j];  // IIR 이전 ADC값에 b계수 곱하여 더해주기
   for(k=1;k<3;k++) IIR[0] -= a_iir[k]*IIR[k];        // IIR 이전 출력값에 a계수 곱하여 빼주기
    
   iir_save=IIR[1];                /////////////////////////
   IIR[1]=IIR[0];                  /////////////////////////
   IIR[2]=iir_save;                //이전adc값과 출력값 저장//
   iir_save=IIR_input[1];          /////////////////////////
   IIR_input[1]=IIR_input[0];      /////////////////////////
   IIR_input[2]=iir_save;          /////////////////////////
   
   return IIR[0];
}
*/

double Moving_Average_Filter_bp(double data)
{
   MAF_bp = 0;                  // 표본들을 누산하는 변수 초기화
   MAF_sample_bp[m_bp] = data;      // Sampling 값을 변수에 순서대로 저장
   int j=0;
   for(j=0;j<20;j++)
   {
      MAF_bp += MAF_sample_bp[j];      // MAF_sample 배열변수에 저장된 Sample을 누산
   }
   if (m_bp==19) m_bp=0;            // 이 작업을  20번 반복하면 다시 m=0으로 초기화
   else m_bp++;               //  왜?: sampling 갯수를 10개로 잡았기 때문에
   
   return (MAF_bp/20);            // 최종 MAF 적용값 반환
}

double Moving_Average_Filter_mp(double data)
{
   MAF_mp = 0;                  // 표본들을 누산하는 변수 초기화
   MAF_sample_mp[m_mp] = data;      // Sampling 값을 변수에 순서대로 저장
   int k=0;
   for(k=0;k<20;k++)
   {
      MAF_mp += MAF_sample_mp[k];      // MAF_sample 배열변수에 저장된 Sample을 누산
   }
   if (m_mp==19) m_mp=0;            // 이 작업을  20번 반복하면 다시 m=0으로 초기화
   else m_mp++;               //  왜?: sampling 갯수를 10개로 잡았기 때문에
   
   return (MAF_mp/20);            // 최종 MAF 적용값 반환
}

///////////////// Function To use IMU ///////////////////////

int FindComma(char * buffer)
{
   int n;
   for(n=0;n<100;n++)
    {
     if(buffer[n]==',') break;
    }

   return n;
}

void deg_find()  
{
  for(int i=0;i<50;i++)
  {
    imu_buffer[i]=rx_buffer[i];
  }
  
  int value,value2;
 
   value=FindComma(imu_buffer);
   imu_buffer[value]='\0';

   value++;
   value2=FindComma(&imu_buffer[value]);
   imu_buffer[value+value2]='\0';
   DegPitch=atof(&imu_buffer[value]);    
}
/////////////////////////////////////////////////////////

////////////////////make pwm/////////////////////////
void Make_PWM_bp(double input)
{  
      curr_estimated_bp = (input-(omega_bp*20*Ke))/Ra;
          
      if(input < 0)
      {
            if(curr_cur_bp<-12) input = (-12*Ra)+(omega_bp*20*Ke) ;
            Motor_ccr_bp = (double)((-input/24)*8999.);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
            TIM3->CCR1 =  Motor_ccr_bp;
      }
      else
      {
            if(curr_cur_bp>12) input = (12*Ra)+(omega_bp*20*Ke) ;
            Motor_ccr_bp = (double)((input/24)*8999.);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
            TIM3->CCR1 =  Motor_ccr_bp;
      }
}

void Make_PWM_mp(double input)
{  
          
      if(input < 0)
      {
            if(curr_cur_mp<-12) input = (-12*Ra)+(omega_mp*10*Ke) ;
            Motor_ccr_mp = (double)((-input/24)*8999.);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
            TIM4->CCR1 =  Motor_ccr_mp;
      }
      else
      {
            if(curr_cur_mp>12) input = (12*Ra)+(omega_mp*10*Ke) ;
            Motor_ccr_mp = (double)((input/24)*8999.);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
            TIM4->CCR1 =  Motor_ccr_mp;
      }
}
/////////////////////////////////////////////////////////

///////////////button interrupt test/////////////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if(GPIO_Pin==GPIO_PIN_13)
   {
     HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
     if(mode == 0) mode = 1;
     //else if(mode == 1) mode = 0;
     
            if(button_cnt >2) pos_ref_bp = 0;
              
            button_cnt++;
            if(button_cnt > 1 && flag==0){
            ctrl_ref += 1;
            if(ctrl_ref == 2){
              flag = 1;
            }
          }
            else if(button_cnt > 1 && flag == 1){
                ctrl_ref -= 1;
                if(ctrl_ref == -2){
                  flag = 0;
                }
            }
   
   }
 
}
     

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////control period////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim == &htim9 && mode == 1)//20000Hz period
  {
    cnt_ctrl++; // control period count v
    
    /////////control period start//////////
    
    
    //////////////////get current////////////////////////////
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_curr1,1);   // To get adc1  20000Hz continous mode:disable, Discon : enable
    HAL_ADC_Start_DMA(&hadc2,(uint32_t *)ADC_curr2,1);   // 센서로 들어오는 전압이 3.3을 넘으면 같은 AD컨버터에 있는 채널에 노이즈 낌. 따라서  ADC분리
    
    /////ADC1 _mp filter/////
    //ADC_curr1_filtered = (0.04)*ADC_curr1[0] + (0.96)*ADC_curr1_pre;
    //ADC_curr1_pre = ADC_curr1_filtered;
    ADC_curr1_filtered = Moving_Average_Filter_mp((double)ADC_curr1[0]);
    curr_cur_mp = ((ADC_curr1_filtered*3.3 / 4096.) -2.4968197) * V2A_value1;
    
   //ADC_curr1_IIR = IIR_LPF(ADC_curr2[0]);
    
   /////ADC2 _bp filter/////
    
    //ADC_curr2_filtered = (0.04)*(double)ADC_curr2[0] + (0.96) * ADC_curr2_pre;
    //ADC_curr2_pre = ADC_curr2_filtered;
    ADC_curr2_filtered = Moving_Average_Filter_bp((double)ADC_curr2[0]);
    curr_cur_bp = ((ADC_curr2_filtered*3.3 / 4096.) - 2.4968197) *  V2A_value2;
    
    curr_cur_real_bp  =  (((double)ADC_curr2[0]*3.3 / 4096.) - 2.503198 )*  V2A_value2;
    
    ///////////////////////////////////////////////////////// 
    
    /////////////////get encoder&IMU//////////////////////////  값 튀는것으로 인하여 초기에 미리 받기
    //////////get speed///////// 
        if(cnt_ctrl% 200 == 0&&cnt_start<10000) // 10ms  
    {
      angle_pre_bp = angle_bp;
      encoder_cnt_pre_bp = encoder_cnt_bp;
      angle_pre_mp = angle_mp;
      encoder_cnt_pre_mp = encoder_cnt_mp;
      /////get omega/////
      encoder_cnt_bp = TIM1->CNT;
      encoder_cnt_mp = TIM2->CNT;
      
    
      encoder_err_bp = encoder_cnt_bp - encoder_cnt_pre_bp;
      encoder_err_mp = encoder_cnt_mp - encoder_cnt_pre_mp;

      
      //encoder exception handling
     if(encoder_cnt_bp>65000 && encoder_cnt_pre_bp< 500)  encoder_err_bp -= 65535 ;
     else if(encoder_cnt_bp< 500 && encoder_cnt_pre_bp > 65000)  encoder_err_bp += 65535 ;
     
      angle_bp += (encoder_err_bp/ 20000.) * 2 * pi;
      omega_bp = (angle_bp - angle_pre_bp) / dt_vel;
      
      if(encoder_cnt_mp>65000 && encoder_cnt_pre_mp< 500)  encoder_err_mp -= 65535 ;
     else if(encoder_cnt_mp< 500 && encoder_cnt_pre_mp > 65000)  encoder_err_mp += 65535 ;
     
      angle_mp += (encoder_err_mp/ 20000.) * 2 * pi;
      omega_mp = (angle_mp - angle_pre_mp) / dt_vel;
      
      
     ///////////get deg //////////
      HAL_UART_Transmit(&huart6, tx_buffer, 1, HAL_MAX_DELAY);
      HAL_UART_Receive_DMA(&huart6, rx_buffer, RX_BUFFER_SIZE);
      deg_find();
    }
    
    ////////////////////////////////////////////////////////
    
  if(cnt_start > 10000)  //센서의 필터등에서 초기값을 안정시키기위해 0.5s이후 시작
  {
    //////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////stm tp PC /////////////////////////////////////////////////
                                 ///////////serial packet///////////////////
        if(cnt_ctrl% 2000 == 0) // 100ms  stm tpoPC data sending period
    {
      packet.data.size = sizeof(Packet_data_t);		        // size = 20
      packet.data.id = 1;							        // g_ID = 1
      packet.data.mode = 3;							// mode = 3
      packet.data.check = 0;							// checksum 저장할 곳을 0으로 초기화
      
      track_point = 1000;
      track_vel = 1.111;
      track_deg = 2.222;
      
      packet.data.t_point = track_point;
      packet.data.t_vel = track_vel * 1000;
      packet.data.t_deg = track_deg * 1000;
      
      
      for (int i = 8; i < sizeof(Packet_t); i++)
      packet.data.check += packet.buffer[i];			     // checksum 제작: 데이터를 다 더하고 저장
      
      
      for(int i =0; i<packet.data.size;i++)                                //Data save in transmit buffer
      {		
        tx_buffer_uart3[i] = packet.buffer[i]+'0';
      }
      
      HAL_UART_Transmit(&huart3, tx_buffer_uart3, packet.data.size,10);  //transmit to PC
      
      
      
      cnt_ctrl=0; // control period count variable reset
    }
    
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////// positon control /////////////////////////////////////
    
    if(cnt_ctrl% 400 == 0) // 20ms  period
    {
        /////////////Get MP Degree/////////////////////////////////
      HAL_UART_Transmit(&huart6, tx_buffer, 1, HAL_MAX_DELAY);                // transmit '*' to IMU for get deg data
      HAL_UART_Receive_DMA(&huart6, rx_buffer, RX_BUFFER_SIZE);             // get data from IMU
      deg_find();                                                                                      // data processing for get deg
      
    }
    
    if(cnt_ctrl% 1000 == 0) // 50ms control period
    {
      ////////////////////////////////////////////////////////
      pos_err_pre_bp = pos_err_cur_bp;
      //pos_ref_bp =  ctrl_ref ; 
      pos_cur_bp = DegPitch;
      pos_err_cur_bp = pos_ref_bp - pos_cur_bp;
      pos_err_dif_bp = (pos_err_cur_bp-pos_err_pre_bp)/dt_pos ;
      
      pos_ctrl_out_bp = Kpp_bp * pos_err_cur_bp + Kdp_bp * pos_err_dif_bp ; 

    }
    
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    
    
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////// speed control ///////////////////////////////////////  200Hz
    
    if(cnt_ctrl% 100 == 0) // 5ms control period
    {
      
        angle_pre_bp = angle_bp;
        encoder_cnt_pre_bp = encoder_cnt_bp;
        angle_pre_mp = angle_mp;
        encoder_cnt_pre_mp = encoder_cnt_mp;
        /////get omega/////
        encoder_cnt_bp = TIM1->CNT;
        encoder_cnt_mp = TIM2->CNT;
        
      
        encoder_err_bp = encoder_cnt_bp - encoder_cnt_pre_bp;
        encoder_err_mp = encoder_cnt_mp - encoder_cnt_pre_mp;

        
        //encoder exception handling
       if(encoder_cnt_bp>65000 && encoder_cnt_pre_bp< 500)  encoder_err_bp -= 65535 ;
       else if(encoder_cnt_bp< 500 && encoder_cnt_pre_bp > 65000)  encoder_err_bp += 65535 ;
       
        angle_bp += (encoder_err_bp/ 20000.) * 2 * pi;
        omega_bp = (angle_bp - angle_pre_bp) / dt_vel;
        
        if(encoder_cnt_mp>65000 && encoder_cnt_pre_mp< 500)  encoder_err_mp -= 65535 ;
       else if(encoder_cnt_mp< 500 && encoder_cnt_pre_mp > 65000)  encoder_err_mp += 65535 ;
       
        angle_mp += (encoder_err_mp/ 20000.) * 2 * pi;
        omega_mp = (angle_mp - angle_pre_mp) / dt_vel;
        
      
        
        //vel ctrl_bp
        vel_cur_bp = omega_bp;
        vel_ref_bp =  pos_ctrl_out_bp;
        
        /////speed ctrl test///
        //vel_ref_bp =  ctrl_ref;
        
        //speed limit
        if(vel_ref_bp > vel_limit) vel_ref_bp = vel_limit;
        else if(vel_ref_bp < -vel_limit) vel_ref_bp = -vel_limit;
        
        
        vel_err_bp = vel_ref_bp -  vel_cur_bp;
        vel_err_sum_bp += vel_err_bp;
        vel_ctrl_out_bp = vel_err_bp * Kps_bp + Kis_bp * vel_err_sum_bp *dt_vel ;  
       
        
       ////////////////////////////////////////////////////////
       ///////////////////////DOB////////////////////////////
       y_cur_DOB = vel_cur_bp;
       u_cur_DOB = vel_ctrl_out_bp;
        
       w_cur_DOB = (Jm*y_cur_DOB-Jm*y_pre_DOB+tau_DOB*w_pre_DOB)/(tau_DOB+dt_vel); 
       u_filtered_cur_DOB = (dt_vel*u_cur_DOB+tau_DOB*u_filtered_pre_DOB)/(tau_DOB+dt_vel);  
       
       d_DOB = w_cur_DOB - u_filtered_cur_DOB;
       
       w_pre_DOB = w_cur_DOB;
       y_pre_DOB = y_cur_DOB;
       u_filtered_pre_DOB = u_filtered_cur_DOB;
        
       vel_ctrl_out_bp -= d_DOB;
          
       // current saturation & anti wind-up
       if(vel_ctrl_out_bp > curr_limit) 
       {
              vel_err_sum_bp  -= (vel_ctrl_out_bp - curr_limit) * (3./Kps_bp);  // anti wind-up
              vel_ctrl_out_bp = curr_limit;
       }
       else if(vel_ctrl_out_bp < -curr_limit)
       {
              vel_ctrl_out_bp -= (vel_ctrl_out_bp + curr_limit) * (3./Kps_bp);
              vel_ctrl_out_bp = -curr_limit;
       }
       
             //vel ctrl_mp
        vel_cur_mp = omega_mp;
        
        /////speed ctrl test///
        //vel_ref_mp =  ctrl_ref;
        
        //speed limit
        if(vel_ref_mp > vel_limit) vel_ref_mp = vel_limit;
        else if(vel_ref_mp < -vel_limit) vel_ref_mp = -vel_limit;
        
        
        vel_err_mp = vel_ref_mp -  vel_cur_mp;
        vel_err_sum_mp += vel_err_mp;
        vel_ctrl_out_mp = -(vel_err_mp * Kps_mp + Kis_mp * vel_err_sum_mp *dt_vel ); 
        
        // current saturation & anti wind-up
       if(vel_ctrl_out_mp > curr_limit_mp_up) 
       {
              vel_err_sum_mp  -= (vel_ctrl_out_mp - curr_limit_mp_up) * (3./Kps_mp);  // anti wind-up
              vel_ctrl_out_mp = curr_limit_mp_up;
       }
       else if(vel_ctrl_out_mp < -curr_limit_mp_down)
       {
              vel_ctrl_out_mp -= (vel_ctrl_out_mp +curr_limit_mp_down) * (3./Kps_mp);
              vel_ctrl_out_mp = -curr_limit_mp_down;
       }
     
    }
    
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    
     
      /////////////////////////////////////////////////////////////////////////////
      /////////////////////// torque control ///////////////////////////////////////    1kHz
       if(cnt_ctrl% 20 == 0) 
      {
     
        ///////test bp ref//////////
         //curr_ref_bp = ctrl_ref ;
       
         ////bp torq ctrl /////
           
         curr_ref_bp = vel_ctrl_out_bp ;
        
          curr_err_bp = curr_ref_bp -  curr_cur_bp;
          curr_err_sum_bp += curr_err_bp;
          curr_ctrl_out_bp = curr_err_bp * Kpc_bp + Kic_bp*curr_err_sum_bp * dt_curr ;
          
          curr_ctrl_out_bp += Ke*omega_bp ;
        
         // voltage saturation & anti wind-up
          
         if(curr_ctrl_out_bp > 24.) 
         {
                curr_err_sum_bp  -= (curr_ctrl_out_bp - 24.) * (3./Kpc_bp);  // anti wind-up
                curr_ctrl_out_bp = 24.;
         }
         else if(curr_ctrl_out_bp < -24.)
         {
                curr_err_sum_bp -= (curr_ctrl_out_bp + 24.) * (3./Kpc_bp);
                curr_ctrl_out_bp = -24.;
         }
         
         //////  input voltage bp
         
        Make_PWM_bp(curr_ctrl_out_bp);     
       // Make_PWM_bp(voltage);
         
         
      ///////test mp ref//////////
        // curr_ref_mp = ctrl_ref ;
       
         ////mp torq ctrl /////
           
          //curr_ref_mp = vel_ctrl_out_mp ;
        
          curr_err_mp = curr_ref_mp -  curr_cur_mp;
          curr_err_sum_mp += curr_err_mp;
          curr_ctrl_out_mp = curr_err_mp * Kpc_mp + Kic_mp*curr_err_sum_mp * dt_curr ;
          
          //curr_ctrl_out_mp += Ke*omega_mp ;
        
         // voltage saturation & anti wind-up
          
         if(curr_ctrl_out_mp > 24.) 
         {
                curr_err_sum_mp  -= (curr_ctrl_out_mp - 24.) * (3./Kpc_mp);  // anti wind-up
                curr_ctrl_out_mp = 24.;
         }
         else if(curr_ctrl_out_bp < -24.)
         {
                curr_err_sum_mp -= (curr_ctrl_out_mp + 24.) * (3./Kpc_mp);
                curr_ctrl_out_mp = -24.;
         }
         
         
         //////  input voltage mp
         Make_PWM_mp(curr_ctrl_out_mp);
         //Make_PWM_mp(voltage);
         
         
         
         cnt_start = 50000 ;  //////////////fix cnt_start(cnt for start delay to stabilizr initial sensor value)
       }
     }  
    cnt_start++;
     //////////////////////
 
    
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    
    
    
    /////////control period end////////// 
  }
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
  packet.data.header[0] = packet.data.header[1] = packet.data.header[2] = packet.data.header[3] = 0xFE;  	// 패킷을 보낼 때 맨 앞 header 데이터 설정
			
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_UART_Receive_DMA(&huart6,rx_buffer,RX_BUFFER_SIZE);
  
  
  //HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_curr1,1);
  //HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_curr2,1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    cnt++;
    cnt1--;
    cnt3++;
    HAL_Delay(50);

    if(cnt==10000) cnt = 0;
    */

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 179;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 49;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
