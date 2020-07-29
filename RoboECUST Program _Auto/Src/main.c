/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define Light_Sensor_1  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)
#define Light_Sensor_2  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)
#define Light_Sensor_3  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)
#define Light_Sensor_4  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)
#define Switch_1        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)
#define Switch_2        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)
#define Switch_3        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define Switch_4        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)
#define Switch_5        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)
#define Switch_6        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)
#define Switch_7        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
#define Distance_Front  UltrasonicDistance[0]
#define Distance_Back   UltrasonicDistance[1]
#define Distance_Left   UltrasonicDistance[2]
#define Distance_Right  UltrasonicDistance[3]
#define Light_RED(x)    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,x);
#define Light_YELLOW(x) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,x);
#define Light_GREEN(x)  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,x);
#define Buzzer(x)       HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,x);
#define PID0123()\
    MotorPidFunction(Motor_PARM[0]);\
    MotorPidFunction(Motor_PARM[1]);\
    MotorPidFunction(Motor_PARM[2]);\
    MotorPidFunction(Motor_PARM[3])


typedef struct{
    uint8_t Motor_ID;
    int32_t etSum;
    int32_t lastErr;
    double kp;
    double ki;
    double kd;
    uint32_t PrevTime;
    int32_t SetPoint;
}Motor_PID_PARM;
typedef struct{
    uint8_t Motor_ID;
    int32_t etSum;
    int32_t lastErr;
    double kp;
    double ki;
    double kd;
    uint32_t PrevTime;
    int32_t SetPoint;
    int32_t PresAngle;
    int32_t LastAngle;
    int32_t AngleCount;
}Motor_PID_Angle_PARM;
typedef struct{
    int32_t RefVoltage;
    int32_t x;
    int32_t y;
    int32_t a;
    int32_t b;
    int32_t x1;
    int32_t y1;
    int32_t a1;
    int32_t b1;
    uint8_t* Button;
}JoystickTypeDef;
typedef struct{
    double ax;
    double ay;
    double az;
    double wx;
    double wy;
    double wz;
    double Roll;
    double Pitch;
    double Yaw;
    uint8_t state;
    uint8_t counter;
}MPU6050TypeDef;
typedef struct {
    uint32_t PrevTime;
    int32_t etSum;
    int32_t lastErr;
    double kp;
    double ki;
    double kd;
    int32_t Delta_SetPoint;                                           //增量输出
    int32_t SetAngle;
} Angle_Pid_Parm;

static uint32_t SYS_Time_us;
int32_t PowerVoltage;
uint32_t ADC_Value[6];

static uint8_t RxBuffer2[11];
static uint8_t RxBuffer6[2];
static int16_t MotorSpeed[8];
static int16_t MotorAngle[8];
static int16_t MotorTrueCurrent[8];
static int8_t  MotorTemperature[8];
static int16_t MotorCurrentValue[8] = {0};

static int8_t UltrasonicState[4] = {0};
static uint32_t UltrasonicUP[4] = {0};
static uint32_t UltrasonicDOWN[4] = {0};
static uint32_t UltrasonicTime[4] = {0};
static double UltrasonicDistance[4];
static int8_t UltrasonicTXsymbol = 0;
// 0:前 1:后 2:左 3:右

int8_t TraceState[40] = {0};

static Motor_PID_PARM* Motor_PARM[8] = {NULL};
static Motor_PID_Angle_PARM Motor_Angle_PARM;
static JoystickTypeDef Joystick;
static MPU6050TypeDef MPU6050;
static Angle_Pid_Parm Angle_PARM;

static int8_t Ball_Type;
static int8_t Ball_Counter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM5_Init(void);
static void MX_CAN1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void MotorDriver(int16_t MotorCurrentValue[], CanTxMsgTypeDef *TxMessage1);
void int2str(int16_t n, uint8_t *str);
void MotorPidFunction(Motor_PID_PARM* PID_PARM);
void MotorPidAngle(Motor_PID_Angle_PARM* PID_Angle_PARM);
void MotorAngleSTAT(Motor_PID_Angle_PARM* PID_Angle_PARM);
void Motor_CAN_Reveive(CAN_HandleTypeDef* hcan, uint8_t FIFONumber);
void JoystickGet();
void JoystickMapping();
void FlyToSky();
void UltrasonicGet();
void MPU6050Decode();
void AnglePidFunction();

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_CAN2_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM5_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */

    static CanRxMsgTypeDef RxMessage1;
    hcan1.pRxMsg = &RxMessage1;
    static CanTxMsgTypeDef TxMessage1;
    hcan1.pTxMsg = &TxMessage1;
    static CanTxMsgTypeDef TxMessage2;
    hcan2.pTxMsg = &TxMessage2;
    static CanRxMsgTypeDef RxMessage2;
    hcan2.pRxMsg = &RxMessage2;
    
    static uint8_t ButtonState[8];
    Joystick.Button = ButtonState;
    
    /*CAN过滤器初始化*/
    
    CAN_FilterConfTypeDef sFilterConfig;
    
    sFilterConfig.FilterNumber = 0;                         //使用过滤器1  
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;       //设为列表模式  
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;      //位宽设置为16位  
    sFilterConfig.FilterIdHigh     = 0x205<<5;              //4个标准CAN ID分别放入到4个存储中  
    sFilterConfig.FilterIdLow      = 0x206<<5;  
    sFilterConfig.FilterMaskIdHigh = 0x207<<5;  
    sFilterConfig.FilterMaskIdLow  = 0x208<<5;
    sFilterConfig.FilterFIFOAssignment = 0;                 //接收到的报文放入到FIFO0中  
    sFilterConfig.FilterActivation = ENABLE;  
    sFilterConfig.BankNumber = 15;  
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);           //设置过滤器
    
    sFilterConfig.FilterNumber = 3;                         //使用过滤器1  
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;       //设为列表模式  
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;      //位宽设置为16位  
    sFilterConfig.FilterIdHigh     = 0x201<<5;              //4个标准CAN ID分别放入到4个存储中  
    sFilterConfig.FilterIdLow      = 0x202<<5;  
    sFilterConfig.FilterMaskIdHigh = 0x203<<5;  
    sFilterConfig.FilterMaskIdLow  = 0x204<<5;
    sFilterConfig.FilterFIFOAssignment = 0;                 //接收到的报文放入到FIFO0中  
    sFilterConfig.FilterActivation = ENABLE;  
    sFilterConfig.BankNumber = 14;  
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);           //设置过滤器
    
    
    if(HAL_UART_Transmit_IT(&huart1, (uint8_t *)"USART1 is OK\r\n", 15) != HAL_OK)
        HAL_UART_Transmit(&huart1, (uint8_t *)&"ERROR\r\n", 7, 10);
    HAL_Delay(100);
    HAL_Delay(10);
    
    static Motor_PID_PARM Motor_0_PARM;
    Motor_PARM[0] = &Motor_0_PARM;
    static Motor_PID_PARM Motor_1_PARM;
    Motor_PARM[1] = &Motor_1_PARM;
    static Motor_PID_PARM Motor_2_PARM;
    Motor_PARM[2] = &Motor_2_PARM;
    static Motor_PID_PARM Motor_3_PARM;
    Motor_PARM[3] = &Motor_3_PARM;
    static Motor_PID_PARM Motor_4_PARM;
    Motor_PARM[4] = &Motor_4_PARM;
    
    Motor_PARM[0]->Motor_ID = 0;
    Motor_PARM[0]->kp = 2;
    Motor_PARM[0]->ki = 0.2;
    Motor_PARM[0]->kd = 1;
    Motor_PARM[0]->SetPoint = 0;
    Motor_PARM[1]->Motor_ID = 1;
    Motor_PARM[1]->kp = 2;
    Motor_PARM[1]->ki = 0.2;
    Motor_PARM[1]->kd = 1;
    Motor_PARM[1]->SetPoint = 0;
    Motor_PARM[2]->Motor_ID = 2;
    Motor_PARM[2]->kp = 2;
    Motor_PARM[2]->ki = 0.2;
    Motor_PARM[2]->kd = 1;
    Motor_PARM[2]->SetPoint = 0;
    Motor_PARM[3]->Motor_ID = 3;
    Motor_PARM[3]->kp = 2;
    Motor_PARM[3]->ki = 0.2;
    Motor_PARM[3]->kd = 1;
    Motor_PARM[3]->SetPoint = 0;
    Motor_PARM[4]->Motor_ID = 4;
    Motor_PARM[4]->kp = 2;
    Motor_PARM[4]->ki = 0.2;
    Motor_PARM[4]->kd = 0;
    Motor_PARM[4]->SetPoint = 0;
    
    Motor_Angle_PARM.Motor_ID = 4;
    Motor_Angle_PARM.kp = 0.3;
    Motor_Angle_PARM.ki = 0.0008;
    Motor_Angle_PARM.kd = 7;
    Motor_Angle_PARM.SetPoint = 0;
    
    Angle_PARM.kd = 0;
    Angle_PARM.ki = 0;
    Angle_PARM.kp = 50;
    
    int32_t Speed_1 = 3000;                                                  //  前进1初始速度
    int32_t Speed_2 = 1500;                                                  //  前进1减速速度
    int32_t Speed_3 = 0;                                             //  左转速度
    int32_t Speed_4 = 2000;                                                  //  前进2速度
    int32_t Speed_5 = 2000;                                             //  左平移速度
    int32_t Speed_6 = 0;                                               //  贴墙左转速度
    int32_t Speed_7 = 2000;                                               //  向右平移速度
    int32_t Speed_8 = 0;                                               //  贴墙右转速度
    int32_t Speed_9 = 2000;                                                  //  后退速度
    
    uint32_t Time_Mark;
    int32_t Symbol;
    
    //HAL_UART_Receive_IT(&huart1, (uint8_t*)RxBuffer1, 10);
    HAL_Delay(10);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim9);
    //HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
    //HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&ADC_Value, 6);
    //HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    //HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
    //HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
    //HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
    //HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
    HAL_Delay(10);
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,2000);
    HAL_UART_Receive_IT(&huart2, (uint8_t*)RxBuffer2, 11);
    HAL_UART_Receive_IT(&huart6, (uint8_t*)RxBuffer6, 2);
    
    Symbol = 0;
    HAL_Delay(3000);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while(1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        SYS_Time_us = __HAL_TIM_GetCounter(&htim5);
        //printf("T=%d\r\n",SYS_Time_us);
        HAL_UART_Receive_IT(&huart2, (uint8_t*)RxBuffer2, 11);
        //UltrasonicGet();
        Motor_CAN_Reveive(&hcan1, CAN_FIFO0);
        
        Motor_CAN_Reveive(&hcan1, CAN_FIFO0);
        Motor_CAN_Reveive(&hcan1, CAN_FIFO0);
        Motor_CAN_Reveive(&hcan1, CAN_FIFO0);
        Motor_CAN_Reveive(&hcan1, CAN_FIFO0);
        
        //printf("Symbol=%d\r\n",Symbol);
        //printf("1=%d,2=%d,3=%d,4=%d\r\n",Light_Sensor_1,Light_Sensor_2,Light_Sensor_3,Light_Sensor_4);
        //printf("1=%d,2=%d,3=%d,4=%d\r\n",Switch_1,Switch_2,Switch_3,Switch_4);
        /*if(MotorSpeed[0] > 8000||MotorSpeed[1] > 8000||MotorSpeed[2] > 8000||MotorSpeed[3] > 8000){
            Symbol = -1;
        }*/
        
        /*if(MotorSpeed[4] > 6000){
            Symbol = -1;
        }*/
        
        /*for(int i = 0; i<5 ;i++){
            if(MotorTrueCurrent[i] > 12000){
                Symbol = -1;
            }
        }*/
        if(Symbol == -1){
            Motor_PARM[0]->SetPoint = 0;
            Motor_PARM[1]->SetPoint = 0;
            Motor_PARM[2]->SetPoint = 0;
            Motor_PARM[3]->SetPoint = 0;
            Motor_PARM[4]->SetPoint = 0;
            MotorPidFunction(Motor_PARM[4]);
        }
        
        if(Symbol == 0) {
            Motor_PARM[0]->SetPoint = 0;
            Motor_PARM[1]->SetPoint = 0;
            Motor_PARM[2]->SetPoint = 0;
            Motor_PARM[3]->SetPoint = 0;
            Motor_PARM[4]->SetPoint = 0;

            if(Switch_1 == 0) {
                Time_Mark = SYS_Time_us;
                Symbol = 10;
            }
        }
        
        if(Symbol == 10) {
            Motor_PARM[0]->SetPoint =  4000 - 400;
            Motor_PARM[1]->SetPoint =  4000 + 400;
            Motor_PARM[2]->SetPoint = -4000 - 400;
            Motor_PARM[3]->SetPoint = -4000 + 400;
            if(SYS_Time_us > Time_Mark+2700000) {
                Symbol = 11;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 11) {                                                     //  停止，刹车
            Motor_PARM[0]->SetPoint = 0;
            Motor_PARM[1]->SetPoint = 0;
            Motor_PARM[2]->SetPoint = 0;
            Motor_PARM[3]->SetPoint = 0;
            if(SYS_Time_us > Time_Mark+500000) {
                Symbol = 20;
                Time_Mark = SYS_Time_us ;
            }
        }
        /*if(Symbol == 20) {                                                      //  停止后，左转
            Angle_PARM.SetAngle = 90;
            AnglePidFunction();                                                 //  左转，设置4个电机速度
            Motor_PARM[0]->SetPoint = - Angle_PARM.Delta_SetPoint;
            Motor_PARM[1]->SetPoint = - Angle_PARM.Delta_SetPoint;
            Motor_PARM[2]->SetPoint = - Angle_PARM.Delta_SetPoint;
            Motor_PARM[3]->SetPoint = - Angle_PARM.Delta_SetPoint;
            if(SYS_Time_us > Time_Mark + 3000000) {          //  转到90°时停止
                Symbol = 21;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 21) {                                                     //  停止，刹车
            Motor_PARM[0]->SetPoint = 0;
            Motor_PARM[1]->SetPoint = 0;
            Motor_PARM[2]->SetPoint = 0;
            Motor_PARM[3]->SetPoint = 0;
            if(SYS_Time_us > Time_Mark+500000) {
                Symbol = 30;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 30) {                                                     //  检查是否进入自动区域
            Motor_PARM[0]->SetPoint = 2000;
            Motor_PARM[1]->SetPoint = 2000;
            Motor_PARM[2]->SetPoint = -2000;
            Motor_PARM[3]->SetPoint = -2000;
            if(Light_Sensor_1 + Light_Sensor_2 + Light_Sensor_3 + Light_Sensor_4 < 2) {
                Symbol = 31;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 31) {                                                     //  继续走0.5秒
            if(SYS_Time_us > Time_Mark+500000) {
                Symbol = 32;
                Time_Mark = SYS_Time_us ;
            }
        }*/
        if(Symbol == 20) {                                                     //  停止，刹车
            Motor_PARM[0]->SetPoint = 0;
            Motor_PARM[1]->SetPoint = 0;
            Motor_PARM[2]->SetPoint = 0;
            Motor_PARM[3]->SetPoint = 0;
            if(SYS_Time_us > Time_Mark+500000) {
                Symbol = 21;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 21){                                                   //抛射电机归位
            Motor_PARM[4]->SetPoint = 200;
            MotorPidFunction(Motor_PARM[4]);
            if(Switch_5 == 1){
                Time_Mark = SYS_Time_us ;
                Symbol = 22;
                printf("s=%d",Symbol);
            }
        }
        if(Symbol == 22){
            Motor_PARM[4]->SetPoint = -150;
            MotorPidFunction(Motor_PARM[4]);
            if(SYS_Time_us > Time_Mark+350000){
                Motor_PARM[4]->SetPoint = 0;
                Time_Mark = SYS_Time_us ;
                Symbol = 30;
            }
        }
        if(Symbol == 30) {                                                      //  等待对接
            MotorPidFunction(Motor_PARM[4]);
            if(Switch_1 + Switch_2 + Switch_3 + Switch_4 == 2) {              //  触碰开关闭合
                Time_Mark = SYS_Time_us;
                Symbol = 31;
                Light_RED(GPIO_PIN_SET);
            }
        }
        if(Symbol == 31) {
            MotorPidFunction(Motor_PARM[4]);
            if(Switch_1 + Switch_2 + Switch_3 + Switch_4 == 2) {
                if(SYS_Time_us > Time_Mark + 2000000) {                           //  触碰开关闭合后等待1秒
                    Light_RED(GPIO_PIN_RESET);
                    Symbol = 32;
                }
            }else{
                Light_RED(GPIO_PIN_RESET);
                Symbol = 30;
            }
        }
        if(Symbol == 32) {                                                      //  收紧舵机
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 1200);
            Symbol = 33;
            Time_Mark = SYS_Time_us;
        }
        if(Symbol == 33){                                           //亮绿灯 等待手动松舵机
            MotorPidFunction(Motor_PARM[4]);
            Light_GREEN(GPIO_PIN_SET);
            if(SYS_Time_us > Time_Mark+5000000){
                Time_Mark = SYS_Time_us ;
                Symbol = 34;
            }
        }
        if(Symbol == 34){                                           //灭绿灯
            MotorPidFunction(Motor_PARM[4]);
            Light_GREEN(GPIO_PIN_RESET);
            Time_Mark = SYS_Time_us ;
            Symbol = 40;
        }
        if(Symbol == 40) {                                                      //  停止后，左转
            MotorPidFunction(Motor_PARM[4]);
            Motor_PARM[0]->SetPoint = - 1000;
            Motor_PARM[1]->SetPoint = - 1000;
            Motor_PARM[2]->SetPoint = - 1000;
            Motor_PARM[3]->SetPoint = - 1000;
            if(MPU6050.Yaw > 175) {          //  转到180°时停止
                Symbol = 41;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 41) {                                                     //  停止，刹车
            MotorPidFunction(Motor_PARM[4]);
            Motor_PARM[0]->SetPoint = 0;
            Motor_PARM[1]->SetPoint = 0;
            Motor_PARM[2]->SetPoint = 0;
            Motor_PARM[3]->SetPoint = 0;
            if(SYS_Time_us > Time_Mark+500000) {
                Symbol = 50;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 50){           //右移
            Motor_PARM[0]->SetPoint = 2000;
            Motor_PARM[1]->SetPoint = -2000;
            Motor_PARM[2]->SetPoint = -2000;
            Motor_PARM[3]->SetPoint = 2000;
            if(SYS_Time_us > Time_Mark+3000000){
                Symbol = 51;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 51){           //前贴边
            Motor_PARM[0]->SetPoint = 1200;
            Motor_PARM[1]->SetPoint = 1200;
            Motor_PARM[2]->SetPoint = -1200;
            Motor_PARM[3]->SetPoint = -1200;
            if(SYS_Time_us > Time_Mark+1000000){
                Symbol = 52;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 52){                           //对准投掷白线 
            MotorPidFunction(Motor_PARM[4]);
            Motor_PARM[0]->SetPoint = 500+2000;
            Motor_PARM[1]->SetPoint = 500-2000;
            Motor_PARM[2]->SetPoint = -500-2000;
            Motor_PARM[3]->SetPoint = -500+2000;
            if(!Light_Sensor_2||!Light_Sensor_3){
                Symbol = 60;
            }
        }
        if(Symbol == 60){                               //刹车,灯亮，响蜂鸣器
            Light_RED(GPIO_PIN_SET);
            Buzzer(GPIO_PIN_SET);
            MotorPidFunction(Motor_PARM[4]);
            Motor_PARM[0]->SetPoint = 0;
            Motor_PARM[1]->SetPoint = 0;
            Motor_PARM[2]->SetPoint = 0;
            Motor_PARM[3]->SetPoint = 0;
            Motor_PARM[4]->SetPoint = 0;
            if(SYS_Time_us > Time_Mark + 500000) {         
                Symbol = 70;
                Time_Mark = SYS_Time_us ;
                Buzzer(GPIO_PIN_RESET);
            }
        }
        if(Symbol == 70){                           //放下抛射杆
            MotorCurrentValue[4]=100;
            if(Switch_5==1){
                Symbol = 71;
            }
        }
        if(Symbol == 71){                           //加速
            Time_Mark = SYS_Time_us;
            MotorCurrentValue[4] = -12000;
            Symbol = 72;
        }
        if(Symbol == 72) {                          //松舵机
            if(SYS_Time_us > Time_Mark + 180000) {
                __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 1800);
                Symbol = 73 ;
            }
        }
        if(Symbol == 73) {                          //减速
            if(SYS_Time_us > Time_Mark + 300000) {
                MotorCurrentValue[4] = 12000;
                Symbol = 74;
            }
        }
        if(Symbol == 74) {                          //延时
            if(SYS_Time_us > Time_Mark + 330000) {
                Time_Mark = SYS_Time_us;
                Symbol = 75;
            }
        }
        if(Symbol == 75) {                          //刹车
            Motor_PARM[4]->SetPoint = 0;
            MotorPidFunction(Motor_PARM[4]);
            if(SYS_Time_us > Time_Mark + 1000000) {
                Time_Mark = SYS_Time_us;
                Symbol = 80;
            }
        }
        if(Symbol == 80){                                                   //抛射电机归位
            Motor_PARM[4]->SetPoint = 150;
            MotorPidFunction(Motor_PARM[4]);
            if(Switch_5 == 1){
                Time_Mark = SYS_Time_us ;
                Symbol = 81;
            }
        }
        if(Symbol == 81){
            Motor_PARM[4]->SetPoint = -150;
            MotorPidFunction(Motor_PARM[4]);
            if(SYS_Time_us > Time_Mark+350000){
                Motor_PARM[4]->SetPoint = 0;
                Time_Mark = SYS_Time_us ;
                Symbol = 82;
            }
        }
        if(Symbol == 82){
            MotorPidFunction(Motor_PARM[4]);
        }
        /*
        if(Symbol == 60) {                                                      //  停止后，左转
            MotorPidFunction(Motor_PARM[4]);
            Angle_PARM.SetAngle = 180;
            AnglePidFunction();                                                 //  左转，设置4个电机速度
            Motor_PARM[0]->SetPoint = - Angle_PARM.Delta_SetPoint;
            Motor_PARM[1]->SetPoint = - Angle_PARM.Delta_SetPoint;
            Motor_PARM[2]->SetPoint = - Angle_PARM.Delta_SetPoint;
            Motor_PARM[3]->SetPoint = - Angle_PARM.Delta_SetPoint;
            if(SYS_Time_us > Time_Mark + 3000000) {          //  转到90°时停止
                Symbol = 61;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 61){                               //刹车
            MotorPidFunction(Motor_PARM[4]);
            Motor_PARM[0]->SetPoint = 0;
            Motor_PARM[1]->SetPoint = 0;
            Motor_PARM[2]->SetPoint = 0;
            Motor_PARM[3]->SetPoint = 0;
            Motor_PARM[4]->SetPoint = 0;
            if(SYS_Time_us > Time_Mark + 500000) {         
                Symbol = 70;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 70){                           //前进直到贴边
            MotorPidFunction(Motor_PARM[4]);
            Motor_PARM[0]->SetPoint = 2000;
            Motor_PARM[1]->SetPoint = 2000;
            Motor_PARM[2]->SetPoint = -2000;
            Motor_PARM[3]->SetPoint = -2000;
            if(SYS_Time_us > Time_Mark + 1000000){
                Symbol = 71;
                Time_Mark = SYS_Time_us ;
            }
        }
        if(Symbol == 71){                           //对准投掷白线 
            MotorPidFunction(Motor_PARM[4]);
            Motor_PARM[0]->SetPoint = 500+2000;
            Motor_PARM[1]->SetPoint = 500-2000;
            Motor_PARM[2]->SetPoint = -500-2000;
            Motor_PARM[3]->SetPoint = -500+2000;
            if(!Light_Sensor_2||!Light_Sensor_3){
                Symbol = 72;
            }
        }
        if(Symbol == 72){                               //刹车,灯亮，响蜂鸣器
            Light_RED(GPIO_PIN_SET);
            Buzzer(GPIO_PIN_SET);
            MotorPidFunction(Motor_PARM[4]);
            Motor_PARM[0]->SetPoint = 0;
            Motor_PARM[1]->SetPoint = 0;
            Motor_PARM[2]->SetPoint = 0;
            Motor_PARM[3]->SetPoint = 0;
            Motor_PARM[4]->SetPoint = 0;
            if(SYS_Time_us > Time_Mark + 500000) {         
                Symbol = 80;
                Time_Mark = SYS_Time_us ;
                Buzzer(GPIO_PIN_RESET);
            }
        }
        if(Symbol == 80){                           //放下抛射杆
            MotorCurrentValue[4]=0;
            if(Switch_5==1){
                Symbol = 81;
            }
        }
        if(Symbol == 81){                           //加速
            Time_Mark = SYS_Time_us;
            MotorCurrentValue[4] = -16384;
            Symbol = 82;
        }
        if(Symbol == 82) {                          //松舵机
            if(SYS_Time_us > Time_Mark + 100000) {
                __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 1800);
                Symbol = 83 ;
            }
        }
        if(Symbol == 83) {                          //减速
            if(SYS_Time_us > Time_Mark + 250000) {
                MotorCurrentValue[4] = 16384;
                Symbol = 84;
            }
        }
        if(Symbol == 84) {                          //延时
            if(SYS_Time_us > Time_Mark + 280000) {
                Time_Mark = SYS_Time_us;
                Symbol = 85;
            }
        }
        if(Symbol == 85) {                          //刹车
            Motor_PARM[4]->SetPoint = 0;
            MotorPidFunction(Motor_PARM[4]);
            if(SYS_Time_us > Time_Mark + 1000000) {
                Time_Mark = SYS_Time_us;
                Symbol = 90;
            }
        }
        if(Symbol == 90){                                                   //抛射电机归位
            Motor_PARM[4]->SetPoint = 150;
            MotorPidFunction(Motor_PARM[4]);
            if(Switch_5 == 1){
                Time_Mark = SYS_Time_us ;
                Symbol = 91;
            }
        }
        if(Symbol == 91){
            Motor_PARM[4]->SetPoint = -150;
            MotorPidFunction(Motor_PARM[4]);
            if(SYS_Time_us > Time_Mark+350000){
                Motor_PARM[4]->SetPoint = 0;
                Time_Mark = SYS_Time_us ;
                Symbol = 92;
            }
        }
        */
        
        
        
        MotorPidFunction(Motor_PARM[0]);
        MotorPidFunction(Motor_PARM[1]);
        MotorPidFunction(Motor_PARM[2]);
        MotorPidFunction(Motor_PARM[3]);
        //MotorPidFunction(Motor_PARM[4]);

        MotorDriver(MotorCurrentValue, hcan1.pTxMsg);
    }

  /* USER CODE END 3 */

}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_2);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 6;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_5TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_5TQ;
  hcan2.Init.BS2 = CAN_BS2_2TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 159;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 79;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 39999;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2400;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 64000;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 19;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 64;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA8   ------> RCC_MCO_1
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF11 PF12 PF13 PF14 
                           PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3 
                           PG4 PG5 PG6 PG7 
                           PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 PB8 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void MotorDriver(int16_t MotorCurrentValue[], CanTxMsgTypeDef * TxMessage1) {
    uint8_t MotorCurrentValue_8bit[16];
    for(uint8_t i = 0 ; i < 8 ; ++i) {
        if(MotorCurrentValue[i] > 16384)
            MotorCurrentValue[i] = 16384;
        else if(MotorCurrentValue[i] < -16384)
            MotorCurrentValue[i] = -16384;
    }
    for(uint8_t i = 0 ; i < 8 ; ++i) {
        MotorCurrentValue_8bit[2 * i] = MotorCurrentValue[i] >> 8;
    }                                                       //取电机控制电流值高8位
    for(uint8_t i = 0 ; i < 8 ; ++i) {
        MotorCurrentValue_8bit[2 * i + 1] = MotorCurrentValue[i] & 0x00FF;
    }                                                       //取电机控制电流值低8位
    
    TxMessage1->StdId = 0x200;                              //设置标准标识符
    TxMessage1->ExtId = 0x200;                              //设置扩展标示符
    TxMessage1->IDE = CAN_ID_STD;                           //标准帧
    TxMessage1->RTR = CAN_RTR_DATA;                         //数据帧
    TxMessage1->DLC = 8;                                    //要发送的数据长度
    for(uint8_t i = 0 ; i < 8 ; ++i)
        TxMessage1->Data[i] = MotorCurrentValue_8bit[i];    //写入第1到4号电机控制电流值
    HAL_CAN_Transmit(&hcan1, 1);                            //发送第1到4号电机控制电流值
    
    TxMessage1->StdId = 0x1FF;                              //设置标准标识符
    TxMessage1->ExtId = 0x1FF;                              //设置扩展标示符
    for(uint8_t i = 0 ; i < 8 ; ++i)
        TxMessage1->Data[i] = MotorCurrentValue_8bit[i + 8];//写入第5到8号电机控制电流值
    HAL_CAN_Transmit(&hcan1, 1);                            //发送第5到8号电机控制电流值
    //printf("Now Time = %d ,Control Current = %d\r\n",SYS_Time_us,MotorCurrentValue[2]);
}
int fputc(int ch, FILE *f){
      HAL_UART_Transmit(&huart1, (uint8_t *)&ch,1, 0xFFFF);
      return ch;
}
int fgetc(FILE *f){
    uint8_t  ch;
    HAL_UART_Receive(&huart1,(uint8_t *)&ch, 1, 0xFFFF);
    return  ch;
}
void Motor_CAN_Reveive(CAN_HandleTypeDef* hcan, uint8_t FIFONumber){
    CanRxMsgTypeDef* pRxMsg = NULL;
    if(FIFONumber == CAN_FIFO0){
        pRxMsg = hcan->pRxMsg;
    }else /* FIFONumber == CAN_FIFO1 */{
        pRxMsg = hcan->pRx1Msg;
    }
    HAL_CAN_Receive(hcan,FIFONumber,1);
    //printf("CAN_STATE1 = %d,CAN_STATE2 = %d\r\n",hcan1.State,TempState);
    uint32_t TempID = pRxMsg->StdId;
    MotorAngle[TempID-0x201] = (pRxMsg->Data[0] << 8) + pRxMsg->Data[1];
    MotorSpeed[TempID-0x201] = (pRxMsg->Data[2] << 8) + pRxMsg->Data[3];
    MotorTrueCurrent[TempID-0x201] = (pRxMsg->Data[4] << 8) + pRxMsg->Data[5];
    MotorTemperature[TempID-0x201] = pRxMsg->Data[6];
    //printf("ID= 0x%x\t Speed= %d\t Angle= %d\t FIFO= %d\r\n",TempID,MotorSpeed[TempID-0x201],MotorAngle[TempID-0x201],FIFONumber);
}
void MotorPidFunction(Motor_PID_PARM* PID_PARM){
    if(PID_PARM->SetPoint>9500)
        PID_PARM->SetPoint = 9500;
    else if(PID_PARM->SetPoint<-9500)
        PID_PARM->SetPoint = -9500;
    //output = kp*et + ki*etSum + kd*det;
    int32_t Input = MotorSpeed[PID_PARM->Motor_ID];
    int32_t DeltaTime = SYS_Time_us - PID_PARM->PrevTime;              //得到当前时间与上次时间之间的间隔
    int32_t et = PID_PARM->SetPoint - Input;                            //反馈值与输入值的差值 e(t) = 比例
    PID_PARM->etSum += et;                                              //差值*时间间隔乘积累加 = 积分  理论是零
    double dEt = (et - PID_PARM->lastErr) / DeltaTime;                  //差值-上一次差值 = 微分
    MotorCurrentValue[PID_PARM->Motor_ID] = PID_PARM->kp * et + PID_PARM->ki * PID_PARM->etSum + PID_PARM->kd * dEt;	//输出 = 比例 + 积分 + 微分
    PID_PARM->lastErr = et;                                             //下次循环: 当前比例成为过去比例
    PID_PARM->PrevTime = SYS_Time_us;                                   //下次循环: 当前时间成为过去时间
    //printf("ID=%d,Speed=%d,Current=%d,SetPoint=%d,\r\n",PID_PARM->Motor_ID,MotorSpeed[PID_PARM->Motor_ID],MotorCurrentValue[PID_PARM->Motor_ID],PID_PARM->SetPoint);
}
void MotorPidAngle(Motor_PID_Angle_PARM* PID_Angle_PARM) {
    
    int32_t Input = PID_Angle_PARM->PresAngle;
    int32_t DeltaTime = SYS_Time_us - PID_Angle_PARM->PrevTime;
    int32_t et = PID_Angle_PARM->SetPoint - Input;
    PID_Angle_PARM->etSum += et;
    double dEt = (et - PID_Angle_PARM->lastErr) / DeltaTime;
    //MotorCurrentValue[PID_Angle_PARM->Motor_ID] = PID_Angle_PARM->kp * et + PID_Angle_PARM->ki * PID_Angle_PARM->etSum + PID_Angle_PARM->kd * (dEt-PID_Angle_PARM->LastdEt);
    MotorCurrentValue[PID_Angle_PARM->Motor_ID] = PID_Angle_PARM->kp * et + PID_Angle_PARM->ki * PID_Angle_PARM->etSum - PID_Angle_PARM->kd * MotorSpeed[PID_Angle_PARM->Motor_ID];
    PID_Angle_PARM->lastErr = et;
    PID_Angle_PARM->PrevTime = SYS_Time_us;
    PID_Angle_PARM->LastAngle = MotorAngle[PID_Angle_PARM->Motor_ID];
    //printf("Angle= %d Speed= %d\r\n",PID_Angle_PARM->PresAngle,MotorSpeed[PID_Angle_PARM->Motor_ID]);
}
void MotorAngleSTAT(Motor_PID_Angle_PARM* PID_Angle_PARM){
    if((MotorSpeed[PID_Angle_PARM->Motor_ID] > 0) && (MotorAngle[PID_Angle_PARM->Motor_ID] < PID_Angle_PARM->LastAngle) && MotorAngle[PID_Angle_PARM->Motor_ID] < 4096 && PID_Angle_PARM->LastAngle > 4096) {
        PID_Angle_PARM->AngleCount += 1;
    } else if((MotorSpeed[PID_Angle_PARM->Motor_ID] < 0) && (MotorAngle[PID_Angle_PARM->Motor_ID] > PID_Angle_PARM->LastAngle) && MotorAngle[PID_Angle_PARM->Motor_ID] > 4096 && PID_Angle_PARM->LastAngle < 4096) {
        PID_Angle_PARM->AngleCount -= 1;
    }
    PID_Angle_PARM->PresAngle = MotorAngle[PID_Angle_PARM->Motor_ID] + PID_Angle_PARM->AngleCount * 8192;
    //printf("PresAngle=%d",PID_Angle_PARM->PresAngle);
}
void JoystickGet(){
    Joystick.x = ADC_Value[0];
    Joystick.y = ADC_Value[1];
    Joystick.a = ADC_Value[2];
    Joystick.b = ADC_Value[3];
    Joystick.RefVoltage = ADC_Value[4];
    PowerVoltage = ADC_Value[5];
    /*
    JoystickPARM.AVG_x = PowerVoltage / 2;
    JoystickPARM.AVG_y = PowerVoltage / 2;
    JoystickPARM.AVG_a = PowerVoltage / 2;
    JoystickPARM.AVG_b = PowerVoltage / 2;
    JoystickPARM.MAX_x = PowerVoltage;
    JoystickPARM.MAX_y = PowerVoltage;
    JoystickPARM.MAX_a = PowerVoltage;
    JoystickPARM.MAX_b = PowerVoltage;*/
    
    Joystick.Button[0] = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_1);
    Joystick.Button[1] = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_2);
    Joystick.Button[2] = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_3);
    Joystick.Button[3] = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_4);
    Joystick.Button[4] = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_5);
    Joystick.Button[5] = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6);
    Joystick.Button[6] = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_7);
    Joystick.Button[7] = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_8);
    JoystickMapping();
    //printf("x=%d,y1=%d,a1=%d,b=%d,b1=%d,Joystick.RefVoltage=%d,\r\n",Joystick.x1,Joystick.y1,Joystick.a1,Joystick.b,Joystick.b1,Joystick.RefVoltage);
}
void JoystickMapping() {
    //-8192~8192
    uint8_t DeadZone = 100;
    
    if(Joystick.x < Joystick.RefVoltage/2 + DeadZone && Joystick.x > Joystick.RefVoltage/2 - DeadZone) {
        Joystick.x1 = 0;
    } else if(Joystick.x > Joystick.RefVoltage/2 + DeadZone) {
        Joystick.x1 = 8192 * (Joystick.x - Joystick.RefVoltage/2 - DeadZone) / (Joystick.RefVoltage - Joystick.RefVoltage/2 - DeadZone);
    } else if(Joystick.x < Joystick.RefVoltage/2 - DeadZone) {
        Joystick.x1 = -8192 * (- Joystick.x + Joystick.RefVoltage/2 - DeadZone) / (Joystick.RefVoltage/2 - DeadZone);
    }

    if(Joystick.y < Joystick.RefVoltage/2 + DeadZone && Joystick.y > Joystick.RefVoltage/2 - DeadZone) {
        Joystick.y1 = 0;
    } else if(Joystick.y > Joystick.RefVoltage/2 + DeadZone) {
        Joystick.y1 = 8192 * (Joystick.y - Joystick.RefVoltage/2 - DeadZone) / (Joystick.RefVoltage - Joystick.RefVoltage/2 - DeadZone);
    } else if(Joystick.y < Joystick.RefVoltage/2 - DeadZone) {
        Joystick.y1 = -8192 * (- Joystick.y + Joystick.RefVoltage/2 - DeadZone) / (Joystick.RefVoltage - Joystick.RefVoltage/2 - DeadZone);
    }

    if(Joystick.a < Joystick.RefVoltage/2 + DeadZone && Joystick.a > Joystick.RefVoltage/2 - DeadZone) {
        Joystick.a1 = 0;
    } else if(Joystick.a > Joystick.RefVoltage/2 + DeadZone) {
        Joystick.a1 = 8192 * (Joystick.a - Joystick.RefVoltage/2 - DeadZone) / (Joystick.RefVoltage - Joystick.RefVoltage/2 - DeadZone);
    } else if(Joystick.a < Joystick.RefVoltage/2 - DeadZone) {
        Joystick.a1 = -8192 * (-Joystick.a + Joystick.RefVoltage/2 - DeadZone) / (Joystick.RefVoltage - Joystick.RefVoltage/2 - DeadZone);
    }

    if(Joystick.b <= Joystick.RefVoltage/2+150 + DeadZone && Joystick.b >= Joystick.RefVoltage/2+150 - DeadZone) {
        Joystick.b1 = 0;
    } else if(Joystick.b > Joystick.RefVoltage/2+150 + DeadZone) {
        Joystick.b1 = 8192 * (Joystick.b - (Joystick.RefVoltage/2+150) - DeadZone) / (Joystick.RefVoltage - (Joystick.RefVoltage/2+150) - DeadZone);
    } else if(Joystick.b < Joystick.RefVoltage/2+150 - DeadZone) {
        Joystick.b1 = 8192 * (Joystick.b - (Joystick.RefVoltage/2+150) + DeadZone) / (Joystick.RefVoltage - (Joystick.RefVoltage/2+150) - DeadZone);
    }

}
void UltrasonicGet(){
    if(UltrasonicState[0] == 0 && UltrasonicState[1] == 0 && UltrasonicState[2] == 0 && UltrasonicState[3] == 0 && UltrasonicTXsymbol == 1){        //发送上升沿
    //if(UltrasonicState[1] == 0 && UltrasonicTXsymbol == 1){        //发送上升沿
        UltrasonicTXsymbol = 0;
        printf("1=%f,2=%f,3=%f,4=%f\r\n",UltrasonicDistance[0],UltrasonicDistance[1],UltrasonicDistance[2],UltrasonicDistance[3]);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);
        UltrasonicUP[0] = SYS_Time_us;
        UltrasonicState[0] = 1;
        UltrasonicState[1] = 1;
        UltrasonicState[2] = 1;
        UltrasonicState[3] = 1;
        //printf("0\r\n");
    }else if(UltrasonicState[0] == 1){  //发送下降沿
        if(SYS_Time_us>UltrasonicUP[0]+20){
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
            UltrasonicState[0] = 2;
            UltrasonicState[1] = 2;
            UltrasonicState[2] = 2;
            UltrasonicState[3] = 2;
        }
        //printf("1\r\n");
    }
    for(int i = 0; i < 4; ++i){
        if(UltrasonicState[i] == 4){  //计算时间
            //UltrasonicTime[i] = (uint16_t)((uint16_t)UltrasonicDOWN[i] - (uint16_t)UltrasonicUP[i]);
            double TempDistance = (double)UltrasonicTime[i]*0.000172;
            if(TempDistance < 5 && TempDistance > 0.03){
                UltrasonicDistance[i] = TempDistance;    //UltrasonicTime[i]/1000000*344/2
            }
            //printf("Time[%d]=%d,Distance=%f\r\n",i,UltrasonicTime[i],UltrasonicDistance[i]);
            UltrasonicState[i] = 5;
            //printf("4\r\n");
        }else if(UltrasonicState[i] == 5){  //接收完成
            UltrasonicState[i] = 0;
            //printf("%d 5\r\n",i);
        }
    }
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM1){
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
            if(UltrasonicState[0] == 2){
                //UltrasonicUP[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
                UltrasonicUP[0] = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
                //__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
                UltrasonicState[0] = 3;
                //printf("02\r\n");
            }else if(UltrasonicState[0] == 3){
                //UltrasonicDOWN[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
                UltrasonicDOWN[0] = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
                //HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);
                //__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
                UltrasonicState[0] = 4;
                //printf("03\r\n");
            }
        }else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
            if(UltrasonicState[1] == 2){
                UltrasonicUP[1] = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
                UltrasonicState[1] = 3;
                //printf("12\r\n");
            }else if(UltrasonicState[1] == 3){
                UltrasonicDOWN[1] = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
                UltrasonicState[1] = 4;
                //printf("13\r\n");
            }
        }else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
            if(UltrasonicState[2] == 2){
                UltrasonicUP[2] = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
                UltrasonicState[2] = 3;
                //printf("22\r\n");
            }else if(UltrasonicState[2] == 3){
                UltrasonicDOWN[2] = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
                UltrasonicState[2] = 4;
                //printf("23\r\n");
            }
        }else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
            if(UltrasonicState[3] == 2){
                UltrasonicUP[3] = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
                UltrasonicState[3] = 3;
                //printf("32\r\n");
            }else if(UltrasonicState[3] == 3){
                UltrasonicDOWN[3] = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
                UltrasonicState[3] = 4;
                //printf("33\r\n");
            }
        }
    }
    
}
void MPU6050Decode(){
    if(RxBuffer2[0] == 0x55){
        switch(RxBuffer2[1]) {
            case 0x51:
                MPU6050.ax = (int16_t)(RxBuffer2[3]<<8|RxBuffer2[2]) / 32768.0 * 16;
                MPU6050.ay = (int16_t)(RxBuffer2[5]<<8|RxBuffer2[4]) / 32768.0 * 16;
                MPU6050.az = (int16_t)(RxBuffer2[7]<<8|RxBuffer2[6]) / 32768.0 * 16;
                //printf("ax=%f,ay=%f,az=%f\r\n",MPU6050.ax,MPU6050.ay,MPU6050.az);
                break;
            case 0x52:
                MPU6050.wx = (int16_t)(RxBuffer2[3]<<8|RxBuffer2[2])/32768.0*2000;
                MPU6050.wy = (int16_t)(RxBuffer2[5]<<8|RxBuffer2[4])/32768.0*2000;
                MPU6050.wz = (int16_t)(RxBuffer2[7]<<8|RxBuffer2[6])/32768.0*2000;
                //printf("wx=%f,wy=%f,wz=%f\r\n",MPU6050.wx,MPU6050.wy,MPU6050.wz);
                break;
            case 0x53:
                MPU6050.Roll = (int16_t)(RxBuffer2[3]<<8|RxBuffer2[2])/32768.0*180;
                MPU6050.Pitch = (int16_t)(RxBuffer2[5]<<8|RxBuffer2[4])/32768.0*180;
                MPU6050.Yaw = (int16_t)(RxBuffer2[7]<<8|RxBuffer2[6])/32768.0*180;
                /*if(MPU6050.Yaw<0){
                    MPU6050.Yaw += 360;
                }*/
                //printf("Yaw=%f\r\n",MPU6050.Yaw);
                //printf("Roll=%f,Pitch=%f,Yaw=%f\r\n",MPU6050.Roll,MPU6050.Pitch,MPU6050.Yaw);
                //printf("ax=%f,ay=%f,az=%f\r\nwx=%f,wy=%f,wz=%f\r\nRoll=%f,Pitch=%f,Yaw=%f\r\n",MPU6050.ax,MPU6050.ay,MPU6050.az,MPU6050.wx,MPU6050.wy,MPU6050.wz,MPU6050.Roll,MPU6050.Pitch,MPU6050.Yaw);
                break;
        }
    }
}
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
    UNUSED(hcan);
    uint32_t TempID;
    TempID = hcan1.pRxMsg->StdId;
    MotorAngle[TempID-0x201] = (hcan1.pRxMsg->Data[0] << 8)+hcan->pRxMsg->Data[1];
    MotorSpeed[TempID-0x201] = (hcan1.pRxMsg->Data[2] << 8)+hcan->pRxMsg->Data[3];
    MotorTrueCurrent[TempID-0x201] = (hcan1.pRxMsg->Data[4] << 8)+hcan->pRxMsg->Data[5];
    MotorTemperature[TempID-0x201] = hcan1.pRxMsg->Data[6];
    printf("ID= 0x%x\t Speed= %d\t Angle= %d\t Current= %d\t Temperature= %d\r\n",TempID,MotorSpeed[TempID-0x201],MotorAngle[TempID-0x201],MotorTrueCurrent[TempID-0x201],MotorTemperature[TempID-0x201]);
    //HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);                            //重新打开CAN1接收中断
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM9) {
        UltrasonicTXsymbol = 1;
        //printf("TIM9\r\n");
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
    UNUSED(huart);
    //while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_RX) ;
    if(huart == &huart1){
        //HAL_UART_Receive_IT(&huart1, (uint8_t*)RxBuffer1, 1);          //重新打开串口接收
    }else if(huart == &huart2){
        MPU6050Decode();
        HAL_UART_Receive_IT(&huart2, (uint8_t*)RxBuffer2, 11);          //重新打开串口接收
        /*
        if(MPU6050.counter==0&&RxBuffer2[0]!=0x55)
            return;         //第 0 号数据不是帧头，跳过
        MPU6050_Buffer[MPU6050.counter] = RxBuffer[0];
        ++MPU6050.counter;
        if(MPU6050.counter = 11){   //接收到 11 个数据
            MPU6050.counter = 0;    //重新赋值，准备下一帧数据的接收
            MPU6050Decode();
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t*)RxBuffer2, 1);
        */
    }else if(huart == &huart6){
        if(RxBuffer6[0] == 0x6F){
            if(RxBuffer6[1]==Ball_Type){
                ++Ball_Counter;
            }else{
                Ball_Counter=0;
            }
            Ball_Type = RxBuffer6[1];
            printf("BallType=%d",Ball_Type);
        }
        while(HAL_UART_Receive_IT(&huart6, (uint8_t*)RxBuffer6, 2) != HAL_OK);
    }
}
void AnglePidFunction() {
    int32_t Input = MPU6050.Yaw;
    int32_t DeltaTime = SYS_Time_us - Angle_PARM.PrevTime;
    int32_t et = Angle_PARM.SetAngle - Input;
    Angle_PARM.etSum += et;
    double dEt = (et - Angle_PARM.lastErr) / DeltaTime;
    Angle_PARM.Delta_SetPoint = Angle_PARM.kp * et + Angle_PARM.ki * Angle_PARM.etSum + Angle_PARM.kd * dEt;
    Angle_PARM.lastErr = et;
    Angle_PARM.PrevTime = SYS_Time_us;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1) {
    }

  /* USER CODE END Error_Handler_Debug */ 
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
