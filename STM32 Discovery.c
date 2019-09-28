#include "stm32F10x.h"
#include "STM32vldiscovery.h"
#include "stm32f10x_conf.h"
#include "stdio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include <stm32f10x_tim.h>
#include "stm32f10x_usart.h"
//#include <stm32f10x_lib.h>
 
/* Private typedef -----------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
USART_InitTypeDef USART_InitStructure;

/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001244C)
#define  LSE_FAIL_FLAG  0x80
#define  LSE_PASS_FLAG  0x100
#define  N  100      //no. of averaging samples
#define  M  1      //no. of channels
 
/* Private variables ---------------------------------------------------------*/
u32 LSE_Delay = 0;
u32 count = 0;
u32 BlinkSpeed = 0;
u32 KeyState = 0;
u8 sendingsignal=0;
u8 checkflag=0;
u8 cf=0;
u8 countno=0;
u16 numberno=0;
 
//PI variables
vu16 NextPoint=0;
u16 SetPoint=5000; //Desired Value
u16 Proportion=5; //Proportional Const
u16 Integral=60; //Integral Const
 
int LastError=0; //Error[-1]
int iError=0;
int iIncpid=0;
s32 LastDuty=0;
s32 currentduty;
 
s32 dutycount=0;
u8 dutycycle=0;
 
vu16 pavg;
vu16 AD_Value[N][M];
vu16 After_filter[M];
 
vu32 pnow=0;
vu32 plast=0;
 
int i;
vu32 sum;
vu16 value[M];
 
static __IO uint32_t TimingDelay;
/* Private function prototypes -----------------------------------------------*/
void Delay(u16 time);
void TimingDelay_Decrement(void);
void RCC_Configuration(void);
void NVIC_Configuration(void);
void USART_Config(void);
void ADC_Config(void);
void Put_String(u8 *p);
void GPIO_Config(void);
 
int main(void)
{  STM32vldiscovery_LEDInit(LED4);
RCC_ADCCLKConfig(RCC_PCLK2_Div2);
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
 
 
 
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
NVIC_Configuration();
 
 
USART_InitStructure.USART_BaudRate = 9600;  //Bode rate 9600
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
USART_Init(USART1, &USART_InitStructure);
USART_Cmd(USART1, ENABLE);

 
GPIO_StructInit(&GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin   =   GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
GPIO_InitStructure.GPIO_Mode    =   GPIO_Mode_AF_PP;
GPIO_InitStructure.GPIO_Speed   =   GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);
 
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //Pin 9
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //Push-Pull output
GPIO_Init(GPIOA, &GPIO_InitStructure);              //TX init
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          //Pin 10
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Floating Input
GPIO_Init(GPIOA, &GPIO_InitStructure);              //RX Init
 
TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
TIM_TimeBaseStructure.TIM_Period   =   480; //480 is for 50kHz, 285 is for 83.89kHz, 
TIM_TimeBaseStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
TIM_TimeBaseStructure.TIM_ClockDivision =   0;
TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);

TIM_OCStructInit(&TIM_OCInitStructure);
TIM_OCInitStructure.TIM_OCMode          =   TIM_OCMode_PWM1;
TIM_OCInitStructure.TIM_OutputState     =   TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_OutputNState    =   TIM_OutputNState_Enable;
TIM_OCInitStructure.TIM_OCPolarity      =   TIM_OCPolarity_High;
TIM_OCInitStructure.TIM_OCNPolarity     =   TIM_OCNPolarity_High;
TIM_OCInitStructure.TIM_OCIdleState     =   TIM_OCIdleState_Set;
TIM_OCInitStructure.TIM_OCNIdleState    =   TIM_OCNIdleState_Reset;

TIM_OC1Init(TIM16, &TIM_OCInitStructure);
(TIM17, &TIM_OCInitStructure);
TIM_CtrlPWMOutputs(TIM16,ENABLE);
TIM_CtrlPWMOutputs(TIM17,ENABLE);
 
TIM_BDTRStructInit(&TIM_BDTRInitStructure);
TIM_BDTRInitStructure.TIM_OSSRState     =   TIM_OSSRState_Enable;
TIM_BDTRInitStructure.TIM_OSSIState     =   TIM_OSSIState_Enable;
TIM_BDTRInitStructure.TIM_LOCKLevel     =   TIM_LOCKLevel_OFF;
TIM_BDTRInitStructure.TIM_DeadTime      =   25; //sets the dead time as 1us
TIM_BDTRInitStructure.TIM_Break         =   TIM_Break_Disable;
TIM_BDTRInitStructure.TIM_AutomaticOutput=  TIM_AutomaticOutput_Enable;
TIM_BDTRConfig(TIM16, &TIM_BDTRInitStructure);
TIM_BDTRConfig(TIM17, &TIM_BDTRInitStructure);
 
TIM_SetCompare1(TIM16, 240); // for the duty cycle; has to be half of the TIM Period
 TIM_SetCompare1(TIM17, 240); // for the duty cycle; has to be half of the TIM Period
 
TIM_Cmd(TIM16, ENABLE);
TIM_Cmd(TIM17, ENABLE);

TIM_SetCounter(TIM16, 0);
TIM_SetCounter(TIM17, 8);
