#include "main.h"
#include "stm32f10x.h"

#define ADC1_DR_Address                 ((uint32_t)0x4001244C)
#define USART1_DR_Base                  ((uint32_t)0x40013804)

void Configure_RCC();
void Configure_GPIOs();
void Configure_NVIC();
void Configure_DMA1_1();
void Configure_DMA1_4();
void Configure_ADC1();
void Configure_TIM3();
void Configure_UART1();

volatile unsigned short ADCValues[1000];
volatile uint8_t UART_TXData[2000];

int main(void)
{ 
  Configure_RCC();
  Configure_GPIOs();
  Configure_NVIC();
  Configure_DMA1_1();
  Configure_ADC1();
  Configure_TIM3();
  Configure_UART1();
  
  __CLEAR_BIT(GPIOC->ODR, 8);
  __CLEAR_BIT(GPIOC->ODR, 9);
  
  DMA_Cmd(DMA1_Channel1, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  TIM_Cmd(TIM3,ENABLE);
  
  USART_Cmd(USART1, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  
  while(1)
  {
  }
}

void ADC_ready(uint16_t idxDataEnd)
{
  __SET_BIT(GPIOA->ODR, 12);
  uint16_t idxADCValue = 0;
  uint16_t idxTxData = 0;
  uint16_t idxCRCValue = 0;
  uint32_t CRCValue = 0;
  
  if (idxDataEnd == 500)
    idxADCValue = 0;
  else if (idxDataEnd == 1000)
    idxADCValue = 500;
  else
    return;
  
  CRC_ResetDR();
  CRCValue = CRC_CalcBlockCRC((uint32_t *)ADCValues, 125);
  
  UART_TXData[idxTxData++] = 0x7E;
  while (idxADCValue < idxDataEnd)
  {
    if ((uint8_t)(ADCValues[idxADCValue]>>8) == 0x7E)
    {
      UART_TXData[idxTxData++] = 0x7D;
      UART_TXData[idxTxData++] = 0x5E;
    }
    else if ((uint8_t)(ADCValues[idxADCValue]>>8) == 0x7D)
    {
      UART_TXData[idxTxData++] = 0x7D;
      UART_TXData[idxTxData++] = 0x5D;
    }
    else
      UART_TXData[idxTxData++] = (uint8_t)(ADCValues[idxADCValue]>>8);
    
    if ((uint8_t)(ADCValues[idxADCValue]) == 0x7E)
    {
      UART_TXData[idxTxData++] = 0x7D;
      UART_TXData[idxTxData++] = 0x5E;
    }
    else if ((uint8_t)(ADCValues[idxADCValue]>>8) == 0x7D)
    {
      UART_TXData[idxTxData++] = 0x7D;
      UART_TXData[idxTxData++] = 0x5D;
    }
    else
      UART_TXData[idxTxData++] = (uint8_t)(ADCValues[idxADCValue]>>8);
    
    idxADCValue++;
  }
  while (idxCRCValue < 4)
  {
    if ((uint8_t)(CRCValue>>(8*(3-idxCRCValue))) == 0x7E)
    {
      UART_TXData[idxTxData++] = 0x7D;
      UART_TXData[idxTxData++] = 0x5E;
    }
    else if ((uint8_t)(CRCValue>>(8*(3-idxCRCValue))) == 0x7D)
    {
      UART_TXData[idxTxData++] = 0x7D;
      UART_TXData[idxTxData++] = 0x5D;
    }
    else
      UART_TXData[idxTxData++] = (uint8_t)(CRCValue>>(8*(3-idxCRCValue)));
    
    idxCRCValue++;
  }
  UART_TXData[idxTxData++] = 0x7E;
  
  Configure_DMA1_4();
  DMA_Cmd(DMA1_Channel4, ENABLE);
  
  __CLEAR_BIT(GPIOA->ODR, 12);
}

void Configure_RCC()
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                                           // Enable GPIOC Clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                                           // Enable ADC1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                                           // Enable GPIOA Clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                                           // Enable Alternate Fuctions Clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                                           // Enable TIM3 Clock
  RCC->AHBENR |= RCC_AHBPeriph_DMA1;                                            // Enable DMA1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                                         // Enable UART1 Clock
  RCC->AHBENR |= RCC_AHBPeriph_CRC;                                             // Enable CRC Clock
}

void Configure_GPIOs()
{
  GPIOC->CRH = 0x44444422;                                                      // PC8 and PC9 as General Purpose Push Pull Output Max Speed = 2MHz
  GPIOA->CRL = 0x44444404;                                                      // PA1 as input analog mode
  GPIOA->CRH = 0x22224444;                                                      // PA12, PA13, PA14 & PA15 as General Purpose Push Pull Output Max Speed = 2MHz
  
  __CLEAR_BIT(GPIOC->ODR, 8);
  __CLEAR_BIT(GPIOC->ODR, 9);
  __CLEAR_BIT(GPIOA->ODR, 12);
  __CLEAR_BIT(GPIOA->ODR, 13);
  __CLEAR_BIT(GPIOA->ODR, 14);
  __CLEAR_BIT(GPIOA->ODR, 15);
}

void Configure_NVIC()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;                      // Enable DMA1 channel1 IRQ Channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;                      // Enable DMA1 channel4 IRQ Channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Configure_DMA1_1()
{
  DMA_InitTypeDef DMA_InitStructure;
  
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCValues[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1000;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
}

void Configure_DMA1_4()
{
  DMA_InitTypeDef DMA_InitStructure;
  
  DMA_DeInit(DMA1_Channel4);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&UART_TXData[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 500;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
}

void Configure_ADC1()
{
  ADC_InitTypeDef ADC_InitStructure;
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
}

void Configure_TIM3()
{
  TIM_TimeBaseInitTypeDef TimeBaseInitStructure; 
  
  TimeBaseInitStructure.TIM_Prescaler = 24 - 1;
  TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TimeBaseInitStructure.TIM_Period = 125;                                       // in microseconds
  TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  
  TIM_TimeBaseInit(TIM3,&TimeBaseInitStructure);
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
}

void Configure_UART1()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 921600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &USART_InitStructure);
}