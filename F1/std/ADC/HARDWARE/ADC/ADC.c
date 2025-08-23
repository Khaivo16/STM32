#include "ADC.h"
void ADCx_init(ADC_TypeDef* ADCx,uint8_t channel)
{
		ADC_InitTypeDef ADC_InitStructure;
		__IO uint16_t ADCConvertedValue;
	
		if (ADCx==ADC1) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);}
		else if (ADCx==ADC2) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);}
		else if (ADCx==ADC3) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);}
			
		if (channel==ADC_Channel_0)	{GPIO_SETUP(GPIOA,GPIO_Pin_0,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_1) {GPIO_SETUP(GPIOA,GPIO_Pin_1,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_2) {GPIO_SETUP(GPIOA,GPIO_Pin_2,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_3) {GPIO_SETUP(GPIOA,GPIO_Pin_3,GPIO_Mode_AIN,GPIO_Speed_50MHz);}		
		else if (channel==ADC_Channel_4) {GPIO_SETUP(GPIOA,GPIO_Pin_4,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_5) {GPIO_SETUP(GPIOA,GPIO_Pin_5,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_6) {GPIO_SETUP(GPIOA,GPIO_Pin_6,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_7) {GPIO_SETUP(GPIOA,GPIO_Pin_7,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_8) {GPIO_SETUP(GPIOB,GPIO_Pin_0,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_9) {GPIO_SETUP(GPIOB,GPIO_Pin_1,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_10) {GPIO_SETUP(GPIOC,GPIO_Pin_0,GPIO_Mode_AIN,GPIO_Speed_50MHz);}	
		else if (channel==ADC_Channel_11) {GPIO_SETUP(GPIOC,GPIO_Pin_1,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_12) {GPIO_SETUP(GPIOC,GPIO_Pin_2,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_13) {GPIO_SETUP(GPIOC,GPIO_Pin_3,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_14) {GPIO_SETUP(GPIOC,GPIO_Pin_4,GPIO_Mode_AIN,GPIO_Speed_50MHz);}
		else if (channel==ADC_Channel_15) {GPIO_SETUP(GPIOC,GPIO_Pin_5,GPIO_Mode_AIN,GPIO_Speed_50MHz);}	
			
/* System clocks configuration ---------------------------------------------*/
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);


  /* GPIO configuration ------------------------------------------------------*/
// GPIO_SETUP(GPIOB,GPIO_Pin_1,GPIO_Mode_AIN,GPIO_Speed_50MHz);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADCx, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime_239Cycles5);
  
  /* Enable ADC1 */
  ADC_Cmd(ADCx, ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADCx);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADCx));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADCx);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADCx));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADCx, ENABLE);
}
uint16_t ADC_read(ADC_TypeDef* ADCx,uint8_t channel)
{
	ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime_239Cycles5);
	ADC_SoftwareStartConvCmd(ADCx,ENABLE);
	/* Wait till done */
	while (ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);
	
	/* Return result */
	//return ADC_GetConversionValue(ADC1);
		
	return ADC_GetConversionValue(ADCx);
}

uint16_t ADC_avr(ADC_TypeDef* ADCx,uint8_t channel,uint8_t lan)
{
	uint32_t tong=0;
	uint8_t i;
	for (i=0;i<lan;i++){
	
	tong+= ADC_read(ADCx,channel);
	}
	return tong/lan;
}

