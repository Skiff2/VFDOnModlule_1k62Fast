#include "stm32f10x.h"                  // Device header
#include "RTE_Components.h"             // Component selection
#include "DMA_STM32F10x.h"              // Keil::Device:DMA
#include "GPIO_STM32F10x.h"             // Keil::Device:GPIO
#include "RTE_Device.h"                 // Keil::Device:Startup
#include "misc.h"                       // Keil::Device:StdPeriph Drivers:Framework
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "math.h"

#define PI 				 3.14159
#define TIM_PERIOD 57     

#define NegativeActiv false

/*
int mTIM_Prescaler =  850;  //500 - 50гц(2,5), 330 - 75гц(3.7), 850 - 30гц (1.5)			
int TIM_STEPS	= 50; 				//500 = 5гц
*/

int mTIM_Prescaler =  300;  //300 - 29гц(4.1), 850 - 5гц (1.5)	
int TIM_STEPS	= 340; 				

#define max_TIM_STEPS  340

//100 - 2000

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

#define poEnable GPIOA, GPIO_Pin_12 //Выход на активацию выхода драйвера
#define piFault GPIOA, GPIO_Pin_11 //Вход ошибки с драйвера
#define piIn1 GPIOB, GPIO_Pin_8 //Старт Инвертированный
#define piIn2 GPIOB, GPIO_Pin_9 //Стоп Инвертированный



void adc_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// настройки ADC
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // режим работы - одиночный, независимый
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // не сканировать каналы, просто измерить один канал
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // однократное измерение
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // без внешнего триггера
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //выравнивание битов результат - прижать вправо
	ADC_InitStructure.ADC_NbrOfChannel = 1; //количество каналов - одна штука
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);

	// настройка канала
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5); //Крутилка на B1

	// калибровка АЦП
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
}

void initIO(void)
{
	 //------Назначение GPIO------------- 
	 //-----------Ввод структур--------------// 
	 GPIO_InitTypeDef gpio;
	 GPIO_InitTypeDef gpio_n;

 //------------------------------------
	GPIO_StructInit(&gpio); // GPIO A - Tim1 Channel 1P, 2P, 3P, Enable
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	gpio.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &gpio);
 //------------------------------------ 
	GPIO_StructInit(&gpio_n); /* GPIOB Configuration: Channel 1N, 2N , 3N as alternate function push-pull */ 
	gpio_n.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
	gpio_n.GPIO_Mode = GPIO_Mode_AF_PP; 
	gpio_n.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_Init(GPIOB, &gpio_n);

}

void timers(void) { 
 //-----------Ввод структур--------------// 
 TIM_TimeBaseInitTypeDef TIM_Base_1;
 TIM_OCInitTypeDef TIM_PWM;

 //-----Назначение TIM1---------------- 
 TIM_TimeBaseStructInit(&TIM_Base_1);
 TIM_Base_1.TIM_Prescaler = mTIM_Prescaler; //18
 TIM_Base_1.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_Base_1.TIM_Period = TIM_PERIOD;
 TIM_Base_1.TIM_ClockDivision = TIM_CKD_DIV1;
 TIM_TimeBaseInit(TIM1,&TIM_Base_1);
	
 //-------Назначение ШИМ------------------- TIM_OCStructInit(&TIM_PWM);
 TIM_PWM.TIM_OCMode = TIM_OCMode_PWM1;
	
 TIM_PWM.TIM_OutputState = TIM_OutputState_Enable;
 TIM_PWM.TIM_OutputNState = TIM_OutputNState_Enable;
 
if (NegativeActiv)
{
 TIM_PWM.TIM_OCPolarity = TIM_OCPolarity_Low;
 TIM_PWM.TIM_OCNPolarity = TIM_OCNPolarity_Low;
}
	
 TIM_OC1Init(TIM1, &TIM_PWM);
 TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
 TIM_OC2Init(TIM1, &TIM_PWM);
 TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
 TIM_OC3Init(TIM1, &TIM_PWM);
 TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
 TIM_Cmd(TIM1, ENABLE);
 TIM_CtrlPWMOutputs(TIM1, ENABLE);
 //разнесение инверсионных выходов
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = 254; //Много - не мало. Но мало - дорого =)
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
 } 
/*********************************************************************/ 
 //---Фаза А ---// 
 uint16_t sinA[max_TIM_STEPS] = {0}; //{56,84,113,141,169,197,225,253,281,309,337,365,392, 420,447,474,502,529,556,583,609,636,662,688,714,740,766,791,817,842,867, 891,916,940,964,988,1011,1035,1058,1080,1103,1125,1147,1169,1190,1211, 1232,1252,1272,1292,1312,1331,1350,1368,1386,1404,1422,1439,1456,1472, 1488,1504,1519,1534,1549,1563,1577,1590,1603,1616,1628,1640,1651,1662, 1673,1683,1693,1702,1711,1720,1728,1736,1743,1750,1756,1762,1768,1773, 1777,1782,1785,1789,1792,1794,1796,1798,1799,1799,1800,1799,1799,1798, 1796,1794,1792,1789,1785,1782,1777,1773,1768,1762,1756,1750,1743,1736, 1728,1720,1711,1702,1693,1683,1673,1662,1651,1640,1628,1616,1603,1590, 1577,1563,1549,1534,1519,1504,1488,1472,1456,1439,1422,1404,1386,1368, 1350,1331,1312,1292,1272,1252,1232,1211,1190,1169,1147,1125,1103,1080, 1058,1035,1011,988,964,940,916,891,867,842,817,791,766,740,714,688,662, 636,609,583,556,529,502,474,447,420,392,365,337,309,281,253,225,56};
 //---Фаза B ---// 
 uint16_t sinB[max_TIM_STEPS] = {0}; //{1529,1514,1499,1483,1467,1450,1433,1416,1398,1380, 1362,1343,1324,1305,1286,1266,1245,1225,1204,1183,1161,1140,1118,1095, 1073,1050,1027,1003,980,956,932,908,883,858,833,808,783,757,732,706,680, 653,627,600,574,547,520,493,465,438,411,383,355,328,300,272,244,216,188, 160,131,103,75,47,18,9,37,65,94,122,150,178,206,234,262,290,318,346,374, 401,429,456,484,511,538,565,591,618,645,671,697,723,749,774,800,825,850, 875,900,924,948,972,996,1019,1042,1065,1088,1110,1132,1154,1176,1197, 1218,1239,1259,1279,1299,1318,1337,1356,1374,1392,1410,1428,1445,1461, 1478,1494,1509,1524,1539,1568,1581,1595,1608,1620,1632,1644,1655,1666, 1677,1687,1696,1705,1714,1723,1731,1745,1752,1758,1764,1769,1774,1779, 1783,1786,1790,1792,1795,1797,1798,1799,1799,1799,1799,1798,1797,1795, 1793,1791,1788,1784,1780,1776,1771,1766,1760,1748,1741,1733,1725,1717, 1708,1699,1690,1680,1670,1659,1529};
 //---Фаза C ---// 
 uint16_t sinC[max_TIM_STEPS] = {0};//{1599,1612,1624,1636,1648,1659,1670,1680,1690,1699, 1708,1717,1725,1733,1741,1748,1754,1760,1766,1771,1776,1780,1784,1788, 1791,1793,1795,1797,1798,1799,1799,1799,1799,1798,1797,1795,1792,1790, 1786,1783,1779,1774,1769,1764,1758,1752,1745,1738,1731,1723,1714,1705, 1696,1687,1677,1666,1655,1644,1632,1620,1608,1595,1581,1568,1554,1539, 1524,1509,1494,1478,1461,1445,1428,1410,1392,1374,1356,1337,1318,1299, 1279,1259,1239,1218,1197,1176,1154,1132,1110,1088,1065,1042,1019,996, 972,948,924,900,875,850,825,800,774,749,723,697,671,645,618,591,565,538, 511,484,456,429,401,374,346,318,290,262,234,206,178,150,122,94,65,37,9, 18,47,75,103,131,160,188,216,244,272,300,328,355,383,411,438,465,493, 520,547,574,600,627,653,680,706,732,757,783,808,833,858,883,908,932,956, 980,1003,1027,1050,1073,1095,1118,1140,1161,1183,1204,1225,1245,1266, 1286,1305,1324,1343,1362,1380,1398,1416,1433,1586,1586};
 /**********************************************************************/ 

//Расчёт таблиц синуса
static void sin_table_update (uint16_t * data, uint16_t steps, uint16_t period, uint16_t phase, float_t koeff)
{
    for(uint16_t i = 0; i < steps; ++i)
    {
    	data[i] = (uint16_t)(period * koeff * (1 + sin(PI * (2.0f * i / steps + phase / 180.0f))));
    }
}

//Инициализация каналов DMA для каждой фазы
void sinDMA_PhaseA(void) { 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitTypeDef DMA_struct;
	DMA_StructInit(&DMA_struct);
	DMA_struct.DMA_PeripheralBaseAddr =(uint32_t)&TIM1->CCR1;
	DMA_struct.DMA_MemoryBaseAddr = (uint32_t)&sinA[0];
	DMA_struct.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_struct.DMA_BufferSize=TIM_STEPS;
	DMA_struct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_struct.DMA_Mode=DMA_Mode_Circular;
	DMA_Init(DMA1_Channel2,&DMA_struct);
	DMA_Cmd(DMA1_Channel2, ENABLE);
	TIM_DMACmd(TIM1,TIM_DMA_CC1, ENABLE);
 } 
void sinDMA_PhaseB(void) { 
	DMA_InitTypeDef DMA_struct1;
	DMA_StructInit(&DMA_struct1);
	DMA_struct1.DMA_PeripheralBaseAddr =(uint32_t)&TIM1->CCR2;
	DMA_struct1.DMA_MemoryBaseAddr = (uint32_t)&sinB[0];
	DMA_struct1.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_struct1.DMA_BufferSize=TIM_STEPS;
	DMA_struct1.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct1.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct1.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_struct1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_struct1.DMA_Mode=DMA_Mode_Circular;
	DMA_Init(DMA1_Channel3,&DMA_struct1);
	DMA_Cmd(DMA1_Channel3, ENABLE);
	TIM_DMACmd(TIM1,TIM_DMA_CC2, ENABLE);
 } 
void sinDMA_PhaseC(void) { 
	DMA_InitTypeDef DMA_struct2;
	DMA_StructInit(&DMA_struct2);
	DMA_struct2.DMA_PeripheralBaseAddr =(uint32_t)&TIM1->CCR3;
	DMA_struct2.DMA_MemoryBaseAddr = (uint32_t)&sinC[0];
	DMA_struct2.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_struct2.DMA_BufferSize=TIM_STEPS;
	DMA_struct2.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct2.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct2.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_struct2.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_struct2.DMA_Mode=DMA_Mode_Circular;
	DMA_Init(DMA1_Channel6,&DMA_struct2);
	DMA_Cmd(DMA1_Channel6, ENABLE);
	TIM_DMACmd(TIM1,TIM_DMA_CC3, ENABLE);
 }

 ////////----------------------------------- Логика
 int Abs(int i1)
 {
	 if (i1 < 0)
		 return i1 * -1;
	 else 
		 return i1;
 }
 
 //Функция задержки
 void delay_ms(uint32_t ms)
{
	int32_t ms_count_tick =  ms * (SystemCoreClock/1000);
	//разрешаем использовать счётчик
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
         //обнуляем значение счётного регистра
	DWT_CYCCNT  = 0;
        //запускаем счётчик
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; 
	while(DWT_CYCCNT < ms_count_tick);
        //останавливаем счётчик
	DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

//замер значения регулятора частоты и приведение его к значениям для делителя таймера
int GetValFriq()
{
	double ret;
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	ret = ADC_GetConversionValue(ADC1);
	
	//ret = 160 + (4000 - ret)/4.7;
	ret = (4096-ret) /300;
	ret = 190 + exp(ret);
	return ret;
}

//Инициализация основных пинов управления. Ручное управление портами
void InitIO2Manual()
{
	adc_init();
	
	GPIO_InitTypeDef gpioP;
	GPIO_InitTypeDef gpioN;
	
	GPIO_InitTypeDef gpiom;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE); //----------Включение тактирования-------// 

	
	//Общие входы-выходы
	GPIO_StructInit(&gpiom); // GPIO A - Enable
	gpiom.GPIO_Mode = GPIO_Mode_Out_PP;
	gpiom.GPIO_Pin = GPIO_Pin_12;
	gpiom.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpiom);
	
	// GPIO A - Fault
	gpiom.GPIO_Mode = GPIO_Mode_IPD;
	gpiom.GPIO_Pin = GPIO_Pin_11;
	gpiom.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpiom);
	
	// GPIO B - in 1, in 2 (кнопки)
	gpiom.GPIO_Mode = GPIO_Mode_IPD;
	gpiom.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpiom.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &gpiom);
	
	GPIO_StructInit(&gpioP); // GPIO A - Tim1 Channel 1P, 2P, 3P
	gpioP.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioP.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	gpioP.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &gpioP);
	
	GPIO_StructInit(&gpioN); // GPIO B - Tim1 Channel 1N, 2N, 3N
	gpioN.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioN.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	gpioN.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioN);	
}

//Изменение делителя частоты (увеличивает частоту срабатывания таймера = увеличивает частоту синуса)
void SetPrescaler(int val)
{
	DMA_Cmd(DMA1_Channel6, DISABLE);
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	TIM_Cmd(TIM1, DISABLE);

	mTIM_Prescaler = val;
	TIM1->PSC = val;

	TIM_Cmd(TIM1, ENABLE);
	DMA_Cmd(DMA1_Channel6, ENABLE);
	DMA_Cmd(DMA1_Channel3, ENABLE);
	DMA_Cmd(DMA1_Channel2, ENABLE);
}

//Метод остановки частотника
void Stop(void)
{
	DMA_Cmd(DMA1_Channel6, DISABLE);
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	TIM_Cmd(TIM1, DISABLE);
	TIM1->EGR = TIM_EGR_UG;

	InitIO2Manual();
//	
	GPIO_ResetBits(poEnable);

	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	GPIO_ResetBits(GPIOA,GPIO_Pin_9);
	GPIO_ResetBits(GPIOA,GPIO_Pin_10);
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	GPIO_SetBits(GPIOB,GPIO_Pin_15);

}

//Метод запуска частотника
void Start()
{
	int i, val;
	
////////// инициализация 
	val = GetValFriq();
	
	mTIM_Prescaler =  900;  
	TIM_STEPS	= 120;  
	
	//0.5 - 100% напряжения на выходе. 0.25 - 50%
	sin_table_update((uint16_t*)sinA, TIM_STEPS, TIM_PERIOD, 0, 0.5);
	sin_table_update((uint16_t*)sinB, TIM_STEPS, TIM_PERIOD, 120, 0.5); //120
	sin_table_update((uint16_t*)sinC, TIM_STEPS, TIM_PERIOD, 240, 0.5); //240
	
	timers();
	initIO();
	
	sinDMA_PhaseA();
	sinDMA_PhaseB();
	sinDMA_PhaseC();
//////////

	GPIO_SetBits(poEnable);
	
	//Выход на рабочие
	for (i = 1200; i > val; i = i - 5) // 2000 = 5hz@650hz 110 = 83hz@9600hz  205 = 50hz@6000hz  350 = 30hz@3500hz
	{
		if (i > 400)
		{
			i = i-50;
		}
		
		SetPrescaler(i);
		delay_ms(10);
//		if (!GPIO_ReadInputDataBit(piIn2) && GPIO_ReadInputDataBit(poEnable))
//		{
//			Stop();
//			delay_ms(100);
//			return;
//		}
	}
}

//Метод измерения частоты
void SetFrequency()
{
	int nVal = GetValFriq();
	
	///////////// Запуск и остановка по значению потенциометра
	if (!GPIO_ReadInputDataBit(poEnable) && nVal < 740)//543)
	{
		Start();
		delay_ms(100);
	}
	
		if (GPIO_ReadInputDataBit(poEnable) && nVal > 840)//548)
			Stop();
	/////////////
	
	if (Abs(nVal - mTIM_Prescaler) > 7)
	{
		SetPrescaler(nVal);
		delay_ms(80);
	}
}

//Основной цикл
int main(void)
{
	Stop();	
	
	while(1)
	{
		if (!GPIO_ReadInputDataBit(piIn1) && !GPIO_ReadInputDataBit(poEnable))
		{
			Start();
			delay_ms(50);
		}
		
		if (!GPIO_ReadInputDataBit(piIn2) && GPIO_ReadInputDataBit(poEnable))
		{
			Stop();
			delay_ms(50);
		}
		
		//Проверка регулятора частоты
		//if (GPIO_ReadInputDataBit(poEnable))
			SetFrequency();
			
		delay_ms(10);
	}
}

