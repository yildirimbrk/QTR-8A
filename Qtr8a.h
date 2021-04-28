/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: QTR-8A 8'li Kizilötesi Sensör - Analog 																																										*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name: QTR-8A																																																											*	
*																																																																						*
*		File Name: Qtr8a.h																																																											*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification Dates: Mart 6, 2021																																																				*
*																																																																						*	
*		Description:																																																														*
* 																																																																					*	
*		Bu, Kizilötesi Sensör Çizgi izleyen modülü "QTR-8A"için header dosyasidir.																															*
* 																																																																					*
*		Notes:																																																																	*
*																																																																						*
*	 [1] 																																																																			*
*	 																																																																					*
*	 [2] 																																																																			*
*																																																																						*
* 																																																																					*
* 																																																																					*	
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
/*																																						*/
#ifndef __Qtr8a
#define __Qtr8a
/*																																						*/
/* Includes ------------------------------------------------------------------*/
/*																																						*/
#include "main.h"
/*																																						*/
/* Private includes ----------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Exported types ------------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Exported constants --------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Exported macro ------------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Exported functions prototypes ---------------------------------------------*/
/*																																						*/
////////////////////////////////////////////////////[1-BOLUM]	SENSOR ICIN KULLANILAN ÇEVRE BIRIMLERININ BASLATILMASI///////////////////////////////////////////////////////////
void ADC1_PinInit(void);
void ADC1_Init(void);
void ADC1_ChannelInit(void);
void DMA1_AdcInit(void);
////////////////////////////////////////////////////[2-BOLUM]	SENSOR ICIN OLUSTURULAN VE KULLANIYA SUNULAN FONKSIYONLAR///////////////////////////////////////////////////////////
void QTR_SensorsInit(void) ;
void QTR_PercentageValue(uint16_t* sensorPercentRate) ;
void QTR_SensorCalibrate(uint16_t* Sensor_MaxValue, uint16_t* Sensor_MinValue ) ;
void QRT_GetCalibratedVales(uint16_t* Sensor_MaxValue,uint16_t* Sensor_MinValue,uint16_t* CalibratedValues) ;
int32_t QTR_GetLinePosition(uint16_t* calibratedValues) ;
/*																																						*/
/* Private defines -----------------------------------------------------------*/
/*																																						*/
#define ADC_RESOLUTION 4096
#define PERCENT        100
#define NR_OF_SENSOR 8 //	sensor adeti
#define DATA_LENGTH 10 // data uzunlugu
#define OUTMAX 100
#define OUTMIN  0
#define LINESENSOR0_PIN GPIO_PIN_0
#define LINESENSOR1_PIN GPIO_PIN_1
#define LINESENSOR2_PIN GPIO_PIN_2
#define LINESENSOR3_PIN GPIO_PIN_3
#define LINESENSOR4_PIN GPIO_PIN_4
#define LINESENSOR5_PIN GPIO_PIN_5
#define LINESENSOR6_PIN GPIO_PIN_6
#define LINESENSOR7_PIN GPIO_PIN_7
#define LINESENSOR_PORT GPIOA
/*																																						*/

#endif
