/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: QTR-8A 8'li Kizilötesi Sensör - Analog 																																										*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name: QTR-8A																																																											*	
*																																																																						*
*		File Name: Qtr8a.c																																																											*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification Dates: Mart 6, 2021																																																				*
*																																																																						*	
*		Description:																																																														*
* 																																																																					*	
*		 Bu 8 adet analog çikisa sahip sensör "QTR-8A" modülü için Temel Kaynak Koddur(Base Source-Code). 																			*
*		 Sensörler için stm32f103c8t6 gelistirme kartinda bulunan ADC ve DMA baslatma(initiation) fonksiyonlarini, 															*
*		 pin konfigürasyon fonksiyonlarini, Alinan ADC verilerin yüzdelik degerlerini veren fonksiyon, Okunan degerler 													*
*		 üzerinden max ve min degerleri veren fonksiyon, Bulunan max ve min degerler arasi sensör kalbirasyonunu saglayan fonksiyon 						*
*		 ve Sensör katsayilarina göre durum pozisyonunu veren fonksiyonlar içermektedir.																												*
* 																																																																					*
*		Notes:																																																																	*
*																																																																						*
*	 [1] Burada kullanilan özellikler stm32f103c8t6 gelistirme karti baz alinarak olusturulmustur ancak																				*
*			 ADC ve DMA baslatma(initiation) fonksiyonlarini ve pin konfigürasyon fonksiyonlarini, üzerindeki 																		*
*			 kücük degisiklikler ile uyumlu diger mikrodenetleyiciler ile kullanilabilir.																													*
*	 																																																																					*
*	 [2] sensor kalibrasyon fonksiyonu belirli bir örnekleme alinarak kullanilmasi tavsiye edilir. Mesela; Bir süre kalbirasyon 							*
*				fonskiyonunu ile kalibre degerleri elde ettikten sonra fonksiyonu sonlandirmak.																											*
*																																																																						*
* 																																																																					*
* 																																																																					*	
*		ADC1 hattinin ADC_IN0 - ADC_IN7																																																					*
*		kanallari yapilandirilmistir.																																																						*
*		ADC1 									 ---> APB2 clock hattina baglidir																																									*
*		APB2 Peripheral clocks ---> 9MHz																																																				*
*		APB2 timer clocks      ---> 9MHz																																																				*
*		To ADC1,2              ---> 4.5MHz																																																			*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
/*																																						*/
#include "Qtr8a.h"
/*																																						*/
/* Private includes ----------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Private typedef -----------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Private define ------------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Private macro -------------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Private variables ---------------------------------------------------------*/
/*																																						*/
ADC_HandleTypeDef ADC1InitStruct	;
DMA_HandleTypeDef DMA_ADCINITSTRUCT ;
uint16_t DmaSensorValues[NR_OF_SENSOR];	
uint32_t adc_values[NR_OF_SENSOR]={0} ; 
uint8_t sensorArray = 0 ; // sensor sirasi
int16_t sensorDataBuffer[NR_OF_SENSOR][DATA_LENGTH]= {0} ; // 8 sensor için 10 ar adet veri depolama yeri
/*																																						*/
/* Private function prototypes -----------------------------------------------*/
/*																																						*/

/*																																						*/
////////////////////////////////////////////////////[1-BOLUM]	SENSOR ICIN KULLANILAN ÇEVRE BIRIMLERININ BASLATILMASI///////////////////////////////////////////////////////////
/* This function handles DMA1 channel1 global interrupt. */
void DMA1_Channel1_IRQHandler(void)
{
	/*tüm sensörler tek tek okununca interrupt bayragi kalkar böylece okunan degerler sirasi ile ayni boyuttaki baska bir diziye atanir böylece sensör sirasi sasmaz*/
	uint8_t sensorNumber = 0;
  HAL_DMA_IRQHandler(&DMA_ADCINITSTRUCT);
	
	for(sensorNumber=0;sensorNumber<8;sensorNumber++)
		DmaSensorValues[sensorNumber] = adc_values[sensorNumber] ;
}


void ADC1_PinInit(void)
{
/** Fonksiyon Tanimi 	 : Bu fonksiyon adc modülü için gpio pin konfigüresini yapilandirir.
	*											 Qtr8a 8 ayri analog çikis saglamakta 
	*											 Bu proje de stm32f103c8 ADC1 hattindaki ADC_IN0 - ADC_IN7 kanallari kullanildi
	*										   ADC1 APB2 hattina baglidir bu hat 16Mhz dir ancak ADC1,2 hatlari 8Mhzdir
	*											 Kanallar A portunda ve A0-A7 sirali pinleridir
	*
	*											-Konfigüre edilen pinler-  
	*															* Analog 
	*												PA0   ------> ADC_IN0
	*												PA1   ------> ADC_IN1
	*												PA2   ------> ADC_IN2
	*												PA3   ------> ADC_IN3
	*												PA4   ------> ADC_IN4
	*												PA5   ------> ADC_IN5
	*												PA6   ------> ADC_IN6
	*												PA7   ------> ADC_IN7
	*/

	//GPIO PORT SAATI
	__HAL_RCC_GPIOA_CLK_ENABLE() ;
	
	GPIO_InitTypeDef ADC_GPIOInitStruct ;
	
	//GPIO konfigürasyonu : PA0-PA7
	ADC_GPIOInitStruct.Pin    = LINESENSOR0_PIN|LINESENSOR1_PIN|LINESENSOR2_PIN|LINESENSOR3_PIN|
															LINESENSOR4_PIN|LINESENSOR5_PIN|LINESENSOR6_PIN|LINESENSOR7_PIN ;
	ADC_GPIOInitStruct.Mode 	= GPIO_MODE_ANALOG ;
	ADC_GPIOInitStruct.Pull   = GPIO_NOPULL ;
	HAL_GPIO_Init(LINESENSOR_PORT,&ADC_GPIOInitStruct);
	
}


void ADC1_Init(void)
{
/**
	* Fonksiyon Tanimi 	 : Bu fonksiyon ADC1 Modülünün özelliklerini yapilandirir 
	* 
	*/
	
	//ADC1 clock saati
	__HAL_RCC_ADC1_CLK_ENABLE() ;
	//ADC1
	ADC1InitStruct.Instance = ADC1 ;
	
	//ADC özellikleri konfigürasyonu
	ADC1InitStruct.Init.ContinuousConvMode    = ENABLE ;
	ADC1InitStruct.Init.DataAlign						  = ADC_DATAALIGN_RIGHT ;
	ADC1InitStruct.Init.DiscontinuousConvMode = DISABLE ;
	ADC1InitStruct.Init.ExternalTrigConv      = ADC_SOFTWARE_START ;
	ADC1InitStruct.Init.NbrOfConversion       = NR_OF_SENSOR ;
	ADC1InitStruct.Init.ScanConvMode          = ENABLE ;
	HAL_ADC_Init(&ADC1InitStruct);
	//DMA kesme
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
}

void ADC1_ChannelInit(void)
{
/**
	* Fonksiyon Tanimi 	 	: Bu fonksiyon ADC kanallarini siralari ve özellikleri konfigüre eder 
	* 
	*/
	
	ADC_ChannelConfTypeDef ADC1ChannelInitStruct ;
	//ADC_IN0
	ADC1ChannelInitStruct.Channel      = ADC_CHANNEL_0 ;
	ADC1ChannelInitStruct.Rank         = ADC_REGULAR_RANK_1;
	ADC1ChannelInitStruct.SamplingTime = ADC_SAMPLETIME_239CYCLES5_SMPR1ALLCHANNELS ; // 239.5 cycle
	HAL_ADC_ConfigChannel(&ADC1InitStruct, &ADC1ChannelInitStruct) ;
	//ADC_IN1
	ADC1ChannelInitStruct.Channel      = ADC_CHANNEL_1 ;
	ADC1ChannelInitStruct.Rank         = ADC_REGULAR_RANK_2;
	HAL_ADC_ConfigChannel(&ADC1InitStruct, &ADC1ChannelInitStruct) ;
	//ADC_IN2
	ADC1ChannelInitStruct.Channel      = ADC_CHANNEL_2 ;
	ADC1ChannelInitStruct.Rank         = ADC_REGULAR_RANK_3;
	HAL_ADC_ConfigChannel(&ADC1InitStruct, &ADC1ChannelInitStruct) ;
	//ADC_IN3
	ADC1ChannelInitStruct.Channel      = ADC_CHANNEL_3 ;
	ADC1ChannelInitStruct.Rank         = ADC_REGULAR_RANK_4;
	HAL_ADC_ConfigChannel(&ADC1InitStruct, &ADC1ChannelInitStruct) ;
	//ADC_IN4
	ADC1ChannelInitStruct.Channel      = ADC_CHANNEL_4 ;
	ADC1ChannelInitStruct.Rank         = ADC_REGULAR_RANK_5;
	HAL_ADC_ConfigChannel(&ADC1InitStruct, &ADC1ChannelInitStruct) ;
	//ADC_IN5
	ADC1ChannelInitStruct.Channel      = ADC_CHANNEL_5 ;
	ADC1ChannelInitStruct.Rank         = ADC_REGULAR_RANK_6;
	HAL_ADC_ConfigChannel(&ADC1InitStruct, &ADC1ChannelInitStruct) ;
	//ADC_IN6
	ADC1ChannelInitStruct.Channel      = ADC_CHANNEL_6 ;
	ADC1ChannelInitStruct.Rank         = ADC_REGULAR_RANK_7;
	HAL_ADC_ConfigChannel(&ADC1InitStruct, &ADC1ChannelInitStruct) ;
	//ADC_IN7
	ADC1ChannelInitStruct.Channel      = ADC_CHANNEL_7 ;
	ADC1ChannelInitStruct.Rank         = ADC_REGULAR_RANK_8;
	HAL_ADC_ConfigChannel(&ADC1InitStruct, &ADC1ChannelInitStruct) ;

}


void DMA1_AdcInit(void)
{
/**
	* Fonksiyon Tanimi 	  : Bu fonksiyon DMA özelliklerini konfigüre eder 
	*	
	*					ADC modülü DMA kanal 1  
	*					DMA yönü ADC cevre biriminden hafizaya
	*					Cevre biriminin adresi ayni
	*					Hafiza adresi arttirilacak
	*					Cevre biriminin veri uzunlugu WORD bit
	*					Hafiza veri uzunlugu WORD
	*					Dönen mod(sürekli)
	*					Öncelik düsük
	*/
	__HAL_RCC_DMA1_CLK_ENABLE();

	DMA_ADCINITSTRUCT.Instance = DMA1_Channel1; 
  DMA_ADCINITSTRUCT.Init.Direction = DMA_PERIPH_TO_MEMORY;
  DMA_ADCINITSTRUCT.Init.PeriphInc = DMA_PINC_DISABLE;
  DMA_ADCINITSTRUCT.Init.MemInc = DMA_MINC_ENABLE;
  DMA_ADCINITSTRUCT.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  DMA_ADCINITSTRUCT.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  DMA_ADCINITSTRUCT.Init.Mode = DMA_CIRCULAR;
  DMA_ADCINITSTRUCT.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&DMA_ADCINITSTRUCT) ;
	
	__HAL_LINKDMA(&ADC1InitStruct,DMA_Handle,DMA_ADCINITSTRUCT);//ADC ve DMA arasi köprü olusturma
	
	HAL_ADC_Start_DMA(&ADC1InitStruct,(uint32_t*)adc_values,NR_OF_SENSOR);//DMA baslat
	
}

////////////////////////////////////////////////////[2-BOLUM]	SENSOR ICIN OLUSTURULAN VE KULLANIYA SUNULAN FONKSIYONLAR///////////////////////////////////////////////////////////

void QTR_SensorsInit(void)
{
	/**
		*	Fonksiyon Tanimi : Bu Fonksiyon QTR-8A sensörü için kullanilan çevre birimlerinin ayarlarini baslatir.
		* Parametre        : Bu Fonksiyon Parametre Almaz
		* Döntürülen Deger : Bu Fonksiyon Deger Döndürmez
		*/	
	ADC1_PinInit();
	ADC1_Init();
	ADC1_ChannelInit();
	DMA1_AdcInit();
}
	

void  QTR_PercentageValue(uint16_t* sensorPercentRate)
{
	/**
		* Fonksiyon Tanimi 	 : 12 Bitlik ADC kullanuldigi için sensörler 0-4096 arasi deger vermektedir.
		*											 Bu fonksiyon sensor degerlerini 0-100 arasinda degerlere çevirir.Bu islem sensorlerin 
		*											 verilerinin okunabilirligi ve yorumlanabilirligini arttirir.
		* Parametre        	 : Bu Fonksiyon isaretsiz 16 bitlik 8 Elemanli bir dizi alir. Yüzdelik degerler bu diziye atanir.
		* Döntürülen Deger 	 : Bu Fonksiyon Deger Döndürmez
		*/	
	
	/** 12BitVeri x 100/12BitÇözünürlük = YüzdeDeger 
	  */
	for(uint8_t sensorArray=0 ;sensorArray< NR_OF_SENSOR;sensorArray++)
		sensorPercentRate[sensorArray] = DmaSensorValues[sensorArray] * PERCENT / ADC_RESOLUTION ;
}


void QTR_SensorCalibrate(uint16_t* Sensor_MaxValue, uint16_t* Sensor_MinValue ) 
{/**
	 * Fonksiyon Tanimi 	 : Bu fonksiyonu ile sensorlerin okudugu max ve min degerlerini elde edebilirsiniz.											
	 * Parametre        	 : 1. parametre 16 bitlik 8 Elemanli bir dizi alir ve bu diziye sensorlerin Max degerleri atanir, 
	 *											 2. parametre 16 bitlik 8 Elemanli bir dizi alir ve bu diziye sensorlerin Min degerleri atanir
	 * Döntürülen Deger 	 : Bu Fonksiyon Deger Döndürmez
	 */

	uint16_t arrayShifter = 0 ;
	uint8_t dataArray = 0 ; // veri sirasi
	uint16_t sensorPercentRate[NR_OF_SENSOR]={0};
	int16_t maxValues[NR_OF_SENSOR],minValues[NR_OF_SENSOR] = {0} ;
 

	/**bir buffer olusturularak alinan veri örnekleri bu bufferda tutulur
		*/	
	for( dataArray=0;dataArray < DATA_LENGTH;dataArray++)
	{
		for(sensorArray=0 ;sensorArray<NR_OF_SENSOR; sensorArray++)
		{
			QTR_PercentageValue(sensorPercentRate) ;
			sensorDataBuffer[sensorArray][dataArray] = sensorPercentRate[sensorArray] ;

		}
	}
		
	/**veriler kücükden büyüge siralanir		
		*/	
	for ( sensorArray = 0; sensorArray < NR_OF_SENSOR; sensorArray++) // herbir sensor için 
	{
		for (uint8_t lap = 1; lap < DATA_LENGTH; lap++) // verileri veri sayisis kadar turlariz 
		{
			for(dataArray = 0; dataArray < DATA_LENGTH; dataArray++) // verileri tek tek gezeriz
			{
				if ( sensorDataBuffer[sensorArray][dataArray] > sensorDataBuffer[sensorArray][dataArray+1] ) 
				{ // siralama mantigi
					arrayShifter = sensorDataBuffer[sensorArray][dataArray]; /* sira kaydirma */
					sensorDataBuffer[sensorArray][dataArray] = sensorDataBuffer[sensorArray][dataArray+1];
					sensorDataBuffer[sensorArray][dataArray+1] = arrayShifter ;
				}
			}
		}
	}
	
	/** Önceki taramalarda göz önüne alinarak en büyük deger ve en küçük deger seçilir
		*/
	for(sensorArray = 0; sensorArray < NR_OF_SENSOR; sensorArray++)
	{
		maxValues[sensorArray] = sensorDataBuffer[sensorArray][(DATA_LENGTH - 1)];
		minValues[sensorArray] = sensorDataBuffer[sensorArray][0];
		
		if(maxValues[sensorArray] > Sensor_MaxValue[sensorArray])
			Sensor_MaxValue[sensorArray] = maxValues[sensorArray] ;
		
		if( minValues[sensorArray] < Sensor_MinValue[sensorArray] || Sensor_MinValue[sensorArray] == 0 )
				Sensor_MinValue[sensorArray] = minValues[sensorArray];
			
	}
	
}

void QRT_GetCalibratedVales(uint16_t* Sensor_MaxValue,uint16_t* Sensor_MinValue,uint16_t* CalibratedValues)
{
	/** Fonksiyon Tanimi 	 : Girilen Max ve Min degerler baz alinarak sensor degerleri bu degerler arasinda kalibre edilir.  
		* Parametre        	 : 1. parametre 16 bitlik 8 Elemanli bir dizi alir ve Bu dizilerdeki degerler Max degerler olarak kabul edilir, 
	  *											 2. parametre 16 bitlik 8 Elemanli bir dizi alir ve Bu dizilerdeki degerler Min degerler olarak kabul edilir,
		*											 3. parametre 16 bitlik 8 Elemanli bir dizi alir MAX ve MIN degerlerine göre sensorlerin kalibre edilmis degerleri atanir. 
		* Döntürülen Deger 	 : Bu Fonksiyon Deger Döndürmez  
		*/
	
	uint16_t PercentRate[NR_OF_SENSOR] = {0} ;
	
	for(sensorArray = 0; sensorArray < NR_OF_SENSOR; sensorArray++)
	{
	 QTR_PercentageValue(PercentRate);
	 CalibratedValues[sensorArray]=(PercentRate[sensorArray] - Sensor_MinValue[sensorArray])*(OUTMAX-OUTMIN)/(Sensor_MaxValue[sensorArray]-Sensor_MinValue[sensorArray])+ OUTMIN;
	}
	
}

int32_t QTR_GetLinePosition(uint16_t* calibratedValues)
{
	/** Fonksiyon Tanimi 	 : Kalibre edilen degerlerine göre matematiksel islem sonucu sensorun cizgi pozisyonunu bildirir. 
		* Parametre        	 : 1. parametre 16 bitlik 8 Elemanli bir dizi alir ve Kalibre edilmis degerlerdir. 
		* Döntürülen Deger 	 : 0-700 arasi tamsayi deger döndürür .    
	
	*/
	int32_t sumofSensorsValues,index = 0 ;
	int32_t linePosition = 0;
	
	sumofSensorsValues = calibratedValues[0]+calibratedValues[1]+calibratedValues[2]+calibratedValues[3]+calibratedValues[4]+calibratedValues[5]+calibratedValues[6]+calibratedValues[7] ;
	index = 0*calibratedValues[0]+100*calibratedValues[1]+200*calibratedValues[2]+300*calibratedValues[3]+400*calibratedValues[4]+500*calibratedValues[5]+600*calibratedValues[6]+700*calibratedValues[7] ;
	
	linePosition = index/sumofSensorsValues ;
	return linePosition;
}



