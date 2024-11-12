/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Final_Sistemas_Control
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

_eProtocolo estadoProtocolo;

typedef enum{
       ACK=0x0D,
       ALIVE=0xF0,
//       IR_SENSOR=0xA0,
       OTHERS
   }_eID;

typedef struct{
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t indexStart; 	 ///////////////////// AGREGAR ///////////////////////////////////
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t cheksumtx;       //!< Cheksumm Tx
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   //!< Buffer circular de recepción
    uint8_t bufferTx[256];   //!< Buffer circular de transmisión
    // uint8_t payload[32];     //!< Buffer para el Payload de datos recibidos
}_sDato ;

_sDato datosComProtocol;

typedef union {
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
}_udat;

_udat myWord;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
void onDataRx(void);
void decodeProtocol(_sDato *);
void decodeData(_sDato *);
void encodeData(uint8_t id);
void sendData(void);

void USBReceive(uint8_t *but, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USBReceive(uint8_t *buf, uint16_t len){
	for (int i = 0; i < len; ++i) {
		datosComProtocol.bufferRx[datosComProtocol.indexWriteRx] = buf[i];
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim1){
		if(htim1->Instance == TIM1){
			uint32_t captured_value = HAL_TIM_ReadCapturedValue(htim1, TIM_CHANNEL_1);
		}
}

/*'<' header
//byte1
//byte2
//byte3
//byte4
//checksum = suma de todos los bytes transmitidos
//'>' tail

//  4      1      1    1    N     1
//HEADER NBYTES TOKEN ID PAYLOAD CKS

//HEADER 4 bytes
//'U' 'N' 'E' 'R'

//NBYTES = ID+PAYLOAD+CKS = 2 + nbytes de payload

//TOKEN: ':'

//CKS: xor de todos los bytes enviados menos el CKS
*/
void decodeProtocol(_sDato *datosCom){
	static uint8_t nBytes = 0;
	while (datosCom->indexReadRx != datosCom->indexWriteRx){
		switch (estadoProtocolo){
		case START:
			if (datosCom->bufferRx[datosCom->indexReadRx++]=='U') {
				estadoProtocolo = HEADER_1;
				datosCom->cheksumRx = 0;
			}
			break;
		case HEADER_1:
			if (datosCom->bufferRx[datosCom->indexReadRx++]=='N')
			   {
				   estadoProtocolo=HEADER_2;
			   }
			else{
				datosCom->indexReadRx--;
				estadoProtocolo=START;
			}
			break;
		case HEADER_2:
			if (datosCom->bufferRx[datosCom->indexReadRx++]=='E')
			{
				estadoProtocolo=HEADER_3;
			}
			else{
				datosCom->indexReadRx--;
			   estadoProtocolo=START;
			}
			break;
		case HEADER_3:
			if (datosCom->bufferRx[datosCom->indexReadRx++]=='R')
				{
					estadoProtocolo=NBYTES;
				}
			else{
				datosCom->indexReadRx--;
			   estadoProtocolo=START;
			}
			break;
		case NBYTES:
			datosCom->indexStart=datosCom->indexReadRx;
			nBytes=datosCom->bufferRx[datosCom->indexReadRx++];
			estadoProtocolo=TOKEN;
			break;
		case TOKEN:
			if (datosCom->bufferRx[datosCom->indexReadRx++]==':'){
			   estadoProtocolo=PAYLOAD;
				datosCom->cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
				// datosCom->payload[0]=nBytes;
				// indice=1;
			}
			else{
				datosCom->indexReadRx--;
				estadoProtocolo=START;
			}
			break;
		case PAYLOAD:
			if (nBytes>1){
				// datosCom->payload[indice++]=datosCom->bufferRx[datosCom->indexReadRx];
				datosCom->cheksumRx ^= datosCom->bufferRx[datosCom->indexReadRx++];
			}
			nBytes--;
			if(nBytes<=0){
				estadoProtocolo=START;
				if(datosCom->cheksumRx == datosCom->bufferRx[datosCom->indexReadRx]){
					decodeData(datosCom);
				}
			}
			break;
		default:
			estadoProtocolo=START;
			break;
		}
	}
}

void decodeData(_sDato *datosCom){
	#define POSID   2
    #define POSDATA 3

	switch (datosCom->bufferRx[(datosCom->indexStart+POSID)]) {
		case ALIVE:
			encodeData(ALIVE);
			break;
//		case MOTOR_ACTION:
//			datosCom->indexStart++;
//			myWord.ui8[0] = datosCom->bufferRx[datosCom->indexStart++];
//			myWord.ui8[1] = datosCom->bufferRx[datosCom->indexStart++];
//			motor.canal1 = (uint16_t)myWord.ui32;
//
//			myWord.ui8[0] = datosCom->bufferRx[datosCom->indexStart++];
//			myWord.ui8[1] = datosCom->bufferRx[datosCom->indexStart++];
//			motor.canal2 = (uint16_t)myWord.ui32;
//
//			myWord.ui8[0] = datosCom->bufferRx[datosCom->indexStart++];
//			myWord.ui8[1] = datosCom->bufferRx[datosCom->indexStart++];
//			motor.canal3 = (uint16_t)myWord.ui32;
//
//			myWord.ui8[0] = datosCom->bufferRx[datosCom->indexStart++];
//			myWord.ui8[1] = datosCom->bufferRx[datosCom->indexStart++];
//			motor.canal4 = (uint16_t)myWord.ui32;
//
//			encodeData(MOTOR_ACTION);
//			break;

		default:
//			auxBuffTx[indiceAux++]=0xDD;
//			auxBuffTx[NBYTES]=0x02;
			break;
	}
}

void encodeData(uint8_t id){
	uint8_t auxBuffTx[50], indiceAux=0, cheksum;
	auxBuffTx[indiceAux++]='U';
	auxBuffTx[indiceAux++]='N';
	auxBuffTx[indiceAux++]='E';
	auxBuffTx[indiceAux++]='R';
	auxBuffTx[indiceAux++]=0;
	auxBuffTx[indiceAux++]=':';

	switch (id) {
	case ALIVE:
		auxBuffTx[indiceAux++]=ALIVE;
		auxBuffTx[indiceAux++]=ACK;
		auxBuffTx[NBYTES]=0x03;
		break;
//	case IR_SENSOR:
//		auxBuffTx[indiceAux++]=IR_SENSOR;
//		auxBuffTx[NBYTES]=0x12; //decimal= 18
//
//		//myWord.ui32 = ir.sensor1[ir.count];
//		myWord.ui32 = ir.promS0;
//		auxBuffTx[indiceAux++] = myWord.ui8[0];
//		auxBuffTx[indiceAux++] = myWord.ui8[1];
//		break;

		default:
			auxBuffTx[indiceAux++]=0xDD;
			auxBuffTx[NBYTES]=0x02;
			break;
	}
	cheksum=0;
	for(uint8_t a=0 ;a < indiceAux ;a++)
	{
		cheksum ^= auxBuffTx[a];
		datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
	}
		datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=cheksum;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  CDC_AttachRxData(USBReceive);
  HAL_UART_Receive_IT(&huart1, &datosComProtocol.bufferRx[datosComProtocol.indexWriteRx], 1);


  datosComProtocol.indexWriteRx = 0;		//Init indice recepión del Buffer de Recepción
  datosComProtocol.indexReadRx = 0;			//Init indice de lectura del Buffer de Recepción
  datosComProtocol.indexWriteTx = 0;
  datosComProtocol.indexReadTx = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
	  HAL_Delay(1000);

	  if(datosComProtocol.indexReadRx != datosComProtocol.indexWriteRx){
	  		  decodeProtocol(&datosComProtocol);
	  }

	  if(datosComProtocol.indexReadTx != datosComProtocol.indexWriteTx){
		  lengthTx = datosComProtocol.indexWriteTx - datosComProtocol.indexReadTx;
		  if((CDC_Transmit_FS(&datosComProtocol.bufferTx[datosComProtocol.indexReadTx], lengthTx) == USBD_OK))
			  datosComProtocol.indexReadTx++;
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
