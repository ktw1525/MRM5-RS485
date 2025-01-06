/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RECORD_LEN 4
#define DIFF_STEP 2
#define TIME_MAX 1000000000
#define KEY_BUFLEN 256
#define CFG_ADDRESS 0x800F800

#define PAGE_SIZE 2048
#define CFG_PAGE_INDEX 31

enum {
	CFG_ADDRESS_RMS = 0,
	CFG_ADDRESS_PHASE,
	CFG_ADDRESS_OFFSET,
};

typedef struct {
	float rms;
	float phase;
	float cfg_rms;
	float cfg_phase;
	float cfg_offset;
} Configs;

typedef struct {
	uint32_t time;
	float val;
} timeData;

typedef struct {
	char keyInput[KEY_BUFLEN];
	int cursor;
	uint32_t keyTimeout;
	int flag_keyInput;
	int flag_uartDMAReady;
	int menuId;
} keyBoard;

typedef struct {
	timeData record[RECORD_LEN];
	int cursor;
} ADCDATA_Records;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
keyBoard Kb;
ADCDATA_Records Record;
Configs Cfg;

uint16_t adcBuf[8];
uint8_t uartBuf[2]={0,};
uint8_t adcflag = 1;

float *cfg_rms, *cfg_phase;
uint64_t _cfg_rms, _cfg_phase;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len);
uint32_t micros(void);
void FLASH_Words(uint32_t Address, uint64_t Data);
float diffval(ADCDATA_Records *rcd);
float ampNdeg2val(float amp, float deg, ADCDATA_Records *rcd);
uint32_t val2dacData(float val);
void flashWrite(int index, uint64_t value);
float flashRead(int index);
void processMenu(keyBoard *_Kb);
void printHelp(keyBoard *_Kb);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t dacval = 0;
	Record.cursor=0;
	Kb.keyTimeout=0;
	Kb.cursor=0;
	Kb.flag_uartDMAReady=0;
	Kb.menuId=0;

	Cfg.rms = 1;
	Cfg.phase = 0;
	Cfg.cfg_rms = flashRead(CFG_ADDRESS_RMS);
	Cfg.cfg_phase = flashRead(CFG_ADDRESS_PHASE);
	Cfg.cfg_offset = flashRead(CFG_ADDRESS_OFFSET);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  printHelp(&Kb);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){
  	  if(adcflag == 1){
   		  adcflag = 0;
   		  Record.record[Record.cursor].time = micros();
   		  Record.record[Record.cursor].val = (float)(adcBuf[0]-adcBuf[1]);
   		  Record.cursor=(Record.cursor + 1)%RECORD_LEN;

   		  dacval = val2dacData(ampNdeg2val(Cfg.rms, Cfg.phase, &Record));

   		  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacval);
       	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, 4);
   	  }

  	  if((Kb.keyTimeout > HAL_GetTick()) && (Kb.flag_keyInput==1)){
  		  Kb.flag_keyInput = 0;

  		  processMenu(&Kb);

  		  memset(Kb.keyInput,0,KEY_BUFLEN);
  		  Kb.cursor=0;
  		  printHelp(&Kb);
  	  }

  	  if(Kb.flag_uartDMAReady==0){
  		  Kb.flag_uartDMAReady=1;
  		  HAL_UART_Receive_DMA(&huart1, uartBuf, 1);
  	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_12CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */
  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */
  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  /* USER CODE END DAC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_NRST_Pin */
  GPIO_InitStruct.Pin = MCU_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MCU_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SYNC_INT_Pin IFLAG_Z_Pin TFLAG_Z_Pin IFLAG_1V6_Pin */
  GPIO_InitStruct.Pin = SYNC_INT_Pin|IFLAG_Z_Pin|TFLAG_Z_Pin|IFLAG_1V6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TFLAG_1V6_Pin */
  GPIO_InitStruct.Pin = TFLAG_1V6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TFLAG_1V6_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcflag = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
		Kb.keyTimeout = HAL_GetTick()+30000;
		if(uartBuf[0] == (uint8_t)'\n' || uartBuf[0] == (uint8_t)'\r'){
			Kb.flag_keyInput=1;
		}else{
			Kb.keyInput[Kb.cursor++]=uartBuf[0];
		}
		Kb.flag_uartDMAReady=0;
	}
}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, len);
  return len;
}

uint32_t micros(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);  // TIM2 카운터 값 읽기
}

void FLASH_Words(uint32_t Address, uint64_t Data)
{
  SET_BIT(FLASH->CR, FLASH_CR_PG);
  *(uint32_t *)Address = (uint32_t)Data;
  __ISB();
  *(uint32_t *)(Address + 4U) = (uint32_t)(Data >> 32U);
}

float diffval(ADCDATA_Records *rcd){
	int idx0, idx1;
	float result;
	idx0 = (rcd->cursor+RECORD_LEN-DIFF_STEP-1)%RECORD_LEN;
	idx1 = (rcd->cursor+RECORD_LEN-1)%RECORD_LEN;
	uint32_t difftime = (rcd->record[idx1].time + TIME_MAX - rcd->record[idx0].time) % TIME_MAX;
	float diffval = (rcd->record[idx1].val - rcd->record[idx0].val);
	result = diffval * 3000 / (float)difftime;
	if(difftime == 0){
		return 0;
	}
	return result;
}

float ampNdeg2val(float amp, float deg, ADCDATA_Records *rcd){
	int idx1 = (rcd->cursor+RECORD_LEN-2)%RECORD_LEN;
	float val = rcd->record[idx1].val;
	float a, b;
	a = amp * Cfg.cfg_rms * sqrt(2) * cos(M_PI * (deg + Cfg.cfg_phase) / 180.0f);
	b = amp * Cfg.cfg_rms * sqrt(2) * sin(M_PI * (deg + Cfg.cfg_phase) / 180.0f);
	float result = a * val + b * diffval(&Record);
	return result;
}

uint32_t val2dacData(float val){
	uint32_t result;
	result = (uint32_t)val + (uint32_t)Cfg.cfg_offset;
	if(result < 0) return 0;
	if(result > 4095) return 4095;
	return result;
}

void flashWrite(int index, uint64_t value){
	static int DoubleWordsNum = PAGE_SIZE/8;
	uint64_t cfg_datas[DoubleWordsNum];
	memcpy(cfg_datas, (void*)CFG_ADDRESS, PAGE_SIZE);
	cfg_datas[index] = value;

	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t pageError = 0;
	eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInitStruct.Page = CFG_PAGE_INDEX;
	eraseInitStruct.NbPages = 1;

	if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
		HAL_FLASH_Lock();
		return;
	}
	for(int i=0;i<DoubleWordsNum;i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CFG_ADDRESS + sizeof(uint64_t)*i, cfg_datas[i]);
	}
	HAL_FLASH_Lock();
}

float flashRead(int index){
	uint64_t cfg_datas[256];
	float *result;
	memcpy(cfg_datas, (void*)CFG_ADDRESS, 2048);
	result = (float*)&cfg_datas[index];
	return *result;
}

void processMenu(keyBoard *_Kb){
	uint64_t val;
	float result;
	float *value = (float*)&val;

	switch(_Kb->menuId){
	case 0:
		if(atoi(_Kb->keyInput)==1){
			_Kb->menuId=1;
		}else if(atoi(_Kb->keyInput)==2){
			_Kb->menuId=2;
		}else if(atoi(_Kb->keyInput)==3){
			_Kb->menuId=3;
		}else if(atoi(_Kb->keyInput)==4){
			_Kb->menuId=4;
		}else if(atoi(_Kb->keyInput)==5){
			_Kb->menuId=5;
		}else if(_Kb->keyInput[0]=='r'){
			printf("System Reset!\r\n");
			NVIC_SystemReset();
		}else{
			printf("Input Error\r\n");
		}
		break;
	case 1:
		Cfg.rms = atof(_Kb->keyInput);
		_Kb->menuId=0;
		printf("RMS set to %f\r\n", Cfg.rms);
		break;
	case 2:
		Cfg.phase = atof(_Kb->keyInput);
		_Kb->menuId=0;
		printf("Phase set to %f\r\n", Cfg.phase);
		break;
	case 3:
		*value = atof(_Kb->keyInput);
		flashWrite(CFG_ADDRESS_RMS, val);
		result = flashRead(CFG_ADDRESS_RMS);
		printf("Config RMS set to %f\r\n", result);
		_Kb->menuId=0;
		Cfg.cfg_rms = flashRead(CFG_ADDRESS_RMS);
		break;
	case 4:
		*value = atof(_Kb->keyInput);
		flashWrite(CFG_ADDRESS_PHASE, val);
		result = flashRead(CFG_ADDRESS_PHASE);
		printf("Config PHASE set to %f\r\n", result);
		_Kb->menuId=0;
		Cfg.cfg_phase = flashRead(CFG_ADDRESS_PHASE);
		break;
	case 5:
		*value = atof(_Kb->keyInput);
		flashWrite(CFG_ADDRESS_OFFSET, val);
		result = flashRead(CFG_ADDRESS_OFFSET);
		printf("Config OFFSET set to %f\r\n", result);
		_Kb->menuId=0;
		Cfg.cfg_offset = flashRead(CFG_ADDRESS_OFFSET);
		break;
	default:
		break;
	}
}

void printHelp(keyBoard *_Kb){
	switch(_Kb->menuId){
	case 0:
		printf("\e[2J\e[H");
		printf("[Calibaba v1.0]\r\n"); // 0
		printf("1. Config Output RMS(mA) [Current : %f]\r\n", Cfg.rms); // 1
		printf("2. Config Output Phase(Deg.) [Current : %f]\r\n", Cfg.phase); // 2
		printf("3. Calibrate RMS(mA) [Current : %f]\r\n", Cfg.cfg_rms); // 3
		printf("4. Calibrate PHASE(Deg.) [Current : %f]\r\n", Cfg.cfg_phase); // 4
		printf("5. Calibrate OFFSET [Current : %f]\r\n", Cfg.cfg_offset); // 5
		break;
	case 1:
		printf("Input RMS(mA) : \r\n"); //
		break;
	case 2:
		printf("Input Phase(Deg.) : \r\n"); //
		break;
	case 3:
		printf("Input Config RMS Value(mA) : \r\n"); //
		break;
	case 4:
		printf("Input Config PHASE Value(Deg.) : \r\n"); //
		break;
	case 5:
		printf("Input Config Offset : \r\n"); //
		break;
	default:
		break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
