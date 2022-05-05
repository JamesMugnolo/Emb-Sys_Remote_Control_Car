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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32F413h_discovery_lcd.h"
#include <stdbool.h>
#include "stdio.h"
#include "rc_input_sbus.h"
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESISTOR_MAX 4095


//RX DEFS
#define RX_MAX 1810
#define RX_MIN 172
#define RX_MID_POINT 992
#define RX_SWITCH_LOW_THRESH 500
#define RX_SWITCH_HIGH_THRESH 1500
#define RX_DEADZONE_THRESH 50
#define RX_MAX_DIF_PER_FRAME 50

//RX CHANNEL DEFS
#define RX_VERTICAL_CH 1
#define RX_HORIZONTAL_CH 0
#define RX_ARM 4

//SWITCH POS DEFS
#define MAP_SWITCH_LOW 0
#define MAP_SWITCH_MID 1
#define MAP_SWITCH_HIGH 2

//MOTOR DEFS
#define MAP_MAX 100
#define MAP_MIN -100
#define MAP_MID ((MAP_MAX + MAP_MIN) / 2)
#define DUTY_CYCLE_MAX 2000
#define DUTY_CYCLE_MIN 1000
#define DUTY_CYCLE_THROTTLE_OFF 1488
#define DUTY_CYCLE_DISARM 1000 //THIS VAL MAYBE WRONG. TESTING REQUIRED
#define THROTTLE_SCALAR 0.1

//BATTERY DEFS
#define BAT_MAX_CELL_V 4.35
#define BAT_LOW_CELL_V 3.5 //If under low V for longer than 1 second set BAT_LVL_FG
#define BAT_MIN_CELL_V 3.2 //If ever goes under min set BAT_LVL_FG

//BUFFER FLAG DEFS
#define ARM_FG 0
#define RX_CON_FG 1
#define RX_FAILSAFE_FG 2
#define BAT_LVL_FG 3
#define THROTTLE_FG 4

//ARRAY DEFS
#define CHAN_VALS_SIZE 8
#define MAP_VALS_SIZE 8
#define MOTOR_VALS_SIZE 4
#define FLAG_BUFF_SIZE 5

//TIM DEFS
#define RADIO_READ_PERIOD 50
#define LCD_WRITE_PERIOD 3000


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char MapBuffer1[20];
char MapBuffer2[20];
char MapBuffer3[20];
char ChanBuffer1[20];
char ChanBuffer2[20];
char ChanBuffer3[20];
char MotorBuffer1[20];
char MotorBuffer2[20];
char FlagBufferChar[20];

uint16_t lastFewFrames[2][4] = {{RX_MID_POINT, RX_MID_POINT, RX_MID_POINT, RX_MID_POINT}, {RX_MID_POINT, RX_MID_POINT, RX_MID_POINT, RX_MID_POINT}};
int count = 0;

uint16_t ChannelVals[CHAN_VALS_SIZE] = {RX_MID_POINT, RX_MID_POINT, RX_MID_POINT, RX_MID_POINT, RX_MID_POINT, RX_MID_POINT, RX_MID_POINT, RX_MID_POINT};//Value of inputs as radio. Values [RX_MIN, RX_MAX].
float MappedVals[MAP_VALS_SIZE] = {MAP_MID,MAP_MID,MAP_MID,MAP_MID,MAP_SWITCH_LOW,MAP_SWITCH_LOW,MAP_SWITCH_LOW,MAP_SWITCH_LOW};//Value of inputs as percentage. Values within MAP_* or SWITCH_*.
int MotorVals[MOTOR_VALS_SIZE] = {DUTY_CYCLE_DISARM, DUTY_CYCLE_DISARM, DUTY_CYCLE_DISARM, DUTY_CYCLE_DISARM}; //2 or 4 size (how many signals). Length of duty cycle in microsec.
bool FlagBuffer[FLAG_BUFF_SIZE] = {0, 0, 0, 1, 1};//Flags! See *_FG defines.

bool CheckFlags()
{
	return FlagBuffer[ARM_FG] && FlagBuffer[RX_CON_FG]
		&& FlagBuffer[RX_FAILSAFE_FG] && FlagBuffer[BAT_LVL_FG];
}

bool CheckValidFrame(int channel)
{
	if((lastFewFrames[0][channel] + RX_MAX_DIF_PER_FRAME) <  lastFewFrames[1][channel])
	{
		return false;
	}
	if((lastFewFrames[0][channel] - RX_MAX_DIF_PER_FRAME) >  lastFewFrames[1][channel])
	{
		return false;
	}
	return true;
}

int MapRxToSwitch(uint16_t swVal)
{
	if(swVal < RX_SWITCH_LOW_THRESH)
		return MAP_SWITCH_LOW;
	else if(swVal > RX_SWITCH_HIGH_THRESH)
		return MAP_SWITCH_HIGH;
	else
		return MAP_SWITCH_MID;
}

float MapRxToPercent(uint16_t rxVal)
{
	float val = rxVal;

	//if we are within deadzone
	if (val <= (RX_MID_POINT + RX_DEADZONE_THRESH)
			&& val >= (RX_MID_POINT - RX_DEADZONE_THRESH))
	{
		return 0;
	}
	else
	{
		return ((((val - RX_MIN) * (MAP_MAX - MAP_MIN))
					/ (RX_MAX - RX_MIN)) + MAP_MIN);
	}
}

int MapPercentToMotor(float perVal)
{
	int retVal = DUTY_CYCLE_DISARM;
	//If any critical flag is unset, do not arm.
	if (CheckFlags())
	{
		float upperRange = DUTY_CYCLE_MAX - DUTY_CYCLE_THROTTLE_OFF;
		float lowerRange = DUTY_CYCLE_THROTTLE_OFF - DUTY_CYCLE_MIN;
		//If throttle scalar flag is set, normalize to scaled range.
		if(FlagBuffer[THROTTLE_FG])
		{
			upperRange = upperRange * THROTTLE_SCALAR;
			lowerRange = lowerRange * THROTTLE_SCALAR;
		}
		//If midpoint, turn off.
		if(perVal == MAP_MID)
		{
			retVal = DUTY_CYCLE_THROTTLE_OFF;
		}
		//Reverse
		else if(perVal < MAP_MID)
		{
			retVal = (DUTY_CYCLE_THROTTLE_OFF - (((perVal * -1)) * lowerRange)
					/ (MAP_MID - MAP_MIN));
		}
		//Forward
		else
		{
			retVal = ((((perVal - MAP_MID) * upperRange)
					/ (MAP_MAX - MAP_MID)) + DUTY_CYCLE_THROTTLE_OFF);
		}
	}
	return retVal;
}

uint8_t CalcBatterySize(int volts)
{
	uint8_t batSize = 0;
	if (volts > 2 * BAT_LOW_CELL_V)
	{
		if (volts < 2 * BAT_MAX_CELL_V)
			batSize = 2;
		else if (volts < 3 * BAT_MAX_CELL_V)
			batSize = 3;
		else if (volts < 4 * BAT_MAX_CELL_V)
			batSize = 4;
		else if (volts < 5 * BAT_MAX_CELL_V)
			batSize = 5; //SHOULD NEVER USE A 5S BATTERY
		else if (volts < 6 * BAT_MAX_CELL_V)
			batSize = 6; //THE CURRENT ESC CAN'T HANDLE OVER 5S BATTERIES
		else {} //WE HAVE A PROBLEM

	}
	else {} //Battery voltage is below minimum allowable size for ESC to operate

}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel7;

FMPI2C_HandleTypeDef hfmpi2c1;

I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s2;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart10;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart7_rx;

SRAM_HandleTypeDef hsram1;
SRAM_HandleTypeDef hsram2;

/* Definitions for Run_Motors */
osThreadId_t Run_MotorsHandle;
const osThreadAttr_t Run_Motors_attributes = {
  .name = "Run_Motors",
  .priority = (osPriority_t) osPriorityNormal4,
  .stack_size = 128 * 4
};
/* Definitions for Radio_Receiver */
osThreadId_t Radio_ReceiverHandle;
const osThreadAttr_t Radio_Receiver_attributes = {
  .name = "Radio_Receiver",
  .priority = (osPriority_t) osPriorityNormal7,
  .stack_size = 128 * 4
};
/* Definitions for Rx_Mapping */
osThreadId_t Rx_MappingHandle;
const osThreadAttr_t Rx_Mapping_attributes = {
  .name = "Rx_Mapping",
  .priority = (osPriority_t) osPriorityNormal6,
  .stack_size = 128 * 4
};
/* Definitions for Battery_Monitor */
osThreadId_t Battery_MonitorHandle;
const osThreadAttr_t Battery_Monitor_attributes = {
  .name = "Battery_Monitor",
  .priority = (osPriority_t) osPriorityNormal3,
  .stack_size = 128 * 4
};
/* Definitions for Data_To_LCD */
osThreadId_t Data_To_LCDHandle;
const osThreadAttr_t Data_To_LCD_attributes = {
  .name = "Data_To_LCD",
  .priority = (osPriority_t) osPriorityNormal2,
  .stack_size = 128 * 4
};
/* Definitions for Map_To_Motors */
osThreadId_t Map_To_MotorsHandle;
const osThreadAttr_t Map_To_Motors_attributes = {
  .name = "Map_To_Motors",
  .priority = (osPriority_t) osPriorityNormal5,
  .stack_size = 128 * 4
};
/* Definitions for RadioReadTim */
osTimerId_t RadioReadTimHandle;
const osTimerAttr_t RadioReadTim_attributes = {
  .name = "RadioReadTim"
};
/* Definitions for LCDDelayTim */
osTimerId_t LCDDelayTimHandle;
const osTimerAttr_t LCDDelayTim_attributes = {
  .name = "LCDDelayTim"
};
/* Definitions for RxInputMutex */
osMutexId_t RxInputMutexHandle;
const osMutexAttr_t RxInputMutex_attributes = {
  .name = "RxInputMutex"
};
/* USER CODE BEGIN PV */
uint16_t motor1Val;
uint16_t horizontalPos;
uint16_t vertiaclPos;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DFSDM2_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2S2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_UART10_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART7_Init(void);
void Start_Run_Motors(void *argument);
void Receive_Radio_Signal(void *argument);
void Start_Rx_Mapping(void *argument);
void Start_Battery_Monitor(void *argument);
void Start_Data_To_LCD(void *argument);
void Start_Map_To_Motors(void *argument);
void RadioReadTimCallBack(void *argument);
void LCDDelayTimCallback(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_DAC_Init();
  MX_DFSDM1_Init();
  MX_DFSDM2_Init();
  MX_FMPI2C1_Init();
  MX_FSMC_Init();
  MX_I2S2_Init();
  MX_QUADSPI_Init();
  MX_SDIO_SD_Init();
  MX_UART10_Init();
  MX_USART6_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_PB_Init(BUTTON_WAKEUP, BUTTON_MODE_GPIO);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  //BSP_LCD_DisplayStringAtLine(1, "Hello");
  //BSP_LCD_DisplayStringAt(0, 112, (uint8_t*)"Starting Project...", CENTER_MODE);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of RxInputMutex */
  RxInputMutexHandle = osMutexNew(&RxInputMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of RadioReadTim */
  RadioReadTimHandle = osTimerNew(RadioReadTimCallBack, osTimerPeriodic, NULL, &RadioReadTim_attributes);

  /* creation of LCDDelayTim */
  LCDDelayTimHandle = osTimerNew(LCDDelayTimCallback, osTimerPeriodic, NULL, &LCDDelayTim_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(RadioReadTimHandle, RADIO_READ_PERIOD);
  osTimerStart(LCDDelayTimHandle, LCD_WRITE_PERIOD);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Run_Motors */
  Run_MotorsHandle = osThreadNew(Start_Run_Motors, NULL, &Run_Motors_attributes);

  /* creation of Radio_Receiver */
  Radio_ReceiverHandle = osThreadNew(Receive_Radio_Signal, NULL, &Radio_Receiver_attributes);

  /* creation of Rx_Mapping */
  Rx_MappingHandle = osThreadNew(Start_Rx_Mapping, NULL, &Rx_Mapping_attributes);

  /* creation of Battery_Monitor */
  Battery_MonitorHandle = osThreadNew(Start_Battery_Monitor, NULL, &Battery_Monitor_attributes);

  /* creation of Data_To_LCD */
  Data_To_LCDHandle = osThreadNew(Start_Data_To_LCD, NULL, &Data_To_LCD_attributes);

  /* creation of Map_To_Motors */
  Map_To_MotorsHandle = osThreadNew(Start_Map_To_Motors, NULL, &Map_To_Motors_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48
                              |RCC_PERIPHCLK_FMPI2C1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 12;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_APB2;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_SYSCLK;
  PeriphClkInitStruct.PLLI2SSelection = RCC_PLLI2SCLKSOURCE_PLLSRC;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.Fmpi2c1ClockSelection = RCC_FMPI2C1CLKSOURCE_APB;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_MANCHESTER_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief DFSDM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM2_Init(void)
{

  /* USER CODE BEGIN DFSDM2_Init 0 */

  /* USER CODE END DFSDM2_Init 0 */

  /* USER CODE BEGIN DFSDM2_Init 1 */

  /* USER CODE END DFSDM2_Init 1 */
  hdfsdm2_channel1.Instance = DFSDM2_Channel1;
  hdfsdm2_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm2_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm2_channel1.Init.OutputClock.Divider = 2;
  hdfsdm2_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm2_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm2_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm2_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_MANCHESTER_RISING;
  hdfsdm2_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm2_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm2_channel1.Init.Awd.Oversampling = 1;
  hdfsdm2_channel1.Init.Offset = 0;
  hdfsdm2_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm2_channel7.Instance = DFSDM2_Channel7;
  hdfsdm2_channel7.Init.OutputClock.Activation = ENABLE;
  hdfsdm2_channel7.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm2_channel7.Init.OutputClock.Divider = 2;
  hdfsdm2_channel7.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm2_channel7.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm2_channel7.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm2_channel7.Init.SerialInterface.Type = DFSDM_CHANNEL_MANCHESTER_RISING;
  hdfsdm2_channel7.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm2_channel7.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm2_channel7.Init.Awd.Oversampling = 1;
  hdfsdm2_channel7.Init.Offset = 0;
  hdfsdm2_channel7.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM2_Init 2 */

  /* USER CODE END DFSDM2_Init 2 */

}

/**
  * @brief FMPI2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMPI2C1_Init(void)
{

  /* USER CODE BEGIN FMPI2C1_Init 0 */

  /* USER CODE END FMPI2C1_Init 0 */

  /* USER CODE BEGIN FMPI2C1_Init 1 */

  /* USER CODE END FMPI2C1_Init 1 */
  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0x00303D5B;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMPI2C1_Init 2 */

  /* USER CODE END FMPI2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 100000;
  huart7.Init.WordLength = UART_WORDLENGTH_9B;
  huart7.Init.StopBits = UART_STOPBITS_2;
  huart7.Init.Parity = UART_PARITY_EVEN;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART10_Init(void)
{

  /* USER CODE BEGIN UART10_Init 0 */

  /* USER CODE END UART10_Init 0 */

  /* USER CODE BEGIN UART10_Init 1 */

  /* USER CODE END UART10_Init 1 */
  huart10.Instance = UART10;
  huart10.Init.BaudRate = 115200;
  huart10.Init.WordLength = UART_WORDLENGTH_8B;
  huart10.Init.StopBits = UART_STOPBITS_1;
  huart10.Init.Parity = UART_PARITY_NONE;
  huart10.Init.Mode = UART_MODE_TX_RX;
  huart10.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart10.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART10_Init 2 */

  /* USER CODE END UART10_Init 2 */

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_RED_Pin|MEMS_LED_Pin|LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CTP_RST_Pin|LCD_TE_Pin|WIFI_WKUP_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_OTG_FS_PWR_EN_Pin|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_RED_Pin MEMS_LED_Pin LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LED1_RED_Pin|MEMS_LED_Pin|LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D5_Pin */
  GPIO_InitStruct.Pin = ARD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(ARD_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_INT_Pin */
  GPIO_InitStruct.Pin = CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B_USER_Pin */
  GPIO_InitStruct.Pin = B_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_GREEN_Pin */
  GPIO_InitStruct.Pin = LED2_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_Detect_Pin */
  GPIO_InitStruct.Pin = SD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D13_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
  HAL_GPIO_Init(ARD_D13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CTP_RST_Pin LCD_TE_Pin WIFI_WKUP_Pin PB8 */
  GPIO_InitStruct.Pin = LCD_CTP_RST_Pin|LCD_TE_Pin|WIFI_WKUP_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_Pin CODEC_INT_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_Pin|CODEC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin PG13 */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D12_Pin */
  GPIO_InitStruct.Pin = ARD_D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(ARD_D12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the SRAM2 memory initialization sequence
  */
  hsram2.Instance = FSMC_NORSRAM_DEVICE;
  hsram2.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram2.Init */
  hsram2.Init.NSBank = FSMC_NORSRAM_BANK3;
  hsram2.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram2.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram2.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram2.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram2.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram2.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram2.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram2.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram2.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram2.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram2.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram2.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram2.Init.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
  hsram2.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	__NOP(); //checks if we receive all data
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_Run_Motors */
/**
  * @brief  Function implementing the Run_Motors thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Run_Motors */
void Start_Run_Motors(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  //char buffer[20];
  //__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,5); //this compares tim3 channel 2s pulse  and changes the pulse if its not the same
  /* Infinite loop */
  //uint16_t resistorVal = 0;
  //uint16_t printVal;
  for(;;)
  {
	  //BSP_LCD_DisplayStringAtLine(2, "RUN MOTORS");
	  //HAL_ADC_Start(&hadc1);
	  //HAL_ADC_PollForConversion(&hadc1,50);
	  //resistorVal = HAL_ADC_GetValue(&hadc1);
	  //motor1Val = (DUTY_CYCLE_MAX * resistorVal) / RESISTOR_MAX;
	  //BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  //BSP_LCD_DisplayStringAtLine(1, buffer);
	  //itoa(motor1Val,buffer,10);
	  //BSP_LCD_SetTextColor(LCD_COLOR_RED);
	  //BSP_LCD_DisplayStringAtLine(1, buffer);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, MotorVals[RX_VERTICAL_CH]);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, MotorVals[RX_HORIZONTAL_CH]);
	  osThreadSuspend(Run_MotorsHandle);
	  //osDelay(5000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Receive_Radio_Signal */
/**
* @brief Function implementing the Radio_Receiver thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Receive_Radio_Signal */
void Receive_Radio_Signal(void *argument)
{
  /* USER CODE BEGIN Receive_Radio_Signal */
	/* Infinite loop */
	//char buffer[20];
	//itoa(motor1Val,buffer,10);
	SBUS sbus;
	sbus.arm = 0;
	sbus.disarm = 0;

	FlagBuffer[RX_CON_FG] = 0;
	FlagBuffer[RX_FAILSAFE_FG] = 0;
	FlagBuffer[RX_ARM] = 0;
	FlagBuffer[THROTTLE_FG] = 1;

	for(;;)
	{
		//BSP_LCD_ClearStringLine(3);
		//BSP_LCD_DisplayStringAtLine(3, "RECEIVE RADIO");
		osStatus_t status = osMutexAcquire(RxInputMutexHandle,osWaitForever);
		if (RC_READ_SBUS(&huart7 ,&sbus))
		{
			//verifying that sbus is reading properly and we are connected(not failsafing)
			//If we enter, we are connected now.
			if(!FlagBuffer[RX_CON_FG]){
				FlagBuffer[RX_CON_FG] = 1;
			}
			if(!sbus.failsafe){
				FlagBuffer[RX_FAILSAFE_FG] = 1;
			}
			else{
				FlagBuffer[RX_FAILSAFE_FG] = 0;
			}


			for(int i = 0; i < CHAN_VALS_SIZE; i++ ) {
			//	if(i < 4)
				//	lastFewFrames[count][i] = sbus.PWM_US_RC_CH[i];
				ChannelVals[i] = sbus.PWM_US_RC_CH[i];
			}

			if(MapRxToSwitch(ChannelVals[RX_ARM]) == MAP_SWITCH_HIGH)
				FlagBuffer[ARM_FG] = true;
			else
				FlagBuffer[ARM_FG] = false;

			count = (count + 1) % 2;
	  	}
		//Too many frames without connection.
		else if(sbus.error) {
			FlagBuffer[RX_CON_FG] = 0;
		}
		status = osMutexRelease(RxInputMutexHandle);
		osThreadSuspend(Radio_ReceiverHandle);
		//osDelay(5000);
	}
  /* USER CODE END Receive_Radio_Signal */
}

/* USER CODE BEGIN Header_Start_Rx_Mapping */
/**
* @brief Function implementing the Rx_Mapping thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Rx_Mapping */
void Start_Rx_Mapping(void *argument)
{
  /* USER CODE BEGIN Start_Rx_Mapping */
	/* Infinite loop */
	for(;;)
	{
		//BSP_LCD_DisplayStringAtLine(4, "RX MAPPING");
		if(CheckFlags())
		{
			for(int i = 0; i < MAP_VALS_SIZE; i++ ) {
				if(i < 4 && CheckValidFrame(i)) //First 4 channels are sticks, all others are switches
					MappedVals[i] = MapRxToPercent(ChannelVals[i]);
				else if(i >= 4)
					MappedVals[i] = MapRxToSwitch(ChannelVals[i]);
				//else keep the last value
			}
		}
		else //Flags are not good, set values to safe values
		{
			for(int i = 0; i < 8; i++)
			{
				if(i < 4) //First 4 channels are sticks, all others are switches
					MappedVals[i] = MAP_MID;
				else
					MappedVals[i] = MAP_SWITCH_LOW;
			}
		}
		//TODO:CALL MAP_TO_MOTOR WHEN VALUES CHANGE
		osThreadSuspend(Rx_MappingHandle);
		//osDelay(5000);
	}
  /* USER CODE END Start_Rx_Mapping */
}

/* USER CODE BEGIN Header_Start_Battery_Monitor */
/**
* @brief Function implementing the Battery_Monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Battery_Monitor */
void Start_Battery_Monitor(void *argument)
{
  /* USER CODE BEGIN Start_Battery_Monitor */
	/* Infinite loop */
	for(;;)
	{
		//BSP_LCD_DisplayStringAtLine(5, "BATTERY");
		//__NOP();
		osThreadSuspend(Battery_MonitorHandle);
		//osDelay(5000);
	}
  /* USER CODE END Start_Battery_Monitor */
}

/* USER CODE BEGIN Header_Start_Data_To_LCD */
/**
* @brief Function implementing the Data_To_LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Data_To_LCD */
void Start_Data_To_LCD(void *argument)
{
  /* USER CODE BEGIN Start_Data_To_LCD */
  /* Infinite loop */
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	for(;;)
	{
		//TODO: new order to display
		//Flags -> battery level -> motor values


/*		//Clearing
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAtLine(0, ChanBuffer1);
		BSP_LCD_DisplayStringAtLine(1, ChanBuffer2);
		BSP_LCD_DisplayStringAtLine(2, ChanBuffer3);

		BSP_LCD_DisplayStringAtLine(4, MapBuffer1);
		BSP_LCD_DisplayStringAtLine(5, MapBuffer2);
		BSP_LCD_DisplayStringAtLine(6, MapBuffer3);

		BSP_LCD_DisplayStringAtLine(8, "MotorVals:");
		BSP_LCD_DisplayStringAtLine(9, MotorBuffer1);
		BSP_LCD_DisplayStringAtLine(10, MotorBuffer2);

		BSP_LCD_DisplayStringAtLine(12, FlagBufferChar);*/


		sprintf(ChanBuffer1, "ChanVals: 1:%04d", ChannelVals[0]);
		sprintf(ChanBuffer2, "2:%04d 3:%04d", ChannelVals[1], ChannelVals[2]);
		sprintf(ChanBuffer3, "4:%04d 5:%04d", ChannelVals[3], ChannelVals[4]);

		sprintf(MapBuffer1, "MappedVals: 1:%04d", (int)MappedVals[0]);
		sprintf(MapBuffer2, "2:%04d 3:%04d", (int)MappedVals[1], (int)MappedVals[2]);
		sprintf(MapBuffer3, "4:%04d 5:%04d", (int)MappedVals[3], (int)MappedVals[4]);

		sprintf(MotorBuffer1, "1:%04d 2:%04d", MotorVals[0], MotorVals[1]);
		sprintf(MotorBuffer2, "3:%04d 4:%04d", MotorVals[2], MotorVals[3]);

		sprintf(FlagBufferChar, "FgBuf:[%1d,%1d,%1d,%1d,%1d]", FlagBuffer[0], FlagBuffer[1], FlagBuffer[2], FlagBuffer[3], FlagBuffer[4]);

		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DisplayStringAtLine(0, ChanBuffer1);
		BSP_LCD_DisplayStringAtLine(1, ChanBuffer2);
		BSP_LCD_DisplayStringAtLine(2, ChanBuffer3);

		BSP_LCD_DisplayStringAtLine(4, MapBuffer1);
		BSP_LCD_DisplayStringAtLine(5, MapBuffer2);
		BSP_LCD_DisplayStringAtLine(6, MapBuffer3);

		BSP_LCD_DisplayStringAtLine(8, "MotorVals:");
		BSP_LCD_DisplayStringAtLine(9, MotorBuffer1);
		BSP_LCD_DisplayStringAtLine(10, MotorBuffer2);

		BSP_LCD_DisplayStringAtLine(12, FlagBufferChar);

		osThreadSuspend(Data_To_LCDHandle);
		//osDelay(5000);
	}
  /* USER CODE END Start_Data_To_LCD */
}

/* USER CODE BEGIN Header_Start_Map_To_Motors */
/**
* @brief Function implementing the Map_To_Motors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Map_To_Motors */
void Start_Map_To_Motors(void *argument)
{
  /* USER CODE BEGIN Start_Map_To_Motors */
	/* Infinite loop */
	for(;;)
	{
		//BSP_LCD_DisplayStringAtLine(7, "MAP MOTORS");
		for(int i = 0; i < MOTOR_VALS_SIZE; i++)
		{
			MotorVals[i] = MapPercentToMotor(MappedVals[i]);
		}
		//BSP_LCD_DisplayStringAtLine(7, "MAP MOTORS");
		osThreadSuspend(Map_To_MotorsHandle);
		//osDelay(5000);
	}
  /* USER CODE END Start_Map_To_Motors */
}

/* RadioReadTimCallBack function */
void RadioReadTimCallBack(void *argument)
{
  /* USER CODE BEGIN RadioReadTimCallBack */
	osThreadResume(Radio_ReceiverHandle);
	osThreadResume(Rx_MappingHandle);
	osThreadResume(Map_To_MotorsHandle);
	osThreadResume(Run_MotorsHandle);
  /* USER CODE END RadioReadTimCallBack */
}

/* LCDDelayTimCallback function */
void LCDDelayTimCallback(void *argument)
{
  /* USER CODE BEGIN LCDDelayTimCallback */
	osThreadResume(Data_To_LCDHandle);
  /* USER CODE END LCDDelayTimCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
