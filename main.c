/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "usbd_audio_if.h"
#include "usbd_audio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
// Audio processing structures
typedef struct {
    int16_t samples[7][512];  // 7 microphones, 512 samples each
    uint32_t timestamp;
    uint8_t mic_levels[7];  // LED brightness levels for each mic
} AudioFrame_t;

// LED control structures
typedef struct {
    uint8_t brightness[7];  // 7 LEDs corresponding to 7 microphones
    uint8_t pattern;        // LED pattern mode
    uint8_t update_flag;    // Flag to indicate LED update needed
} LEDState_t;

// Audio processing functions
static void AudioProcessor_Init(void);
static void AudioProcessor_ProcessFrame(AudioFrame_t* frame);
static uint8_t AudioProcessor_GetMicLevel(uint8_t mic_index);
static void AudioProcessor_UpdateStats(void);

// LED control functions
static void LEDController_Init(void);
static void LEDController_SetBrightness(uint8_t led_index, uint8_t brightness);
static void LEDController_UpdatePattern(uint8_t pattern);
static void LEDController_Refresh(void);

// Microphone array functions
static void MicrophoneArray_Init(void);
static int MicrophoneArray_Capture(int16_t* buffer, uint32_t size);
static void MicrophoneArray_ConfigurePDM(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_BUFFER_SIZE 512
#define SAMPLE_RATE 48000
#define NUM_MICROPHONES 7
#define LED_REFRESH_RATE 100  // Hz
#define AUDIO_PROCESS_RATE 1000  // Hz
#define USB_AUDIO_START 0x01
#define USB_AUDIO_STOP 0x02

// Sipeed 6+1 Microphone Array Pin Definitions
#define MIC_D0_PIN GPIO_PIN_11
#define MIC_D0_PORT GPIOD
#define MIC_D1_PIN GPIO_PIN_0
#define MIC_D1_PORT GPIOA
#define MIC_D2_PIN GPIO_PIN_6
#define MIC_D2_PORT GPIOE
#define MIC_D3_PIN GPIO_PIN_3
#define MIC_D3_PORT GPIOE
#define MIC_CK_PIN GPIO_PIN_13
#define MIC_CK_PORT GPIOD
#define MIC_WS_PIN GPIO_PIN_12
#define MIC_WS_PORT GPIOD

// LED Control Pin Definitions
#define LED_CK_PIN GPIO_PIN_5
#define LED_CK_PORT GPIOA
#define LED_DA_PIN GPIO_PIN_7
#define LED_DA_PORT GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai2_a;
DMA_HandleTypeDef hdma_sai2_b;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AudioCapture */
osThreadId_t AudioCaptureHandle;
uint32_t AudioCaptureBuffer[ 2048 ];
osStaticThreadDef_t AudioCaptureControlBlock;
const osThreadAttr_t AudioCapture_attributes = {
  .name = "AudioCapture",
  .cb_mem = &AudioCaptureControlBlock,
  .cb_size = sizeof(AudioCaptureControlBlock),
  .stack_mem = &AudioCaptureBuffer[0],
  .stack_size = sizeof(AudioCaptureBuffer),
  .priority = (osPriority_t) osPriorityHigh6,
};
/* Definitions for USBAudio */
osThreadId_t USBAudioHandle;
uint32_t USBAudioBuffer[ 1024 ];
osStaticThreadDef_t USBAudioControlBlock;
const osThreadAttr_t USBAudio_attributes = {
  .name = "USBAudio",
  .cb_mem = &USBAudioControlBlock,
  .cb_size = sizeof(USBAudioControlBlock),
  .stack_mem = &USBAudioBuffer[0],
  .stack_size = sizeof(USBAudioBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal5,
};
/* Definitions for AudioProcess */
osThreadId_t AudioProcessHandle;
uint32_t AudioProcessBuffer[ 4096 ];
osStaticThreadDef_t AudioProcessControlBlock;
const osThreadAttr_t AudioProcess_attributes = {
  .name = "AudioProcess",
  .cb_mem = &AudioProcessControlBlock,
  .cb_size = sizeof(AudioProcessControlBlock),
  .stack_mem = &AudioProcessBuffer[0],
  .stack_size = sizeof(AudioProcessBuffer),
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for LEDControl */
osThreadId_t LEDControlHandle;
uint32_t LEDControlBuffer[ 512 ];
osStaticThreadDef_t LEDControlControlBlock;
const osThreadAttr_t LEDControl_attributes = {
  .name = "LEDControl",
  .cb_mem = &LEDControlControlBlock,
  .cb_size = sizeof(LEDControlControlBlock),
  .stack_mem = &LEDControlBuffer[0],
  .stack_size = sizeof(LEDControlBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal2,
};
/* Definitions for AudioDataQueue */
osMessageQueueId_t AudioDataQueueHandle;
const osMessageQueueAttr_t AudioDataQueue_attributes = {
  .name = "AudioDataQueue"
};
/* Definitions for LEDCommandQueue */
osMessageQueueId_t LEDCommandQueueHandle;
const osMessageQueueAttr_t LEDCommandQueue_attributes = {
  .name = "LEDCommandQueue"
};
/* Definitions for USBControlQueue */
osMessageQueueId_t USBControlQueueHandle;
const osMessageQueueAttr_t USBControlQueue_attributes = {
  .name = "USBControlQueue"
};
/* Definitions for LEDRefreshTimer */
osTimerId_t LEDRefreshTimerHandle;
const osTimerAttr_t LEDRefreshTimer_attributes = {
  .name = "LEDRefreshTimer"
};
/* Definitions for AudioStatsTimer */
osTimerId_t AudioStatsTimerHandle;
const osTimerAttr_t AudioStatsTimer_attributes = {
  .name = "AudioStatsTimer"
};
/* Definitions for USBHeartbeatTimer */
osTimerId_t USBHeartbeatTimerHandle;
const osTimerAttr_t USBHeartbeatTimer_attributes = {
  .name = "USBHeartbeatTimer"
};
/* Definitions for AudioBufferMutex */
osMutexId_t AudioBufferMutexHandle;
const osMutexAttr_t AudioBufferMutex_attributes = {
  .name = "AudioBufferMutex"
};
/* Definitions for USBStateMutex */
osMutexId_t USBStateMutexHandle;
const osMutexAttr_t USBStateMutex_attributes = {
  .name = "USBStateMutex"
};
/* Definitions for LEDDataMutex */
osMutexId_t LEDDataMutexHandle;
const osMutexAttr_t LEDDataMutex_attributes = {
  .name = "LEDDataMutex"
};
/* Definitions for AudioBufferReady */
osSemaphoreId_t AudioBufferReadyHandle;
const osSemaphoreAttr_t AudioBufferReady_attributes = {
  .name = "AudioBufferReady"
};
/* Definitions for USBTransferComplete */
osSemaphoreId_t USBTransferCompleteHandle;
const osSemaphoreAttr_t USBTransferComplete_attributes = {
  .name = "USBTransferComplete"
};
/* Definitions for LEDUpdateReady */
osSemaphoreId_t LEDUpdateReadyHandle;
const osSemaphoreAttr_t LEDUpdateReady_attributes = {
  .name = "LEDUpdateReady"
};
/* Definitions for AudioBufferCount */
osSemaphoreId_t AudioBufferCountHandle;
const osSemaphoreAttr_t AudioBufferCount_attributes = {
  .name = "AudioBufferCount"
};
/* Definitions for ProcessingComplete */
osSemaphoreId_t ProcessingCompleteHandle;
const osSemaphoreAttr_t ProcessingComplete_attributes = {
  .name = "ProcessingComplete"
};
/* USER CODE BEGIN PV */
// Global audio and LED state
AudioFrame_t current_audio_frame;
LEDState_t led_state;
volatile uint8_t usb_audio_ready = 0;
volatile uint8_t audio_capture_active = 0;

// Add this external declaration
extern USBD_HandleTypeDef hUsbDeviceFS;

// Audio processing variables
static uint32_t mic_energy[7] = {0};
static uint8_t mic_levels[7] = {0};
static uint32_t audio_stats_counter = 0;

// LED control variables
static uint8_t led_brightness[7] = {0};
static uint8_t led_pattern = 0;

// Audio buffers for DMA capture
static int16_t mic_buffer_0[AUDIO_BUFFER_SIZE * 2];  // For SAI1 Block A (2 mics)
static int16_t mic_buffer_1[AUDIO_BUFFER_SIZE * 2];  // For SAI1 Block B (2 mics)
static int16_t mic_buffer_2[AUDIO_BUFFER_SIZE * 2];  // For SAI2 Block A (2 mics)
static int16_t mic_buffer_3[AUDIO_BUFFER_SIZE];      // For SAI2 Block B (1 mic)

// Sound localization variables
static float mic_angles[7] = {0.0f, 51.4f, 102.8f, 154.3f, 205.7f, 257.1f, 308.6f}; // Degrees for circular array
static uint8_t dominant_mic = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SAI1_Init(void);
static void MX_SAI2_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void LEDRefreshCallback(void *argument);
void AudioStatsCallback(void *argument);
void USBHeartbeatCallback(void *argument);

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Initialize microphone array and LED controller
  MicrophoneArray_ConfigurePDM();
  LEDController_Init();
  
  // Start LED refresh timer
  osTimerStart(LEDRefreshTimerHandle, 1000 / LED_REFRESH_RATE);
  osTimerStart(AudioStatsTimerHandle, 1000);
  osTimerStart(USBHeartbeatTimerHandle, 100);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of AudioBufferMutex */
  AudioBufferMutexHandle = osMutexNew(&AudioBufferMutex_attributes);

  /* creation of USBStateMutex */
  USBStateMutexHandle = osMutexNew(&USBStateMutex_attributes);

  /* creation of LEDDataMutex */
  LEDDataMutexHandle = osMutexNew(&LEDDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of AudioBufferReady */
  AudioBufferReadyHandle = osSemaphoreNew(1, 1, &AudioBufferReady_attributes);

  /* creation of USBTransferComplete */
  USBTransferCompleteHandle = osSemaphoreNew(1, 1, &USBTransferComplete_attributes);

  /* creation of LEDUpdateReady */
  LEDUpdateReadyHandle = osSemaphoreNew(1, 1, &LEDUpdateReady_attributes);

  /* creation of AudioBufferCount */
  AudioBufferCountHandle = osSemaphoreNew(4, 0, &AudioBufferCount_attributes);

  /* creation of ProcessingComplete */
  ProcessingCompleteHandle = osSemaphoreNew(2, 0, &ProcessingComplete_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of LEDRefreshTimer */
  LEDRefreshTimerHandle = osTimerNew(LEDRefreshCallback, osTimerPeriodic, NULL, &LEDRefreshTimer_attributes);

  /* creation of AudioStatsTimer */
  AudioStatsTimerHandle = osTimerNew(AudioStatsCallback, osTimerPeriodic, NULL, &AudioStatsTimer_attributes);

  /* creation of USBHeartbeatTimer */
  USBHeartbeatTimerHandle = osTimerNew(USBHeartbeatCallback, osTimerPeriodic, NULL, &USBHeartbeatTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of AudioDataQueue */
  AudioDataQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &AudioDataQueue_attributes);

  /* creation of LEDCommandQueue */
  LEDCommandQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &LEDCommandQueue_attributes);

  /* creation of USBControlQueue */
  USBControlQueueHandle = osMessageQueueNew (8, sizeof(uint16_t), &USBControlQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of AudioCapture */
  AudioCaptureHandle = osThreadNew(StartTask02, NULL, &AudioCapture_attributes);

  /* creation of USBAudio */
  USBAudioHandle = osThreadNew(StartTask03, NULL, &USBAudio_attributes);

  /* creation of AudioProcess */
  AudioProcessHandle = osThreadNew(StartTask04, NULL, &AudioProcess_attributes);

  /* creation of LEDControl */
  LEDControlHandle = osThreadNew(StartTask05, NULL, &LEDControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.Sai23ClockSelection = RCC_SAI23CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockA2.Init.Synchro = SAI_SYNCHRONOUS_EXT_SAI1;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA2.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB2.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockB2.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB2.Init.MonoStereoMode = SAI_MONOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Configure microphone array pins
  // MIC D0 = PD11 (SAI1_SD_A)
  GPIO_InitStruct.Pin = MIC_D0_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(MIC_D0_PORT, &GPIO_InitStruct);

  // MIC D1 = PA0 (SAI1_SD_B)
  GPIO_InitStruct.Pin = MIC_D1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(MIC_D1_PORT, &GPIO_InitStruct);

  // MIC D2 = PE6 (SAI2_SD_A)
  GPIO_InitStruct.Pin = MIC_D2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(MIC_D2_PORT, &GPIO_InitStruct);

  // MIC D3 = PE3 (SAI2_SD_B)
  GPIO_InitStruct.Pin = MIC_D3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(MIC_D3_PORT, &GPIO_InitStruct);

  // MIC CK = PD13 (SAI1_SCK_A - Master Clock)
  GPIO_InitStruct.Pin = MIC_CK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(MIC_CK_PORT, &GPIO_InitStruct);

  // MIC WS = PD12 (SAI1_FS_A - Word Select/Frame Sync)
  GPIO_InitStruct.Pin = MIC_WS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(MIC_WS_PORT, &GPIO_InitStruct);

  // Configure LED control pins
  // LED CK = PA5 (SPI1_SCK)
  GPIO_InitStruct.Pin = LED_CK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(LED_CK_PORT, &GPIO_InitStruct);

  // LED DA = PA7 (SPI1_MOSI)
  GPIO_InitStruct.Pin = LED_DA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(LED_DA_PORT, &GPIO_InitStruct);

  // Configure LED chip select pin (PA4)
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS high initially
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Audio processing functions
static void AudioProcessor_Init(void) {
    memset(mic_energy, 0, sizeof(mic_energy));
    memset(mic_levels, 0, sizeof(mic_levels));
    audio_stats_counter = 0;
    dominant_mic = 0;
}

static void AudioProcessor_ProcessFrame(AudioFrame_t* frame) {
    uint32_t max_energy = 0;
    uint8_t max_mic = 0;
    
    // Calculate energy for each microphone
    for (int mic = 0; mic < NUM_MICROPHONES; mic++) {
        uint32_t energy = 0;
        for (int sample = 0; sample < AUDIO_BUFFER_SIZE; sample++) {
            int32_t sample_val = frame->samples[mic][sample];
            energy += (uint32_t)(sample_val * sample_val);
        }
        mic_energy[mic] = energy / AUDIO_BUFFER_SIZE;

        // Find dominant microphone for sound localization
        if (energy > max_energy) {
            max_energy = energy;
            max_mic = mic;
        }

        // Convert energy to LED brightness (0-255)
        uint32_t log_energy = (uint32_t)(log10f((float)energy + 1.0f) * 50.0f);
        mic_levels[mic] = (uint8_t)(log_energy > 255 ? 255 : log_energy);
    }
    
    dominant_mic = max_mic;
}

static uint8_t AudioProcessor_GetMicLevel(uint8_t mic_index) {
    if (mic_index < NUM_MICROPHONES) {
        return mic_levels[mic_index];
    }
    return 0;
}

static void AudioProcessor_UpdateStats(void) {
    audio_stats_counter++;
}

// LED control functions
static void LEDController_Init(void) {
    // Initialize SPI for LED control
    HAL_SPI_Init(&hspi1);
    memset(led_brightness, 0, sizeof(led_brightness));
    led_pattern = 0;
    
    // Clear all LEDs initially
    LEDController_Refresh();
}

static void LEDController_SetBrightness(uint8_t led_index, uint8_t brightness) {
    if (led_index < NUM_MICROPHONES) {
        led_brightness[led_index] = brightness;
    }
}

static void LEDController_UpdatePattern(uint8_t pattern) {
    led_pattern = pattern;
}

static void LEDController_Refresh(void) {
    // Send LED data via SPI to LED controller
    // Format: Start frame + LED data + End frame for WS2812B-like LEDs
    uint8_t led_data[NUM_MICROPHONES * 3 + 8]; // RGB data + start/end frames
    uint32_t idx = 0;
    
    // Start frame (4 bytes of 0x00)
    for (int i = 0; i < 4; i++) {
        led_data[idx++] = 0x00;
    }
    
    // LED data (3 bytes per LED: G, R, B)
    for (int i = 0; i < NUM_MICROPHONES; i++) {
        uint8_t brightness = led_brightness[i];
        
        // Create color based on microphone position and level
        uint8_t red = 0, green = 0, blue = 0;
        
        if (i == dominant_mic && brightness > 50) {
            // Dominant microphone - bright white/yellow
            red = brightness;
            green = brightness;
            blue = brightness / 4;
        } else if (brightness > 20) {
            // Active microphone - colored by position
            float angle = mic_angles[i] * M_PI / 180.0f;
            red = (uint8_t)(brightness * (0.5f + 0.5f * cosf(angle)));
            green = (uint8_t)(brightness * (0.5f + 0.5f * cosf(angle + 2.0f * M_PI / 3.0f)));
            blue = (uint8_t)(brightness * (0.5f + 0.5f * cosf(angle + 4.0f * M_PI / 3.0f)));
        }
        
        led_data[idx++] = green;  // Green first for WS2812B
        led_data[idx++] = red;
        led_data[idx++] = blue;
    }
    
    // End frame (4 bytes of 0xFF)
    for (int i = 0; i < 4; i++) {
        led_data[idx++] = 0xFF;
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS low
    HAL_SPI_Transmit(&hspi1, led_data, idx, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS high
}

// Microphone array functions
static void MicrophoneArray_ConfigurePDM(void) {
    // Configure SAI for PDM microphone input
    // SAI1 Block A is master, others are slaves synchronized to it
    
    // Configure SAI1 for PDM input (4 microphones via 2 blocks)
    hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
    hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
    hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
    
    hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai_BlockB1.Init.DataSize = SAI_DATASIZE_16;
    hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
    hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE; // Opposite edge for stereo
    
    // Configure SAI2 for additional microphones (3 more microphones)
    hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai_BlockA2.Init.DataSize = SAI_DATASIZE_16;
    hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
    hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
    
    hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai_BlockB2.Init.DataSize = SAI_DATASIZE_16;
    hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
    hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
}

static void MicrophoneArray_Init(void) {
    // Initialize SAI for audio capture
    HAL_SAI_Init(&hsai_BlockA1);
    HAL_SAI_Init(&hsai_BlockB1);
    HAL_SAI_Init(&hsai_BlockA2);
    HAL_SAI_Init(&hsai_BlockB2);

    // Start SAI DMA reception for all microphone channels
    HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)mic_buffer_0, AUDIO_BUFFER_SIZE * 2 * 2); // 2 channels, 2 bytes per sample
    HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)mic_buffer_1, AUDIO_BUFFER_SIZE * 2 * 2);
    HAL_SAI_Receive_DMA(&hsai_BlockA2, (uint8_t*)mic_buffer_2, AUDIO_BUFFER_SIZE * 2 * 2);
    HAL_SAI_Receive_DMA(&hsai_BlockB2, (uint8_t*)mic_buffer_3, AUDIO_BUFFER_SIZE * 2);     // 1 channel
}

static int MicrophoneArray_Capture(int16_t* buffer, uint32_t size) {
    // Interleave audio data from all 7 microphones
    // mic_buffer_0: mics 0,1 (stereo)
    // mic_buffer_1: mics 2,3 (stereo)  
    // mic_buffer_2: mics 4,5 (stereo)
    // mic_buffer_3: mic 6 (mono)
    
    for (uint32_t sample = 0; sample < AUDIO_BUFFER_SIZE && sample * NUM_MICROPHONES < size; sample++) {
        // Microphones 0 and 1 from SAI1 Block A
        buffer[sample * NUM_MICROPHONES + 0] = mic_buffer_0[sample * 2];     // Left channel
        buffer[sample * NUM_MICROPHONES + 1] = mic_buffer_0[sample * 2 + 1]; // Right channel
        
        // Microphones 2 and 3 from SAI1 Block B
        buffer[sample * NUM_MICROPHONES + 2] = mic_buffer_1[sample * 2];
        buffer[sample * NUM_MICROPHONES + 3] = mic_buffer_1[sample * 2 + 1];
        
        // Microphones 4 and 5 from SAI2 Block A
        buffer[sample * NUM_MICROPHONES + 4] = mic_buffer_2[sample * 2];
        buffer[sample * NUM_MICROPHONES + 5] = mic_buffer_2[sample * 2 + 1];
        
        // Microphone 6 from SAI2 Block B (mono)
        buffer[sample * NUM_MICROPHONES + 6] = mic_buffer_3[sample];
    }
    
    return 0; // Success
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the AudioCapture thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  // Initialize microphone array
  MicrophoneArray_Init();

  // Start audio capture
  audio_capture_active = 1;

  /* Infinite loop */
  for(;;)
  {
    if (audio_capture_active) {
      // Capture audio from all 7 microphones via SAI
      int16_t audio_buffer[AUDIO_BUFFER_SIZE * NUM_MICROPHONES];

      if (MicrophoneArray_Capture(audio_buffer, AUDIO_BUFFER_SIZE * NUM_MICROPHONES) == 0) {

        // Copy audio data to frame structure for processing
        for (int mic = 0; mic < NUM_MICROPHONES; mic++) {
          for (int sample = 0; sample < AUDIO_BUFFER_SIZE; sample++) {
            current_audio_frame.samples[mic][sample] =
                audio_buffer[sample * NUM_MICROPHONES + mic];
          }
        }

        current_audio_frame.timestamp = HAL_GetTick();

        // Send audio data to processing queue
        uint16_t audio_data_ready = 1;
        osMessageQueuePut(AudioDataQueueHandle, &audio_data_ready, 0, 0);

        // Signal audio buffer is ready
        osSemaphoreRelease(AudioBufferReadyHandle);
        
        // Send audio data to USB host (interleaved format)
        if (usb_audio_ready) {
          // The USB audio interface expects interleaved audio data
          // This will be handled by the USB audio callback functions
        }
      }
    }

    osDelay(1); // 1ms delay for 1kHz capture rate
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the USBAudio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  // Initialize USB audio interface
  USBD_AUDIO_RegisterInterface(&hUsbDeviceFS, &USBD_AUDIO_fops_FS);

  /* Infinite loop */
  for(;;)
  {
    // Wait for USB audio to be ready
    if (usb_audio_ready) {
      // Process USB audio streaming
      uint16_t usb_control_msg;
      if (osMessageQueueGet(USBControlQueueHandle, &usb_control_msg, NULL, 10) == osOK) {
        // Handle USB audio control messages
        switch(usb_control_msg) {
          case USB_AUDIO_START:
            audio_capture_active = 1;
            break;
          case USB_AUDIO_STOP:
            audio_capture_active = 0;
            break;
        }
      }
    }

    osDelay(10); // 10ms delay
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the AudioProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  // Initialize audio processor
  AudioProcessor_Init();

  /* Infinite loop */
  for(;;)
  {
    // Wait for audio data
    uint16_t audio_ready;
    if (osMessageQueueGet(AudioDataQueueHandle, &audio_ready, NULL, osWaitForever) == osOK) {

      // Process audio for sound localization
      AudioProcessor_ProcessFrame(&current_audio_frame);

      // Calculate LED levels based on microphone activity
      for (int i = 0; i < NUM_MICROPHONES; i++) {
        led_state.brightness[i] = AudioProcessor_GetMicLevel(i);
      }

      // Signal LED update
      osSemaphoreRelease(LEDUpdateReadyHandle);

      // Signal processing complete
      osSemaphoreRelease(ProcessingCompleteHandle);
    }

    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the LEDControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  // Initialize LED controller
  LEDController_Init();

  /* Infinite loop */
  for(;;)
  {
    // Wait for LED update signal
    if (osSemaphoreAcquire(LEDUpdateReadyHandle, 100) == osOK) {

      // Update LED brightness based on microphone levels
      for (int i = 0; i < NUM_MICROPHONES; i++) {
        LEDController_SetBrightness(i, led_state.brightness[i]);
      }

      // Apply LED pattern
      LEDController_UpdatePattern(led_state.pattern);
    }

    osDelay(10); // 10ms delay
  }
  /* USER CODE END StartTask05 */
}

/* LEDRefreshCallback function */
void LEDRefreshCallback(void *argument)
{
  /* USER CODE BEGIN LEDRefreshCallback */
  // Refresh LED display
  LEDController_Refresh();

  // Signal LED update ready
  osSemaphoreRelease(LEDUpdateReadyHandle);
  /* USER CODE END LEDRefreshCallback */
}

/* AudioStatsCallback function */
void AudioStatsCallback(void *argument)
{
  /* USER CODE BEGIN AudioStatsCallback */
  // Update audio statistics
  AudioProcessor_UpdateStats();

  // Signal processing complete
  osSemaphoreRelease(ProcessingCompleteHandle);
  /* USER CODE END AudioStatsCallback */
}

/* USBHeartbeatCallback function */
void USBHeartbeatCallback(void *argument)
{
  /* USER CODE BEGIN USBHeartbeatCallback */
  // Check USB connection status
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    usb_audio_ready = 1;
  } else {
    usb_audio_ready = 0;
  }

  // Signal USB transfer complete
  osSemaphoreRelease(USBTransferCompleteHandle);
  /* USER CODE END USBHeartbeatCallback */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
  if (htim->Instance == TIM6)
  {
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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