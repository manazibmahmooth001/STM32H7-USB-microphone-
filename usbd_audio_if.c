/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_audio_if.c
  * @version        : v1.0_Cube
  * @brief          : Generic media access layer.
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
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */
#include "cmsis_os.h"
extern volatile uint8_t audio_capture_active;
extern osSemaphoreId_t USBTransferCompleteHandle;
extern osMessageQueueId_t USBControlQueueHandle;
extern osMessageQueueId_t LEDCommandQueueHandle;

// External audio frame structure
typedef struct {
    int16_t samples[7][512];  // 7 microphones, 512 samples each
    uint32_t timestamp;
    uint8_t mic_levels[7];  // LED brightness levels for each mic
} AudioFrame_t;

extern AudioFrame_t current_audio_frame;
extern volatile uint8_t usb_audio_ready;

// Audio streaming buffer for USB transmission
static int16_t usb_audio_buffer[512 * 7]; // Interleaved audio data for 7 channels
static volatile uint8_t buffer_ready = 0;
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_AUDIO_IF
  * @{
  */

/** @defgroup USBD_AUDIO_IF_Private_TypesDefinitions USBD_AUDIO_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Defines USBD_AUDIO_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Macros USBD_AUDIO_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Variables USBD_AUDIO_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_FunctionPrototypes USBD_AUDIO_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_FS(uint32_t options);
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol);
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_GetState_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static void PrepareAudioDataForUSB(void);
static void UpdateLEDsBasedOnAudio(void);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_FS =
{
  AUDIO_Init_FS,
  AUDIO_DeInit_FS,
  AUDIO_AudioCmd_FS,
  AUDIO_VolumeCtl_FS,
  AUDIO_MuteCtl_FS,
  AUDIO_PeriodicTC_FS,
  AUDIO_GetState_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the AUDIO media low layer over USB FS IP
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
  /* USER CODE BEGIN 0 */
  UNUSED(AudioFreq);
  UNUSED(Volume);
  UNUSED(options);
  
  // Initialize USB audio streaming for 7-channel microphone input
  usb_audio_ready = 1;
  audio_capture_active = 1;
  buffer_ready = 0;
  
  // Signal USB audio start
  uint16_t usb_start_msg = 0x01; // USB_AUDIO_START
  osMessageQueuePut(USBControlQueueHandle, &usb_start_msg, 0, 0);
  
  return (USBD_OK);
  /* USER CODE END 0 */
}

/**
  * @brief  De-Initializes the AUDIO media low layer
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_DeInit_FS(uint32_t options)
{
  /* USER CODE BEGIN 1 */
  UNUSED(options);
  
  // Stop USB audio streaming
  usb_audio_ready = 0;
  audio_capture_active = 0;
  
  // Signal USB audio stop
  uint16_t usb_stop_msg = 0x02; // USB_AUDIO_STOP
  osMessageQueuePut(USBControlQueueHandle, &usb_stop_msg, 0, 0);
  
  return (USBD_OK);
  /* USER CODE END 1 */
}

/**
  * @brief  Handles AUDIO command.
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 2 */
  switch(cmd)
  {
    case AUDIO_CMD_START:
      // Start audio streaming - enable microphone capture
      audio_capture_active = 1;
      usb_audio_ready = 1;
      
      // Prepare initial audio data for USB transmission
      PrepareAudioDataForUSB();
      
      // Update LEDs to show microphone activity
      UpdateLEDsBasedOnAudio();
      break;

    case AUDIO_CMD_PLAY:
      // For microphone input, this handles the streaming of captured audio to USB host
      if (size > 0 && pbuf != NULL && buffer_ready) {
        // Copy interleaved audio data to USB buffer
        memcpy(pbuf, usb_audio_buffer, size);
        buffer_ready = 0;
        
        // Prepare next audio frame
        PrepareAudioDataForUSB();
        
        // Update LEDs based on current microphone levels
        UpdateLEDsBasedOnAudio();
      }
      break;

    case AUDIO_CMD_STOP:
      // Stop audio streaming
      audio_capture_active = 0;
      usb_audio_ready = 0;
      buffer_ready = 0;
      
      // Turn off all LEDs
      uint16_t led_off_cmd = 0x00;
      osMessageQueuePut(LEDCommandQueueHandle, &led_off_cmd, 0, 0);
      break;
  }
  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Controls AUDIO Volume.
  * @param  vol: volume level (0..100)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol)
{
  /* USER CODE BEGIN 3 */
  UNUSED(vol);
  // Volume control for microphone input - could be used for gain control
  // For now, just return OK as gain is handled in hardware
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  Controls AUDIO Mute.
  * @param  cmd: command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd)
{
  /* USER CODE BEGIN 4 */
  if (cmd == 1) {
    // Mute - stop audio capture
    audio_capture_active = 0;
    
    // Turn off LEDs to indicate muted state
    uint16_t led_mute_cmd = 0x00;
    osMessageQueuePut(LEDCommandQueueHandle, &led_mute_cmd, 0, 0);
  } else {
    // Unmute - resume audio capture
    audio_capture_active = 1;
    
    // Resume LED activity indication
    uint16_t led_resume_cmd = 0x01;
    osMessageQueuePut(LEDCommandQueueHandle, &led_resume_cmd, 0, 0);
  }
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  AUDIO_PeriodicT_FS
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 5 */
  if (cmd == AUDIO_OUT_TC) {
    // This is for audio output (speaker), not used for microphone input
    // But we can use this for periodic USB audio data transmission
    
    if (usb_audio_ready && buffer_ready) {
      // Audio data is ready for transmission
      // The USB audio class driver will handle the actual transmission
      
      // Signal that transfer is complete
      osSemaphoreRelease(USBTransferCompleteHandle);
    }
  }
  
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Gets AUDIO State.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_GetState_FS(void)
{
  /* USER CODE BEGIN 6 */
  // Return the current state of the audio interface
  if (usb_audio_ready && audio_capture_active) {
    return (USBD_OK); // Audio is active and ready
  } else {
    return (USBD_FAIL); // Audio is not ready or inactive
  }
  /* USER CODE END 6 */
}

/**
  * @brief  Manages the DMA full transfer complete event.
  * @retval None
  */
void TransferComplete_CallBack_FS(void)
{
  /* USER CODE BEGIN 7 */
  // Signal USB transfer complete
  osSemaphoreRelease(USBTransferCompleteHandle);

  // Sync audio with USB timing
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_FULL);
  
  // Prepare next audio buffer for transmission
  PrepareAudioDataForUSB();
  /* USER CODE END 7 */
}

/**
  * @brief  Manages the DMA Half transfer complete event.
  * @retval None
  */
void HalfTransfer_CallBack_FS(void)
{
  /* USER CODE BEGIN 8 */
  // Sync audio with USB timing for half transfer
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_HALF);
  
  // Update LEDs based on current microphone activity
  UpdateLEDsBasedOnAudio();
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @brief  Prepares audio data from 7 microphones for USB transmission
  * @retval None
  */
static void PrepareAudioDataForUSB(void) {
  if (!audio_capture_active) {
    return;
  }
  
  // Interleave audio data from all 7 microphones for USB transmission
  for (int sample = 0; sample < 512; sample++) {
    for (int mic = 0; mic < 7; mic++) {
      usb_audio_buffer[sample * 7 + mic] = current_audio_frame.samples[mic][sample];
    }
  }
  
  buffer_ready = 1;
}

/**
  * @brief  Updates LEDs based on microphone audio levels
  * @retval None
  */
static void UpdateLEDsBasedOnAudio(void) {
  // Send LED update command based on microphone levels
  // The LED pattern will be handled by the LED control thread
  
  // Calculate average microphone activity
  uint32_t total_activity = 0;
  for (int i = 0; i < 7; i++) {
    total_activity += current_audio_frame.mic_levels[i];
  }
  
  // Send LED command based on activity level
  uint16_t led_cmd = (uint16_t)(total_activity / 7); // Average activity
  if (led_cmd > 255) led_cmd = 255;
  
  osMessageQueuePut(LEDCommandQueueHandle, &led_cmd, 0, 0);
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */