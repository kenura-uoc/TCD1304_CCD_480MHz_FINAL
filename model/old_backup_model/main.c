/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body — TCD1304 CCD + Chlorophyll-a Predictor
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 * 
 * CHLA PREDICTOR INTEGRATION:
 *   Added USB commands for chlorophyll-a concentration prediction:
 *     'P'   - Predict: capture one frame and predict concentration
 *     'B'   - Background: capture and store dark background spectrum
 *     'S'   - Status: report predictor status
 *     'M'   - Multi-predict: average N frames then predict (higher accuracy)
 *     'I<n>'- Set integration time (existing command, preserved)
 *
 *   New files to add to project:
 *     preprocessing.c, pls_predict.c, chla_predictor.c
 *     preprocessing.h, pls_predict.h, chla_predictor.h, pls_model_data.h
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "chla_predictor.h"
#include "pls_model_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
#define CCD_BUFFER_SIZE 3694  // 32 Dummies + 3648 Pixels + 14 Dummies

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;     // 0xABCD
    uint16_t frame_num; // Rolling frame counter
    uint16_t pixels[CCD_BUFFER_SIZE];
} CCD_Frame_t;
#pragma pack(pop)

// Buffer in SRAM3 (non-cached on H7)
__attribute__((section(".sram3"), aligned(32))) uint16_t Buffer_A[CCD_BUFFER_SIZE];

// Application State
volatile uint8_t frame_ready = 0;
uint16_t frame_counter = 0;
CCD_Frame_t ccd_frame;

// Mode Control
volatile uint8_t ccd_mode = 0;  // 0=Stream, 1=Predict
volatile uint8_t mode_update_pending = 0;

// Integration Time Control (in milliseconds)
volatile uint32_t integration_time_ms = 18;  // Default 18ms

// ============================================
// CHLA PREDICTOR VARIABLES
// ============================================
// Place predictor in D1 DTCM RAM for fast access (~44 KB)
// If DTCM is too small, it will go into regular SRAM (AXI)
static chla_predictor_t chla_pred;

// Predict mode control
volatile uint8_t predict_request = 0;     // 0=none, 'P'=predict, 'B'=bg, 'M'=multi
volatile uint8_t predict_avg_count = 5;   // Number of frames to average for multi-predict

// USB response buffer
static char usb_msg[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
void Process_USB_Command(uint8_t *buf, uint32_t len);
/* USER CODE BEGIN PFP */

// ============================================================
// ESP32-STYLE BIT-BANGING IMPLEMENTATION
// ============================================================

#define ICG_PORT GPIOA
#define ICG_PIN  GPIO_PIN_0  // PA0
#define SH_PORT  GPIOA
#define SH_PIN   GPIO_PIN_2  // PA2

#define ICG_HIGH() (ICG_PORT->BSRR = ICG_PIN)
#define ICG_LOW()  (ICG_PORT->BSRR = (uint32_t)ICG_PIN << 16)
#define SH_HIGH()  (SH_PORT->BSRR = SH_PIN)
#define SH_LOW()   (SH_PORT->BSRR = (uint32_t)SH_PIN << 16)

static inline void delay_ns(uint32_t ns) {
    uint32_t cycles = (ns * 480) / 1000;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

static inline void delay4ns(uint32_t count) {
    uint32_t cycles = count * 2;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

static inline void delay_ms_precise(uint32_t ms) {
    uint32_t output_cycles = ms * 480000;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < output_cycles);
}

void Configure_BitBang_GPIO(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    ICG_HIGH();
    SH_LOW();
}

void Enable_DWT_Counter(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void readCCD(void) {
    __disable_irq();

    ICG_LOW();
    delay4ns(9);
    SH_HIGH();
    delay4ns(96);
    SH_LOW();
    delay4ns(48);
    ICG_HIGH();
    TIM4->CNT = 0;

    __enable_irq();
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ============================================================
// USB SEND HELPER (waits for CDC ready, sends string)
// ============================================================
static void USB_SendString(const char *str) {
    uint16_t len = (uint16_t)strlen(str);
    uint32_t timeout = 100000;

    while (timeout > 0) {
        USBD_CDC_HandleTypeDef *hcdc =
            (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
        if (hcdc && hcdc->TxState == 0) {
            CDC_Transmit_FS((uint8_t *)str, len);
            return;
        }
        timeout--;
    }
}

// ============================================================
// ADC/DMA CALLBACK
// ============================================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        ccd_frame.magic = 0xABCD;
        ccd_frame.frame_num = frame_counter++;
        memcpy(ccd_frame.pixels, (void *)Buffer_A, CCD_BUFFER_SIZE * 2);
        frame_ready = 1;
    }
}

// ============================================================
// SEND CCD FRAME (BINARY) — existing streaming mode
// ============================================================
void Send_CCD_Frame_Binary(void) {
    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0) return;

    uint8_t *ptr = (uint8_t *)&ccd_frame;
    uint16_t remaining = sizeof(CCD_Frame_t);

    while (remaining > 0) {
        uint16_t chunk = (remaining > 64) ? 64 : remaining;
        uint8_t status = CDC_Transmit_FS(ptr, chunk);
        if (status == USBD_OK) {
            ptr += chunk;
            remaining -= chunk;
        } else {
            for (volatile int i = 0; i < 1000; i++);
        }
    }
}

// ============================================================
// CAPTURE ONE FRAME (blocking)
// Used by predict commands — captures a single fresh frame
// ============================================================
static void Capture_One_Frame(void) {
    /* Step 1: Flush */
    readCCD();

    /* Step 2: Fixed integration */
    delay_ms_precise(integration_time_ms);

    /* Step 3: Arm DMA */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Buffer_A, CCD_BUFFER_SIZE);

    /* Step 4: Trigger readout */
    readCCD();

    /* Step 5: Wait for DMA complete */
    frame_ready = 0;
    while (!frame_ready);

    /* Step 6: Stop DMA */
    HAL_ADC_Stop_DMA(&hadc1);
}

// ============================================================
// PREDICT COMMAND HANDLER
// Captures a frame and predicts concentration
// ============================================================
static void Handle_Predict(void) {
    float concentration = 0.0f;
    chla_status_t status;
    uint32_t start_tick, end_tick;

    /* Capture a fresh frame */
    Capture_One_Frame();

    /* Record timing */
    start_tick = DWT->CYCCNT;

    /* Run prediction on the captured pixels */
    status = chla_predict_from_adc(&chla_pred, ccd_frame.pixels,
                                    (float)integration_time_ms,
                                    &concentration);

    end_tick = DWT->CYCCNT;
    float predict_time_ms = (float)(end_tick - start_tick) / 480000.0f;

    if (status == CHLA_OK) {
        snprintf(usb_msg, sizeof(usb_msg),
                 "PREDICT:%.4f ug/L (%.1f ms)\r\n",
                 concentration, predict_time_ms);
    } else {
        snprintf(usb_msg, sizeof(usb_msg),
                 "PREDICT:ERROR %d\r\n", (int)status);
    }
    USB_SendString(usb_msg);
}

// ============================================================
// MULTI-PREDICT: Average N frames then predict
// Higher accuracy by reducing noise
// ============================================================
static void Handle_Multi_Predict(void) {
    float concentration = 0.0f;
    chla_status_t status;
    int n = predict_avg_count;
    int i, j;

    /* Use work_buf1 in predictor to accumulate (we'll call predict after) */
    /* First, zero the accumulator */
    for (j = 0; j < SPECTRUM_LENGTH; j++) {
        chla_pred.work_buf1[j] = 0.0f;
    }

    /* Capture and accumulate N frames */
    for (i = 0; i < n; i++) {
        Capture_One_Frame();
        for (j = 0; j < SPECTRUM_LENGTH; j++) {
            chla_pred.work_buf1[j] += (float)ccd_frame.pixels[j];
        }
    }

    /* Average */
    float inv_n = 1.0f / (float)n;
    for (j = 0; j < SPECTRUM_LENGTH; j++) {
        chla_pred.work_buf1[j] *= inv_n;
    }

    /* Background subtraction */
    if (chla_pred.background_loaded) {
        for (j = 0; j < SPECTRUM_LENGTH; j++) {
            chla_pred.work_buf1[j] -= chla_pred.background[j];
        }
    }

    /* Integration time normalization */
    float inv_int_time = 1.0f / ((float)integration_time_ms + 1e-8f);
    for (j = 0; j < SPECTRUM_LENGTH; j++) {
        chla_pred.work_buf1[j] *= inv_int_time;
    }

    /* Preprocessing: SG smooth -> 1st derivative -> SNV */
    sg_smooth(chla_pred.work_buf1, chla_pred.work_buf2, SPECTRUM_LENGTH);
    sg_first_derivative(chla_pred.work_buf2, chla_pred.work_buf1, SPECTRUM_LENGTH);
    snv_normalize(chla_pred.work_buf1, SPECTRUM_LENGTH);

    /* PLS prediction */
    concentration = pls_predict(chla_pred.work_buf1);

    snprintf(usb_msg, sizeof(usb_msg),
             "PREDICT_AVG(%d):%.4f ug/L\r\n",
             n, concentration);
    USB_SendString(usb_msg);
}

// ============================================================
// BACKGROUND CAPTURE HANDLER
// Captures a dark frame and stores as background
// ============================================================
static void Handle_Background(void) {
    chla_status_t status;

    /* Capture a fresh dark frame */
    Capture_One_Frame();

    /* Store as background */
    status = chla_set_background_u16(&chla_pred, ccd_frame.pixels);

    if (status == CHLA_OK) {
        USB_SendString("BG:OK (background captured)\r\n");
    } else {
        snprintf(usb_msg, sizeof(usb_msg), "BG:ERROR %d\r\n", (int)status);
        USB_SendString(usb_msg);
    }
}

// ============================================================
// STATUS COMMAND HANDLER
// ============================================================
static void Handle_Status(void) {
    snprintf(usb_msg, sizeof(usb_msg),
        "=== CHLA PREDICTOR v1.0 ===\r\n"
        "Model: PLS (comp=%d, feat=%d)\r\n"
        "Preprocess: SG(w=%d)->D1->SNV\r\n"
        "IntTime: %lu ms\r\n"
        "Background: %s\r\n"
        "Ready: %s\r\n"
        "===========================\r\n",
        NUM_PLS_COMPONENTS, NUM_FEATURES,
        SG_WINDOW_SIZE,
        integration_time_ms,
        chla_pred.background_loaded ? "YES" : "NO",
        chla_pred.initialized ? "YES" : "NO"
    );
    USB_SendString(usb_msg);
}

// ============================================================
// PROCESS USB COMMAND
// Extended with prediction commands
//
// Commands:
//   I<n>   - Set integration time (existing)
//   P      - Predict: single frame capture + predict
//   B      - Background: capture dark frame
//   S      - Status: report predictor info
//   M<n>   - Multi-predict: average N frames (default 5)
//   (any other data) - treated as streaming mode
// ============================================================
void Process_USB_Command(uint8_t *buf, uint32_t len) {
    if (len == 0) return;

    switch (buf[0]) {
        case 'I':
        case 'i': {
            /* Integration Time Command: I<time_ms> */
            if (len > 1) {
                uint32_t new_time = 0;
                for (uint32_t i = 1; i < len; i++) {
                    if (buf[i] >= '0' && buf[i] <= '9') {
                        new_time = new_time * 10 + (buf[i] - '0');
                    } else {
                        break;
                    }
                }
                if (new_time < 15) new_time = 15;
                if (new_time > 10000) new_time = 10000;
                integration_time_ms = new_time;
                snprintf(usb_msg, sizeof(usb_msg), "INT_TIME:%lu ms\r\n", integration_time_ms);
                USB_SendString(usb_msg);
            }
            break;
        }

        case 'P':
        case 'p':
            /* Predict: capture + predict concentration */
            predict_request = 'P';
            break;

        case 'B':
        case 'b':
            /* Background: capture dark spectrum */
            predict_request = 'B';
            break;

        case 'S':
        case 's':
            /* Status */
            predict_request = 'S';
            break;

        case 'M':
        case 'm': {
            /* Multi-predict: M<n> where n = number of frames to average */
            if (len > 1) {
                uint32_t count = 0;
                for (uint32_t i = 1; i < len; i++) {
                    if (buf[i] >= '0' && buf[i] <= '9') {
                        count = count * 10 + (buf[i] - '0');
                    } else {
                        break;
                    }
                }
                if (count < 1) count = 1;
                if (count > 100) count = 100;
                predict_avg_count = (uint8_t)count;
            }
            predict_request = 'M';
            break;
        }

        default:
            break;
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MPU Configuration */
    MPU_Config();

    /* Enable I-Cache */
    SCB_EnableICache();
    /* Enable D-Cache */
    SCB_EnableDCache();

    /* MCU Configuration */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USB_DEVICE_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();

    /* USER CODE BEGIN 2 */

    HAL_Delay(1000);  // Wait for USB

    Enable_DWT_Counter();
    Configure_BitBang_GPIO();

    // ADC Calibration
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    // Start fM clock (TIM3) - 1MHz continuous
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    TIM3->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    // Start ADC trigger timer (TIM4) - 250kHz
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    TIM4->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    // ============================================
    // INITIALIZE CHLA PREDICTOR
    // ============================================
    chla_predictor_init(&chla_pred);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        // ============================================
        // CHECK FOR PREDICT COMMANDS
        // ============================================
        if (predict_request != 0) {
            uint8_t cmd = predict_request;
            predict_request = 0;

            switch (cmd) {
                case 'P':
                    Handle_Predict();
                    break;
                case 'B':
                    Handle_Background();
                    break;
                case 'S':
                    Handle_Status();
                    break;
                case 'M':
                    Handle_Multi_Predict();
                    break;
            }

            /* After handling command, skip the normal stream frame
             * to avoid sending stale data */
            continue;
        }

        // ============================================
        // NORMAL CCD STREAMING MODE (existing behavior)
        // ============================================

        // Step 1: Flush variable integration charge
        readCCD();

        // Step 2: Fixed Integration Time
        delay_ms_precise(integration_time_ms);

        // Step 3: Arm DMA
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Buffer_A, CCD_BUFFER_SIZE);

        // Step 4: Trigger Readout
        readCCD();

        // Step 5: Wait for DMA complete
        while (!frame_ready);

        // Step 6: Stop DMA
        HAL_ADC_Stop_DMA(&hadc1);

        // Step 7: Send frame
        Send_CCD_Frame_Binary();
        frame_ready = 0;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  // LED Heartbeat

        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 20;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 */
static void MX_ADC1_Init(void) {
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_CC4;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.Oversampling.Ratio = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief TIM2 Initialization Function
 */
static void MX_TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4320000 - 1;  // 18ms ICG @ 240MHz
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 100;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 */
static void MX_TIM3_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 240 - 1;  // 1MHz fM @ 240MHz
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 120;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 */
static void MX_TIM4_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 960 - 1;  // 250kHz ADC trigger @ 240MHz
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 480;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief TIM5 Initialization Function
 */
static void MX_TIM5_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 4320000 - 1;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) { Error_Handler(); }

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_ITR1;
    if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 4320000 - 50;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim5);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 */
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* MPU Configuration */
void MPU_Config(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x30000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }

    if (htim->Instance == TIM2) {
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Buffer_A, CCD_BUFFER_SIZE);
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
    /* User can add: printf("Wrong parameters: file %s on line %d\r\n", file, line) */
}
#endif
