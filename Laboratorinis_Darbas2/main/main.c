/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "math.h"
#include "driver/ledc.h"


#define PIN_GLED GPIO_NUM_9
#define PIN_YLED GPIO_NUM_8
#define PIN_DAC GPIO_NUM_3
#define PIN_ADC GPIO_NUM_2
#define PIN_I2S_CLOCK GPIO_NUM_42
#define PIN_I2S_DATA GPIO_NUM_41


#define ADCTASK
//#define MICTASK
#define DACTASK

#define PI 3.1415926

/*Function declaration*/
void ADCTask(void*);
void DACTask(void*);
void MicTask(void*);


/*Task definitions*/
typedef struct {
    TaskFunction_t pvTaskCode;
    const char* const pcName;
    const configSTACK_DEPTH_TYPE uxStackDepth;
    void* pvParameters;
    UBaseType_t uxPriority;
} taskInitStruct;


TaskHandle_t* ADCTaskHandle;
taskInitStruct ADCTaskInit = {
    ADCTask,
    "ADC Task",
    configMINIMAL_STACK_SIZE * 3,
    NULL,
    5
};

TaskHandle_t* DACTaskHandle;
taskInitStruct DACTaskInit = {
    DACTask,
    "DAC Task",
    configMINIMAL_STACK_SIZE * 3,
    NULL,
    6
};

TaskHandle_t* MicTaskHandle;
taskInitStruct MicTaskInit = {
    MicTask,
    "Mic Task",
    configMINIMAL_STACK_SIZE * 8,
    NULL,
    5
};


/*ADC definitions */
adc_oneshot_unit_handle_t adc1Handle;
adc_oneshot_unit_init_cfg_t initConfig1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};
adc_oneshot_chan_cfg_t adc1config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
};

/*DAC definitions*/
// Prepare and then apply the LEDC PWM channel configuration
ledc_channel_config_t dacChannel = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = PIN_DAC,
    .duty = 0, // Set duty to 0%
    .hpoint = 0
};
ledc_timer_config_t dacTimer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 4000,  // Set output frequency at 4 kHz
    .clk_cfg = LEDC_AUTO_CLK
};
/*Mic i2s definitions*/
i2s_chan_handle_t rx_handle;
i2s_chan_config_t chaCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);


/* Init the channel into PDM RX mode */
i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(16000),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
        .clk = GPIO_NUM_42,
        .din = GPIO_NUM_41,
        .invert_flags = {
            .clk_inv = false,
        },
    },
};


/*Main app just initializes tasks and ADC peripheral*/
void app_main(void) {

    /*GPIO initialisation*/
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);

    /*ADC initialisation*/
    adc_oneshot_new_unit(&initConfig1, &adc1Handle);
    adc_oneshot_config_channel(adc1Handle, ADC_CHANNEL_1, &adc1config);


    /*I2S initialisation*/
    i2s_new_channel(&chaCfg, NULL, &rx_handle);
    i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg);
    i2s_channel_enable(rx_handle);

    /*PWM */

    ledc_timer_config(&dacTimer);
    ledc_channel_config(&dacChannel);


    xTaskCreate(ADCTaskInit.pvTaskCode,
                ADCTaskInit.pcName,
                ADCTaskInit.uxStackDepth,
                ADCTaskInit.pvParameters,
                ADCTaskInit.uxPriority,
                ADCTaskHandle);

    xTaskCreate(MicTaskInit.pvTaskCode,
                MicTaskInit.pcName,
                MicTaskInit.uxStackDepth,
                MicTaskInit.pvParameters,
                MicTaskInit.uxPriority,
                MicTaskHandle);

    xTaskCreate(DACTaskInit.pvTaskCode,
                DACTaskInit.pcName,
                DACTaskInit.uxStackDepth,
                DACTaskInit.pvParameters,
                DACTaskInit.uxPriority,
                DACTaskHandle);
}


void LUTCreate(uint16_t* bufferOut) {

    uint16_t fd = 100; //diskretizavimo daznis
    uint8_t f1 = 1; //Pirmo sinuso daznis
    uint8_t f2 = 3; //Antro sinuso daznis
    float timestep = 1.0 / fd;
    float time = 0;
    float radian;
    float buffer[100];
    for (uint16_t i = 0; i < 100; i++) {
        radian = 2 * PI * f1 * time;
        buffer[i] = 0.7 * sin(radian);
        radian = 2 * PI * f2 * time + PI / 2; //antras sinusas perstumtas per 90 laipsniu
        buffer[i] += 0.5 * sin(radian);
        time += timestep;

    }

    float max = 0;
    float min = 20;

    //Normalizavimas nuo -1 iki 1
    for (uint16_t i = 0; i < 100; i++) {
        if (max < buffer[i]) {
            max = buffer[i];
        }
        if (min > buffer[i]) {
            min = buffer[i];
        }
    }


    for (uint16_t i = 0; i < 100; i++) {
        buffer[i] /= max;
        buffer[i] += 1;
    }

    //Perstumimas nuo 0 - 1 iki 0-3.3

    for (uint16_t i = 0; i < 100; i++) {
        buffer[i] = buffer[i] * (3.3 / 2);
    }

    //Konvertavimas i 10 bitu PWM

    for (uint16_t i = 0; i < 100; i++) {
        bufferOut[i] = buffer[i] * 1024.0 / 3.3;
    }


}


void ADCTask(void*) {
    int adcRaw = 0;
    float voltage;
#ifdef ADCTASK
    while (1) {

        adc_oneshot_read(adc1Handle, ADC_CHANNEL_1, &adcRaw);
        voltage = ((float)(adcRaw) / 4095) * 3.3;
        printf(">ADC: %d\n", adcRaw);

        vTaskDelay(10 / portTICK_PERIOD_MS);


    }
#else
    while (1) {

        vTaskDelay(250 / portTICK_PERIOD_MS);


    }
#endif

}


void MicTask(void*) {

#ifdef MICTASK
    while (1) {

        uint16_t i2s_read[2048];
        if (i2s_channel_read(rx_handle, i2s_read, 2048, NULL, 2048) == ESP_OK) {
            for (int i = 0; i < 2048;i++) {

                printf("%d\n", i2s_read[i]);
            }

        }



    }
#else
    while (1) {

        vTaskDelay(250 / portTICK_PERIOD_MS);


    }
#endif

}



void DACTask(void*) {
    uint16_t dataOut[100];
    LUTCreate(dataOut);

    static uint8_t i = 0;
#ifdef DACTASK
    while (1) {

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dataOut[i]);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        //printf(">DAC:%d\r\n", dataOut[i]);
        i = (i >= 99) ? 0 : (i + 1);

        vTaskDelay(10 / portTICK_PERIOD_MS);


    }
#else
    while (1) {

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
#endif

}
