/* Pirmas laboratorinis Simonas Rozanovas */
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "driver/adc.h"

#define GREEN_LED GPIO_NUM_3
#define YELLOW_LED GPIO_NUM_4


void YellowLEDTask(void*); 
void GreenLEDTask(void*);

typedef struct { 
    TaskFunction_t pvTaskCode;
    const char * const pcName;
    const configSTACK_DEPTH_TYPE uxStackDepth;
    void *pvParameters;
    UBaseType_t uxPriority;
} taskInitStruct;

TaskHandle_t *yellowLEDTaskHandle;
TaskHandle_t *greenLEDTaskHandle;
taskInitStruct yellowLEDTaskInit = { 
    YellowLEDTask, 
    "Yellow LED Task", 
    configMINIMAL_STACK_SIZE, 
    NULL,
    5
}; 
taskInitStruct greenLEDTaskInit = { 
    GreenLEDTask, 
    "Green LED Task", 
    configMINIMAL_STACK_SIZE, 
    NULL,
    5
}; 


void ESPInit(void){ 
    gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT); 
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT); 
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT); 
    
    xTaskCreate(yellowLEDTaskInit.pvTaskCode,
                yellowLEDTaskInit.pcName, 
                yellowLEDTaskInit.uxStackDepth, 
                yellowLEDTaskInit.pvParameters, 
                yellowLEDTaskInit.uxPriority,
                yellowLEDTaskHandle);
    xTaskCreate(greenLEDTaskInit.pvTaskCode,
                greenLEDTaskInit.pcName, 
                greenLEDTaskInit.uxStackDepth, 
                greenLEDTaskInit.pvParameters, 
                greenLEDTaskInit.uxPriority,
                greenLEDTaskHandle);
        
}

//3 uzduotis hello world paleidimas 
void ReadESP(void){  
    printf("Pirmas Laboratorinis Simonas Rozanovas!\n");
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
            (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
            (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
}

/*4 uzduotis. Nuskaityti ADC vertes ir Isvesti i terminala. 
Tuo paciu naudojant serial monitor stebimas signalas. 
(Naudotas sviesos sensorius ir signalu generatorius)*/ 
void ReadADC() { 
    int adcValue;
    float voltage; 
    adcValue = adc1_get_raw(ADC1_CHANNEL_1);
    voltage = 3.3 * adcValue / 4095;
    printf(">Voltage:%f\r\n", voltage);     
}


/*Free RTOS uzduotis dvi skirtingos uzduotys kviecia bendra LED ijungimo funkcija*/
void SetLED(int ledNum, int level){
    gpio_set_level(ledNum, level);  
}



void app_main(void)
{
    ReadESP();
    ESPInit();
    
    while (1) {
        ReadADC(); 
        vTaskDelay(200/portTICK_PERIOD_MS); 
    }
    
    esp_restart();
}



void YellowLEDTask(void*){
    int level = 0; 
    while(1){ 
        SetLED(YELLOW_LED, level);
        level = level == 1 ? 0 : 1; 
        vTaskDelay(250/portTICK_PERIOD_MS);
    }

} 
void GreenLEDTask(void*){
    int level = 0; 
    while(1){ 
        SetLED(GREEN_LED, level);
        level = level == 1 ? 0 : 1; 
        vTaskDelay(500/portTICK_PERIOD_MS);
    }

}