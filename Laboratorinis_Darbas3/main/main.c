/*3rd laboratory FIR & IIR filter implementation on EP32S3  using ESP-IDF platform */



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
#include "driver/gptimer.h"
#include "esp_intr_alloc.h"

/*Pin definitions*/

#define PIN_GLED GPIO_NUM_9
#define PIN_YLED GPIO_NUM_8
#define PIN_DAC GPIO_NUM_3
#define PIN_ADC GPIO_NUM_2


/*Define how many seconds to capture signal can increase orr decrease. Reads is sample rate in HZ (CANNOT CHANGE OR HAVE TO CHANGE TIMER SETTINGS) */

#define SECONDS 3
#define READS 1000
#define SIMULATION //Uncomment this if want to use real ADC. Simulation is squareWave fom real adc. Just in case you have no generator at home 

#include "adcSimulation.h"

//ADC and filtered adc buffers. Just pointers. Buffers are alocated dynamicaly in main function 

uint16_t* adcBuf[SECONDS] = { 0 };
int16_t* adcBufFIRFiltered[SECONDS] = { 0 };
int16_t* adcBufIIRFiltered[SECONDS] = { 0 };


//FIR and IIR filters coefficients 

float firFilterACoefs[] = { 0,	0.00692099368127822,	-0.0627110568684960,	0.0484469557689477,	0.507343107418270,	0.507343107418270,	0.0484469557689477,	-0.0627110568684960,	0.00692099368127822,	0 };

float iirFilterBCoefs[] = {0.0188869179526077,	0.169982261573470,	0.679929046293878,	1.58650110801905,	2.37975166202857,	2.37975166202857,	1.58650110801905,	0.679929046293878,	0.169982261573470,	0.0188869179526077 };
float iirFilterACoefs[] = {1,	1.79158135278860,	2.53189988089812,	2.11822942034193,	1.37075629439323,	0.609038913076474,	0.199331556962956,	0.0431047310152814,	0.00580426165430882,	0.000355580604257625};


//Main task finite state machine. Data read - adc data aquisition, then filtering with both filters, than sending ADC data, and filtered data via UART 

typedef enum eFsmStates {
    eFsmInit = 0,
    eFsmDataRead,
    eFsmDataFIRFilter,
    eFsmDataIIRFilter,
    eFsmDataSend
} eFsmStates;

eFsmStates FSM = eFsmInit;

/*Function declaration*/
void ADCTask(void*);

/*ADC alarm function. When in this function check if FSM is data read and read one ADC sample. This gets 1kHz sample rate*/

bool IRAM_ATTR ADC_Timer_ISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);


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
    tskIDLE_PRIORITY
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

/*Timer definitions*/

gptimer_handle_t adcTimerhandle;
gptimer_config_t timerCfg = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 10000, //10kHz clock but will be counting only for 1000 ticks
    .intr_priority = 0
};
gptimer_alarm_config_t timerISR = {
    .alarm_count = 10, /*1000 hz */
    .reload_count = 0, /*Reset timer*/
    .flags.auto_reload_on_alarm = true
};
gptimer_event_callbacks_t cbs = {
    .on_alarm = ADC_Timer_ISR, // register user callback
};




/*Main app just initializes tasks, ADC peripheral, Timer peripheral*/
void app_main(void) {


    /*Allocate memoy for adc reads*/
    for (int i = 0; i < SECONDS; i++) {
        adcBuf[i] = malloc(READS * sizeof(uint16_t));
        adcBufFIRFiltered[i] = malloc(READS * sizeof(uint16_t));
        adcBufIIRFiltered[i] = malloc(READS * sizeof(uint16_t));
    }
    /*GPIO initialisation*/
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);


    /*ADC initialisation*/
    adc_oneshot_new_unit(&initConfig1, &adc1Handle);
    adc_oneshot_config_channel(adc1Handle, ADC_CHANNEL_1, &adc1config);

    /*timer initialisation*/
    gptimer_new_timer(&timerCfg, &adcTimerhandle);
    gptimer_register_event_callbacks(adcTimerhandle, &cbs, NULL);
    gptimer_set_alarm_action(adcTimerhandle, &timerISR);
    gptimer_enable(adcTimerhandle);
    gptimer_start(adcTimerhandle);

    xTaskCreate(ADCTaskInit.pvTaskCode,
                ADCTaskInit.pcName,
                ADCTaskInit.uxStackDepth,
                ADCTaskInit.pvParameters,
                ADCTaskInit.uxPriority,
                ADCTaskHandle);
}


/*Get one sample from buffer without specifying second and sample number*/
uint16_t ADCGetValue(uint32_t count) {

    uint8_t second = count / READS;
    uint16_t read = count % READS;

    return adcBuf[second][read];
}


/*Read one ADC sample. If buffer is filled than raise Filtering flag and stop timer */
void ADCReadValue() {
    static uint16_t sec = 0;
    static uint16_t cnt = 0;
    static uint16_t simulationCount = 0;
    if (FSM == eFsmDataRead) {

        int adcRaw = 0;
#ifdef SIMULATION

        adcBuf[sec][cnt] = adcSimulationBuff[simulationCount];
        simulationCount++;


#else

        adc_oneshot_read(adc1Handle, ADC_CHANNEL_1, &adcRaw);
        adcBuf[sec][cnt] = adcRaw;

#endif
        if (sec == SECONDS - 1 && cnt == READS - 1) {
            FSM = eFsmDataFIRFilter;
            sec = 0;
            cnt = 0;
            simulationCount = 0;
        } else {
            if (cnt == READS - 1) {
                cnt = 0;
                sec++;
            }

            else cnt++;
        }
    }
}

//9th order FIR Filter 
void FIRFilter() {


    if (FSM != eFsmDataFIRFilter) {
        return;
    }
    uint16_t tempFIR[10] = { 0 };
    float sum = 0;

    for (uint32_t j = 0; j < READS * SECONDS; j++) {
        for (uint8_t i = 9; i > 0; i--) {
            tempFIR[i] = tempFIR[i - 1];
        }
        tempFIR[0] = ADCGetValue(j);


        sum = 0;
        for (uint8_t i = 0; i < 10; i++) {
            sum += (tempFIR[i] * firFilterACoefs[i]);
        }

        uint8_t second = j / READS;
        uint16_t read = j % READS;
        
        adcBufFIRFiltered[second][read] = sum;
    }
}

//9th order butterworth IIR filter
void IIRFilter() {

    if (FSM != eFsmDataIIRFilter) {
        return;
    }

    int16_t tempIIRForward[10] = { 0 };
    int16_t tempIIRBackward[10] = { 0 };
    float sumForward = 0;
    float sumBackward = 0;
    for (uint32_t j = 0; j < READS * SECONDS; j++) {



        tempIIRForward[0] = ADCGetValue(j);

        sumForward = 0;
        sumBackward = 0;



        for (uint8_t i = 0; i < 10; i++) {
            sumForward += (tempIIRForward[i] * 1.0 * iirFilterBCoefs[i]);
        }

        for (uint8_t i = 1; i < 10; i++) {
            sumBackward -= (tempIIRBackward[i] * 1.0 * iirFilterACoefs[i]);
        }



        uint8_t second = j / READS;
        uint16_t read = j % READS;

        if ((sumForward + sumBackward) < 0) adcBufIIRFiltered[second][read] = 0;
        else adcBufIIRFiltered[second][read] = sumForward + sumBackward;
        
        adcBufIIRFiltered[second][read] = sumForward + sumBackward;


        tempIIRBackward[0] = adcBufIIRFiltered[second][read];

        for (uint8_t i = 9; i > 0; i--) {
            tempIIRForward[i] = tempIIRForward[i - 1];
            tempIIRBackward[i] = tempIIRBackward[i - 1];
        }
    }
}


/*Send All buffers*/
void DataSend() {
    printf("Printing last %d secs data \r\n", SECONDS);
    for (int i = 0; i < SECONDS; i++){
        for (int j = 0; j<READS; j++){
            printf(">ADC: %d\r\n", adcBuf[i][j]);
        }
    }

    for (int i = 0; i < SECONDS; i++){
        for (int j = 0; j<READS; j++){
            printf(">ADCFIR: %d\r\n", adcBufFIRFiltered[i][j]);
        }
    }

    for (int i = 0; i < SECONDS; i++) {
        for (int j = 0; j < READS; j++) {
            printf(">ADCIIR: %d\r\n", adcBufIIRFiltered[i][j]);
        }
    }
}


/*Main function Do nothing until FSM is filtering. Then filter with both filters, than send data and start timer again. Raise FSM to data read to start new data aquisition */

void ADCTask(void*) {

    if (FSM == eFsmInit) FSM = eFsmDataRead;
    while (1) {

        if (FSM == eFsmDataFIRFilter) {
            FIRFilter();
            vTaskDelay(200 / portTICK_PERIOD_MS);
            FSM = eFsmDataIIRFilter;
        }


        if (FSM == eFsmDataIIRFilter) {
            IIRFilter();
            vTaskDelay(200 / portTICK_PERIOD_MS);
            FSM = eFsmDataSend;
        }

        if (FSM == eFsmDataSend) {
            DataSend();
            vTaskDelay(200 / portTICK_PERIOD_MS);
            FSM = eFsmDataRead;
            gptimer_start(adcTimerhandle);
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);

    }
}

/*timer interupt function */
bool ADC_Timer_ISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
    ADCReadValue();
    if (FSM == eFsmDataFIRFilter) {
        gptimer_stop(adcTimerhandle);
        //printf("ADC\r\n");
    }
    return 1;
}
