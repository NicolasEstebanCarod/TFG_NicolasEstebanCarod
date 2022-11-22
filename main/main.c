#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define TX_GPIO_NUM         GPIO_NUM_13
#define RX_GPIO_NUM         GPIO_NUM_12
#define SLAVE                     0b111
#define MASTER                    0b011
#define ID                        0x4105cc
#define MSG_DATAIN                0
#define MSG_DATAIN_SLAVE          0
#define MSG_DATAIN_MASTER         2
#define MSG_GOONBUS               0
#define MSG_CONFIG                1
#define MSG_DATAOUT               2
#define MSG_DATAOUT_SLAVE         2
#define MSG_DATAOUT_MASTER        0
#define MSG_CARDTYPE              5
#define STATE_GOONBUS             0
#define STATE_CARDTYPE1           1
#define STATE_CARDTYPE2           2
#define STATE_DATAOUT_SLAVE       3

//####  INPUTS CONFIG  ####
#define INPUT1 GPIO_NUM_4
#define INPUT2 GPIO_NUM_5
#define INPUT3 GPIO_NUM_6
#define INPUT4 GPIO_NUM_7
#define INPUT5 GPIO_NUM_15
#define INPUT6 GPIO_NUM_16
#define INPUT7 GPIO_NUM_17
#define INPUT8 GPIO_NUM_18
#define INPUT9 GPIO_NUM_8
#define INPUT10 GPIO_NUM_19
#define INPUT11 GPIO_NUM_20
#define INPUT12 GPIO_NUM_3
#define INPUT13 GPIO_NUM_46

//####  OUTPUTS CONFIG  ####
#define OUTPUT1 GPIO_NUM_42
#define OUTPUT2 GPIO_NUM_41
#define OUTPUT3 GPIO_NUM_40
#define OUTPUT4 GPIO_NUM_39
#define OUTPUT5 GPIO_NUM_38
#define OUTPUT6 GPIO_NUM_37
#define OUTPUT7 GPIO_NUM_36
#define OUTPUT8 GPIO_NUM_35
#define OUTPUT9 GPIO_NUM_45
#define OUTPUT10 GPIO_NUM_48
#define OUTPUT11 GPIO_NUM_47
#define OUTPUT12 GPIO_NUM_21

twai_message_t TRANSMIT_GOONBUS         = {.identifier = ((SLAVE) << 26) + ((ID) << 3) + (0) , .data_length_code = 0,
                                            .ss = 1,.extd = 1, .data = {0 , 0 , 0 , 0 ,0 ,0 ,0 ,0}};
twai_message_t TRANSMIT_CARDTYPE1       = {.identifier = ((SLAVE) << 26) + ((ID) << 3) + (5) , .data_length_code = 2,
                                            .ss = 1,.extd = 1, .data = {0x00 , 0x03 , 0 , 0 ,0 ,0 ,0 ,0}};
twai_message_t TRANSMIT_CARDTYPE2       = {.identifier = ((SLAVE) << 26) + ((ID) << 3) + (5) , .data_length_code = 4,
                                            .ss = 1,.extd = 1, .data = {0x00 , 0x0a , 0x00 , 0x03 ,0 ,0 ,0 ,0}};
twai_message_t TRANSMIT_CONFIG          = {.identifier = 503852642 , .data_length_code = 0,
                                            .ss = 1,.extd = 1, .data = {0 , 0 , 0 , 0 ,0 ,0 ,0 ,0}};
twai_message_t TRANSMIT_DATAOUT_SLAVE   = {.identifier = ((SLAVE) << 26) + ((ID) << 3) + (2) , .data_length_code = 0,
                                            .ss = 1,.extd = 1, .data = {0 , 0 , 0 , 0 ,0 ,0 ,0 ,0}};
twai_message_t TRANSMIT_INPUT           = {.identifier = ((SLAVE) << 26) + ((ID) << 3) + (2) , .data_length_code = 0,
                                            .ss = 1,.extd = 1, .data = {0 , 0 , 0 , 0 ,0 ,0 ,0 ,0}};
uint8_t state;

uint8_t inputsGPIO[13]  = { INPUT1,INPUT2,INPUT3,INPUT4,INPUT5,INPUT6,INPUT7,INPUT8,INPUT9,INPUT10,INPUT11,INPUT12,INPUT12};

typedef struct{
    gpio_num_t outputPin;
    uint8_t pin;
    uint8_t subIndex;
    bool status;
    bool fixed;
    bool flashingOrPulsed; //flashing == 0, Pulsed == 1
    uint64_t timer;
    uint64_t timerValue;
}output_t;

output_t outputs[12];

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();


static const twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                              .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,
                                              .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,
                                              .tx_queue_len = 10, .rx_queue_len = 10,
                                              .alerts_enabled = TWAI_ALERT_ALL,
                                              .clkout_divider = 0};

/* --------------------------- Tasks and Functions -------------------------- */
static void twai_receive_task(void *arg)
{
    uint32_t deviceType;
    uint32_t canIdentifier;
    uint32_t messageType;
    vTaskDelay(1000/ portTICK_PERIOD_MS);
    while (1) {
        twai_message_t rx_msg;
        twai_receive(&rx_msg,portMAX_DELAY);
        deviceType = (rx_msg.identifier    & 0b11100000000000000000000000000) >> 26;
        canIdentifier = (rx_msg.identifier & 0b00011111111111111111111111000) >> 3;
        messageType = rx_msg.identifier    & 0b00000000000000000000000000111;
        printf("Dispositivo: %x, Identificador: %x, Mensaje %x datos: %x %x %x %x Data length: %d\n",deviceType, canIdentifier,messageType, rx_msg.data[0],rx_msg.data[1], rx_msg.data[2],rx_msg.data[2]
                ,rx_msg.data_length_code);
        if(deviceType == MASTER && canIdentifier == ID)
        {
            switch (messageType)
            {
                case MSG_CARDTYPE:
                    if(rx_msg.data_length_code == 4)
                    {
                        state = STATE_CARDTYPE1;
                    }
                    else
                    {
                        state = STATE_CARDTYPE2;
                    }
                    break;
                case MSG_CONFIG:
                    state = STATE_DATAOUT_SLAVE;
                    break;
                case MSG_DATAOUT_MASTER:
                    if(rx_msg.data_length_code == 2)
                    {
                        outputs[rx_msg.data[1]].status = true;
                        outputs[rx_msg.data[1]].timer = 50;
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

static void mainTask(void *arg)
{
    state = STATE_GOONBUS;
    vTaskDelay(3000/ portTICK_PERIOD_MS);
    while(1)
    {
        switch (state)
        {
            case STATE_GOONBUS:
                printf("GOONBUS slave transmit\n");
                twai_transmit(&TRANSMIT_GOONBUS,portMAX_DELAY);
                break;
            case STATE_CARDTYPE1:
                printf("CARDTYPE1 slave transmit \n");
                twai_transmit(&TRANSMIT_CARDTYPE1,portMAX_DELAY);
                twai_transmit(&TRANSMIT_GOONBUS,portMAX_DELAY);
                break;
            case STATE_CARDTYPE2:
                printf("CARDTYPE2 slave transmit\n");
                twai_transmit(&TRANSMIT_CARDTYPE2,portMAX_DELAY);
                twai_transmit(&TRANSMIT_GOONBUS,portMAX_DELAY);
                break;
            case STATE_DATAOUT_SLAVE:
                printf("DATAOUT_SLAVE slave transmit \n");
                twai_transmit(&TRANSMIT_DATAOUT_SLAVE,portMAX_DELAY);
                break;
            default:
                break;
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void inputsManage()
{
    uint16_t inputsCounter = 0;
    vTaskDelay(3000/ portTICK_PERIOD_MS);
    while(1)
    {
        if(gpio_get_level(inputsGPIO[inputsCounter]) == 0)
        {
            TRANSMIT_INPUT.data[0] = inputsGPIO[inputsCounter] ;
            twai_transmit(&TRANSMIT_INPUT,portMAX_DELAY);
            printf("\n \n ENTRADA ACTIVA %d  %d \n \n", inputsGPIO[inputsCounter], gpio_get_level(inputsGPIO[inputsCounter]));
        }
        inputsCounter++;
        if(inputsCounter > 12) inputsCounter = 0;
        vTaskDelay(30/ portTICK_PERIOD_MS);
    }
}

void outputsTask()
{
    uint16_t outputsCounter = 0;
    vTaskDelay(3000/ portTICK_PERIOD_MS);
    while(1)
    {
        if(outputs[outputsCounter].status == true )
        {
            if(outputs[outputsCounter].timer > 0)
            {
                gpio_set_level(outputs[outputsCounter].outputPin, 1);
                outputs[outputsCounter].timer -= 10;
                printf("\n \n SALIDA ACTIVA %d  %d \n \n", outputs[outputsCounter].outputPin, gpio_get_level(outputs[outputsCounter].outputPin));
            }
            else
            {
                gpio_set_level(outputs[outputsCounter].pin, 0);
                outputs[outputsCounter].timer = 0;
                outputs[outputsCounter].status = false;
            }
        }
        outputsCounter++;
        if(outputsCounter > 11) outputsCounter = 0;
        vTaskDelay(20/ portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    //Config inputs
    gpio_set_direction(INPUT1,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT2,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT3,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT4,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT5,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT6,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT7,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT8,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT9,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT10,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT11,GPIO_MODE_INPUT);
	gpio_set_direction(INPUT12,GPIO_MODE_INPUT);
    gpio_set_direction(INPUT13,GPIO_MODE_INPUT);
    gpio_pullup_en(INPUT5);
    gpio_pullup_en(INPUT6);
    gpio_pullup_en(INPUT7);
    gpio_pullup_en(INPUT8);
    gpio_pullup_en(INPUT9);
    gpio_pullup_en(INPUT10);
    gpio_pullup_en(INPUT11);
    gpio_pullup_en(INPUT12);
    //Config outputs
    outputs[0].outputPin = OUTPUT1;
	outputs[1].outputPin = OUTPUT2;
	outputs[2].outputPin = OUTPUT3;
	outputs[3].outputPin = OUTPUT4;
	outputs[4].outputPin = OUTPUT5;
	outputs[5].outputPin = OUTPUT6;
	outputs[6].outputPin = OUTPUT7;
	outputs[7].outputPin = OUTPUT8;
	outputs[8].outputPin = OUTPUT9;
	outputs[9].outputPin = OUTPUT10;
	outputs[10].outputPin = OUTPUT11;
	outputs[11].outputPin = OUTPUT12;

    gpio_set_direction(OUTPUT1,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT2,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT3,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT4,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT5,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT6,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT7,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT8,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT9,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT10,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT11,GPIO_MODE_OUTPUT);
	gpio_set_direction(OUTPUT12,GPIO_MODE_OUTPUT);

	ESP_LOGI("InOutPins","Outputs initialized");
    
    xTaskCreatePinnedToCore(twai_receive_task, "twai_receive_task", 4096, NULL, 2, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(mainTask, "mainTask", 4096, NULL, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(inputsManage, "inputsManage", 4096, NULL, 1, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(outputsTask, "outputsTask", 4096, NULL, 1, NULL, tskNO_AFFINITY);
    //Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI("TAG", "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI("TAG", "Driver started");
}