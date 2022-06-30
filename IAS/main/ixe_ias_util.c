/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/****************************************************************************
* This is a demo for bluetooth config wifi connection to ap. You can config ESP32 to connect a softap
* or config ESP32 as a softap to be connected by other device. APP can be downloaded from github 
* android source code: https://github.com/EspressifApp/EspBlufi
* iOS source code: https://github.com/EspressifApp/EspBlufiForiOS
****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "ixe_blufi.h"
#include "ixe_main.h"

#ifdef   PRODUCT_IAS 
#include "ixe_ias_util.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
 
extern  IxeBle                    x_ble;
extern  IxeParam                  x_params;
extern  volatile IxeData          x_datas;


#define GPIO_OUTPUT_SPI_EN      33
#define GPIO_OUTPUT_4G_EN       21

#define GPIO_OUTPUT_LED_RED      23
#define GPIO_OUTPUT_LED_GREEN    22


#define GPIO_OUTPUT_MASK  ((1ULL<<GPIO_OUTPUT_LED_RED) | (1ULL<<GPIO_OUTPUT_LED_GREEN) | (1ULL<<GPIO_OUTPUT_SPI_EN) | (1ULL<<GPIO_OUTPUT_4G_EN))


#define GPIO_INPUT_RESET_KEY      36



volatile uint8_t    led_green = 0;
volatile uint8_t    led_red = 0;

typedef enum {
	KEY_SHORT_PRESS = 1, 
	KEY_LONG_PRESS,
} alink_key_t;



/*ADC所接的通道
ADC1_CH0 (GPIO 36)
ADC1_CH1 (GPIO 37)
ADC1_CH2 (GPIO 38)
ADC1_CH3 (GPIO 39)
ADC1_CH4 (GPIO 32)
ADC1_CH5 (GPIO 33)
ADC1_CH6 (GPIO 34)
ADC1_CH7 (GPIO 35)
*/

#define ADC1_TEST_CHANNEL3 ADC1_CHANNEL_3  

//ADC初始化
//ADC_ATTEN_DB_0:表示参考电压为1.1V
//ADC_ATTEN_DB_2_5:表示参考电压为1.5V
//ADC_ATTEN_DB_6:表示参考电压为2.2V
//ADC_ATTEN_DB_11:表示参考电压为3.9V
void ias_adc_Init()
{	
  adc1_config_width(ADC_WIDTH_12Bit);  // 12位分辨率	
  adc1_config_channel_atten(ADC1_TEST_CHANNEL3,ADC_ATTEN_DB_6);  // 设置通道3和2.2V参考电压
}	

void ias_power_on(gpio_num_t gpio_num)
{
  gpio_set_level(gpio_num, 1);
}

void ias_power_off(gpio_num_t gpio_num)
{
  gpio_set_level(gpio_num, 0);
}

void ias_led_on(gpio_num_t gpio_num)
{
  gpio_set_level(gpio_num, 0);
}

void ias_led_off(gpio_num_t gpio_num)
{
  gpio_set_level(gpio_num, 1);
}


#define  IAS_LED_TASK_DELAY   (100 / portTICK_RATE_MS)
void ias_led_display(uint8_t LedOn_Cnt,uint8_t LedOff_Cnt,gpio_num_t gpio_num)
{
  static uint8_t cnt = 0;
  if(++cnt >= LedOff_Cnt)
  {
    cnt = 0;
    ias_led_on(gpio_num);
	vTaskDelay(LedOn_Cnt* IAS_LED_TASK_DELAY);
    ias_led_off(gpio_num);
  }
}

void ias_led_blink_slow(gpio_num_t gpio_num)
{
  uint8_t On = 2;
  uint8_t Off = 10;
  ias_led_display(On,Off,gpio_num);
}

void ias_led_blink_quick(gpio_num_t gpio_num)
{
  uint8_t On = 1;
  uint8_t Off = 1;
  ias_led_display(On,Off,gpio_num);
}

static void ias_led_gpio_init()
{
  gpio_config_t io_conf;    
  //disable interrupt    
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;    
  //set as output mode    
  io_conf.mode = GPIO_MODE_OUTPUT;    
  //bit mask of the pins that you want to set,e.g.GPIO18/19    
  io_conf.pin_bit_mask = GPIO_OUTPUT_MASK;    
  //disable pull-down mode    
  io_conf.pull_down_en = 0;    
  //disable pull-up mode    
  io_conf.pull_up_en = 0;    
  //configure GPIO with the given settings    
  gpio_config(&io_conf);
  //gpio init,set led off
  gpio_set_level(GPIO_OUTPUT_LED_RED, 1);
  gpio_set_level(GPIO_OUTPUT_LED_GREEN, 1);
  //BLUFI_ERROR("[%s:%d]...Test flag ....\n", __FILE__,__LINE__);
    
}

static uint8_t ias_set_leds_status(void)
{
   uint8_t  value = 0;
   if(x_datas.ble_con)
     value = GREEN_BLE_CONING;
   else
   	 value = GREEN_BLE_NOCON;
   
   return value;
}

static xQueueHandle gpio_evt_queue = NULL;

void IRAM_ATTR gpio_isr_handler(void *arg) {
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void ias_key_gpio_init(uint32_t key_gpio_pin) {

	//配置GPIO，下降沿和上升沿触发中断
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.pin_bit_mask = 1ULL << GPIO_INPUT_RESET_KEY;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	gpio_set_intr_type(GPIO_INPUT_RESET_KEY, GPIO_INTR_ANYEDGE);
	gpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));

	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_INPUT_RESET_KEY, gpio_isr_handler, (void *) GPIO_INPUT_RESET_KEY);
}

esp_err_t alink_key_scan(TickType_t ticks_to_wait) {

	uint32_t io_num;
	BaseType_t press_key = pdFALSE;
	BaseType_t lift_key = pdFALSE;
	int backup_time = 0;

	while (1) {

		//接收从消息队列发来的消息
		xQueueReceive(gpio_evt_queue, &io_num, ticks_to_wait);

		//记录下用户按下按键的时间点
		if (gpio_get_level(io_num) == 0) {
			press_key = pdTRUE;
			backup_time = esp_timer_get_time();
			//如果当前GPIO口的电平已经记录为按下，则开始减去上次按下按键的时间点
		} else if (press_key) {
			//记录抬升时间点
			lift_key = pdTRUE;
			backup_time = esp_timer_get_time() - backup_time;
		}

		//近当按下标志位和按键弹起标志位都为1时候，才执行回调
		if (press_key & lift_key) {
			press_key = pdFALSE;
			lift_key = pdFALSE;

			//如果大于1s则回调长按，否则就短按回调
			if (backup_time > 2000000) {
				return KEY_LONG_PRESS;
			} else {
				return KEY_SHORT_PRESS;
			}
		}
	}
}

void ias_key_trigger(void *arg) 
{
	esp_err_t ret = 0;
	ias_key_gpio_init(GPIO_INPUT_RESET_KEY);

	while (1) {
		ret = alink_key_scan(portMAX_DELAY);
		if (ret == -1)
			vTaskDelete(NULL);

		switch (ret) {
		case KEY_SHORT_PRESS:
			printf("短按触发回调 ... \r\n");
			BLUFI_ERROR("[%s:%d]...ESP Restart ....\n", __FILE__,__LINE__);
		    sleep(1);
            esp_restart(); 
			break;

		case KEY_LONG_PRESS:
			printf("长按触发回调 ... \r\n");
			break;

		default:
			break;
		}
	}

	vTaskDelete(NULL);
}

void ias_led_key_task(void* arg)
{
  uint8_t    leds_val = 0;
  
  xTaskCreate(ias_key_trigger, "key_trigger", 1024 * 2, NULL, 10,NULL);
  
  ias_led_gpio_init();
  while(1)
  {    
    vTaskDelay(IAS_LED_TASK_DELAY);
    // led display
    leds_val = ias_set_leds_status();
	switch(leds_val)
	{
       case SYSTEM_STARTING:
	   	                    break;
	   case GREEN_BLE_NOCON:
	   	                    ias_led_off(GPIO_OUTPUT_LED_RED);
	   	                    ias_led_blink_slow(GPIO_OUTPUT_LED_GREEN);
	   	                    break;
	   case GREEN_BLE_CONING:
	   	                    ias_led_off(GPIO_OUTPUT_LED_GREEN);
	   	                    ias_led_on(GPIO_OUTPUT_LED_RED);
	   	                    break;
	                 default:
						    break;
	  }
	}
}

uint8_t ias_check_power_voltage(void)
{
  int adc_read = 0;
  int power_voltage = 0;
  static uint8_t test_cnts = 0; 
  static uint8_t low_cnts = 0;  

  adc_read = adc1_get_raw(ADC1_TEST_CHANNEL3);//采集ADC
  power_voltage = (adc_read*2200)/4096;      //mv
  if(power_voltage <= 1460)
  	low_cnts++;
  //BLUFI_ERROR("[%s:%d]...adc_read: %d power_voltage: %d mv....\n", __FILE__,__LINE__,read_raw1,(read_raw1*2200)/4096);
  if(test_cnts++ == 5)
  {
     if(low_cnts >= 5)
     {
	    BLUFI_ERROR("[%s:%d]...power test low! ....\n", __FILE__,__LINE__);
	 }
     test_cnts = 0;
	 low_cnts = 0;
  }
  return 0;
}

void ias_util_task(void* arg)
{
 
  //ise_read_params();
 

  //ias_adc_Init();

  sleep(2);
  
 
  ias_power_on(GPIO_OUTPUT_SPI_EN);
  while(1)
  {
    sleep(1);
	
  }
}


#endif

