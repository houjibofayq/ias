/*# Connection with the RF module
By default, the pins used to control the RF transceiver are--

Pin | Signal
--- | ------
CS | IO15
RST | IO32
MISO | IO13 
MOSI | IO12
SCK | IO14

but you can reconfigure the pins using ```make menuconfig``` and changing the options in the "LoRa Options --->"
*/


/*
## Basic usage
A simple **sender** program...
```c
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"

#include "esp_event.h"
#include "esp_log.h"

#include "ixe_main.h"
#include "ixe_ias_lora.h"
#include "esp_lora_config.h"


static const char *TAG = "ixe_lora";
extern IxeBle      x_ble;

static int sent_cnts = 0;
static int recv_cnts = 0;

void ias_lora_get_cnts(int* sent,int* recv)
{
   *sent = sent_cnts;
   *recv = recv_cnts;
   return;
}

uint8_t buf[32];
void ias_lora_task_rx(void *p)
{
   int x;
   uint32_t i = 0;
   char buf[32] = {0};
   
   for(;;) {
      lora_receive();    // put into receive mode
      while(lora_received()) {
         x = lora_receive_packet((uint8_t*)buf, sizeof(buf));
         buf[x] = '\0';
         ESP_LOGI(TAG,"%s received %d packet: %s\n",(char*)&x_ble.ble_name[0],++i, buf);
         recv_cnts = i;
		 bzero(buf,32);
		 lora_receive();
      }
      vTaskDelay(1);
   }
}

void ias_lora_task_tx(void *p)
{
   uint32_t i=0;
   char   str[32] = {0};
   
   for(;;) {
      vTaskDelay(pdMS_TO_TICKS(10000));
	  sprintf(str,"%s-%04d",(char*)&x_ble.ble_name[0],++i);
      lora_send_packet((uint8_t*)str, strlen(str));
      ESP_LOGI(TAG,"%s lora packet: %d sent %s\n",(char*)&x_ble.ble_name[0],i,str);
      sent_cnts = i;
   }
}

void ias_lora_task(void *pvParameter)
{
   sleep(5);
   lora_init();
   ESP_LOGI(TAG,"lora init over!\n");
   //lora_set_frequency(915e6);
   lora_set_frequency(435e6);
   lora_set_tx_power(17);
   lora_set_spreading_factor(12);
   lora_set_bandwidth(125e3);
   lora_set_coding_rate(5);
   lora_set_preamble_length(0x0005);
   lora_enable_crc();
   xTaskCreate(&ias_lora_task_tx, "ias_lora_task_tx", 2048, NULL, 5, NULL);
   //xTaskCreate(&ias_lora_task_rx, "ias_lora_task_rx", 2048, NULL, 5, NULL);
   while(1)
   {
     sleep(1);
   }
}

