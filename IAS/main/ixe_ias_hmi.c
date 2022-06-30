#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"

#include "esp_event.h"
#include "esp_log.h"

#include "driver/gpio.h"


#include "esp_uart_config.h"
#include "ixe_ias_hmi.h"
#include "esp_max1704.h"
#include "ixe_ias_lora.h"


RTC_DATA_ATTR int  boot_count;

#define uart_used          UART_NUM_1

static const char *TAG = "uart_events";

static QueueHandle_t uart0_queue;

#define  UART_PIN_TX   19
#define  UART_PIN_RX   18

#define GPIO_OUTPUT_HMI_EN      21
#define GPIO_OUTPUT_POWER_MASK  (1ULL<<GPIO_OUTPUT_HMI_EN)

static void ias_hmi_gpio_init()
{

  gpio_config_t io_conf;    
  //disable interrupt    
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;    
  //set as output mode    
  io_conf.mode = GPIO_MODE_OUTPUT;    
  //bit mask of the pins that you want to set,e.g.GPIO18/19    
  io_conf.pin_bit_mask = GPIO_OUTPUT_POWER_MASK;    
  //disable pull-down mode    
  io_conf.pull_down_en = 0;    
  //disable pull-up mode    
  io_conf.pull_up_en = 0;    
  //configure GPIO with the given settings    
  gpio_config(&io_conf);
  //gpio init,set led off
  gpio_set_level(GPIO_OUTPUT_HMI_EN, 1);
}


// uart_tx_task 任务。配置uart0-txrx，使用队列接收串口消息。（可以发送，但为了不影响手动发送的演示效果，给屏蔽了）
void ias_hmi_tx_task(void* arg)
{ 
   char  buf[16];
   char  end[3]={0xff,0xff,0xff};
   int   len = 0;
   int   val;
   float vol_value = 3.3;
   float soc_per = 50.0;

    sleep(6);
    while (1) {
		memset(buf,0x00,16);
		val = (int)(vol_value*1000/10);
		sprintf(&buf[0],"%s%d","x0.val=",val);
		//buf[10] = '\n';
		len = 10;
		uart_write_bytes(uart_used, buf, len);
		uart_write_bytes(uart_used, end, 3);
	    printf("\nuart send voltage:%s\n",buf);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
		
		memset(buf,0x00,16);
		if(soc_per > 100.0)
		  soc_per = 99.9;
		val = (int)(soc_per*100/10);
		sprintf(&buf[0],"%s%03d","x1.val=",val);
		//buf[10] = '\n';
		len = 10;
		
		uart_write_bytes(uart_used, &buf[0], len);
		uart_write_bytes(uart_used, &end[0], 3);
		printf("uart send soc percent:%s\n",buf);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
		
		memset(buf,0x00,16);
		sprintf(&buf[0],"%s%03d","n0.val=",boot_count);
		//buf[10] = '\n';
		len = 10;
		uart_write_bytes(uart_used, buf, len);
		uart_write_bytes(uart_used, end, 3);
	    printf("uart send boot_count:%s\n",buf);
		
        vTaskDelay(10*1000 / portTICK_PERIOD_MS);
		ias_get_vol_soc(&vol_value,&soc_per);
    }
}

// uart_rx_task 任务。通过串口接收数据，使用队列的方式。实现数据接收。
void ias_hmi_rx_task(void* arg)
{
    uart_event_t event;
    int length = 0;
	int i;
	
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE+1);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            //bzero(dtmp, RX_BUF_SIZE); // 不用清空也可以
            //ESP_LOGI(TAG, "uart[%d] event:", uart_used); // 使用 ESP_LOGI 和 uart_sendData，单次接收字段长度超过120时，显示效果是不一样的
            //uart_sendData(uart_used, "\r\nuart0 event::\r\n");
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    uart_get_buffered_data_len(uart_used, (size_t*)&length);
                    printf(TAG, "[UART DATA]: event.size=%d    length=%d: ", event.size, length);
                    //uart_sendData(uart_used, "\r\n+RECV:\r\n");
                    uart_read_bytes(uart_used, dtmp, event.size, portMAX_DELAY);
                    //ESP_LOGI(TAG, "[DATA EVT]:");
                    for(i=0;i<event.size;i++)
                    {
                      //printf(" %02x",dtmp[i]);
                    }
					//printf(" \n");
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uart_used);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uart_used);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}


void ias_hmi_rv_task(void *pvParameters)
{
    uart_event_t event;
    int i;
	uint8_t  recv_buf[64] = {0};
	
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
				if(event.size)
                {
                    uint8_t *temp = NULL;
					temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    if(temp == NULL){
                        printf("%s malloc.1 failed\n", __func__);
                        break;
                    }
                    memset(temp,0x0,event.size);
                    uart_read_bytes(uart_used,temp,event.size,portMAX_DELAY);
                    //igs cmd: 0x55 0xaa 0x03... 
					printf( "\n%s recv at cmd len = %d: ", __func__,event.size);
	                for(i=0;i<event.size;i++)
	                   printf("%02x ",recv_buf[i]);
	                printf("\n");
				    free(temp);
				 }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}



void ias_hmi_task(void *pvParameter)
{
    sleep(3);
	ias_hmi_gpio_init();
    uart_init_no_hwfc(uart_used, 115200, UART_PIN_TX , UART_PIN_RX , 10, &uart0_queue);
    // 创建 uart_tx_task 任务。
    xTaskCreate(ias_hmi_tx_task, "ias_uart_tx_task", 2048, NULL, 3, NULL);
    // 创建 uart_rx_task 任务。
    xTaskCreate(ias_hmi_rv_task, "ias_uart_rx_task", 2048, NULL, 3, NULL);
	while(1)
	{
      sleep(1);
	}
}
