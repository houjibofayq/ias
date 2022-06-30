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

#include   "esp_uart_config.h"
#include   "ixe_ias_485.h"
#include   "ixe_main.h"


static const char *TAG = "ixe_ias_485";

#define uart_485          UART_NUM_2

static QueueHandle_t uart2_queue;

#define  UART_PIN_TX   16
#define  UART_PIN_RX   17

#define GPIO_OUTPUT_485_POWER      33
#define GPIO_485_POWER_MASK  (1ULL<<GPIO_OUTPUT_485_POWER)


#ifdef SOIL_TEST_EN

typedef  struct soil_datas_{
    uint16_t    tem;
	uint16_t    wet;
	uint16_t    ec;
	uint8_t     testing;
}Soil_data;

Soil_data   soil_data;

static void ias_485_gpio_init()
{

  gpio_config_t io_conf;    
  //disable interrupt    
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;    
  //set as output mode    
  io_conf.mode = GPIO_MODE_OUTPUT;    
  //bit mask of the pins that you want to set,e.g.GPIO18/19    
  io_conf.pin_bit_mask = GPIO_485_POWER_MASK;    
  //disable pull-down mode    
  io_conf.pull_down_en = 0;    
  //disable pull-up mode    
  io_conf.pull_up_en = 0;    
  //configure GPIO with the given settings    
  gpio_config(&io_conf);
  //gpio init,set led off
  gpio_set_level(GPIO_OUTPUT_485_POWER, 1);
}


void soil_data_init()
{
  memset(&soil_data,0x00,sizeof(Soil_data));
}

void soil_data_recv(uint8_t *temp)
{
    ESP_LOGI(TAG,"RS485 recv data: %02x %02x, %02x %02x, %02x %02x,\n", temp[5],temp[6],temp[3],temp[4],temp[7],temp[8]);
    memset(&soil_data,0x00,sizeof(Soil_data));
	soil_data.wet = (temp[3]*256 + temp[4])/10; 
	soil_data.tem = (temp[5]*256 + temp[6])/10 + 1000; 
	soil_data.ec = temp[7]*256 + temp[8];
	soil_data.testing = 1;
    ESP_LOGI(TAG,"RS485 recv soil data:tep=%d,wet=%d,ec=%d\n", soil_data.tem,soil_data.wet,soil_data.ec);
	return;
}

void soil_data_get(int *tem,int *wet,int *ec,int *flag)
{
	*wet = soil_data.wet; 
	*tem = soil_data.tem; 
	*ec = soil_data.ec;
	*flag = soil_data.testing ;
    ESP_LOGI(TAG,"mqtt get soil data:tep=%d,wet=%d,ec=%d testing=%d\n", *tem,*wet,*ec,*flag);
	return;
}

#if 0
void soil_json_data_get(char *pub_buf)
{
    BLUFI_INFO("mqtt get soil data:tep=%d,wet=%d,ec=%d testing=%d\n", soil_data.tem,soil_data.wet,soil_data.ec,soil_data.testing);
    cJSON *root = NULL;
	cJSON *array = NULL;
	char  *out = NULL;
	int   data[4] = {soil_data.tem,soil_data.wet,soil_data.ec,soil_data.testing};

	root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "sn", g_params.sys.blue_name);
	#if 1
	array = cJSON_CreateIntArray(data, 4);
	cJSON_AddItemToObject(root, "data",array);
    #endif
	out = cJSON_Print(root);
	cJSON_Delete(root);
	
	ESP_LOGI(TAG,"RS485 send json data len = %d :%s\n",strlen(out),out);
    strcpy(pub_buf,out);
	free(out);

	return;
}
#endif

void soil_data_send(uint8_t *temp)
{
    ESP_LOGI(TAG,"RS485 recv soil data:tep=%d,wet=%d,ec=%d testing=%d\n", soil_data.tem,soil_data.wet,soil_data.ec,soil_data.testing);
    temp[0] = soil_data.tem/256;
	temp[1] = soil_data.tem%256;
	temp[2] = soil_data.wet/256;
	temp[3] = soil_data.wet%256;
	temp[4] = soil_data.ec/256;
	temp[5] = soil_data.ec%256;
	temp[6] = soil_data.testing;
	ESP_LOGI(TAG,"RS485 send data: %02x %02x, %02x %02x, %02x %02x, %02x\n", temp[0],temp[1],temp[2],temp[3],temp[4],temp[5],temp[6]);
   
	return;
}
#endif


// uart_tx_task 任务。配置uart0-txrx，使用队列接收串口消息。（可以发送，但为了不影响手动发送的演示效果，给屏蔽了）
void ias_uart_tx_task(void* arg)
{ 
   int ret;

   sleep(10);
   #ifdef SOIL_TEST_EN
     soil_data_init(); 
	 char soil_send[8] = {0xfe,0x03,0x00,0x00,0x00,0x03,0x11,0xc4};
   #endif
   
    while (1) {
		 ret = uart_write_bytes(uart_485,soil_send,8);
	     ESP_LOGI(TAG,"[%s:%d]...soil test send, ret = %d....\n", __FILE__,__LINE__,ret);
        vTaskDelay(10*1000 / portTICK_PERIOD_MS);
    }
}

// uart_rx_task 任务。通过串口接收数据，使用队列的方式。实现数据接收。
void ias_uart_rx_task(void* arg)
{
    uart_event_t event;
    int length = 0;
	int i;
	
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE+1);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart2_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            //bzero(dtmp, RX_BUF_SIZE); // 不用清空也可以
            //ESP_LOGI(TAG, "uart[%d] event:", uart_used); // 使用 ESP_LOGI 和 uart_sendData，单次接收字段长度超过120时，显示效果是不一样的
            //uart_sendData(uart_used, "\r\nuart0 event::\r\n");
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    uart_get_buffered_data_len(uart_485, (size_t*)&length);
                    printf(TAG, "[UART DATA]: event.size=%d    length=%d: ", event.size, length);
                    //uart_sendData(uart_used, "\r\n+RECV:\r\n");
                    uart_read_bytes(uart_485, dtmp, event.size, portMAX_DELAY);
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
                    uart_flush_input(uart_485);
                    xQueueReset(uart2_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uart_485);
                    xQueueReset(uart2_queue);
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


void ias_uart_rv_task(void *pvParameters)
{
    uart_event_t event;
    int i;
	uint8_t  recv_buf[64] = {0};
	
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart2_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
			ESP_LOGI(TAG, "uart2 rs485 recved data!\n");
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
                    uart_read_bytes(uart_485,temp,event.size,portMAX_DELAY);
                    //igs cmd: 0x55 0xaa 0x03... 
					printf( "\n%s recv at cmd len = %d: ", __func__,event.size);
	                for(i=0;i<event.size;i++)
	                   printf("%02x ",temp[i]);
	                printf("\n");
					#ifdef SOIL_TEST_EN
					if(event.size == 11 && temp[0] == 0xfe && temp[2] == 0x06)
					{
					  soil_data_recv(temp);
					}
					#endif
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

static void ixe_uart_485_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(uart_485, 4096, 8192, 10,&uart2_queue,0);
	//uart_driver_install(UART_NUM_2, 4096, 8192, 10,NULL,0);
    //Set UART parameters
    uart_param_config(uart_485, &uart_config);
    //Set UART pins
    //uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_set_pin(uart_485, UART_PIN_TX, UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
}


void ias_uart_task(void *pvParameter)
{
    sleep(10);
	ias_485_gpio_init();
   // uart_init_no_hwfc(uart_485, 9600, UART_PIN_TX , UART_PIN_RX , 10, &uart2_queue);
   ixe_uart_485_init();
    // 创建 uart_tx_task 任务。
    xTaskCreate(ias_uart_tx_task, "ias_uart_tx_task", 2048, NULL, 3, NULL);
    // 创建 uart_rx_task 任务。
    xTaskCreate(ias_uart_rv_task, "ias_uart_rx_task", 2048, NULL, 3, NULL);
	while(1)
	{
      sleep(1);
	}
}
