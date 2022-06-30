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


#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "ixe_blufi.h"
#include "ixe_main.h"
#include "ixe_mqtt.h"
#include "ixe_ota.h"
#include "driver/gpio.h"


#include "lwip/apps/sntp.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "esp_ota_ops.h"
#include "esp_partition.h"


IxeMqttParam                 xm_params;
esp_mqtt_client_handle_t	 xm_client = NULL;
esp_mqtt_client_config_t     xmqtt_cfg;

extern  IxeParam                  x_params;
extern  volatile IxeData          x_datas;
extern  IxeBle                    x_ble;

static volatile  uint8_t xmqtt_status;
static volatile  uint8_t xmqtt_offline;


void xmqtt_inform_online(uint8_t online);

static int myhex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}


int myhex2byte(const char *hex)
{
	int a, b;
	a = myhex2num(*hex++);
	if (a < 0)
		return -1;
	b = myhex2num(*hex++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}


/**
 * hexstr2bin - Convert ASCII hex string into binary data
 * @hex: ASCII hex string (e.g., "01ab")
 * @buf: Buffer for the binary data
 * @len: Length of the text to convert in bytes (of buf); hex will be double
 * this size
 * Returns: 0 on success, -1 on failure (invalid hex string)
 */
int myhexstr2bin(const char *hex, uint8_t*buf, size_t len)
{
	size_t i;
	int a;
	const char *ipos = hex;
	uint8_t *opos = buf;

	for (i = 0; i < len; i++) {
		a = myhex2byte(ipos);
		if (a < 0)
			return -1;
		*opos++ = a;
		ipos += 2;
	}
	return 0;
}


void xmqtt_make_pubbuf(uint8_t* data,uint8_t data_len)
{
   IxeMqttHead     mqtt_head;
   uint8_t         temp_buf[20];
   uint8_t  len;
   uint8_t  i;
   
   memset(temp_buf,0x00,20);
   memset(xm_params.mqtt_pub_buf,0x00,MQTT_PUBBUF_LEN);

   mqtt_head.f_type = FRAME_TYPE_DEFAULT;
   mqtt_head.f_control = FRAME_CONTROL_DEFAULT;
   mqtt_head.f_snum = FRAME_SNUMBER_DEFAULT;
   mqtt_head.data_length = data_len;

   memcpy(temp_buf,&mqtt_head,sizeof(IxeMqttHead));
   memcpy(temp_buf+sizeof(IxeMqttHead),data,data_len);
   len = sizeof(IxeMqttHead)+data_len;
   for(i=0;i<len;i++)
   {    
      sprintf(&xm_params.mqtt_pub_buf[i*2],"%02x",temp_buf[i]);   
   }   
   xm_params.mqtt_pub_buf[len*2] = '\0';
   
   BLUFI_INFO("mqtt_pub_buf : %s\n",xm_params.mqtt_pub_buf);
}

uint8_t xmqtt_get_status()
{
  return xmqtt_status;
}

 void xmqtt_set_status(uint8_t value)
{
  xmqtt_status = value;
}

 uint8_t xmqtt_get_offline()
{
  return xmqtt_offline;
}

 void xmqtt_set_offline(uint8_t value)
{
  xmqtt_offline = value;
}


static void xmqtt_recv_handler(uint8_t *mqtt_data,uint8_t mqtt_data_len)
{
  int      data_len = 0;
  uint8_t  send_buf[64];
  uint8_t  send_len;
  
  data_len = mqtt_data[3];
  BLUFI_INFO("mqtt data len = %d,data_len = %02x", mqtt_data_len,data_len);
  if(mqtt_data[0] == FRAME_TYPE_DEFAULT && mqtt_data_len == (data_len + 4))
  {
    memset(send_buf,0x00,40);
    ixe_recv_app_com_handler(&mqtt_data[4],data_len,send_buf,&send_len);
	xmqtt_make_pubbuf(send_buf, send_len);
	esp_mqtt_client_publish(xm_client, xm_params.mqtt_pub_topic, xm_params.mqtt_pub_buf, 0, 1, 0);
  }
  return;
}

static esp_err_t xmqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
	
    static int sub_id = 0;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED://连接MQTT成功
            BLUFI_INFO("MQTT_EVENT_CONNECTED");
			sub_id = esp_mqtt_client_subscribe(client, xm_params.mqtt_sub_topic, 1);
			//BLUFI_INFO("MQTT_EVENT_SUBSCRIBE, sub_id=%d", sub_id);
            //msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED://断开MQTT
             BLUFI_INFO("MQTT_EVENT_DISCONNECTED");
			 xmqtt_set_status(XMQTT_DISCONNECT);
			 x_datas.wifi_discon = 1;
            break;
 
        case MQTT_EVENT_SUBSCRIBED://订阅成功
        	 //BLUFI_INFO("_---------订阅--------\n");
           BLUFI_INFO("MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
		   if(event->msg_id == sub_id)	 
		   {
		     xmqtt_set_status(XMQTT_INIT_OK);
			 //BLUFI_INFO("[%s:%d]mqtt_status = %d\n",__FILE__,__LINE__,xmqtt_get_status());
		   }
          //int msg_id = esp_mqtt_client_publish(client, mqtt_pub_topic, "start...", 0, 1, 0);
         // BLUFI_INFO("sent publish successful, msg_id=%d", msg_id);
          //  msg_id = esp_mqtt_client_publish(client, "/World", "data", 0, 0, 0);
          //  ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED://取消订阅
             BLUFI_INFO("MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED://发布成功
        	 //BLUFI_INFO("_--------发布----------\n");
             BLUFI_INFO("MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
			 xmqtt_set_status(XMQTT_PUBLISH_OK);
            break;
        case MQTT_EVENT_DATA://数据接收
             BLUFI_INFO("MQTT_EVENT_DATA");
             //BLUFI_INFO("[%s:%d]MQTT TOPIC=%s,DATA=%s, len=%d\r\n",__FILE__,__LINE__,event->topic,event->data,event->data_len);	
            // BLUFI_INFO("event data[0]-data[3] : %c%c%c%c",event->data[0],event->data[1],event->data[2],event->data[3]);
             uint8_t mqtt_data[BLUFI_SEND_LENGTH] = {0};
			 myhexstr2bin(event->data, mqtt_data, event->data_len/2);
			 BLUFI_INFO("mqtt data[0]-data[3] : %02x %02x %02x %02x",mqtt_data[0],mqtt_data[1],mqtt_data[2],mqtt_data[3]);
			 xmqtt_recv_handler(mqtt_data,event->data_len/2);
            break;
        case MQTT_EVENT_ERROR://MQTT错误
             BLUFI_INFO("MQTT_EVENT_ERROR");
			 xmqtt_set_status(XMQTT_DISCONNECT);
            break;
        default:
             BLUFI_INFO("Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}


static void xmqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    //BLUFI_INFO("Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    xmqtt_event_handler_cb(event_data);
}


static void xmqtt_params_init()
{
    xmqtt_cfg.uri = MQTT_URI, //MQTT 地址
    xmqtt_cfg.port = MQTT_PORT,	 //MQTT端口
    //mqtt_cfg.username = "admin",//用户名
    //mqtt_cfg.password = "public",//密码
    xmqtt_cfg.keepalive = 65,
    xmqtt_cfg.username = "lazelaze",//用户名
    xmqtt_cfg.password = "hs1997",//密码
    xmqtt_cfg.client_id = (char*)&x_ble.ble_name,

	memset(&xm_params,0x00,sizeof(IxeMqttParam));
    sprintf(xm_params.mqtt_inform_topic,"%s/inform",x_ble.ble_name);
	sprintf(xm_params.mqtt_heart_topic,"%s/heart",x_ble.ble_name);
	sprintf(xm_params.mqtt_pub_topic,"%s/data",x_ble.ble_name);
	sprintf(xm_params.mqtt_sub_topic,"%s/cmd",x_ble.ble_name);
	
	//BLUFI_INFO("mqtt sub topic = %s,pub topic = %s\n",xm_params.mqtt_sub_topic,xm_params.mqtt_pub_topic);

}

static esp_mqtt_client_handle_t xmqtt_client_init()
{
    esp_mqtt_client_handle_t mqtt_client = NULL;
    mqtt_client = esp_mqtt_client_init(&xmqtt_cfg);
	if(mqtt_client)
	{
		esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, xmqtt_event_handler, mqtt_client);
		//esp_mqtt_client_register_event();
		if( ESP_OK != esp_mqtt_client_start(mqtt_client) )
		{
			BLUFI_INFO("mqtt client start failed @@@@\n");
			return NULL;
		}else
		{
			 BLUFI_INFO("mqtt client start success @@@@\n");
		}
		return mqtt_client;
	}	
	return NULL;
}

void xmqtt_pubbuf_init()
{
  memset(xm_params.mqtt_pub_buf,0x00,64);
  xm_params.mqtt_pub_buf[0] = FRAME_TYPE_DEFAULT;
  xm_params.mqtt_pub_buf[1] = FRAME_CONTROL_DEFAULT;
  xm_params.mqtt_pub_buf[2] = FRAME_SNUMBER_DEFAULT;
}


void xmqtt_make_upline_buf(char *send_buf,uint8_t type)
{
    cJSON *root = NULL;
	char  *out = NULL;

	struct tm   timeinfo = { 0 };
    char        strftime_buf[64];
    time_t      now = 0;

	time(&now);
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
	
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_app_desc_t running_app_info;
	if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
		 BLUFI_INFO( "Running firmware version: %s\n", running_app_info.version);
	 }

	root = cJSON_CreateObject();
	if(type)
	{
	  cJSON_AddNumberToObject(root, "upline", 0);
	  cJSON_AddNumberToObject(root, "type", type);
	}else{
      cJSON_AddNumberToObject(root, "upline", 1);
	  cJSON_AddNumberToObject(root, "type", type);
	}
	
	cJSON_AddStringToObject(root, "version", running_app_info.version);
	cJSON_AddNumberToObject(root, "time-ts", now);
	cJSON_AddStringToObject(root, "time-str", strftime_buf);
	
	out = cJSON_Print(root);
	cJSON_Delete(root);
	
	//BLUFI_INFO("ixe upline send json data len = %d :%s\n",strlen(out),out);
    strcpy(send_buf,out);
	free(out);

	return;
}

void xmqtt_make_update_buf(char *send_buf,uint8_t num)
{
    cJSON *root = NULL;
	char  *out = NULL;
    
	struct tm	timeinfo = { 0 };
	char		strftime_buf[64];
	time_t		now = 0;
    time(&now);
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
	
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "update", 1);
	cJSON_AddNumberToObject(root, "status", num);
	cJSON_AddNumberToObject(root, "time-ts", now);
	cJSON_AddStringToObject(root, "time-str", strftime_buf);
	
	out = cJSON_Print(root);
	cJSON_Delete(root);
	
	//BLUFI_INFO("ixe update send json data len = %d :%s\n",strlen(out),out);
    strcpy(send_buf,out);
	free(out);

	return;
}

void xmqtt_make_notify_buf(char *send_buf,uint8_t num)
{
    cJSON *root = NULL;
	char  *out = NULL;

    struct tm	timeinfo = { 0 };
	char		strftime_buf[64];
	time_t		now = 0;
    time(&now);
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
	
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "notify", 1);
	cJSON_AddNumberToObject(root, "dev_status", num);
	cJSON_AddNumberToObject(root, "time-ts", now);
	cJSON_AddStringToObject(root, "time-str", strftime_buf);
	
	out = cJSON_Print(root);
	cJSON_Delete(root);
	
	//BLUFI_INFO("ixe update send json data len = %d :%s\n",strlen(out),out);
    strcpy(send_buf,out);
	free(out);

	return;
}



void xmqtt_make_heart_buf(char *send_buf,uint8_t num)
{
    cJSON *root = NULL;
	cJSON *array = NULL;
	char  *out = NULL;
	int   data[5] = {x_datas.status,x_params.wifi_onoff,x_datas.wifi_con,x_datas.ble_con,0};

	struct tm   timeinfo = { 0 };
    char        strftime_buf[64];
    time_t      now = 0;

	time(&now);
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);

	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "heart", num);
	
	array = cJSON_CreateIntArray(data, 5);
	cJSON_AddItemToObject(root, "data",array);

	cJSON_AddNumberToObject(root, "time-ts", now);
	cJSON_AddStringToObject(root, "time-str", strftime_buf);
   
	out = cJSON_Print(root);
	cJSON_Delete(root);
	
	//BLUFI_INFO("ixe heart send json data len = %d :%s\n",strlen(out),out);
    strcpy(send_buf,out);
	free(out);

	return;
}

void xmqtt_inform_online(uint8_t type)
{
  memset(xm_params.mqtt_pub_buf,0x00,64);
  xmqtt_make_upline_buf(xm_params.mqtt_pub_buf,type);
  esp_mqtt_client_publish(xm_client, xm_params.mqtt_inform_topic, xm_params.mqtt_pub_buf, 0, 1, 0);
  BLUFI_INFO("[%d]mqtt publish, topic:%s,payload:%s", __LINE__,xm_params.mqtt_inform_topic,xm_params.mqtt_pub_buf);   
}

void ixe_mqtt_task(void *param) 
{
    int i = 0;
	int msg_id = 0;
	uint8_t  mqtt_status;
	uint8_t  mqtt_offline;
	uint8_t  xota_status = NONE_UPDATE;
	uint8_t  x_status;
	uint8_t  dev_status = 0;
	uint32_t  xmqtt_hearts = 0;
	
   // BLUFI_INFO("[%s:%d]mqtt task start >>>\n",__FILE__,__LINE__);
	xmqtt_params_init();
	xmqtt_set_status(XMQTT_NOT_INIT);
	
	while(1)
	{		
	    sleep(1);

		mqtt_offline = xmqtt_get_offline();
		//BLUFI_INFO("[%s:%d]mqtt_cmd = %d\n",__FILE__,__LINE__,mqtt_cmd);
		if(mqtt_offline)
		{
		  if(xm_client)
		  { 
		    xmqtt_inform_online(mqtt_offline);
		    vTaskDelay(1000 / portTICK_PERIOD_MS);
		  	esp_mqtt_client_destroy(xm_client);
		    xm_client = NULL;
		    esp_wifi_stop();
		  }
          BLUFI_INFO("[%s:%d]ixe_mqtt_task delete\n",__FILE__,__LINE__);
		  vTaskDelete(NULL);
		}
		
		if(x_datas.wifi_con == 0)
		{
		  if((x_datas.wifi_discon== 1) && (i++ >= 60*3))
		  {
		     BLUFI_INFO("[%s:%d]mqtt esp wifi retry connect\n",__FILE__,__LINE__);
             x_datas.wifi_discon = 0;
			 esp_wifi_connect();
			 i = 0;
		  }
		  sleep(3);
          continue;
		}
		
		mqtt_status = xmqtt_get_status();
		switch(mqtt_status)
		{
		  case XMQTT_NOT_INIT:
		  	   if(xm_client == NULL)
		       {
		         xm_client = xmqtt_client_init();
			     BLUFI_INFO("mqtt sub topic = %s,pub topic = %s\n",xm_params.mqtt_sub_topic,xm_params.mqtt_pub_topic);
		  	   }else{
				 if(i++ > 5)
			     {
				    BLUFI_INFO("[%s:%d]mqtt esp subscribe fault,wifi disconnect!\n",__FILE__,__LINE__);
                    x_datas.wifi_discon = 1;
				    esp_wifi_disconnect();
					i = 0;
			     }
			   } 
			   break;
		   case XMQTT_INIT_OK:
		         xmqtt_inform_online(OFFLINE_NO_TYPE);
                 xmqtt_set_status(XMQTT_PUBLISH_OK);
				 sleep(3);
				break;
		   case XMQTT_PUBLISH_OK:
		   	    // update status inform
		   	    x_status = xota_get_status();
				//BLUFI_INFO("mqtt publish, x_status = %d,ota_status = %d /n",x_status,xota_status);
				if((x_status > NONE_UPDATE) && (x_status != xota_status))
				{
				  memset(xm_params.mqtt_pub_buf,0x00,MQTT_PUBBUF_LEN);
				  xmqtt_make_update_buf(xm_params.mqtt_pub_buf,x_status);
				  BLUFI_INFO("mqtt publish, msg_id=%d,topic:%s,payload:%s",msg_id,xm_params.mqtt_inform_topic,xm_params.mqtt_pub_buf); 
                  msg_id = esp_mqtt_client_publish(xm_client, xm_params.mqtt_inform_topic, xm_params.mqtt_pub_buf, 0, 1, 0);
				  if(msg_id < 0)
                  {
				    xmqtt_set_status(XMQTT_DISCONNECT);
			      }
		          i = 0;
				  xota_status = x_status;
				  continue;
				}
				// device anomal inform
				if((x_datas.dev_status > 0) && (dev_status != x_datas.dev_status))
				{
				  dev_status = x_datas.dev_status;
				  memset(xm_params.mqtt_pub_buf,0x00,MQTT_PUBBUF_LEN);
				  xmqtt_make_notify_buf(xm_params.mqtt_pub_buf,dev_status);
				  BLUFI_INFO("mqtt publish, msg_id=%d,topic:%s,payload:%s",msg_id,xm_params.mqtt_inform_topic,xm_params.mqtt_pub_buf); 
                  msg_id = esp_mqtt_client_publish(xm_client, xm_params.mqtt_inform_topic, xm_params.mqtt_pub_buf, 0, 1, 0);
				  if(msg_id < 0)
                  {
				    xmqtt_set_status(XMQTT_DISCONNECT);
			      }
		          i = 0;
				  continue;
				}
				// heart
			    if(i++ >= 60*1)
			    {
			       i = 1;
				   memset(xm_params.mqtt_pub_buf,0x00,MQTT_PUBBUF_LEN);
				   xmqtt_make_heart_buf(xm_params.mqtt_pub_buf,xmqtt_hearts++);
                   msg_id = esp_mqtt_client_publish(xm_client, xm_params.mqtt_heart_topic, xm_params.mqtt_pub_buf, 0, 1, 0);
                   BLUFI_INFO("mqtt publish, msg_id=%d,topic:%s,payload:%s", msg_id,xm_params.mqtt_heart_topic,xm_params.mqtt_pub_buf); 
				   if(msg_id < 0)
			       {
                     xmqtt_set_status(XMQTT_DISCONNECT);
			       }
			     }
				break;
			case XMQTT_DISCONNECT:
				 BLUFI_ERROR("[%s:%d]esp_mqtt_client_stop!",__FILE__,__LINE__);  
				 esp_mqtt_client_reconnect(xm_client);
				 break;
			default :
				break;
		 }
	  }
}
