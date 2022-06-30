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

#include "driver/uart.h"

#include <time.h>

#include "lwip/apps/sntp.h"

#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_sleep.h"


#include "ixe_blufi.h"
#include "ixe_main.h"
#include "ixe_mqtt.h"
#include "ixe_ota.h"



#ifdef PRODUCT_IAS
#include "ixe_ias_util.h"
#include "ixe_ias_cmd.h"
#endif

#ifdef IAS_MODEM_4G
#include "ixe_ias_4g.h"
#endif

#ifdef IAS_485
#include "ixe_ias_485.h"
#endif

#ifdef IAS_LORA
#include "ixe_ias_lora.h"
#endif

#ifdef IAS_LTC
#include "ixe_ias_ltc2941.h"
#endif

#ifdef IAS_MAX1704
#include "esp_max1704.h"
#endif

#ifdef IAS_HMI
#include "ixe_ias_hmi.h"
#endif



IxeBle                    x_ble;
IxeParam                  x_params;
volatile IxeData          x_datas;



static void ixe_initialise_wifi(void);
static esp_err_t save_ble_params(void);
static esp_err_t save_ixe_params(void);



QueueHandle_t ixe_blufi_uart_queue = NULL;
//static uint16_t blufi_mtu_size = 23;

static uint8_t ixe_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
};

//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
static esp_ble_adv_data_t ixe_ble_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = ixe_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t ixe_ble_adv_params = {
    .adv_int_min        = 0x800,
    .adv_int_max        = 0x800,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define IXE_WIFI_LIST_NUM   10

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t ixe_wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int ixe_CONNECTED_BIT = BIT0;
#define WIFI_FAIL_BIT      BIT1
#define ESP_WIFI_MAXIMUM_RETRY  3

static int con_retry_num = 0;

/* store the station info for send back to phone */
//static uint8_t ixe_sta_bssid[6];
//static uint8_t ixe_sta_ssid[32];
//static int ixe_sta_ssid_len;

/* connect infor*/
static uint8_t ixe_server_if;
static uint16_t ixe_conn_id;

void ixe_set_wifis(uint8_t value)
{
  x_datas.wifi_con = value;
}

uint8_t ixe_get_wifis()
{
  return x_datas.wifi_con;
}

int ixe_get_time(uint8_t* hour,uint8_t* min,uint8_t* sec)
{
  time_t now; 
  time(&now);
  BLUFI_INFO("The number of seconds since January 1, 1970 is %ld",now);
  struct tm tms = { 0 };
  localtime_r(&now, &tms);
  *hour = tms.tm_hour;
  *min = tms.tm_min;
  *sec = tms.tm_sec;
  BLUFI_INFO("Get system time:%4d-%02d-%02d,%02d:%02d:%02d",tms.tm_year+1900,tms.tm_mon+1,tms.tm_mday,tms.tm_hour,tms.tm_min,tms.tm_sec);
  return 0;
}

int ixe_set_time(uint8_t hour,uint8_t min,uint8_t sec)
{
  time_t now; 
  time(&now);
  //BLUFI_INFO("The number of seconds since January 1, 1970 is %ld",now);
  struct tm tms = { 0 };
  localtime_r(&now, &tms);
  //BLUFI_INFO("Get system time:%4d-%2d-%2d,%2d:%2d:%2d",tms.tm_year+1900,tms.tm_mon+1,tms.tm_mday,tms.tm_hour,tms.tm_min,tms.tm_sec);
  tms.tm_hour = hour;
  tms.tm_min = min;
  tms.tm_sec = sec;

  time_t timep;
  timep = mktime(&tms);
  struct timeval tv;
  tv.tv_sec = timep; 
  //printf("tv_sec; %ld\n", tv.tv_sec);
  settimeofday(&tv,NULL);
  time(&now);
  //BLUFI_INFO("The number of seconds since January 1, 1970 is %ld",now);
  return 0;
}

uint8_t ixe_reset_factory(void)
{
   nvs_handle_t    my_handle;
   esp_err_t       err;

   BLUFI_ERROR("[%s:%d]...ixe reset factory ....\n", __FILE__,__LINE__);
   err = nvs_open("storage", NVS_READWRITE, &my_handle);
   if (err != ESP_OK) return 2;
   nvs_erase_key(my_handle,"ixe_params");
   nvs_erase_key(my_handle,"user_params");
   if(x_params.wifi_onoff == 1)
   {
     BLUFI_ERROR("[%s:%d]...ixe wifi restore ....\n", __FILE__,__LINE__);
     //esp_wifi_disconnect();
     esp_wifi_restore();
   }
   x_datas.restart = 0x01; 
  
  return 1;
}


static void ixe_app_status_resp(uint8_t *send_buf,uint8_t *send_len)
{

  send_buf[0] = IXE_QUERY_RESP;
  send_buf[1] = IXE_QUERY_STATUS;
  send_buf[2] = x_params.wifi_onoff;
  send_buf[3] = x_datas.wifi_con;
  send_buf[4] = x_datas.ble_con;
  send_buf[5] = x_datas.status;
  *send_len = 6;
  
  return;
}

static void ixe_app_AP_resp(uint8_t *send_buf,uint8_t *send_len)
{
 
  send_buf[0] = IXE_QUERY_RESP;
  send_buf[1] = IXE_QUERY_AP;
  
  BLUFI_INFO("[%s:%d]AP ssid:%s ,len = %d\n",__FILE__,__LINE__,x_params.sta_ssid,x_params.ssid_len);
  if(x_params.ssid_len > 0)
  {
    memcpy(&send_buf[2],(char*)&x_params.sta_ssid,x_params.ssid_len);
    *send_len = x_params.ssid_len + 2;
  }else{
    send_buf[2] = 0x00;
	*send_len = 3;
  }

  return;
}

static void ixe_app_time_resp(uint8_t *send_buf,uint8_t *send_len)
{
 
  send_buf[0] = IXE_QUERY_RESP;
  send_buf[1] = IXE_QUERY_TIME;
   uint8_t  hour;
   uint8_t  min;
   uint8_t  sec;

  ixe_get_time(&hour,&min,&sec);
  
  BLUFI_INFO("[%s:%d]APP get time: %02d:%02d:%02d \n",__FILE__,__LINE__,hour,min,sec);
  send_buf[2] = hour;
  send_buf[3] = min;
  send_buf[4] = sec;

  *send_len = 5;

  return;
}


static void ixe_app_set_resp(uint8_t set_type,uint8_t set_result,uint8_t *send_buf,uint8_t *send_len)
{
  send_buf[0] = IXE_SET_ACK;
  send_buf[1] = set_type;
  send_buf[2] = set_result;
  *send_len = 3;
  BLUFI_INFO("[%s:%d]set_type:%d ,result = %d\n",__FILE__,__LINE__,set_type,set_result);
  
  return;
}

static uint8_t ixe_app_set_wifi(uint8_t *data)
{
  uint8_t OnOff;
  uint8_t change_flag = 0;

  
  OnOff = data[0];
  BLUFI_INFO("Ble set, wifi onoff = %d\r\n",OnOff);
  if(x_params.wifi_onoff == OnOff)
  	return 1;

  switch(OnOff){

        case 0x00:
			      //x_datas.restart = 0x01;
				  xmqtt_set_offline(OFFLINE_CLOSE_WIFI);
	              change_flag = 1;
				  break;
		case 0x01:
				  ESP_ERROR_CHECK( esp_wifi_start() );
				  break;
		case 0x02:
				  esp_wifi_connect();
	              change_flag = 1;
				  OnOff = 1;
				  break;
	    case 0x03:
	              change_flag = 1;
				  OnOff = 1;
				  break;
		  default:
		  	      return 2;
  }
  
  if(change_flag)
  {
	 x_params.wifi_onoff = OnOff;
	 save_ixe_params();
  }
  
  return 1;
}

static uint8_t ixe_app_reset_factory(uint8_t *data)
{
   uint8_t         reset_flag;
   nvs_handle_t    my_handle;
   esp_err_t       err;

  reset_flag = data[0];
  BLUFI_INFO("reset_factory =%d\r\n",reset_flag);
  if(reset_flag == 0x01)
  {	 
     err = nvs_open("storage", NVS_READWRITE, &my_handle);
     if (err != ESP_OK) return 2;
	 nvs_erase_key(my_handle,"ixe_params");
	 nvs_erase_key(my_handle,"user_params");
	 x_datas.restart = 0x01; 
	 esp_wifi_stop();
	 esp_wifi_restore();
  }else
     return 2;
  
  return 1;
}

static uint8_t ixe_app_set_ota(uint8_t *data)
{
   uint8_t         set_flag;

  set_flag = data[0];
  BLUFI_INFO("Ble set, set_ota =%d\r\n",set_flag);
  if(set_flag == 1)
  {
    xota_set_status(UPDATE_START);
  }else
    return 2;
  
  return 1;
}

static uint8_t ixe_app_set_time(uint8_t *data)
{
   uint8_t    hour;
   uint8_t    min;
   uint8_t    sec;
   
  hour = data[0];
  min  = data[1];
  sec = data[2];
  BLUFI_INFO("app set time, %d:%d:%d\r\n",hour,min,sec);
  if(hour>= 24 || min>= 60 || sec >= 60)
    return 2;
  ixe_set_time(hour,min,sec);
  
  return 1;
}

static uint8_t ixe_app_set_zone(uint8_t *data)
{
  uint8_t    hour;
  uint8_t    min;
  uint8_t    sign;

  sign = data[0];
  hour = data[1];
  min  = data[2];
  if(hour> 12 && min >59)
  {
    return 2;
  }
  memset(x_params.dev_zone,0x00,16);
  if(sign)
  {
    sprintf(x_params.dev_zone,"GMT-%d:%02d",hour,min);
  }else{
    sprintf(x_params.dev_zone,"GMT+%d:%02d",hour,min);
  }
  BLUFI_INFO("app set zone, %s\n",x_params.dev_zone);

  save_ixe_params();
  x_datas.restart = 0x01; 
  return 1;
}

static uint8_t ixe_app_set_rallback(uint8_t *data)
{
   uint8_t         set_flag;

  set_flag = data[0];
  BLUFI_INFO("Ble set, set_rallback =%d\r\n",set_flag);
  if(set_flag == 1)
  {
    xota_set_status(UPDATE_RALLBACK);
  }else
    return 2;
  
  return 1;
}


static uint8_t ixe_app_reset_ixe(uint8_t *data)
{
   uint8_t         reset_flag;
   nvs_handle_t    my_handle;
   esp_err_t       err;

  reset_flag = data[0];
  BLUFI_INFO("Ble set, reset_ixe =%d\r\n",reset_flag);
  if(reset_flag == 0x01)
  {	 
     err = nvs_open("storage", NVS_READWRITE, &my_handle);
     if (err != ESP_OK) return 2;
     nvs_erase_key(my_handle,"ixe_ble");
	 nvs_erase_key(my_handle,"ixe_params");
	 nvs_erase_key(my_handle,"user_params");
	 x_datas.restart = 0x01; 
  }else
     return 2;
  
  return 1;
}

static int ixe_app_set_ble(uint8_t *data,uint16_t len)
{
  BLUFI_INFO("Ble set, ble name = %s, len=%d\r\n",(char*)data,len);	  
  if(len > 0 && len < 16)
  {
    memset(x_ble.ble_name,0x00,16);
    memcpy(x_ble.ble_name,(char*)data,len);
	save_ble_params();
	x_datas.restart = 0x01;  
  }else
    return 2;
  
  return 1;
}

void ixe_recv_app_com_handler(uint8_t *recv_data,uint32_t recv_len,uint8_t *send_buf,uint8_t *send_len)
{
  uint8_t  main_type;
  uint8_t  sub_type;
  uint8_t  *data;
  uint8_t  len;
  uint8_t  ret = 0;

  
  if(NULL == recv_data || recv_len<2)
  {
    return;
  }
  main_type = recv_data[0];
  sub_type  = recv_data[1];
  data = &recv_data[2];
  len = recv_len -2;
  BLUFI_INFO("[%s:%d]ixe_recv_app_data,len = %d ,cmd type:%02x %02x\n",__FILE__,__LINE__,recv_len,main_type,sub_type);

  //query command
  if(main_type == IXE_QUERY_CMD)
  {
    switch(sub_type)
    {
      case IXE_QUERY_STATUS:
	  	     ixe_app_status_resp(send_buf,send_len);
	  	     break;
	  case IXE_QUERY_AP:
	  	     ixe_app_AP_resp(send_buf,send_len);
	  	     break;
	  case IXE_QUERY_TIME:
	  	     ixe_app_time_resp(send_buf,send_len);
	  	     break;
	  default:
	  	     break;
	}
	return;
  }
  //set command
  if(main_type == IXE_SET_CMD)
  {
    switch(sub_type)
    {
	  case IXE_SET_WIFI_OnOff:
			 ret = ixe_app_set_wifi(data);
	  	     break;
	  case IXE_RESET_PARAM:
			 ret = ixe_app_reset_factory(data);
	  	     break;
	  case IXE_SET_OTA:
			 ret = ixe_app_set_ota(data);
	  	     break;
	  case IXE_SET_TIME:
			 ret = ixe_app_set_time(data);
	  	     break;
	  case IXE_SET_ZONE:
			 ret = ixe_app_set_zone(data);
	  	     break;
	  case IXE_SET_RALLBACK:
			 ret = ixe_app_set_rallback(data);
	  	     break;
	  case IXE_RESET_PROGRAME:
			 ret = ixe_app_reset_ixe(data);
	  	     break;
	  case IXE_SET_BLE_NAME:
			 ret = ixe_app_set_ble(data,len);
	  	     break;
	  default:
	  	     ret = 2;
	  	     break;
	 }
	ixe_app_set_resp(sub_type,ret,send_buf,send_len);
	return;
   }
  
    #ifdef PRODUCT_IAS
	  if(main_type == IAS_CMD_RECV)
	  {
	    printf("ble recvd ias cmd\n");
	    ias_product_comd_handler(recv_data,recv_len,send_buf,send_len);
	  }
	  return;
	#endif  
  
  return;
}


static void ixe_ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    wifi_mode_t mode;

    switch (event_id) {
    case IP_EVENT_STA_GOT_IP: {
        esp_blufi_extra_info_t info;

        xEventGroupSetBits(ixe_wifi_event_group, ixe_CONNECTED_BIT);
        esp_wifi_get_mode(&mode);
        ixe_set_wifis(1);
        memset(&info, 0, sizeof(esp_blufi_extra_info_t));
        memcpy(&info.sta_bssid, (uint8_t*)&x_params.sta_bssid, 6);
        info.sta_bssid_set = true;
        info.sta_ssid = (uint8_t*)&x_params.sta_ssid;
        info.sta_ssid_len = x_params.ssid_len;
		BLUFI_INFO("[%s:%d]BLUFI wifi is  connected yet, AP:%s \n",__FILE__,__LINE__,x_params.sta_ssid);
        if (x_datas.ble_con == true) {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
        } else {
            BLUFI_INFO("BLUFI BLE is not connected yet\n");
        }
		if(x_params.wifi_onoff == 0)
		{
		  esp_wifi_disconnect();
		  BLUFI_INFO("[%s:%d]BLUFI wifi is  disconnected yet! \n",__FILE__,__LINE__);
		}
        break;
    }
    default:
        break;
    }
    return;
}

static void ixe_wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    wifi_event_sta_connected_t *event;
    wifi_mode_t mode;

    switch (event_id) {
    case WIFI_EVENT_STA_START:
		BLUFI_INFO("[%s:%d]esp32 wifi start connect\n",__FILE__,__LINE__);
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        x_datas.wifi_con = true;
        event = (wifi_event_sta_connected_t*) event_data;
        memcpy((char*)&x_params.sta_bssid, (char*)event->bssid, 6);
		x_params.ssid_len = event->ssid_len;
		memset((char*)&x_params.sta_ssid, 0, 32);
        memcpy((char*)&x_params.sta_ssid, (char*)event->ssid, event->ssid_len);
		save_ixe_params();
		con_retry_num = 0;
        break; 
    case WIFI_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        x_datas.wifi_con = false;
		wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t*) event_data;
		BLUFI_ERROR( "[%s:%d]esp32 wif disconnect reason : %d",__FILE__,__LINE__,disconnected->reason);        
		if(con_retry_num++ < ESP_WIFI_MAXIMUM_RETRY) {                                   
			BLUFI_ERROR( "retry to connect to the AP,retry_num = %d",con_retry_num);       
		}else{  
		   x_datas.wifi_discon = 1;
		}
		if(x_datas.wifi_discon == 0 && x_params.wifi_onoff == 1)
          esp_wifi_connect();
        xEventGroupClearBits(ixe_wifi_event_group, ixe_CONNECTED_BIT);
        break;
    case WIFI_EVENT_AP_START:
        esp_wifi_get_mode(&mode);

        /* TODO: get config or information of softap, then set to report extra_info */
        if (x_datas.ble_con == true) {
            if (x_datas.wifi_con) {  
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, NULL);
            } else {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
            }
        } else {
            BLUFI_INFO("BLUFI BLE is not connected yet\n");
        }
        break;
    case WIFI_EVENT_SCAN_DONE: {
        uint16_t apCount = 0;
        esp_wifi_scan_get_ap_num(&apCount);
        if (apCount == 0) {
            BLUFI_INFO("Nothing AP found");
            break;
        }
		BLUFI_INFO("[%d]BLUFI AP found %d !\n",__LINE__,apCount);
        wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);
        if (!ap_list) {
            BLUFI_ERROR("malloc error, ap_list is NULL");
            break;
        }
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, ap_list));
        esp_blufi_ap_record_t * blufi_ap_list = (esp_blufi_ap_record_t *)malloc(apCount * sizeof(esp_blufi_ap_record_t));
        if (!blufi_ap_list) {
            if (ap_list) {
                free(ap_list);
            }
            BLUFI_ERROR("malloc error, blufi_ap_list is NULL");
            break;
        }
        for (int i = 0; i < apCount; ++i)
        {
            blufi_ap_list[i].rssi = ap_list[i].rssi;
            memcpy(blufi_ap_list[i].ssid, ap_list[i].ssid, sizeof(ap_list[i].ssid));
        }
        
        if (x_datas.ble_con == true) {
            esp_blufi_send_wifi_list(apCount, blufi_ap_list);
        } else {
            BLUFI_INFO("BLUFI BLE is not connected yet\n");
        }

        esp_wifi_scan_stop();
		x_datas.wifi_discon = 0;
		esp_wifi_connect();
        free(ap_list);
        free(blufi_ap_list);
        break;
    }
    default:
        break;
    }
    return;
}

static void ixe_initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ixe_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &ixe_wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ixe_ip_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_MIN_MODEM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	
    //ESP_ERROR_CHECK( esp_wifi_start() );
}

static void ixe_blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param)
{
    /* actually, should post to blufi_task handle the procedure,
     * now, as a example, we do it more simply */
   
    static wifi_config_t   sta_config;
    static wifi_config_t   ap_config;
	
    switch (event) {
    case ESP_BLUFI_EVENT_INIT_FINISH:
        BLUFI_INFO("BLUFI init finish\n");
        esp_ble_gap_set_device_name(x_ble.ble_name);
        esp_ble_gap_config_adv_data(&ixe_ble_adv_data);
        break;
    case ESP_BLUFI_EVENT_DEINIT_FINISH:
        BLUFI_INFO("BLUFI deinit finish\n");
        break;
    case ESP_BLUFI_EVENT_BLE_CONNECT:
        x_datas.ble_con = true;
        ixe_server_if = param->connect.server_if;
        ixe_conn_id = param->connect.conn_id;
        esp_ble_gap_stop_advertising();
        blufi_security_init();
		BLUFI_INFO( "[%s:%d]---blufi ble connected!---\r\n",__FILE__,__LINE__);
        break;
    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
        x_datas.ble_con = false;
        blufi_security_deinit();
        esp_ble_gap_start_advertising(&ixe_ble_adv_params);
        BLUFI_INFO( "[%s:%d]---blufi ble disconnect!---\r\n",__FILE__,__LINE__);
        break;
    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
        BLUFI_INFO("BLUFI Set WIFI opmode %d\n", param->wifi_mode.op_mode);
        ESP_ERROR_CHECK( esp_wifi_set_mode(param->wifi_mode.op_mode) );
        break;
    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
        BLUFI_INFO("BLUFI requset wifi connect to AP,[%s,%d]\n",__FILE__,__LINE__);
        /* there is no wifi callback when the device has already connected to this wifi
        so disconnect wifi before connection.
        */
        esp_wifi_disconnect();
        esp_wifi_connect();
        break;
    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
        BLUFI_INFO("BLUFI requset wifi disconnect from AP,[%s,%d]\n",__FILE__,__LINE__);
        esp_wifi_disconnect();
        break;
    case ESP_BLUFI_EVENT_REPORT_ERROR:
        BLUFI_ERROR("[%s:%d]BLUFI report error, error code %d\n",__FILE__,__LINE__,param->report_error.state);
        //esp_blufi_send_error_info(param->report_error.state);
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
        wifi_mode_t mode;
        esp_blufi_extra_info_t info;
        esp_wifi_get_mode(&mode);
        if (x_datas.wifi_con) {  
            memset(&info, 0, sizeof(esp_blufi_extra_info_t));
            memcpy(&info.sta_bssid, (uint8_t*)&x_params.sta_bssid, 6);
            info.sta_bssid_set = true;
            memcpy(&info.sta_ssid, (char*)&x_params.sta_ssid, x_params.ssid_len);
            info.sta_ssid_len = x_params.ssid_len;
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
        } else {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
        }
        BLUFI_INFO("BLUFI get wifi status from AP,[%s,%d]\n",__FILE__,__LINE__);

        break;
    }
    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
        BLUFI_INFO("blufi close a gatt connection");
        esp_blufi_close(ixe_server_if, ixe_conn_id);
        break;
    case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
        /* TODO */
        break;
	case ESP_BLUFI_EVENT_RECV_STA_BSSID:
        memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
        sta_config.sta.bssid_set = 1;
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA BSSID %s,[%s,%d]\n", sta_config.sta.bssid,__FILE__,__LINE__);
        break;
	case ESP_BLUFI_EVENT_RECV_STA_SSID:
        strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid, param->sta_ssid.ssid_len);
        sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA SSID %s,[%s,%d]\n", sta_config.sta.ssid,__FILE__,__LINE__);
        break;
	case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
        strncpy((char *)sta_config.sta.password, (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
        sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA PASSWORD %s,[%s,%d]\n", sta_config.sta.password,__FILE__,__LINE__);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
        strncpy((char *)ap_config.ap.ssid, (char *)param->softap_ssid.ssid, param->softap_ssid.ssid_len);
        ap_config.ap.ssid[param->softap_ssid.ssid_len] = '\0';
        ap_config.ap.ssid_len = param->softap_ssid.ssid_len;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP SSID %s, ssid len %d\n", ap_config.ap.ssid, ap_config.ap.ssid_len);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
        strncpy((char *)ap_config.ap.password, (char *)param->softap_passwd.passwd, param->softap_passwd.passwd_len);
        ap_config.ap.password[param->softap_passwd.passwd_len] = '\0';
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP PASSWORD %s len = %d\n", ap_config.ap.password, param->softap_passwd.passwd_len);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
        if (param->softap_max_conn_num.max_conn_num > 4) {
            return;
        }
        ap_config.ap.max_connection = param->softap_max_conn_num.max_conn_num;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP MAX CONN NUM %d\n", ap_config.ap.max_connection);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
        if (param->softap_auth_mode.auth_mode >= WIFI_AUTH_MAX) {
            return;
        }
        ap_config.ap.authmode = param->softap_auth_mode.auth_mode;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP AUTH MODE %d\n", ap_config.ap.authmode);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
        if (param->softap_channel.channel > 13) {
            return;
        }
        ap_config.ap.channel = param->softap_channel.channel;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP CHANNEL %d\n", ap_config.ap.channel);
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_LIST:{
		BLUFI_INFO("[%s:%d]Get esp32 wifi list,disconnect ap\n",__FILE__,__LINE__);
		x_datas.wifi_discon = 1;
        esp_wifi_disconnect();
		sleep(1);
        wifi_scan_config_t scanConf = {
            .ssid = NULL,
            .bssid = NULL,
            .channel = 0,
            .show_hidden = false
        };
		ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
        break;
    }
    case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
        //BLUFI_INFO("[%s,%d] Recv blufi Custom Data %d,\n", __FILE__,__LINE__,param->custom_data.data_len);
        esp_log_buffer_hex("Custom Data", param->custom_data.data, param->custom_data.data_len);
		uint8_t  send_buf[BLUFI_SEND_LENGTH] = {0};
        uint8_t  send_len = 0;
		ixe_recv_app_com_handler(param->custom_data.data,param->custom_data.data_len,send_buf,&send_len);
		esp_blufi_send_custom_data(send_buf, send_len);
        break;
	case ESP_BLUFI_EVENT_RECV_USERNAME:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CA_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
        /* Not handle currently */
        break;;
	case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
        /* Not handle currently */
        break;
    default:
        break;
    }
}

static void ixe_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&ixe_ble_adv_params);
        break;
    default:
        break;
    }
}

esp_err_t ixe_wifi_connect()
{
   if(x_datas.wifi_con)
   	 return ESP_OK;
   else
   	return -1;
}

static void set_default_ble_params()
{
  #if (defined PRODUCT_IAS) || (defined PRODUCT_IPE)
     memset(&x_ble,0x00,sizeof(IxeBle));
     strcpy(x_ble.ble_name,DEFAULT_BLE_DEVICE_NAME);
  #endif
}

static void set_default_ixe_params()
{
  memset((void*)&x_params,0x00,sizeof(IxeParam));
  x_params.wifi_onoff = DEFAULT_WIFI_ONOFF;
  x_params.ssid_len= 0;
  strcpy(x_params.dev_zone,DEFAULT_DEVICE_ZONE);
}

static esp_err_t save_ble_params(void)
{
	nvs_handle_t	my_handle;
	esp_err_t	   err;
	
	
	err = nvs_open("storage", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) return err;

    err = nvs_set_blob(my_handle, "ixe_ble", &x_ble,sizeof(IxeBle));
    printf((err != ESP_OK) ? "***save param Failed!\n" : "***save ixe ble Done\n");

    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    nvs_close(my_handle);

	return err;
}

static esp_err_t save_ixe_params(void)
{
	nvs_handle_t	my_handle;
	esp_err_t	    err;
	
	
	err = nvs_open("storage", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) return err;

    err = nvs_set_blob(my_handle, "ixe_params", (void*)&x_params,sizeof(IxeParam));
    printf((err != ESP_OK) ? "***save param Failed!\n" : "***save system param Done\n");

    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    nvs_close(my_handle);

	return err;
}


static esp_err_t ixe_read_storage_params(void)
{
   nvs_handle_t    my_handle;
   esp_err_t       err;
   size_t        length;
   
   
   err = nvs_open("storage", NVS_READWRITE, &my_handle);
   if (err != ESP_OK) return err;

   //nvs_erase_key(my_handle,"ixe_ble");
   length = sizeof(IxeBle);
   err = nvs_get_blob(my_handle, "ixe_ble", &x_ble, &length);
   switch (err) {
      case ESP_OK:   
				BLUFI_INFO("read params length = %d ,f_params length = %d\n",length,sizeof(IxeBle));
				BLUFI_INFO("[%s:%d]The ble adv name :%s!\n", __FILE__,__LINE__,x_ble.ble_name);
                break;
      case ESP_ERR_NVS_NOT_FOUND:
	  	        BLUFI_INFO("[%s:%d]The factory param is not initialized yet!\n", __FILE__,__LINE__);
				set_default_ble_params();
				save_ble_params();
                break;
      default :
                BLUFI_ERROR("[%s:%d]Error (%s) reading!\n", __FILE__,__LINE__,esp_err_to_name(err));
   }	

   //nvs_erase_key(my_handle,"user_params");
   length = sizeof(IxeParam);
   err = nvs_get_blob(my_handle, "ixe_params",(void*)&x_params, &length);
   nvs_close(my_handle);
   switch (err) {
      case ESP_OK:   
				BLUFI_INFO("read params length = %d ,s_params length = %d\n",length,sizeof(IxeParam));
				BLUFI_INFO("[%s:%d]The s_params,  wifi_onoff = %d\n", __FILE__,__LINE__,x_params.wifi_onoff);
                break;
      case ESP_ERR_NVS_NOT_FOUND:
	  	        BLUFI_INFO("[%s:%d]The system param is not initialized yet!\n", __FILE__,__LINE__);
				set_default_ixe_params();
				save_ixe_params();
                break;
      default :
                BLUFI_ERROR("[%s:%d]Error (%s) reading!\n", __FILE__,__LINE__,esp_err_to_name(err));
   }
   
   return ESP_OK;
}

void ixe_init_params()
{
   memset(&x_ble,0x00,sizeof(IxeBle));
   memset((void*)&x_params,0x00,sizeof(IxeParam));
   memset((void*)&x_datas,0x00,sizeof(IxeData));
   
   ixe_read_storage_params();
}

static esp_blufi_callbacks_t ixe_blufi_callbacks = {
    .event_cb = ixe_blufi_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

void ixe_ble_start()
{
    esp_err_t ret;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        BLUFI_ERROR("%s initialize bt controller failed: %s\n", __func__, esp_err_to_name(ret));
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        BLUFI_ERROR("%s enable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    BLUFI_INFO("BD ADDR: "ESP_BD_ADDR_STR"\n", ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));

    BLUFI_INFO("BLUFI VERSION %04x\n", esp_blufi_get_version());

    ret = esp_ble_gap_register_callback(ixe_gap_event_handler);
    if(ret){
        BLUFI_ERROR("%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_blufi_register_callbacks(&ixe_blufi_callbacks);
    if(ret){
        BLUFI_ERROR("%s blufi register failed, error code = %x\n", __func__, ret);
        return;
    }

    esp_blufi_profile_init();

}

void ixe_ble_restart()
{
    esp_err_t ret;

   
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		   BLUFI_ERROR("%s initialize bt controller failed: %s\n", __func__, esp_err_to_name(ret));
	}
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        BLUFI_ERROR("%s enable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
	ret = esp_bluedroid_init();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
     
	ret = esp_ble_gap_register_callback(ixe_gap_event_handler);
    if(ret){
        BLUFI_ERROR("%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_blufi_register_callbacks(&ixe_blufi_callbacks);
    if(ret){
        BLUFI_ERROR("%s blufi register failed, error code = %x\n", __func__, ret);
        return;
    }

    esp_blufi_profile_init();

}

static void ixe_initialize_sntp(void)
{
	BLUFI_INFO( "[%s:%d]Initializing SNTP", __FILE__,__LINE__);
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "ntp1.aliyun.com");
	sntp_setservername(1, "210.72.145.44");		// 国家授时中心服务器 IP 地址
    sntp_setservername(2, "1.cn.pool.ntp.org");        

	sntp_init();
}

void ixe_sntp_task(void *param) 
{
  
  struct tm   timeinfo = { 0 };
  char        strftime_buf[64];
  time_t      now = 0;
  int         i = 0;
  int         retry = 0;
  
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  ixe_initialize_sntp();
  
  // set timezone to China Standard Time
  //setenv("TZ", "GMT-8", 1);
  setenv("TZ", &x_params.dev_zone[0], 1);
  tzset();
  BLUFI_INFO("Device time zone : %s\n",x_params.dev_zone);

  while(1)
 { 
	 if(x_datas.wifi_con == false)
	 {
	   vTaskDelay(1000 / portTICK_PERIOD_MS);
	   continue;
	 }
     // wait for time to be set    
     while (timeinfo.tm_year < (2021 - 1900))
    {
       if(x_datas.wifi_con == false)
	     continue;
	   if(++retry >= 20)
	  {
		 BLUFI_ERROR("[%s:%d]...wifi connect enthenet failed!....\n", __FILE__,__LINE__);
		 //break;
	   }
	   //BLUFI_INFO("[%s:%d]Waiting for system time to be set... (%d) %d", __FILE__,__LINE__,retry,timeinfo.tm_year);
	   vTaskDelay(3000 / portTICK_PERIOD_MS);
	   time(&now);
	   localtime_r(&now, &timeinfo);
	   //BLUFI_INFO("The number of seconds since January 1, 1970 is %ld",now);
     }
	 
     //2021-01-07 12:00:00 
     //Thu Jan 14 07:44:10 2021
     strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
     BLUFI_INFO("The current date/time in Shanghai is: %s", strftime_buf);
     //sntp_stop();
     const int deep_sleep_sec = 60*60 - 20;
	 sleep(20);
     BLUFI_INFO("Entering deep sleep for %d seconds", deep_sleep_sec);
     esp_deep_sleep(1000000LL * deep_sleep_sec); 
     for(i=0;i<3600;i++)
     {
       vTaskDelay(1000 / portTICK_PERIOD_MS);
	   if(x_datas.wifi_con == false)
	   	 break;
	 }
   }
  
}


void app_main(void)
{
    esp_err_t ret;

    uint8_t  hour;
	uint8_t  min;
	uint8_t  sec;
	ixe_get_time(&hour,&min,&sec);
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

	ixe_init_params();
	
    //ixe_initialise_wifi();
    if(x_params.wifi_onoff == 1)
	{
	  ESP_ERROR_CHECK( esp_wifi_start() );
      BLUFI_INFO("[%s:%d]...init wifi ok ....\n", __FILE__,__LINE__);
    }
	ixe_ble_start();

	xTaskCreate(ixe_sntp_task, "ixe_sntp_task", 4096, NULL, 4, NULL); 
	//xTaskCreate(ixe_mqtt_task, "ixe_mqtt_task", 4096, NULL, 4, NULL); 
	xTaskCreate(ixe_ota_task, "ixe_ota_task", 4096, NULL, 3, NULL);
	
	#ifdef PRODUCT_IAS
	xTaskCreate(ias_led_key_task, "ias_led_key_task",2048, NULL, 10, NULL);
	xTaskCreate(ias_util_task, "ias_util_pump_task",2048, NULL, 10, NULL);
	#endif

	#ifdef IAS_MODEM_4G
	xTaskCreate(ias_modem_4g_task, "ias_modem_4g_task",4096, NULL, 10, NULL);
	#endif

	#ifdef IAS_485
	xTaskCreate(ias_uart_task, "ias_uart_task",4096, NULL, 10, NULL);
	#endif

	#ifdef IAS_LORA
	xTaskCreate(ias_lora_task, "ias_lora_task",8192, NULL, 10, NULL);
	#endif

	#ifdef IAS_LTC
	xTaskCreate(ias_ltc_task, "ias_ltc_task",4096, NULL, 10, NULL);
	#endif

	#ifdef IAS_MAX1704
	xTaskCreate(ias_max1704_task, "ias_max1704_task",4096, NULL, 10, NULL);
	#endif

	#ifdef IAS_HMI
	xTaskCreate(ias_hmi_task, "ias_uart_task",4096, NULL, 10, NULL);
	#endif

	uint8_t xmqtt_offline;
	uint8_t xota_sta;
	while(1)
	{
	  sleep(1);
      xmqtt_offline = xmqtt_get_offline();
	  xota_sta = xota_get_status();
	  if(x_params.wifi_onoff ==1)
	  {
	    //BLUFI_INFO("[%s:%d] ESP  wifi_con = %d, discon =  %d!\n",__FILE__,__LINE__,x_datas.wifi_con,x_datas.wifi_discon);
	    if(x_datas.wifi_con == false)
	    {
          x_datas.status = 1;
	    }else{
	      if(xota_sta == UPDATE_START)
		  	x_datas.status = 0x10;
		  else
            x_datas.status = 0;
		}
	     
	  }else{
          x_datas.status = 0;
	  }
	  if(xmqtt_offline == OFFLINE_CLOSE_WIFI)
	  {
		sleep(10);
		xmqtt_set_offline(OFFLINE_NO_TYPE);
		xTaskCreate(ixe_mqtt_task, "ixe_mqtt_task", 4096, NULL, 3, NULL);
		//BLUFI_INFO("[%s:%d] ESP  ixe_mqtt_task restart!\n",__FILE__,__LINE__);
	  }
      if((xota_sta == UPDATE_FAIL) || (xota_sta == VERSION_SAME))
	  {
		ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_MIN_MODEM) );
		sleep(2);
        ixe_ble_restart();
		xTaskCreate(ixe_ota_task, "ixe_ota_task", 4096, NULL, 3, NULL);
		//BLUFI_INFO("[%s:%d] ESP  ixe_ota_task restart!\n",__FILE__,__LINE__);
	  }
	  
	  if(x_datas.restart)
	  {
	    BLUFI_INFO("[%s:%d] ESP  restart  after 2s!\n",__FILE__,__LINE__);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	    esp_restart();
	  }
	}
}
