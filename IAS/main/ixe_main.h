#ifndef _IXE_MAIN
#define _IXE_MAIN

#define  DEFAULT_BLE_DEVICE_NAME           "IAS010001"
#define  DEFAULT_WIFI_ONOFF                   0
#define  DEFAULT_DEVICE_ZONE                "GMT-8:00"

#define  BLUFI_SEND_LENGTH                    256

#define  PRODUCT_IAS                        1


#ifdef    PRODUCT_IAS
#define  DEFAULT_BLE_DEVICE_NAME           "IAS010001"
#endif

//#define  IAS_MODEM_4G                         1
#define  IAS_485                             1
//#define  IAS_LORA                             1
//#define  IAS_LTC                             1
#define  IAS_MAX1704                             1
//#define  IAS_HMI                             1
#define  SOIL_TEST_EN                           1



   
typedef struct _IxeBle{ 
	char     ble_name[16];
	char     ble_mac[12];
} IxeBle; 

typedef struct _IxeParam
{ 
   uint8_t    wifi_onoff;
   char       sta_ssid[32];
   uint8_t    ssid_len;
   uint8_t    sta_bssid[6];
   char       dev_zone[16];
}IxeParam; 


typedef struct _IxeData{
	uint8_t    status;  
	uint8_t    dev_status; 
	uint8_t    wifi_con;
	uint8_t    wifi_discon;
	uint8_t    ble_con;
	uint8_t    restart;
}IxeData;


enum ixe_communicate_method{
	NONE_METHOD = 0,
    BLE_METHOD,
    WIFI_METHOD,
    OTHER_METHOD
};

enum ixe_mqtt_status{
	XMQTT_NOT_INIT = 0,
    XMQTT_INIT_OK,
    XMQTT_PUBLISH_OK,
    XMQTT_DISCONNECT,
    XMQTT_RESTART
};

enum ixe_mqtt_offline{
	OFFLINE_NO_TYPE = 0,
    OFFLINE_RESET_KEY,
    OFFLINE_RECOERTY_KEY,
    OFFLINE_CLOSE_WIFI,
    OFFLINE_OTA_OK,
    OFFLINE_OTA_FAULT,
};



enum ixe_frame_main_type{
	IXE_CMD_TYPE_START = 0,
	IXE_QUERY_CMD,
	IXE_QUERY_RESP,
	IXE_SET_CMD,
	IXE_SET_ACK,
	IXE_CMD_TYPE__END
};

enum ixe_frame_query_type{
	IXE_QUERY_START = 0,
	IXE_QUERY_STATUS,
	IXE_QUERY_AP,
	IXE_QUERY_TIME,
	IXE_QUERY_END
};

enum ixe_frame_set_type{
	IXE_SET_CMD_START = 0,
	IXE_SET_WIFI_OnOff,
	IXE_RESET_PARAM,
	IXE_SET_OTA,
	IXE_SET_TIME,
	IXE_SET_ZONE,
	IXE_SET_RALLBACK,
	IXE_RESET_PROGRAME = 0x11,
	IXE_SET_BLE_NAME = 0x12,
	IXE_SET_TYPE__END
};


uint8_t ixe_reset_factory(void);


void ixe_recv_app_com_handler(uint8_t *recv_data,uint32_t recv_len,uint8_t *send_buf,uint8_t *send_len);
esp_err_t ixe_wifi_connect();
void ixe_ble_start();




#endif
