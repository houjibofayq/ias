#ifndef _ISE_MQTT_
#define _ISE_MQTT_

#define  FRAME_TYPE_DEFAULT       0x4d
#define  FRAME_CONTROL_DEFAULT    0x00
#define  FRAME_SNUMBER_DEFAULT    0X01


#define MQTT_URI     "mqtt://sa.lazelaze.com"
//#define MQTT_URI     "mqtt://dev.seunji.com"

#define MQTT_PORT    1883
#define MQTT_PUBBUF_LEN     128

typedef struct _ixe_mqtt_param{
			   char                         mqtt_inform_topic[40];
			   char                         mqtt_heart_topic[40];
               char                         mqtt_pub_topic[40];
               char                         mqtt_sub_topic[40];			   
               char                         mqtt_pub_buf[MQTT_PUBBUF_LEN];
}IxeMqttParam;

typedef struct _ixe_mqtt_head{
   uint8_t  f_type;
   uint8_t  f_control;
   uint8_t  f_snum;
   uint8_t  data_length;
}IxeMqttHead;


uint8_t xmqtt_get_status();

uint8_t xmqtt_get_offline();

void xmqtt_set_status(uint8_t value);
void xmqtt_set_offline(uint8_t value);



void ixe_mqtt_task(void *param); 


#endif
