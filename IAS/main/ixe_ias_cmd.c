
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

#include "ixe_blufi.h"
#include "ixe_main.h"
#include "ixe_ias_util.h"
#include "ixe_ias_cmd.h"

#include "ixe_ias_lora.h"
#include "esp_max1704.h"
extern  volatile IxeData          x_datas;


#ifdef   PRODUCT_IAS 

static void ias_app_query_resp(uint8_t *send_buf,uint8_t *send_len)
{
  int lora_sents = 0;
  int lora_recvs = 0;

  float soc = 0.0;
  float val = 0.0;
  
  ias_lora_get_cnts(&lora_sents,&lora_recvs);
  printf("ias query resp,sents = %d,recvs = %d\n",lora_sents,lora_recvs);

  ias_get_vol_soc(&val,&soc);
  
  send_buf[0] = IAS_CMD_RESP;
  send_buf[1] = IAS_QUERY_PAMS;
  send_buf[2] = (uint8_t)lora_sents;
  send_buf[3] = (uint8_t)lora_recvs;
  send_buf[4] = (uint8_t)(val*10);
  send_buf[5] = (uint8_t)soc;
  
  *send_len = 6;
  
  return;
}

static uint8_t ias_app_set_params(uint8_t *data)
{ 
  return 1;
}

void ias_app_set_resp(uint8_t set_type,uint8_t set_result,uint8_t *send_buf,uint8_t *send_len)
{
  send_buf[0] = IAS_CMD_RESP;
  send_buf[1] = set_type;
  send_buf[2] = set_result;
  *send_len = 3;
  BLUFI_INFO("[%s:%d]set_type:%d ,result = %d\n",__FILE__,__LINE__,set_type,set_result);
  
  return;
}


void ias_product_comd_handler(uint8_t *recv_data,uint32_t recv_len,uint8_t *send_buf,uint8_t *send_len)
{
  uint8_t  main_type;
  uint8_t  sub_type;
  uint8_t  *data;
  uint8_t  ret = 0;

  
  if(NULL == recv_data || recv_len<2)
  {
    return;
  }
  main_type = recv_data[0];
  sub_type  = recv_data[1];
  data = &recv_data[2];
  printf("ias recv cmd,main_type = %d,sub_type = %d\n",main_type,sub_type);
  //query command
  if(main_type == IAS_CMD_RECV)
  {
    switch(sub_type)
    {
      case IAS_QUERY_PAMS:
	  	     ias_app_query_resp(send_buf,send_len);
	  	     break;
	  case IAS_SET_PAMS:
	  	     ret = ias_app_set_params(data);
	  	     ias_app_set_resp(sub_type,ret,send_buf,send_len);
	  	     break;
	  default:
	  	     break;
	}
  }else{
      ias_app_set_resp(sub_type,2,send_buf,send_len);
  }
  return;
}


#endif

