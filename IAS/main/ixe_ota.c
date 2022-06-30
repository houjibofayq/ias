/* OTA example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
//#include "protocol_examples_common.h"
#include "errno.h"

#include "esp_wifi.h"


#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include <esp_bt.h>
#include <esp_gatts_api.h>
#include <esp_gatt_common_api.h>

#include "ixe_main.h"
#include "ixe_mqtt.h"
#include "ixe_ota.h"
#include "ixe_blufi.h"


extern  IxeParam                  x_params;
extern  volatile IxeData          x_datas;
extern  IxeBle                    x_ble;



#define BUFFSIZE 1024
#define HASH_LEN 32 /* SHA-256 digest length */

static const char *TAG = "ixe_ota";
/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256

static volatile uint8_t  xota_status = 0;


void xota_set_status(uint8_t value)
{
    xota_status = value;
}

uint8_t xota_get_status(void)
{
  return xota_status;
}


static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

static void __attribute__((noreturn)) task_fatal_error(void)
{
    xmqtt_set_offline(OFFLINE_OTA_FAULT);
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);
    
    while (1) {
        ;
    }
}

esp_err_t ixe_ble_stop(void)
{
    esp_err_t err;
    ESP_LOGD(TAG, "Free mem at start of simple_ble_stop %d", esp_get_free_heap_size());
	err = esp_blufi_profile_deinit();
	if (err != ESP_OK) {
        return ESP_FAIL;
    }
    err = esp_bluedroid_disable();
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "esp_bluedroid_disable called successfully");
    err = esp_bluedroid_deinit();
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGD(TAG, "esp_bluedroid_deinit called successfully");
    err = esp_bt_controller_disable();
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    /* The API `esp_bt_controller_deinit` will have to be removed when we add support for
     * `reset to provisioning`
     */
    ESP_LOGD(TAG, "esp_bt_controller_disable called successfully");
    err = esp_bt_controller_deinit();
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "esp_bt_controller_deinit called successfully");

    ESP_LOGD(TAG, "Free mem at end of simple_ble_stop %d", esp_get_free_heap_size());
    return ESP_OK;
}


static void ixe_do_ota_task(void *pvParameter)
{
    uint8_t  type = 0;
    char  ota_url[128] = {0};
	
    if(strstr(x_ble.ble_name,"ISE03"))
	{
	  strcpy(ota_url,"https://cosota.lazelaze.com/firmware/ISE03/ISE03_OTA.bin?sn=");
	  type = PRO_ISE03;
    }
	else if(strstr(x_ble.ble_name,"IPE01"))
	{
	  strcpy(ota_url,"https://cosota.lazelaze.com/firmware/newIPE01/IPE01_OTA.bin?sn=");
	  type = PRO_IPE01;
	}else{
	  BLUFI_INFO("[%s:%d]BLE do not have correct name!",__FILE__,__LINE__);
	  strcpy(ota_url,"https://cosota.lazelaze.com/firmware/IPE01/IPE01_OTA.bin?sn=");
	}
    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    esp_http_client_config_t config = {
        //.url = "http://192.168.1.53:8070/blufi_demo.bin",
		.url = strcat(ota_url,x_ble.ble_name),
        .cert_pem = (char *)server_cert_pem_start,
        .timeout_ms = 20000,
    };
    BLUFI_INFO("ixe ota url:%s\n",config.url);

//#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    //config.skip_cert_common_name_check = true;
//#endif

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        task_fatal_error();
    }
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        task_fatal_error();
    }
    esp_http_client_fetch_headers(client);

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);
	#if 0
	BLUFI_INFO("[%d]configued_partition type %d subtype %d at offset 0x%08x,size 0x%08x\n",__LINE__,
		       configured->type, configured->subtype, configured->address,configured->size);
	BLUFI_INFO("[%d]running_partition type %d subtype %d at offset 0x%08x,size 0x%08x \n",__LINE__,
		       running->type, running->subtype, running->address, running->size);
	BLUFI_INFO("[%d]update_partition type %d subtype %d at offset 0x%08x,size 0x%08x \n",__LINE__,
				update_partition->type,update_partition->subtype,update_partition->address,update_partition->size);
    #endif
    int binary_file_length = 0;
    /*deal with all receive packet*/
    bool image_header_was_checked = false;
	BLUFI_INFO("start get ota data [%d]\n",__LINE__);
    while (1) {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        if (data_read < 0) {
            ESP_LOGE(TAG, "Error: SSL data read error");
            http_cleanup(client);
            task_fatal_error();
        } else if (data_read > 0) {
            if (image_header_was_checked == false) {
                esp_app_desc_t new_app_info;
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                    // check current version with downloading
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);
					
		            if((type == PRO_ISE03) && (strstr(new_app_info.version,"ISE03") == NULL))
	                {
                      xota_set_status(VERSION_FAULT);
					  vTaskDelete(NULL);
					  break;
					}  
		            if((type == PRO_IPE01) && (strstr(new_app_info.version,"IPE01") == NULL))
	                {
                      xota_set_status(VERSION_FAULT);
					  vTaskDelete(NULL);
					  break;
					}
                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }

                    // check current version with last invalid partition
                    if (last_invalid_app != NULL) {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                            ESP_LOGW(TAG, "New version is the same as invalid version.");
                            ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                            ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                            http_cleanup(client);
							xota_set_status(VERSION_SAME);
							vTaskDelete(NULL);
						    break;
                        }
                    }
                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0) {
                        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                        http_cleanup(client);
						xota_set_status(VERSION_SAME);
                        vTaskDelete(NULL);
						break;
                    }
                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        http_cleanup(client);
                        task_fatal_error();
						break;
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");
                } else {
                    ESP_LOGE(TAG, "received package is not fit len");
                    http_cleanup(client);
                    task_fatal_error();
					break;
                }
            }
            err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK) {
                http_cleanup(client);
                task_fatal_error();
            }
            binary_file_length += data_read;
            ESP_LOGD(TAG, "Written image length %d", binary_file_length);
        } else if (data_read == 0) {
           /*
            * As esp_http_client_read never returns negative error code, we rely on
            * `errno` to check for underlying transport connectivity closure if any
            */
            if (errno == ECONNRESET || errno == ENOTCONN) {
                ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
				xota_set_status(DOWNLOAD_FAIL);
                break;
            }
            if (esp_http_client_is_complete_data_received(client) == true) {
                ESP_LOGI(TAG, "Connection closed");
				sleep(2);
                break;
            }
        }
    }
    ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);
    if (esp_http_client_is_complete_data_received(client) != true) {
        ESP_LOGE(TAG, "Error in receiving complete file");
        http_cleanup(client);
        task_fatal_error();
		xota_set_status(DOWNLOAD_FAIL);
    }
    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
		xota_set_status(DOWNLOAD_FAIL);
    }
    
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
    }else{
       #if (defined PRODUCT_ISE) || (defined PRODUCT_IPE)
           xota_set_status(DOWNLOAD_OVER);
	   #endif
    }
	sleep(1);
    ESP_LOGI(TAG, "OTA over ,prepare to restart system!");
    x_datas.restart = 0x01;
	#if (defined PRODUCT_ISE) || (defined PRODUCT_IPE)
	   xmqtt_set_offline(OFFLINE_OTA_OK);
	#else
	   esp_restart();
	#endif
    while(1);
}

void ixe_ota_rallback()
{
  esp_err_t err;
  
  const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
  const esp_partition_t *configured = esp_ota_get_boot_partition();
  const esp_partition_t *running = esp_ota_get_running_partition();

  
  BLUFI_INFO("[%d]configued_partition type %d subtype %d at offset 0x%08x,size 0x%08x\n",__LINE__,
				 configured->type, configured->subtype, configured->address,configured->size);
  BLUFI_INFO("[%d]running_partition type %d subtype %d at offset 0x%08x,size 0x%08x \n",__LINE__,
				 running->type, running->subtype, running->address, running->size);
  BLUFI_INFO("[%d]update_partition type %d subtype %d at offset 0x%08x,size 0x%08x \n",__LINE__,
				  update_partition->type,update_partition->subtype,update_partition->address,update_partition->size);

  err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        task_fatal_error();
    }else{
       xota_set_status(DOWNLOAD_OVER);
    }
	sleep(1);
    ESP_LOGI(TAG, "Prepare to restart system!");
    x_datas.restart = 0x01; ;

}

void ixe_ota_task(void *pvParameter)
{
    esp_err_t err;
	uint8_t   ota_t = 0;
   
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
	
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
           ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
           esp_ota_mark_app_valid_cancel_rollback();      
        }
    }

	xota_set_status(NONE_UPDATE);
	
#if (defined PRODUCT_IAS) || (defined PRODUCT_IPE)
	while(1)
    {
      if(xota_get_status() == UPDATE_RALLBACK)
      {
        ixe_ota_rallback();
	  }
      else if(x_datas.wifi_con == 0 || xota_get_status() == NONE_UPDATE)
      {
        sleep(1);
		continue;
	  }else{
 		break;
	  }
	}
#endif

    printf("ixe start ota...[%s:%d]\n",__FILE__,__LINE__);
    ixe_ble_stop();
	sleep(2);
	esp_wifi_set_ps(WIFI_PS_NONE);    //should enable wifi modem sleep when both wifi and blueooth enable

    // Initialize NVS.
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
	
    ESP_ERROR_CHECK( err );
    
    ESP_ERROR_CHECK(esp_netif_init());
	
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(ixe_wifi_connect());
    printf("...ota before check over... [%d:%s]\n",__LINE__,__FILE__);
    
    xTaskCreate(&ixe_do_ota_task, "ixe_do_ota_task", 8192, NULL, 5, NULL);
    while(1)
    {
      sleep(1);
	  if(xota_get_status() == VERSION_SAME)
	  {  
	     sleep(2);
	     BLUFI_INFO("[%s:%d] ESP  ixe_ota_task delete!\n",__FILE__,__LINE__);
	     vTaskDelete(NULL);
		 
	  }
	  if(ota_t++ >= 60*3)
	  {
        BLUFI_INFO("...Ota timout,restart the system!...\n");
		sleep(2);
		xota_set_status(UPDATE_FAIL);
		BLUFI_INFO("[%s:%d] ESP  ixe_ota_task delete!\n",__FILE__,__LINE__);
	    vTaskDelete(NULL);
		ota_t = 0;
	  }
	}
}
