/**
 * @file MAX17043.c
 * @author Mario Aguilar (fernandoaguilar731010@gmail.com)
 * @brief 
 * @version v1
 * @date 2022-02-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "esp_max1704.h"

static const char *TAG = "ixe_ota";


#define MAX1704_ADDR       0x6C    /*!<Direccion del esclavo 8 bits        */
//#define MAX1704_ADDR       0x36    /*!<Direccion del esclavo 7 bits        */

///////////////////////////////////
// MAX17043 Register Definitions //
///////////////////////////////////
// All registers contain two bytes of data and span two addresses.
#define MAX17043_VCELL    0x02 // R - 12-bit A/D measurement of battery voltage
#define MAX17043_SOC      0x04 // R - 16-bit state of charge (SOC)
#define MAX17043_MODE     0x06 // W - Sends special commands to IC
#define MAX17043_VERSION  0x08 // R - Returns IC version
#define MAX17043_CONFIG   0x0C // R/W - Battery compensation (default 0x971C)
#define MAX17043_COMMAND  0xFE // W - Sends special comands to IC

///////////////////////////////////
// MAX17043 Config Register Bits //
///////////////////////////////////
#define MAX17043_CONFIG_SLEEP     7
#define MAX17043_CONFIG_ALERT     5
#define MAX17043_CONFIG_THRESHOLD 0

/////////////////////////////////////
// MAX17043 Mode Register Commands //
/////////////////////////////////////
#define MAX17043_MODE_QUICKSTART 0x4000

////////////////////////////////////////
// MAX17043 Command Register Commands //
////////////////////////////////////////
#define MAX17043_COMMAND_POR 0x5400

////////////////////////////////
// MAX17043 default config //
////////////////////////////////
#define MAX17043_CONFIG_DEFAULT 0x9700

float  vol_value = 0.0;
float  soc_per = 0.0;

void ias_get_vol_soc(float *vol,float *soc)
{
  *vol = vol_value;
  *soc = soc_per;

  ESP_LOGI(TAG,"get reg soc = %f\n",*soc);
  ESP_LOGI(TAG,"get reg vol = %f\n",*vol);
}

/** * @brief i2c master initialization */
static esp_err_t i2c_master_init(void)
{    
   i2c_config_t conf;    
   conf.mode = I2C_MODE_MASTER;    
   conf.sda_io_num = I2C_MASTER_SDA_IO;    
   conf.sda_pullup_en = GPIO_PULLUP_ENABLE;    
   conf.scl_io_num = I2C_MASTER_SCL_IO;    
   conf.scl_pullup_en = GPIO_PULLUP_ENABLE;    
   conf.master.clk_speed = I2C_MASTER_FREQ_HZ;    
   i2c_param_config(I2C_MASTER_NUM, &conf);    
   return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void tgls_max_reg_write16(unsigned char reg,unsigned int data)	
{
    
	uint8_t cmd_data[3];
	cmd_data[0] = reg; // sends register address to read from
	cmd_data[1] = (unsigned char )(data>>8);; // write data high byte
	cmd_data[2] = (unsigned char )(data & 0xFF); // write data low byte;
    //printf("write reg: %02x data data[0] = %02x, data[1] = %02x\n",cmd_data[0],cmd_data[1],cmd_data[2]);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MAX1704_ADDR | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write(cmd, &cmd_data[0], 3, NACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
    
	vTaskDelay(1/portTICK_PERIOD_MS);

	return;
}

void tgls_max_write16(unsigned char reg,unsigned int data)	
{
    
	uint8_t cmd_data[3];
	cmd_data[0] = reg; // sends register address to read from
	cmd_data[1] = (unsigned char )(data>>8);; // write data high byte
	cmd_data[2] = (unsigned char )(data & 0xFF); // write data low byte;
    //printf("write reg: %02x data data[0] = %02x, data[1] = %02x\n",cmd_data[0],cmd_data[1],cmd_data[2]);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX1704_ADDR | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, cmd_data[0], ACK_CHECK_EN);
	i2c_master_write_byte(cmd, cmd_data[1], ACK_CHECK_EN);
	i2c_master_write_byte(cmd, cmd_data[2], NACK_VAL);
	i2c_master_stop(cmd);
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
    
	vTaskDelay(1/portTICK_PERIOD_MS);

	return;
}


uint16_t tgls_max_reg_read16(unsigned char reg)
{
    uint8_t data_temp[2] = {0};
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MAX1704_ADDR, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN));
	printf("read reg: %02x \n",reg);
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MAX1704_ADDR | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data_temp[0], ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data_temp[1], NACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	
	vTaskDelay(30/portTICK_PERIOD_MS);
	//printf("get reg[%02x] data temp[0] = %02x, tem[1] = %02x\n",reg,data_temp[0],data_temp[1]);
	return 	(uint16_t)((data_temp[0]<<8)+data_temp[1]);
}

uint16_t tgls_max_read16(unsigned char reg)
{
    uint8_t data_temp[2] = {0};
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MAX1704_ADDR, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN));
	
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MAX1704_ADDR | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read(cmd, &data_temp[0],2, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	
	vTaskDelay(30/portTICK_PERIOD_MS);
	//printf("get reg[%02x] data temp[0] = %02x, tem[1] = %02x\n",reg,data_temp[0],data_temp[1]);
	return 	(uint16_t)((data_temp[0]<<8)+data_temp[1]);
}


uint16_t tgls_max1704_regs_read(unsigned char reg)
{
    uint8_t data_temp[12] = {0};
	int i = 0;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MAX1704_ADDR, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN));
	
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MAX1704_ADDR | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read(cmd, &data_temp[0],12, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	
	vTaskDelay(30/portTICK_PERIOD_MS);
	//printf("[%s:%d]get regs \n",__FILE__,__LINE__);
	for(i=0;i<6;i++)
	{
	  //printf("get reg data temp[%d] = %02x, temp[%d] = %02x\n",2*i,data_temp[2*i],2*i+1,data_temp[2*i+1]);
	}
	return 	(uint16_t)((data_temp[0]<<8)+data_temp[1]);
}


void tgls_max1704_quickstart(void)
{   
    
   tgls_max_write16(MAX17043_MODE,MAX17043_MODE_QUICKSTART);
    
	return ;
}

void tgls_max1704_set_default(void)
{   
    
   tgls_max_write16(MAX17043_CONFIG,MAX17043_CONFIG_DEFAULT);
    
	return ;
}

void tgls_max1704_reset(void)
{   
    
   tgls_max_write16(MAX17043_COMMAND,MAX17043_COMMAND_POR);
    
	return ;
}


void tgls_max1704_sleep(void)
{   
  uint16_t configReg = tgls_max_reg_read16(MAX17043_CONFIG);
  //printf("get reg config = %x\n",configReg);
  if (configReg & (1<<7))
    return; // Already sleeping, do nothing but return an error
  configReg |= (1<<7); // Set sleep bit
  tgls_max_write16(MAX17043_CONFIG,configReg);
  return;
}

void tgls_max1704_wake(void)
{   
    
    uint16_t configReg = tgls_max_reg_read16(MAX17043_CONFIG);
  if (configReg & (1<<7))
    return; // Already sleeping, do nothing but return an error
  configReg &= ~(1<<7); // Set sleep bit
  tgls_max_write16(MAX17043_CONFIG,configReg);
  return; 
}


float tgls_max1704_get_voltage(void)
{   
   uint16_t vCell;
   float voltage;
  vCell = tgls_max_read16(MAX17043_VCELL);
  // vCell is a 12-bit register where each bit represents 1.25mV
  vCell = (vCell) >> 4;
  voltage = (float) vCell / 800.0;
  //ESP_LOGI(TAG,"get reg voltage = %f\n",voltage);
  return voltage;
}

float tgls_max1704_get_soc(void)
{   
  uint16_t soc;
  float percent;
  soc = tgls_max_read16(MAX17043_SOC);
  percent = (soc & 0xFF00) >> 8;
  percent += (float) (((uint8_t) soc) / 256.0);
  //ESP_LOGI(TAG,"get reg soc = %f\n",percent);
  return percent;
}

uint16_t tgls_max1704GetVersion(void)
{
    uint16_t data=0;
   
    data = tgls_max_reg_read16(MAX17043_VERSION);
    //printf("get reg version = %04x\n",data);
	return data;
}

uint16_t tgls_max1704_get_config(void)
{
    uint16_t data=0;
   
    data = tgls_max_reg_read16(MAX17043_CONFIG);
    ESP_LOGI(TAG,"get reg config = %04x\n",data);
	return data;
}


void tgls_max1704_init(void)
{
  //printf("-----reset-----\n");
  //tgls_max1704_reset();
  vTaskDelay(1000 / portTICK_RATE_MS);	
  tgls_max1704GetVersion();
  tgls_max1704_get_config();
 // tgls_max1704_quickstart();
  //tgls_max1704_set_default();
}

void ias_max1704_task(void *arg)
{	
  int cnt = 0;	  
  printf("*******************\n");    
  printf("ESP32  MASTER READ SENSOR( MAX17043 )\n");    
  printf("*******************\n");

  ESP_ERROR_CHECK(i2c_master_init());
  vTaskDelay(2000 / portTICK_RATE_MS);	
  tgls_max1704_init();
  vTaskDelay(2000 / portTICK_RATE_MS);	
  //tgls_max1704_sleep();
  while (1) {			   		
	vTaskDelay(1000 / portTICK_RATE_MS);		
	if(cnt++ %5 == 0)		
	{		   
	   cnt = 1;
	   //tgls_max1704_wake();
	  // vTaskDelay(100 / portTICK_RATE_MS);	
	   soc_per = tgls_max1704_get_soc();
	   vTaskDelay(100 / portTICK_RATE_MS);	
	   vol_value = tgls_max1704_get_voltage();
	   vTaskDelay(100 / portTICK_RATE_MS);
	   //tgls_max1704_sleep();
	 }    
   }       
  vTaskDelete(NULL);

}

