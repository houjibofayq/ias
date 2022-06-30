#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"


#include  "esp_ltc2941.h"

#define LTC2941_ADDR 	0xC8
#define STATUS_REG		0x00
#define CONTROL_REG		0x01
#define ACC_CHARGE_MSB	0x02
#define ACC_CHARGE_LSB	0x03
#define CH_THR_H_MSB	0x04
#define CH_THR_H_LSB	0x05
#define CH_THR_L_MSB	0x06
#define CH_THR_L_LSB	0x07




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

void tgls_ltc_reg_write(unsigned char reg,unsigned int data)	
{
    
	uint8_t cmd_data[3];
	cmd_data[0] = reg; // sends register address to read from
	cmd_data[1] = (unsigned char )(data>>8);; // write data high byte
	cmd_data[2] = (unsigned char )(data & 0xFF); // write data low byte;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, LTC2941_ADDR | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write(cmd, &cmd_data[0], 3, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
    
	vTaskDelay(1/portTICK_PERIOD_MS);

	return;
}


uint16_t tgls_ltc_reg_read(unsigned char reg)
{
    uint8_t data_temp[2] = {0};
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, LTC2941_ADDR, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN));
	
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, LTC2941_ADDR | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data_temp[0], ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data_temp[1], NACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	
	vTaskDelay(30/portTICK_PERIOD_MS);
	printf("get reg data temp[0] = %02x, tem[1] = %02x\n",data_temp[0],data_temp[1]);
	return 	(uint16_t)((data_temp[0]<<8)+data_temp[1]);
}


uint16_t tgls_ltc_regs_read(unsigned char reg)
{
    uint8_t data_temp[8] = {0};
	int i = 0;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, LTC2941_ADDR, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN));
	
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, LTC2941_ADDR | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read(cmd, &data_temp[0],8, ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	
	vTaskDelay(30/portTICK_PERIOD_MS);
	printf("[%s:%d]get regs \n",__FILE__,__LINE__);
	for(i=0;i<8;i++)
	{
	  printf("get reg data temp[%d] = %02x, \n",i,data_temp[i]);
	}
	return 	(uint16_t)((data_temp[0]<<8)+data_temp[1]);
}

void tgls_ltc_write_Byte(unsigned char reg,uint8_t data)	
{
    
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, LTC2941_ADDR | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
    
	vTaskDelay(1/portTICK_PERIOD_MS);

	return;
}


void tgls_ltc_init(void )	
{
	tgls_ltc_regs_read(STATUS_REG);
	
	//0xffff   full charged
	//tgls_ltc_write_Byte(CONTROL_REG,0xfd);
	//tgls_ltc_reg_write(ACC_CHARGE_MSB,0xffff);
	//0x0000   no power
	//tgls_ltc_write_Byte(CONTROL_REG,0xfd);
	//tgls_ltc_write_Byte(ACC_CHARGE_MSB,0x0000);
	//3V(11),128(111),(10),(0)
	tgls_ltc_write_Byte(CONTROL_REG,0xfc);
	tgls_ltc_regs_read(STATUS_REG);		
}

void tgls_ltc_get_capacity(void )	
{
	uint16_t recv_data;
	float Qlsb = 0;
	float QBAT = 0;
	float Rense = 50;
	float M = 128;
	
	recv_data = tgls_ltc_reg_read(ACC_CHARGE_MSB);
	Qlsb = 0.085*(50/Rense)*(M/128);
	QBAT = Qlsb*recv_data;
	printf("-----------QBAT = %.3f ------------\n",QBAT);     
}

void tgls_ltc_get_percent(void )	
{
	uint16_t cur_bat;
	uint16_t full_bat;
    uint16_t rate;
	
	cur_bat = tgls_ltc_reg_read(ACC_CHARGE_MSB);
	full_bat = tgls_ltc_reg_read(CH_THR_H_MSB);
	rate = cur_bat*100/full_bat;
	printf("-----------rate = %d ------------\n",rate);     
}



void tgls_ltc_task(void *arg)
{	
  int cnt = 0;	  
  printf("*******************\n");    
  printf("ESP32  MASTER READ SENSOR( LTC2194 )\n");    
  printf("*******************\n");

  ESP_ERROR_CHECK(i2c_master_init());
  vTaskDelay(5000 / portTICK_RATE_MS);	
  tgls_ltc_init();	    
  while (1) {		
		
	tgls_ltc_get_percent();	   		
	vTaskDelay(10000 / portTICK_RATE_MS);		
	if(cnt++ >= 5)		
	{		   
	   cnt = 1;	
	   tgls_ltc_get_capacity( );	
	   vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * 5) / portTICK_RATE_MS);		
	 }    
   }       
  vTaskDelete(NULL);

}


