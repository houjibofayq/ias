#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"


#include  "esp_ina219.h"

#define INA219_ADDRESS  (0x40 << 1)     // A0 = GND, A1 = GND


#define INA219_REG_CONFIG                       (uint8_t)(0x00)      
#define INA219_REG_SHUNTVOLTAGE                 (uint8_t)(0x01)      
#define INA219_REG_BUSVOLTAGE                   (uint8_t)(0x02)      
#define INA219_REG_POWER                        (uint8_t)(0x03)     
#define INA219_REG_CURRENT                      (uint8_t)(0x04)      
#define INA219_REG_CALIBRATION                  (uint8_t)(0x05)      
#define INA219_CONFIG_RESET                    (0x8000)  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK       (0x2000)  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V        (0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V        (0x2000)  // 0-32V Range

#define INA219_CONFIG_GAIN_MASK                (0x1800)  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV              (0x0000)  // Gain 1, 40mV Range		
#define INA219_CONFIG_GAIN_2_80MV              (0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV             (0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV             (0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK             (0x0780)  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT             (0x0080)  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT            (0x0100)  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT            (0x0200)  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT            (0x0400)  // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK             (0x0078)  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US     (0x0000)  // 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US   (0x0008)  // 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US   (0x0010)  // 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   (0x0018)  // 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US  (0x0048)	 // 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US  (0x0050)  // 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US  (0x0058)  // 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US (0x0060)  // 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS   (0x0068)  // 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS   (0x0070)  // 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS  (0x0078)  // 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK                (0x0007)  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN           (0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED     (0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED     (0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED (0x0003)
#define INA219_CONFIG_MODE_ADCOFF              (0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    (0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    (0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)	


#if 0
// 32V_2A

#define INA219_CONFIG_value	INA219_CONFIG_BVOLTAGERANGE_32V|INA219_CONFIG_GAIN_8_320MV|INA219_CONFIG_BADCRES_12BIT|INA219_CONFIG_SADCRES_12BIT_32S_17MS|INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS
#define INA_CAL 4096
								
#define IAN_I_LSB	    10.0							
#define INA_Power_LSB   2	
#endif

#if 0
// 32V_1A

#define INA219_CONFIG_value	INA219_CONFIG_BVOLTAGERANGE_32V|INA219_CONFIG_GAIN_8_320MV|INA219_CONFIG_BADCRES_12BIT|INA219_CONFIG_SADCRES_12BIT_1S_532US |INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS
#define INA_CAL 10240
								
#define IAN_I_LSB	    25.0							
#define INA_Power_LSB   0.8f	
#endif

#if 1
// 16V_400MA

#define INA219_CONFIG_value	INA219_CONFIG_BVOLTAGERANGE_16V|INA219_CONFIG_GAIN_1_40MV|INA219_CONFIG_BADCRES_12BIT|INA219_CONFIG_SADCRES_12BIT_1S_532US|INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS
#define INA_CAL 8192
								
#define IAN_I_LSB	    20.0							
#define INA_Power_LSB   1.0	
#endif

 static float mv = 0.0;    
 static float ma = 0.0;   
 static float mw = 0.0;  



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

void i2c_ina_task(void *arg)
{	
  int cnt = 0;	  
  printf("*******************\n");    
  printf("ESP32  MASTER READ SENSOR( INA219 )\n");    
  printf("*******************\n");

  //ESP_ERROR_CHECK(i2c_master_init());
  vTaskDelay(5000 / portTICK_RATE_MS);	
  ise_ina_init();	    
  while (1) {		
  	printf("----------- cnt = %d ------------\n",cnt);       
	mv = ise_ina_get_v();		
	ma = ise_ina_get_ma();		
	mw = ise_ina_get_mw();		
	printf(" mv = %.2f, ma = %.2f, mw = %.2f\n\n\n",mv,ma,mw);		    		
	vTaskDelay(500 / portTICK_RATE_MS);		
	if(cnt++ >= 5)		
	{		   
	   cnt = 1;		   
	   vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * 5) / portTICK_RATE_MS);		
	 }    
   }       
  vTaskDelete(NULL);

}



void ise_ina_reg_write(unsigned char reg,unsigned int data)	
{
    
	uint8_t cmd_data[3];
	cmd_data[0] = reg; // sends register address to read from
	cmd_data[1] = (unsigned char )(data>>8);; // write data high byte
	cmd_data[2] = (unsigned char )(data & 0xFF); // write data low byte;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, INA219_ADDRESS | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write(cmd, &cmd_data[0], 3, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
    
	vTaskDelay(1/portTICK_PERIOD_MS);

	return;
}

uint16_t ise_ina_reg_read(unsigned char reg)
{
    uint8_t data_temp[2];
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, INA219_ADDRESS, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN));
	
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, INA219_ADDRESS | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data_temp[0], ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data_temp[1], NACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	
	vTaskDelay(30/portTICK_PERIOD_MS);
	printf("get reg data temp[0] = %02x, tem[1] = %02x\n",data_temp[0],data_temp[1]);
	return 	(uint16_t)((data_temp[0]<<8)+data_temp[1]);
}


void ise_ina_init(void )	
{
	ise_ina_reg_write(INA219_REG_CONFIG,0x399f);
	ise_ina_reg_write(INA219_REG_CONFIG,INA219_CONFIG_value);
	ise_ina_reg_write(INA219_REG_CALIBRATION,INA_CAL);
}

float ise_ina_get_v(void)	
{
	uint16_t value;
	value = ise_ina_reg_read(INA219_REG_BUSVOLTAGE);
	value = (uint16_t)((value >> 3)*4);
	printf("get mv reg value = %d\n",value);
	return (float)(value/1000.0);	
}
float ise_ina_get_ma(void)		
{
	uint16_t value;
	ise_ina_reg_write(INA219_REG_CALIBRATION,INA_CAL);
	value = ise_ina_reg_read(INA219_REG_CURRENT);
	printf("get ma reg value = %d\n",value);
	if(value > 32767)
	   return (float)((value - 65535)/IAN_I_LSB);	
	else
	  return (float)(value/IAN_I_LSB);
}
float ise_ina_get_mw(void)		
{
	uint16_t value;
	ise_ina_reg_write(INA219_REG_CALIBRATION,INA_CAL);
	value = ise_ina_reg_read(INA219_REG_POWER);
	printf("get mw reg value = %d\n",value);
	if(value > 32767)
	   return (float)((value - 65535)*INA_Power_LSB);	
	else
	  return (float)(value/IAN_I_LSB);
}



