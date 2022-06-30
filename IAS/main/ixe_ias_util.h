#ifndef _IXE_ISE_UTIL
#define _IXE_ISE_UTIL


enum
{
	SYSTEM_STARTING = 0,
	GREEN_BLE_NOCON,
	GREEN_BLE_CONING,
	GREEN_SYS_UPDATING,
	GREEN_END
}LED_GREEN;

enum
{
	RED_WIFI_CONFAULT = 4,
	RED_WATER_LACK = 5,
	RED_PIPE_BLOCKED = 6,
	RED_POWER_LOW = 7,
	RED_SYSTEM_END
}LED_RED;

void ias_led_key_task(void* arg);
void ias_util_task(void* arg);


#endif
