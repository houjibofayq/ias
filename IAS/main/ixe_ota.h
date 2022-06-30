#ifndef  __MY_OTA_TEST__
#define  __MY_OTA_TEST__

enum ixe_product_type{
	NONE_TYPE = 0,
	PRO_ISE03,
    PRO_IPE01,
    END_TYPE
};

enum ixe_update_status{
	NONE_UPDATE = 0,
	UPDATE_START,
    DOWNLOAD_OVER,
    DOWNLOAD_FAIL,
    VERSION_SAME,
    UPDATE_FAIL,
    UPDATE_RALLBACK,
    VERSION_FAULT,
    END_UPDATE
};

void xota_set_status(uint8_t value);
uint8_t xota_get_status(void);

void ixe_ota_task(void *pvParameter);


#endif