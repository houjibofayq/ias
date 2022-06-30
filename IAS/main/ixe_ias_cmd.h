#ifndef _IAS_CMD_
#define _IAS_CMD_


enum ise_cmd_main{
	IAS_CMD_RECV = 0x0d,
	IAS_CMD_RESP = 0x0e,
	IAS_CMD_TYPE__END
};

enum ise_cmd_type{
	IAS_CMD_START = 0,
	IAS_QUERY_PAMS,
	IAS_SET_PAMS,
	IAS_QUERY_END
};


void ias_product_comd_handler(uint8_t *recv_data,uint32_t recv_len,uint8_t *send_buf,uint8_t *send_len);


#endif
