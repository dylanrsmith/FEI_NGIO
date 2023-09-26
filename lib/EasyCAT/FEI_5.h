#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

#define CUST_BYTE_NUM_OUT	4
#define CUST_BYTE_NUM_IN	14
#define TOT_BYTE_NUM_ROUND_OUT	4
#define TOT_BYTE_NUM_ROUND_IN	16


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		uint8_t     command;
		uint8_t     slot_type;
		uint8_t     slot_number;
		uint8_t     board_number;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		uint32_t    checksum_bad;
		uint32_t    checksum_good;
		uint16_t    Data1;
		uint16_t    Data2;
		uint16_t    Data3;
	}Cust;
} PROCBUFFER_IN;

#endif