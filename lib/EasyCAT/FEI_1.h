#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project FEI_4.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	8
#define CUST_BYTE_NUM_IN	12
#define TOT_BYTE_NUM_ROUND_OUT	8
#define TOT_BYTE_NUM_ROUND_IN	12


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		uint32_t    type;
		uint32_t    Output1;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		uint32_t    Slot1;
		uint32_t    Slot1_missed;
		uint32_t    Slot1_valid;
	}Cust;
} PROCBUFFER_IN;

#endif