/**
  ***************************************************************************************************
  * @file    module_modbus_drive_map.h 
  * @author  Regal Myron Mychal
  * @version V1.0
  * @date    15-Jun-2021
  * @brief   Macros for defining register map in drive micro
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MODBUS_DRIVE_MAP_H_
#define _MODULE_MODBUS_DRIVE_MAP_H_


/* Includes ------------------------------------------------------------------*/
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"
#include "module_modbus.h"

extern  Modbus_RTU_Control modbus_RTU_control;					// structure from modbus module

// Template for module modbus address map structure
#define	DRIVE_MAX_COILS		(16)		// maximum number of modbus coils allowed for a single module
#define	DRIVE_MAX_DISCRETES	(16)		// maximum number of modbus discrete registers allowed for a single module
#define DRIVE_MAX_INPUTS	(64)		// maximum number of modbus input registers allowed for a single module
#define	DRIVE_MAX_HOLDINGS	(160)		// maximum bumber of modbus holding registers allowed for a single module

uint16_t	void_drive_start_of_coils_pu16[1];
uint16_t	void_drive_start_of_discretes_pu16[1];
uint16_t	void_drive_start_of_inputs_pu16[1];
uint16_t	void_drive_start_of_holdings_pu16[1];

// number of each type of modbus data element in the module
#define MODULE_DRIVE_NUMBER_OF_COILS			0
#define MODULE_DRIVE_NUMBER_OF_DISCRETES		0
#define MODULE_DRIVE_NUMBER_OF_INPUTS			0
#define MODULE_DRIVE_NUMBER_OF_HOLDINGS		0

// address for the start of each element type within the module
#define MODULE_DRIVE_MODBUS_GROUP					(0)
#define MODULE_DRIVE_MODBUS_GROUP_BASE_ADDRESS	(MODULE_DRIVE_MODBUS_GROUP<<8)
#define	MODULE_DRIVE_START_OF_COILS_ADDRESS		(MODULE_DRIVE_MODBUS_GROUP_BASE_ADDRESS + 0)
#define MODULE_DRIVE_START_OF_DISCRETES_ADDRESS	(MODULE_DRIVE_MODBUS_GROUP_BASE_ADDRESS + DRIVE_MAX_COILS)
#define MODULE_DRIVE_START_OF_INPUTS_ADDRESS	(MODULE_DRIVE_MODBUS_GROUP_BASE_ADDRESS + DRIVE_MAX_DISCRETES)
#define MODULE_DRIVE_START_OF_HOLDINGS_ADDRESS	(MODULE_DRIVE_MODBUS_GROUP_BASE_ADDRESS + DRIVE_MAX_INPUTS)
#define	MODULE_DRIVE_END_OF_GROUP				(MODULE_DRIVE_START_OF_HOLDINGS_ADDRESS + DRIVE_MAX_HOLDINGS)

uint16_t*	module_drive_start_of_coils_pu16;			// first register in coil subgroup
uint16_t*	module_drive_start_of_discretes_pu16;		// first register in discrete subgroup
uint16_t*	module_drive_start_of_inputs_pu16;			// first register in inputs subgroup
uint16_t*	module_drive_start_of_holdings_pu16;		// first register in holdings subgroup

// structure for drive modbus registers

// Dynamic Data
// number of each type of modbus data element in the module
#define MODULE_DRIVE_DYNAMIC_DATA_NUMBER_OF_COILS			0
#define MODULE_DRIVE_DYNAMIC_DATA_NUMBER_OF_DISCRETES		0
#define MODULE_DRIVE_DYNAMIC_DATA_NUMBER_OF_INPUTS			(1) // 
#define MODULE_DRIVE_DYNAMIC_DATA_NUMBER_OF_HOLDINGS		(1) // 

// address for the start of each element type within the module
#define MODULE_DRIVE_DYNAMIC_DATA_GROUP							(0)			// drive dynamic data
#define MODULE_DRIVE_DYNAMIC_DATA_BASE_ADDRESS					(MODULE_DRIVE_DYNAMIC_GROUP<<8)
#define	MODULE_DRIVE_DYNAMIC_DATA_START_OF_COILS_ADDRESS		(MODULE_DRIVE_DYNAMIC_BASE_ADDRESS + 0)
#define MODULE_DRIVE_DYNAMIC_DATA_START_OF_DISCRETES_ADDRESS	(MODULE_DRIVE_DYNAMIC_BASE_ADDRESS + DRIVE_MAX_COILS)
#define MODULE_DRIVE_DYNAMIC_DATA_START_OF_INPUTS_ADDRESS		(MODULE_DRIVE_DYNAMIC_BASE_ADDRESS + DRIVE_MAX_DISCRETES)
#define MODULE_DRIVE_DYNAMIC_DATA_START_OF_HOLDINGS_ADDRESS		(MODULE_DRIVE_DYNAMIC_BASE_ADDRESS + DRIVE_MAX_INPUTS)
#define	MODULE_DRIVE_DYNAMIC_DATA_END_OF_GROUP					(MODULE_DRIVE_DYNAMIC_BASE_ADDRESS + DRIVE_MAX_HOLDINGS)

uint16_t*	module_drive_dynamic_data_start_of_coils_pu16			= (uint16_t*) &(void_drive_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_drive_dynamic_data_start_of_discretes_pu16		= (uint16_t*) &(void_drive_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_drive_dynamic_data_start_of_inputs_pu16			= (uint16_t*) &(void_drive_start_of_inputs_pu16[0]);		// first register in inputs subgroup
uint16_t*	module_drive_dynamic_data_start_of_holdings_pu16		= (uint16_t*) &(void_drive_start_of_holdings_pu16[0]);		// first register in holdings subgroup

// Drive Configuration
// number of each type of modbus data element in the module
#define MODULE_DRIVE_CONFIGURATION_NUMBER_OF_COILS			0
#define MODULE_DRIVE_CONFIGURATION_NUMBER_OF_DISCRETES		0
#define MODULE_DRIVE_CONFIGURATION_NUMBER_OF_INPUTS			(1) // 
#define MODULE_DRIVE_CONFIGURATION_NUMBER_OF_HOLDINGS		(1) // 

// address for the start of each element type within the module
#define MODULE_DRIVE_CONFIGURATION_GROUP							(0)			// drive dynamic data
#define MODULE_DRIVE_CONFIGURATION_BASE_ADDRESS					(MODULE_DRIVE_CONFIGURATION_GROUP<<8)
#define	MODULE_DRIVE_CONFIGURATION_START_OF_COILS_ADDRESS		(MODULE_DRIVE_CONFIGURATION_BASE_ADDRESS + 0)
#define MODULE_DRIVE_CONFIGURATION_START_OF_DISCRETES_ADDRESS	(MODULE_DRIVE_CONFIGURATION_BASE_ADDRESS + DRIVE_MAX_COILS)
#define MODULE_DRIVE_CONFIGURATION_START_OF_INPUTS_ADDRESS		(MODULE_DRIVE_CONFIGURATION_BASE_ADDRESS + DRIVE_MAX_DISCRETES)
#define MODULE_DRIVE_CONFIGURATION_START_OF_HOLDINGS_ADDRESS	(MODULE_DRIVE_CONFIGURATION_BASE_ADDRESS + DRIVE_MAX_INPUTS)
#define	MODULE_DRIVE_CONFIGURATION_END_OF_GROUP					(MODULE_DRIVE_CONFIGURATION_BASE_ADDRESS + DRIVE_MAX_HOLDINGS)

uint16_t*	module_drive_configuration_start_of_coils_pu16			= (uint16_t*) &(void_drive_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_drive_configuration_start_of_discretes_pu16		= (uint16_t*) &(void_drive_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_drive_configuration_start_of_inputs_pu16			= (uint16_t*) &(void_drive_start_of_inputs_pu16[0]);			// first register in inputs subgroup
uint16_t*	module_drive_configuration_start_of_holdings_pu16		= (uint16_t*) &(void_drive_start_of_holdings_pu16[0]);		// first register in holdings subgroup

// Drive Identification
// number of each type of modbus data element in the module
#define MODULE_DRIVE_ID_NUMBER_OF_COILS			0
#define MODULE_DRIVE_ID_NUMBER_OF_DISCRETES		0
#define MODULE_DRIVE_ID_NUMBER_OF_INPUTS		(1) // 
#define MODULE_DRIVE_ID_NUMBER_OF_HOLDINGS		(1) // 

// address for the start of each element type within the module
#define MODULE_DRIVE_ID_GROUP							(0)			// drive dynamic data
#define MODULE_DRIVE_ID_BASE_ADDRESS					(MODULE_DRIVE_ID_GROUP<<8)
#define	MODULE_DRIVE_ID_START_OF_COILS_ADDRESS			(MODULE_DRIVE_ID_BASE_ADDRESS + 0)
#define MODULE_DRIVE_ID_START_OF_DISCRETES_ADDRESS		(MODULE_DRIVE_ID_BASE_ADDRESS + DRIVE_MAX_COILS)
#define MODULE_DRIVE_ID_START_OF_INPUTS_ADDRESS			(MODULE_DRIVE_ID_BASE_ADDRESS + DRIVE_MAX_DISCRETES)
#define MODULE_DRIVE_ID_START_OF_HOLDINGS_ADDRESS		(MODULE_DRIVE_ID_BASE_ADDRESS + DRIVE_MAX_INPUTS)
#define	MODULE_DRIVE_ID_END_OF_GROUP					(MODULE_DRIVE_ID_BASE_ADDRESS + DRIVE_MAX_HOLDINGS)

uint16_t*	module_drive_id_start_of_coils_pu16			= (uint16_t*) &(void_drive_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_drive_id_start_of_discretes_pu16		= (uint16_t*) &(void_drive_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_drive_id_start_of_inputs_pu16		= (uint16_t*) &(void_drive_start_of_inputs_pu16[0]);		// first register in inputs subgroup
uint16_t*	module_drive_id_start_of_holdings_pu16		= (uint16_t*) &(void_drive_start_of_holdings_pu16[0]);		// first register in holdings subgroup

// Drive Test Control and Data
// number of each type of modbus data element in the module
#define MODULE_DRIVE_TEST_NUMBER_OF_COILS			0
#define MODULE_DRIVE_TEST_NUMBER_OF_DISCRETES		0
#define MODULE_DRIVE_TEST_NUMBER_OF_INPUTS			(1) // 
#define MODULE_DRIVE_TEST_NUMBER_OF_HOLDINGS		(1) // 

// address for the start of each element type within the module
#define MODULE_DRIVE_TEST_GROUP							(0)			// drive dynamic data
#define MODULE_DRIVE_TEST_BASE_ADDRESS					(MODULE_DRIVE_TEST_GROUP<<8)
#define	MODULE_DRIVE_TEST_START_OF_COILS_ADDRESS		(MODULE_DRIVE_TEST_BASE_ADDRESS + 0)
#define MODULE_DRIVE_TEST_START_OF_DISCRETES_ADDRESS	(MODULE_DRIVE_TEST_BASE_ADDRESS + DRIVE_MAX_COILS)
#define MODULE_DRIVE_TEST_START_OF_INPUTS_ADDRESS		(MODULE_DRIVE_TEST_BASE_ADDRESS + DRIVE_MAX_DISCRETES)
#define MODULE_DRIVE_TEST_START_OF_HOLDINGS_ADDRESS		(MODULE_DRIVE_TEST_BASE_ADDRESS + DRIVE_MAX_INPUTS)
#define	MODULE_DRIVE_TEST_END_OF_GROUP					(MODULE_DRIVE_TEST_BASE_ADDRESS + DRIVE_MAX_HOLDINGS)

uint16_t*	module_drive_test_start_of_coils_pu16		= (uint16_t*) &(void_drive_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_drive_test_start_of_discretes_pu16	= (uint16_t*) &(void_drive_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_drive_test_start_of_inputs_pu16		= (uint16_t*) &(void_drive_start_of_inputs_pu16[0]);		// first register in inputs subgroup
uint16_t*	module_drive_test_start_of_holdings_pu16	= (uint16_t*) &(void_drive_start_of_holdings_pu16[0]);		// first register in holdings subgroup

#endif /* _MODULE_MODBUS_DRIVE_H_ */

