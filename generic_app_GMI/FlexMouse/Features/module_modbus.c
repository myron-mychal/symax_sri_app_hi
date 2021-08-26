/***************************************************************************
****** Endura Platform (Crosstour) ********************************************
*
*  File:    Modbus_SL.c   Modbus processing routines.
*                   
*
*       !!!!!   REGAL BELOIT COMPANY PROPRIETARY !!!!!
*
*  Author:  B.Beifus, Vijay G
*  Toolchain: V5.20.2.31007
*  Target:	STMicro STM32F103/101
****************************************************************************
****** REVLOG **************************************************************
*   Created 12-1-14
*   
*
****** DO LIST *************************************************************
* Next:
*   
* Completed:
***************************************************************************
* General Notes:
*  
*    
*
***************************************************************************/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart1.h"
#include "module_modbus_application_map.h"
#include "module_modbus_drive_map.h"
#include "module_motor_demand_multiplexer.h"
#include "main.h"
#include "stm32g0xx_ll_iwdg.h" // TODO: Watchdog is for Bootloader Resets only. Move Bootloader Features to a better location


typedef struct ModbusDataRangeStruct
{
  uint16_t	minimum_value_u16;
  uint16_t	maximum_value_u16;
} ModbusDataRange;

typedef struct ModbusMapBlockStruct
{
  uint16_t *start_of_data_pu16;		// a pointer to where the sequential data used by the motor is stored
  uint16_t start_address_u16;	        // the modbus start address
  uint16_t number_of_registers_u16;	// number registers from the start address in this struct
  ModbusDataRange data_range_pu16[];	// store the minimum values for the data in this block (sequentially)
  //uint16_t maximum_values_pu16[];	// store the maximum values for the data in this block (sequentially)	
} ModbusMapBlock;

typedef struct
{
  uint8_t		crc_low_u8;
  uint8_t		crc_hi_u8;
  uint16_t	crc_u16;
} xCheckSum;

// Now each modbus block has to be defined to point to a valid data structure and exactly match its sizes
// MODBUS BLocks:
//
// Monitor
// Operation

uint8_t	index_u8 = 0;
int16_t  hi_amplitude_s16[8]={0}; 		
  uint8_t  hi_angle_multiplier_hi_u8[8]={0};
  uint16_t hi_angle_offset_u16[8]={0}; 		
  uint8_t  hi_is_phase_inverted_u8[8]={0}; 	
  uint16_t hi_min_speed_u16[8]={0}; 		
  uint16_t hi_max_speed_u16[8]={0}; 		
  uint8_t  hi_is_harmonic_injection_allowed_hi_u8 ;

uint64_t tt_ModbusLinkLostStop;
#define MODBUS_LOST_LINK_TIME 10000


#define NUM_MODBUS_REGISTERS_100s 17

enum ModbusRegisterIndexes {
  //
  MODBUS_100S_INDEX_STATUS,
  MODBUS_100S_INDEX_FAULTS,
  MODBUS_100S_INDEX_BUS_VOLTAGE,
  //
  MODBUS_100S_INDEX_DIRECTION,
  MODBUS_100S_INDEX_MEASURED_SPEED,
  MODBUS_100S_INDEX_TORQUE,
  //
  MODBUS_100S_INDEX_POWER_WATTS,
  MODBUS_100S_INDEX_IPM_TEMPERATURE,
  //
  MODBUS_100S_INDEX_UNDEFINED_08,
  MODBUS_100S_INDEX_UNDEFINED_09,
  MODBUS_100S_INDEX_UNDEFINED_10,
  MODBUS_100S_INDEX_UNDEFINED_11,
  MODBUS_100S_INDEX_UNDEFINED_12,
  MODBUS_100S_INDEX_UNDEFINED_13,
  MODBUS_100S_INDEX_UNDEFINED_14,
  MODBUS_100S_INDEX_UNDEFINED_15,
  MODBUS_100S_INDEX_UNDEFINED_16,
};

uint16_t modbusRegisters100s[NUM_MODBUS_REGISTERS_100s] = { // TODO: Register IDs should be ENUMs
  //
  0,
  0,
  0,
  //
  0,
  0,
  0,
  //
  0,
  0,
  // UNIMPLEMENTED BELOW THIS LINE
  80,			// Shaft Power				108
  1300,			// Rated Power				109
  85,			// IPM Temperature			110
  16384,		// DAC Channel 1			111
  4096,			// Iq Current				112
  4096,			// Id Current				113
  1,			// Ia RMS				    114
  1,			// Ib RMS				    115
  1			    // Ic RMS			        116
    //
    //   1234, 		// measured_speed_u16			    100
    //   5678,		// measured_demand_u16		        101
    //   120,		// Bus Voltage			            102
    //   0,		// PWM Input Duty Cycle		            103
    //   50,		// PWM Output Duty Cycle	        104
    //   100,		// PWM Frequency		            105
    //   120,		// Motor Control Micro Temperature	106
    //   100,		// Application Micr Temerature		107
    
};

//Block: Monitor data - all measured data in motor
//uint16_t	measured_speed_u16 		= 1234;  	// modbus map address 100
//uint16_t	measured_demand_u16		= 5678;		// 101

ModbusMapBlock BlockMonitor =
{
  //	(uint16_t *)&measured_speed_u16,	// first data value in block
  modbusRegisters100s,				// first data value in block
  100,						// starting address for block
  NUM_MODBUS_REGISTERS_100s,			// number of elements in block
  // Range (2):
  // TODO: Update for updated register Definitions
  // - TODO: Range limitations are not yet implemented
  // -- REVIEW: These are Measurements received from MCU, is filtering them actually necessary?
  0, 4000,		 // range measured speed  = 0 - 4000 rpm
/*  0, 10000,		 // range measured demand = 0.00 - 100.00 %
  0, 1000,		 // Bus Voltage				        102
  0, 10000,		 // PWM Input Duty Cycle			103
  0, 10000,		 // PWM Output Duty Cycle	        104
  1, 20000,		 // PWM Frequency			        105
  0, 150,		 // Motor Control Micro Temperature	106
  0, 150,		 // Application Micr Temerature		107
  0, 20000,	     // Shaft Power				        108
  0, 20000,		 // Rated Power				        109
  0, 150,		 // IPM Temperature			        110
  0, 65535,		 // DAC Channel 1			        111
  -32767, 32767, // Iq Current				        112
  -32767, 32767, // Id Current				        113
  0, 100,		 // Ia RMS				            114
  0, 100,		 // Ib RMS				            115
  0, 100		 // Ic RMS				            116	  */
};

#define NUM_MODBUS_REGISTERS_200s 19
uint16_t modbusRegisters200s[NUM_MODBUS_REGISTERS_200s] = {
  0, 		// commanded_speed_u16				    200
  0,		// commanded_demand_u16				    201
  0, 		// commanded_start_u16				    202
  0, 		// demand_source_u16				    203 // 0 is autodetect
  0, 		// Direction					        204
  0,		// User Torque Demand				    205
  1,		// Ramp Rate					        206
  1,		// Speed Regulation Kp coefficient		207
  10,		// Speed Regulation Kp exponent			208
  1,		// Speed Regulation Ki coefficient		209
  10,		// Speed Regulation Ki exponent			210
  1,		// Speed Regulation Kd coefficient		211
  10,		// Speed Regulation Kd exponent			212
  1,		// Torque Regulation Kp coefficient		213
  10,		// Torque Regulation Kp exponent		214
  1,		// Torque Regulation Ki coefficient		215
  10,		// Torque Regulation Ki exponent		216
  1,		// Torque Regulation Kd coefficient		217
  10,		// Torque Regulation Kd exponent		218
};

//Block: Operation data - all motor operation commands
ModbusMapBlock BlockOperation =
{
  //(uint16_t *)&commanded_speed_u16,	// first data value in block
  modbusRegisters200s,			// first data value in block
  200,
  NUM_MODBUS_REGISTERS_200s,
  // Range:
  0, 4000,	// range commanded speed  = 0 - 4000 rpm
  0, 10000,	// range commanded demand = 0.00 - 100.00 %
  0, 1,		// range of value of start command = 0, 1
  0, 0xFFFF,	// range of value of demand source = 0, 0xFFFF
  0, 4000,	// User Speed Demand				    204
  0, 20000,	// User Torque Demand				    205
  0, 10000,	// Ramp Rate					        206
  0, 65535,	// Speed Regulation Kp coefficient		207
  0, 32,	// Speed Regulation Kp exponent			208
  0, 65535,	// Speed Regulation Ki coefficient		209
  0, 32,	// Speed Regulation Ki exponent			210
  0, 65535,	// Speed Regulation Kd coefficient		211
  0, 32,	// Speed Regulation Kd exponent			212
  0, 65535,	// Torque Regulation Kp coefficient		213
  0, 32,	// Torque Regulation Kp exponent		214
  0, 65535,	// Torque Regulation Ki coefficient		215
  0, 32,	// Torque Regulation Ki exponent		216
  0, 65535,	// Torque Regulation Kd coefficient		217
  0, 32,	// Torque Regulation Kd exponent		218	  
};

#define NUM_MODBUS_REGISTERS_300s 4
uint16_t modbusRegisters300s[NUM_MODBUS_REGISTERS_300s] = {
  0,        // commanded_speed_u16		            200
  0,	    // commanded_demand_u16		            201
  0, 	    // commanded_start_u16		            202
  0, 	    // demand_source_u16		            203
};

//Block: Operation data - all motor operation commands
//static volatile uint16_t	commanded_speed_u16 	= 1800;	// modbus map address 200	
//static volatile uint16_t	commanded_demand_u16	= 4200;	// 201
//static volatile uint16_t	command_start_u16	    = 0;	// 202

ModbusMapBlock BlockID =
{
  //	(uint16_t *)&commanded_speed_u16,	// first data value in block
  modbusRegisters300s,				// first data value in block
  300,
  NUM_MODBUS_REGISTERS_300s,
  // Range (3):
  0, 4000,	// range commanded speed  = 0 - 4000 rpm
  0, 10000,	// range commanded demand = 0.00 - 100.00 %
  0, 1,		// range of value of start command = 0, 1
  0, 1		// range of value of start command = 0, 1
};

#define NUM_MODBUS_REGISTERS_1000s 3
uint16_t modbusRegisters1000s[NUM_MODBUS_REGISTERS_1000s] = {
  0xFF00,	// Start/Stop motor		1000
  0xFF00,	// Set Direction		1001
  0xFF00,	// Fault clear			1002
};

ModbusMapBlock BlockCoils =
{
  //	(uint16_t *)&measured_speed_u16,	// first data value in block
  modbusRegisters1000s,				// first data value in block
  1000,						// starting address for block
  NUM_MODBUS_REGISTERS_1000s,			// number of elements in block
  // Range
  0, 1,						
  0, 1,
  0, 1,	
};

#define NUM_MODBUS_REGISTERS_2000s 23
uint16_t modbusRegisters2000s[NUM_MODBUS_REGISTERS_2000s] = {
  0x0000,	// Overtemperature state	2000	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Ramp State				2001	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Start State				2002	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Run State				2003	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Cutback State			2004	IN(0xFF00)/OUT(0x0000)
  0x0000,	// Stall State				2005	TRUE(0xFF00)/FALSE(0x0000)	
  0x0000,	// Risk Addressed			2006	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Motor Brake				2007	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Voltage Limited			2008	IN(0xFF00)/OUT(0x0000)
  0x0000,	// Open Phase				2009	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Over Voltage				2010	IN(0xFF00)/OUT(0x0000)
  0x0000,	// Brownout State			2011	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Reserved				    2012
  0x0000,	// UL Fault				    2013	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Mode of Operation	    2014    SPEED(0xFF00)/TORQUE(0x0000)
  0x0000,	// Coherence Check			2015	IN(0xFF00)/OUT(0x0000)
  0x0000,	// Input Phase Loss			2016	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Fault Status in FOC		2017	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Phase A Overcurrent		2018	IN(0xFF00)/OUT(0x0000)
  0x0000,	// Phase B Overcurrent		2019	IN(0xFF00)/OUT(0x0000)
  0x0000,	// Phase C Overcurrent		2020	IN(0xFF00)/OUT(0x0000)
  0x0000,	// Loss of Analog Input		2021	TRUE(0xFF00)/FALSE(0x0000)
  0x0000,	// Failsafe Mode			2022	TRUE(0xFF00)/FALSE(0x0000)
};

ModbusMapBlock BlockDiscrete =
{
  //	(uint16_t *)&measued_speed_u16,	// first data value in block
  modbusRegisters2000s,				// first data value in block
  2000,						// starting address for block
  NUM_MODBUS_REGISTERS_2000s,			// number of elements in block
  // Range
  0, 1,						
/*  
  0, 1,
  0, 1,
  0, 1,
  0, 1,
  0, 1,	
  0, 1,						
  0, 1,
  0, 1,
  0, 1,
  0, 1,
  0, 1,	
  0, 1,						
  0, 1,
  0, 1,
  0, 1,
  0, 1,
  0, 1,	
  0, 1,						
  0, 1,
  0, 1,
  0, 1,
  0, 1	  
*/
};
#define NUM_MODBUS_REGISTERS_5000s 49 // 1
uint16_t modbusRegisters5000s[NUM_MODBUS_REGISTERS_5000s] = {
  0,		// HI ENable/Disable		5000	
  0,		// Amplitude 1				5001	
  0,		// Amplitude 2				5002	
  0,		// Amplitude 3				5003	
  0,		// Amplitude 4				5004	
  0,		// Amplitude 5				5005		
  0,		// Amplitude 6				5006	
  0,		// Amplitude 7				5007	
  0,		// Amplitude 8				5008	
  0,		// Angle Multiplier 1 		5009	
  0,		// Angle Multiplier 2 		5010	
  0,		// Angle Multiplier 3 		5011	
  0,		// Angle Multiplier 4 		5012
  0,		// Angle Multiplier 5 		5013	
  0,		// Angle Multiplier 6 		5014  
  0,		// Angle Multiplier 7 		5015	
  0,		// Angle Multiplier 8 		5016	
  0,		// Angle Offset 1 			5017
  0,		// Angle Offset 2 			5018
  0,		// Angle Offset 3 			5019
  0,		// Angle Offset 4 			5020
  0,		// Angle Offset 5 			5021
  0,		// Angle Offset 6 			5022
  0,		// Angle Offset 7 			5023
  0,		// Angle Offset 8 			5024
  0,		// Phase Inversion 1 		5025
  0,		// Phase Inversion 2 		5026
  0,		// Phase Inversion 3 		5027
  0,		// Phase Inversion 4 		5028
  0,		// Phase Inversion 5 		5029
  0,		// Phase Inversion 6 		5030
  0,		// Phase Inversion 7 		5031
  0,		// Phase Inversion 8 		5032
  0,		// Minimum Speed 1 			5033
  0,		// Minimum Speed 2 			5034
  0,		// Minimum Speed 3 			5035
  0,		// Minimum Speed 4 			5036
  0,		// Minimum Speed 5 			5037
  0,		// Minimum Speed 6 			5038
  0,		// Minimum Speed 7 			5039
  0,		// Minimum Speed 8 			5040
  2250,		// Maximum Speed 1 			5041
  2250,		// Maximum Speed 2 			5042
  2250,		// Maximum Speed 3 			5043
  2250,		// Maximum Speed 4 			5044
  2250,		// Maximum Speed 5 			5045
  2250,		// Maximum Speed 6 			5046
  2250,		// Maximum Speed 7 			5047
  2250		// Maximum Speed 8 			5048  
};
  
ModbusMapBlock BlockHarmonic =
{
  //	(uint16_t *)&measued_speed_u16,	// first data value in block
  modbusRegisters5000s,				// first data value in block
  5000,						// starting address for block
  NUM_MODBUS_REGISTERS_5000s,			// number of elements in block
  // Range
  0, 1,						// Harmonic Injection Enable		5000
  -32767, 32767,			// Harmonic Injection Amplitude 1```5001
  -32767, 32767,			// Amplitude 2	5002
  -32767, 32767,			// Amplitude 3	5003
  -32767, 32767,			// Amplitude 4	5004
  -32767, 32767,			// Amplitude 5	5005
  -32767, 32767,			// Amplitude 6	5006	
  -32767, 32767,			// Amplitude 7	5007
  -32767, 32767,			// Amplitude 8	5008
  0, 255,					// Angle Multiplier 1 5009
  0, 255,					// Angle Multiplier 2 5010
  0, 255,					// Angle Multiplier 3 5011
  0, 255,					// Angle Multiplier 4 5012
  0, 255,					// Angle Multiplier 5 5013
  0, 255,					// Angle Multiplier 6 5014
  0, 255,					// Angle Multiplier 7 5015
  0, 255,					// Angle Multiplier 8 5016
  0, 65535,					// Angle Offset 1 5017
  0, 65535,					// Angle Offset 2 5018
  0, 65535,					// Angle Offset 3 5019
  0, 65535,					// Angle Offset 4 5020
  0, 65535,					// Angle Offset 5 5021
  0, 65535,					// Angle Offset 6 5022
  0, 65535, 				// Angle Offset 7 5023
  0, 65535, 				// Angle Offset 8 5024
  0, 1,						// Phase Inversion 1 5025
  0, 1,						// Phase Inversion 2 5026
  0, 1,						// Phase Inversion 3 5027
  0, 1,						// Phase Inversion 4 5028
  0, 1,						// Phase Inversion 5 5029
  0, 1,						// Phase Inversion 6 5030
  0, 1,						// Phase Inversion 7 5031
  0, 1,						// Phase Inversion 8 5032
  0, 2250,					// Minimum Speed 1 5033
  0, 2250,					// Minimum Speed 2 5034
  0, 2250,					// Minimum Speed 3 5035
  0, 2250,					// Minimum Speed 4 5036
  0, 2250,					// Minimum Speed 5 5037
  0, 2250,					// Minimum Speed 6 5038
  0, 2250,					// Minimum Speed 7 5039
  0, 2250,					// Minimum Speed 8 5040
  0, 2250,					// Maximum Speed 1 5041
  0, 2250,					// Maximum Speed 2 5042
  0, 2250,					// Maximum Speed 3 5043
  0, 2250,					// Maximum Speed 4 5044
  0, 2250,					// Maximum Speed 5 5045
  0, 2250,					// Maximum Speed 6 5046
  0, 2250,					// Maximum Speed 7 5047
  0, 2250					// Maximum Speed 8 5048
};

ModbusMapBlock BlockAnalog0TO10V_Inputs =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_0TO10_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_INPUTS,			// number of elements in block
  // Range
  0, 1						
};

ModbusMapBlock BlockAnalog0TO10V_Holdings =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_0TO10_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_HOLDINGS,		// number of elements in block
  // Range
  0, 1					
};

ModbusMapBlock BlockDigitalInputs_Inputs =
{
  0,				// first data value in block
  MODULE_DIGITAL_INPUTS_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_DIGITAL_INPUTS_NUMBER_OF_INPUTS,			// number of elements in block
  // Range
  0, 1					
};

ModbusMapBlock BlockDigitalInputs_Holdings =
{
  0,				// first data value in block
  MODULE_DIGITAL_INPUTS_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_DIGITAL_INPUTS_NUMBER_OF_HOLDINGS,			// number of elements in block
  // Range
  0, 1						
};

ModbusMapBlock BlockPWMInputs_Inputs =
{
  0,				// first data value in block
  MODULE_PWM_INPUT_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_PWM_INPUT_NUMBER_OF_INPUTS,			// number of elements in block
  // Range
  0, 1						
};

ModbusMapBlock BlockPWMInputs_Holdings =
{
  0,				// first data value in block
  MODULE_PWM_INPUT_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_PWM_INPUT_NUMBER_OF_HOLDINGS,		// number of elements in block
  // Range
  0, 1					
};

ModbusMapBlock BlockModbusRTU_Inputs =
{
  0,				// first data value in block
  MODULE_MODBUS_RTU_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_MODBUS_RTU_NUMBER_OF_INPUTS,			// number of elements in block
  // Range
  0, 1						
};

ModbusMapBlock BlockModbusRTU_Holdings =
{
  0,				// first data value in block
  MODULE_MODBUS_RTU_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_MODBUS_RTU_NUMBER_OF_HOLDINGS,		// number of elements in block
  // Range
  0, 1					
};

ModbusMapBlock BlockAnalog4TO20_Inputs =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_4TO20_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_INPUTS,			// number of elements in block
  // Range
  0, 1						
};

ModbusMapBlock BlockAnalog4TO20_Holdings =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_4TO20_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_HOLDINGS,		// number of elements in block
  // Range
  0, 1					
};

ModbusMapBlock BlockDemandMultiplexer_Inputs =
{
  0,				// first data value in block
  MODULE_DEMAND_MULTIPLEXER_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_INPUTS,			// number of elements in block
  // Range
  0, 1						
};

ModbusMapBlock BlockDemandMultiplexer_Holdings =
{
  0,				// first data value in block
  MODULE_DEMAND_MULTIPLEXER_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_HOLDINGS,		// number of elements in block
  // Range
  0, 1					
};

ModbusMapBlock BlockFirmwareUpdate_Inputs =
{
  0,				// first data value in block
  MODULE_FIRMWARE_UPDATE_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_FIRMWARE_UPDATE_NUMBER_OF_INPUTS,			// number of elements in block
  // Range
  0, 1						
};

ModbusMapBlock BlockFirmwareUpdate_Holdings =
{
  0,				// first data value in block
  MODULE_FIRMWARE_UPDATE_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_FIRMWARE_UPDATE_NUMBER_OF_HOLDINGS,		// number of elements in block
  // Range
  0, 1					
};

ModbusMapBlock BlockFlashBlock_Inputs =
{
  0,				// first data value in block
  MODULE_FLASH_BLOCK_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_FLASH_BLOCK_NUMBER_OF_INPUTS,			// number of elements in block
  // Range
  0, 1						
};

ModbusMapBlock BlockFlashBlock_Holdings =
{
  0,				// first data value in block
  MODULE_MODBUS_RTU_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_MODBUS_RTU_NUMBER_OF_HOLDINGS,		// number of elements in block
  // Range
  0, 1					
};

ModbusMapBlock *masterBlocks[] =
{
  //&BlockTest,
  &BlockMonitor,				// moved to dynamic data
  &BlockOperation,				// moved to drive configuration
  &BlockID,						// moved to drive ID
  //&BlockInterface,
  //&BlockProtection
  //&BlockHistory,
  //&BLockCalibration,
  &BlockCoils,					// segregated into drive groups
  &BlockDiscrete,				// segregated into drive groups
  &BlockHarmonic,				
  // &BlockAnalog0TO10V_Inputs,
  // &BlockAnalog0TO10V_Holdings,  
  // &BlockDigitalInputs_Inputs,
  // &BlockDigitalInputs_Holdings,
  // &BlockPWMInputs_Inputs,
  // &BlockPWMInputs_Holdings,
  // &BlockModbusRTU_Inputs,
  // &BlockModbusRTU_Holdings,
  // &BlockAnalog0TO10V_Inputs,
  // &BlockAnalog0TO10V_Holdings,
  &BlockDemandMultiplexer_Inputs,
  &BlockDemandMultiplexer_Holdings,
  //&BlockFirmwareUpdate_Inputs,
  &BlockFirmwareUpdate_Holdings,
  //&BlockFlashBlock_Inputs,
  //&BlockFlashBlock_Holdings,
};

uint16_t number_of_modbus_blocks_u16 = sizeof(masterBlocks)/sizeof(masterBlocks[0]);

enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Ddditional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// Global variables
extern ProcessInfo processInfoTable[];
Usart1_Control* usart1Control_Modbus;
Modbus_Control modbus_Control;
static Ram_Buf_Handle module_Control_StructMem_u32;

#define ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN 1
#if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN >= 1
// This is a one-shot buffer, that is written to and read from in single calls.
// - it does not currently need to be tracked for current index because of this.
#define FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH USART1_SINGLE_MESSAGE_RX_BUF_SIZE // Inclusive (this value is accepted) 
uint8_t fixedModbus_ProtocolBufRX_Length = 0;
uint8_t fixedModbus_ProtocolBufRX[FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH];
uint8_t* modbus_ProtocolBufRX = fixedModbus_ProtocolBufRX;
#else // if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
uint8_t* modbus_ProtocolBufRX;
#endif // if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0


#define ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN 1
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN >= 1
// This is a one-shot buffer, that is written to and read from in single calls.
// - it does not currently need to be tracked for current index because of this.
#define FIXED_MODBUS_PROTOCOLBUF_TX_MAX_LENGTH USART1_SINGLE_MESSAGE_TX_BUF_SIZE // Inclusive (this value is accepted) 
uint8_t fixedModbus_ProtocolBufTX_Length = 0;
uint8_t fixedModbus_ProtocolBufTX[FIXED_MODBUS_PROTOCOLBUF_TX_MAX_LENGTH];
uint8_t* modbus_ProtocolBufTX = fixedModbus_ProtocolBufTX;
#else // if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
uint8_t* modbus_ProtocolBufTX;
#endif // if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0

uint16_t mb_crcCalc = 0xFFFF;
Modbus_RTU_Control modbus_RTU_control;

// Local Function Prototypes
void Modbus_ParseReceivedMessages(void);
ModbusMapBlock *find_modbus_block(uint16_t desired_address_u16, uint16_t number_of_registers_u16);
MBErrorCode ProcessMBHoldingRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16, MBRegisterMode eMode);
MBErrorCode ProcessMBCoilRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16, MBRegisterMode eMode);
MBErrorCode ProcessMBDiscreteRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16, MBRegisterMode eMode);
void FixChecksum(xCheckSum *xCHK);
void UpdateChecksum(uint8_t element, xCheckSum *xCHK);
uint8_t isChecksumValid(uint8_t *full_frame_pu8, uint8_t packet_length_u8, xCheckSum *xCHK);
void UpdateMotorDemandMultiplexer(void);
//uint16_t calculateMODBUSCRC(uint8_t *modbusFrame_pu8, uint16_t length_u16);
void AssignModuleMemModbus(void);
void Modbus_JumpToBootloader(void);

//uint16_t MB_Valid_crc(uint8_t len); //@ "program"
//void MB_Update_crc(uint8_t thisByte); //@ "program"
uint16_t calculateModbusCRC(uint8_t *buf_pu8, uint16_t length_u16);

void UpdateMotorDemandMultiplexer(void) {

  uint16_t commanded_speed = modbusRegisters200s[0];
  uint16_t commanded_demand = modbusRegisters200s[1];
  uint16_t commanded_start = modbusRegisters200s[2];
  uint16_t demand_source = modbusRegisters200s[3];
  uint16_t direction = modbusRegisters200s[4];
  MotorDemandMux_ModbusUpdate(commanded_speed, commanded_demand, commanded_start, demand_source, direction);
}

void UpdateHarmonicInjectionParameters(void) {
  	hi_is_harmonic_injection_allowed_hi_u8 = modbusRegisters5000s[0]; 
  for(index_u8 = 0; index_u8 < 8; index_u8++) {
  	hi_amplitude_s16[index_u8] 			= (int16_t)  modbusRegisters5000s[1+index_u8]; 
  	hi_angle_multiplier_hi_u8[index_u8] 	= (uint8_t)  modbusRegisters5000s[9+index_u8];
  	hi_angle_offset_u16[index_u8] 			= (uint16_t) modbusRegisters5000s[17+index_u8];
  	hi_is_phase_inverted_u8[index_u8] 		= (uint8_t)  modbusRegisters5000s[25+index_u8];
  	hi_min_speed_u16[index_u8] 			= (uint16_t) modbusRegisters5000s[33+index_u8];
  	hi_max_speed_u16[index_u8] 			= (uint16_t) modbusRegisters5000s[41+index_u8];  
  }
//  HarmonicInjection_ModbusUpdate(hi_is_harmonic_injection_allowed_hi_u8, hi_amplitude_s16, hi_angle_multiplier_hi_u8, hi_angle_offset_u16, hi_is_phase_inverted_u8, hi_min_speed_u16, hi_max_speed_u16);
}

// module functions
uint8_t moduleModbus(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) {
  case MEMORY_INIT_MODULE:
    {
      //AssignModuleMemModbus(); // Assign structured memory to Analog 0-10V setting and data 
      return_state_u8 = INIT_MODULE;
	  // map modbus registers to corresponding modules
	  // BlockAnalog0TO10V_Inputs.start_of_data_pu16 		= module_analog_inputs_0TO10_start_of_inputs_pu16;
	  // BlockAnalog0TO10V_Holdings.start_of_data_pu16 	= module_analog_inputs_0TO10_start_of_holdings_pu16;
	  // BlockDigitalInputs_Inputs.start_of_data_pu16 		= module_digital_inputs_start_of_inputs_pu16;
	  // BlockDigitalInputs_Holdings.start_of_data_pu16 	= module_digital_inputs_start_of_holdings_pu16;
	  // BlockPWMInputs_Inputs.start_of_data_pu16 			= module_pwm_input_start_of_inputs_pu16;
	  // BlockPWMInputs_Holdings.start_of_data_pu16 		= module_pwm_input_start_of_holdings_pu16;
	  // BlockModbusRTU_Inputs.start_of_data_pu16 			= module_modbus_rtu_start_of_inputs_pu16;
	  // BlockModbusRTU_Holdings.start_of_data_pu16 		= module_modbus_rtu_start_of_holdings_pu16;
	  // BlockAnalog4TO20_Inputs.start_of_data_pu16 		= module_analog_inputs_4TO20_start_of_inputs_pu16;
	  // BlockAnalog4TO20_Holdings.start_of_data_pu16 		= module_analog_inputs_4TO20_start_of_holdings_pu16;	
	  BlockDemandMultiplexer_Inputs.start_of_data_pu16 		= module_demand_multiplexer_start_of_inputs_pu16;
	  BlockDemandMultiplexer_Holdings.start_of_data_pu16 	= module_demand_multiplexer_start_of_holdings_pu16;	  	  
	  //BlockFirmwareUpdate_Inputs.start_of_data_pu16 	= module_firmware_update_start_of_inputs_pu16;
	  BlockFirmwareUpdate_Holdings.start_of_data_pu16 	= module_firmware_update_start_of_holdings_pu16;	  
	  //BlockFlashBlock_Inputs.start_of_data_pu16 		= module_flash_block_start_of_inputs_pu16;
	  //BlockFlashBlock_Holdings.start_of_data_pu16 		= module_flash_block_start_of_holdings_pu16;	  	  
	  break;
    }
  case INIT_MODULE: 
    {
      // initialize test registers for modbus
      //initMODBUSRegisterMap();
      /* Attach Uart1 shared memory into this Module */

      tt_ModbusLinkLostStop = 0xFFFFFFFFFFFFFFFF; // Restart Modbus Link Lost Timer
      
      uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1);              //return Process index from processInfo array with the Uart2 driver
      usart1Control_Modbus = (Usart1_Control*) ((*(processInfoTable[Usart1index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      return_state_u8 = RUN_MODULE ;
      break;
    }
  case RUN_MODULE: 
    {
      //unsigned char received_slave_address_u8 = (*usart1Control_Modbus).seqMem_RawRx;
      //unsigned char received_function_code_u8 = (*usart1Control_Modbus).seqMem_RawRx;
      // test CRC
      // test length
      
      //Ring_Buf_Handle seqMem_ModbusRx;
      //Ring_Buf_Handle seqMemTX;
      //Ring_Buf_Handle seqMem_RawRx;
      //int16_t motorSpeed_s16;
      //uint16_t motorStatus_u16;
      //uint8_t errorCode_u8;
      //} Usart1_Control;		

      if(getSysCount() >= tt_ModbusLinkLostStop) // Modbus Link Lost
      {            
        MotorDemandMux_ModbusUpdate(0, 0, 1, 5, 0);
      }  
      
      Modbus_ParseReceivedMessages();
      return_state_u8 = RUN_MODULE ;
      break;
    }
  case KILL_MODULE: 
    {
      // The USART1 driver module must only be executed once.
      // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
      uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
        processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
      }
      return_state_u8 = KILL_MODULE;
      break;
    }
  default: 
    {
      return_state_u8 = KILL_MODULE;
      break;
    }
  }
  return return_state_u8;
}


/**
********************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory for digital outputs module
* @param   None 
* @return  None
********************************************************************************
*/
void AssignModuleMemModbus(void){   
  module_Control_StructMem_u32 =  StructMem_CreateInstance(MODULE_MODBUS, sizeof(Modbus_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*module_Control_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&modbus_Control ;    // Map the ADC1 memory into the structured memory
  uint8_t module_modbus_index_u8 = getProcessInfoIndex(MODULE_MODBUS);
  processInfoTable[module_modbus_index_u8].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)module_Control_StructMem_u32;
}

uint8_t modbus_message_counter_u8 = 0; // !errorTemporary

// Checks Message Length: Returns 0, if length accepted, Returns 1 if buffer overflow
uint8_t Modbus_ReallocateTxBufferForLength(uint8_t message_length) {
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN >= 1
  if (message_length > FIXED_MODBUS_PROTOCOLBUF_TX_MAX_LENGTH) {
    return 1; // Message would overflow buffer
  }
#else // if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
  if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, message_length)) == NULL) {
    reallocError++;
    return 1;
  }
#endif // if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
  return 0; // Message will fit in allocated buffer
}

uint8_t Modbus_ReallocateRxBufferForLength(uint8_t message_length) {
#if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN >= 1
  if (message_length > FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH) {
    return 1; // Message would overflow buffer
  }
#else // if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
  if((modbus_ProtocolBufRX = (uint8_t *) realloc(modbus_ProtocolBufRX, message_length)) == NULL) {
    reallocError++;
    return 1;
  }
#endif // if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
  return 0; // Message will fit in allocated buffer
}


void Modbus_ParseReceivedMessages(void) {
  // TODO: Accept Messages of Various Lengths, will require updates to driver_usart1 as well.
  // TODO: Length Checking for individual Messages
  uint32_t DataLen2_u32		= MODBUS_MIN_MESSAGE_LEN;
  uint32_t responseLength_u32 = (uint32_t) MINIMUM_RESPONSE_LENGTH;
  uint8_t number_of_coil_bytes_u8 = 0;
  
  DataLen2_u32 = RingBuf_GetUsedNumOfElements((*usart1Control_Modbus).seqMem_ModbusRx);
  if(DataLen2_u32 >= MODBUS_MIN_MESSAGE_LEN) {
    // if((modbus_ProtocolBufRX = (unsigned char*) realloc(modbus_ProtocolBufRX,DataLen2_u32)) == NULL) reallocError++;     //allocate the right frame size of memory for buffer
    uint8_t error_occurred = Modbus_ReallocateRxBufferForLength(DataLen2_u32);
    if (error_occurred) {
      // Read All Data (Clear the Buffer),  so we don't get stuck with the Buffer full
      while (DataLen2_u32 > 0) {
        if (DataLen2_u32 > FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH) {
          // REVIEW: Replace with RingBuf_ClearContents? Much less processing
          unsigned int read_length = FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH;
          RingBuf_ReadBlock((*usart1Control_Modbus).seqMem_ModbusRx, modbus_ProtocolBufRX, &read_length); //extract the whole frame
          DataLen2_u32 -= FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH;
        } else {
          unsigned int read_length = DataLen2_u32;
          RingBuf_ReadBlock((*usart1Control_Modbus).seqMem_ModbusRx, modbus_ProtocolBufRX, &read_length); //extract the whole frame
          DataLen2_u32 = 0;
        }
      }
      // Exit Gracefully
      return;
    }
    
    RingBuf_ReadBlock((*usart1Control_Modbus).seqMem_ModbusRx, modbus_ProtocolBufRX, &DataLen2_u32); //extract the whole frame into he temporary modbus_ProtocolBuf
    //decode and perform the CMD function
    uint8_t slave_address_u8 = modbus_ProtocolBufRX[BYTE_0];
    uint8_t function_code_u8 = modbus_ProtocolBufRX[BYTE_1]; // does not include address or checksum, does include this length byte
    uint8_t crc_lo_u8 = modbus_ProtocolBufRX[DataLen2_u32 - 2];
    uint8_t crc_hi_u8 = modbus_ProtocolBufRX[DataLen2_u32 - 1];
    uint16_t register_address_u16 		= modbus_ProtocolBufRX[BYTE_2]*BYTE_TO_WORD_MSB + modbus_ProtocolBufRX[BYTE_3]*BYTE_TO_WORD_LSB;
    uint16_t number_of_registers_u16	= modbus_ProtocolBufRX[BYTE_4]*BYTE_TO_WORD_MSB + modbus_ProtocolBufRX[BYTE_5]*BYTE_TO_WORD_LSB;		
    uint16_t crc_received_u16 = (uint16_t) ((crc_hi_u8*256) | (crc_lo_u8));
    
    //All characters in msg included in checksum
    //for (index_u8 = 0; index_u8 < (DataLen2_u32 - 2); index_u8++) {
    //	UpdateChecksum(modbus_ProtocolBufRX[index_u8], &localCHK);
    //}
    //FixChecksum(&localCHK);
    //uint8_t myBoolean;
    //myBoolean = isChecksumValid(modbus_ProtocolBufRX, DataLen2_u32 - 2, &localCHK);
    
    uint16_t crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufRX, (DataLen2_u32 - 2));
    //uint16_t alt_calculated_u16 = calculateMODBUSCRC(modbus_ProtocolBufRX, (DataLen2_u32 - 2));
    //uint16_t crc_calculated_u16 = Modbus_CalculateCrc((DataLen2_u32 - 2) , modbus_ProtocolBufRX);
    
    // MRM: Validate CRC
    if(crc_received_u16 != crc_calculated_u16) {
      // send exception for bad CRC
      responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
      uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
      if (error_occurred) {
        return;
      }
      // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
      
      // build header
      modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
      modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
      modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_04;
      crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufRX, (DataLen2_u32 - 2));			
      modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
      modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
      
      // For debugging MODBUS:
      //modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (DataLen2_u32 >> 8);									
      //modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (DataLen2_u32 >> 0);									
      //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (((*usart1Control_Modbus).seqMem_ModbusRx)->head_s32);	
      //modbus_ProtocolBufTX[BYTE_5] = (uint8_t) (((*usart1Control_Modbus).seqMem_ModbusRx)->tail_s32);	
      //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (((*usart1Control_Modbus).seqMem_ModbusRx)->usedNumOfElements_s32);
      //modbus_ProtocolBufTX[BYTE_5] = (uint8_t) (((*usart1Control_Modbus).seqMem_ModbusRx)->totalNumOfElements_u32);	
      //crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufRX, (DataLen2_u32 - 2));
      //modbus_ProtocolBufTX[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
      //modbus_ProtocolBufTX[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
      
      // write to seq Mem structure
      RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf								  
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
      if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX,1)) == NULL) reallocError++;	
#endif //ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
      
      return;
    }
    
    // Validate modbus slave address
    if (slave_address_u8 != MODBUS_ADDRESS) {
      // MRM: return exception code for invalid slave address
      responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
      uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
      if (error_occurred) {
        return;
      }
      // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
      
      // build exception header
      modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
      modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
      modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_04;
      crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufRX, (DataLen2_u32 - 2));			
      modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
      modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
      
      // write to seq Mem structure
      RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf								  
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
      if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX,1)) == NULL) reallocError++;	
#endif //ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0		  		  
      return;
    }
    
    // MRM: Validate message length length
    //if (DataLen2_u32 < MODBUS_MIN_MESSAGE_LEN) {
    //  	// MRM: return exception code for invalid message lengths
    //    return;
    //}
    tt_ModbusLinkLostStop = getSysCount() + MODBUS_LOST_LINK_TIME; // Restart Modbus Link Lost Timer
    switch(function_code_u8) 
    {
    case MODBUS_FUNCTION_CODE_UTILITY: {
        if (DataLen2_u32 < MODUBS_FUNCTION_CODE_UTILITY_MIN_MESSAGE_LEN) {
          return;
        } 
        //        
        Utility_ExecuteOperation(modbus_ProtocolBufRX[BYTE_2], modbus_ProtocolBufRX[BYTE_3], modbus_ProtocolBufRX[BYTE_4]);
        // Modbus_JumpToBootloader();
    }
      //case MODBUS_FUNCTION_CODE_READ_COILS:												
    case MODBUS_FUNCTION_CODE_READ_COILS: {
      if (ProcessMBCoilRegister(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for bad address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf									
      }
      else {
        // build response message and send
        if (number_of_registers_u16 % 8 == 0)
          number_of_coil_bytes_u8 = number_of_registers_u16/8;
        else
          number_of_coil_bytes_u8 = number_of_registers_u16/8 + 1;
        responseLength_u32 = READ_RESPONSE_HEADER_LENGTH + number_of_coil_bytes_u8 + CRC_LENGTH;	// exactly one partial byte for coils			  
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) number_of_coil_bytes_u8;								// 1 byte per multiple of 8 coils
        //modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 8);								// starting register
        //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (register_address_u16 >> 0);					
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
        uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
      }  				  
      break;
    }
    //case MODBUS_FUNCTION_CODE_READ_DISCRETE_INPUTS:
    case MODBUS_FUNCTION_CODE_READ_DISCRETE_INPUTS: {
      if (ProcessMBCoilRegister(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for bad address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        if (number_of_registers_u16 % 8 == 0)
          number_of_coil_bytes_u8 = number_of_registers_u16/8;
        else
          number_of_coil_bytes_u8 = number_of_registers_u16/8 + 1;
        // build response message and send
        responseLength_u32 = READ_RESPONSE_HEADER_LENGTH + number_of_coil_bytes_u8 + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (number_of_coil_bytes_u8);								// 2 bytes per registers
        //modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 8);								// starting register
        //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (register_address_u16 >> 0);					
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
        uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
      }  
      break;
    }			  
    //case MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS: {
    case MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS: {
      if (ProcessMBHoldingRegister(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for bad address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        // build response message and send
        responseLength_u32 = READ_RESPONSE_HEADER_LENGTH + 2*number_of_registers_u16 + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (2*number_of_registers_u16);								// 2 bytes per registers
        //modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 8);								// starting register
        //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (register_address_u16 >> 0);					
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
        uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
      }  
      break;
    }			  
    //case MODBUS_FUNCTION_CODE_READ_INPUT_REGISTERS:
    case MODBUS_FUNCTION_CODE_READ_INPUT_REGISTERS: {			
      if (ProcessMBHoldingRegister(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for bad address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        // build response message and send
        responseLength_u32 = READ_RESPONSE_HEADER_LENGTH + 2*number_of_registers_u16 + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (2*number_of_registers_u16);								// 2 bytes per registers
        //modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 8);								// starting register
        //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (register_address_u16 >> 0);					
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
        uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
      }
      break;
    }
    //case MODBUS_FUNCTION_CODE_WRITE_SINGLE_HOLDING_REGISTER:
    case MODBUS_FUNCTION_CODE_WRITE_SINGLE_HOLDING_REGISTER: {
      if (ProcessMBHoldingRegister(modbus_ProtocolBufRX+WRITE_SINGLE_HEADER_LENGTH, register_address_u16, 1, MB_WRITE_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for ilegal address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        responseLength_u32 = WRITE_RESPONSE_HEADER_LENGTH + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (register_address_u16 >> 8);				// starting register MSB/LSB
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 0);					
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (modbus_ProtocolBufRX[WRITE_SINGLE_HEADER_LENGTH]);			// number of registers written MSB/LSB
        modbus_ProtocolBufTX[BYTE_5] = (uint8_t) (modbus_ProtocolBufRX[WRITE_SINGLE_HEADER_LENGTH+1]);				
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
        
        UpdateMotorDemandMultiplexer(); // TODO: Don't call for every write (only when updates detected)
        UpdateHarmonicInjectionParameters(); // TODO: Temporary solution, do an if/else clause to select only one update based on modbus register.
      }
      break;
    }			  
    //case MODBUS_FUNCTION_CODE_WRITE_MULTIPLE_COILS:
    case MODBUS_FUNCTION_CODE_WRITE_MULTIPLE_COILS:{
      if (number_of_registers_u16 % 8 == 0)
        number_of_coil_bytes_u8 = number_of_registers_u16/8;
      else
        number_of_coil_bytes_u8 = number_of_registers_u16/8 + 1;			  
      if (ProcessMBCoilRegister(modbus_ProtocolBufRX+WRITE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_WRITE_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for ilegal address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        responseLength_u32 = WRITE_RESPONSE_HEADER_LENGTH + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build heade
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (register_address_u16 >> 8);				// starting register MSB/LSB
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 0);					
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (number_of_registers_u16 >> 8);			// number of registers written MSB/LSB
        modbus_ProtocolBufTX[BYTE_5] = (uint8_t) (number_of_registers_u16 >> 0);				
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
        
        UpdateMotorDemandMultiplexer(); // TODO: Don't call for every write (only when updates detected)
        UpdateHarmonicInjectionParameters(); // TODO: Temporary solution, do an if/else clause to select only one update based on modbus register.
      }				
      break;
    }
    //case MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS: {
    case MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS: {
      if (ProcessMBHoldingRegister(modbus_ProtocolBufRX+WRITE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_WRITE_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for ilegal address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        responseLength_u32 = WRITE_RESPONSE_HEADER_LENGTH + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (register_address_u16 >> 8);				// starting register MSB/LSB
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 0);					
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (number_of_registers_u16 >> 8);			// number of registers written MSB/LSB
        modbus_ProtocolBufTX[BYTE_5] = (uint8_t) (number_of_registers_u16 >> 0);				
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
        
        UpdateMotorDemandMultiplexer(); // TODO: Don't call for every write (only when updates detected)
        UpdateHarmonicInjectionParameters(); // TODO: Temporary solution, do an if/else clause to select only one update based on modbus register.
      }
      break;
    }			  
    //case MODBUS_FUNCTION_CODE_READ_RECORDS:
    //case MODBUS_FUNCTION_CODE_WRITE_RECORDS: 
    default: 
      {
        // function code is not supported, so send an exception
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_01;									// illegal function code
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf								  
        break;
      }
    }
  }
  // Minimize Size of Dynamically Allocated Buffers
#if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
  if((modbus_ProtocolBufRX = (uint8_t *) realloc(modbus_ProtocolBufRX,1)) == NULL) reallocError++;
#endif // ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
  if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX,1)) == NULL) reallocError++;	
#endif //ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
}

MBErrorCode ProcessMBHoldingRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16, MBRegisterMode eMode) {
  MBErrorCode    eStatus = MB_ILLEGAL_ADDRESS;
  ModbusMapBlock *block = find_modbus_block(starting_address_u16, number_of_registers_u16);
  
  if (block) {
    //block_1 =(uint16_t) ( block->start_address_u16);
    //offset_1 = starting_address_u16;
    uint16_t index_u16 = starting_address_u16 - block->start_address_u16;
    
    while (number_of_registers_u16 > 0 ) {
      if (eMode == MB_WRITE_REGISTER) {
        block->start_of_data_pu16[index_u16] = ((uint16_t)*data_buffer_pu8++) << 8;
        block->start_of_data_pu16[index_u16] |= (uint16_t)*data_buffer_pu8++;
      }
      else {
        *data_buffer_pu8++ = (uint8_t)(block->start_of_data_pu16[index_u16] >> 8);
        *data_buffer_pu8++ = (uint8_t)(block->start_of_data_pu16[index_u16] & 0xff);
      }
      ++index_u16;
      --number_of_registers_u16;
    }
    eStatus = MB_NO_ERROR;
  }
  return eStatus;
}

MBErrorCode ProcessMBCoilRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_coil_registers_u16, MBRegisterMode eMode) {
  MBErrorCode    eStatus = MB_ILLEGAL_ADDRESS;
  ModbusMapBlock *block = find_modbus_block(starting_address_u16, number_of_coil_registers_u16);
  uint8_t		number_of_coil_bits_u8 = 0;		// reduce each coil to a bit and the number of these bits = number of coils_u16 / 8
  uint8_t		partial_final_byte_u8 = 0;			// contents of final byte might be partial (less than 8)
  uint8_t		full_coil_byte_u8 = 0;				// byte for a full 8 coils  
  uint8_t		number_of_full_coil_bytes_u8 = 0;	// bytes that are filled with data for eight coils
  uint8_t		number_of_partial_coil_bits_u8 = 0;	// number of bits in the partial coil byte
  uint8_t     start_number_of_partial_coil_bits_u8 = 0;
  uint8_t		bit_mask_u8 = 0;					// bit mask for converting 1-bit coils to full registers
  
  number_of_full_coil_bytes_u8 	= number_of_coil_registers_u16 / 8;
  number_of_partial_coil_bits_u8 	= number_of_coil_registers_u16 % 8;	
  start_number_of_partial_coil_bits_u8 = number_of_partial_coil_bits_u8;
  
  if (block) {
    //block_1 =(uint16_t) ( block->start_address_u16);
    //offset_1 = starting_address_u16;
    uint16_t index_u16 = starting_address_u16 - block->start_address_u16;
    
    if (eMode == MB_WRITE_REGISTER) {										
      // perform full-byte coil writes
      while(number_of_full_coil_bytes_u8 > 0){
        full_coil_byte_u8 = (uint16_t)*data_buffer_pu8++;					// gather the full coil byte from the write packet
        number_of_coil_bits_u8 = 8;
        bit_mask_u8 = 0x80;			
        while(number_of_coil_bits_u8  > 1) {
          if((full_coil_byte_u8 & bit_mask_u8) == 0)
            block->start_of_data_pu16[index_u16] = ((uint16_t)(0x00)) << 8;		// upper byte of coil = 0x00 because it is off
          else 
            block->start_of_data_pu16[index_u16] = ((uint16_t)(0xFF)) << 8;		// upper byte of coil = 0xFF
          block->start_of_data_pu16[index_u16] |= (uint16_t)(0x00);				// lower byte of coil is always = 0x00
          bit_mask_u8 >>= 1;														// next bit in byte
          --number_of_coil_bits_u8;
          ++index_u16;						// put into next coil register  from data table			
        }
        number_of_full_coil_bytes_u8--;
      }
      // perform partial-byte coil writes
      while(number_of_coil_bits_u8  > 1) {
        if((full_coil_byte_u8 & bit_mask_u8) == 0)
          block->start_of_data_pu16[index_u16] = ((uint16_t)(0x00)) << 8;		// upper byte of coil = 0x00 because it is off
        else 
          block->start_of_data_pu16[index_u16] = ((uint16_t)(0xFF)) << 8;		// upper byte of coil = 0xFF
        block->start_of_data_pu16[index_u16] |= (uint16_t)(0x00);				// lower byte of coil is always = 0x00
        bit_mask_u8 >>= 1;														// next bit in byte
        --number_of_coil_bits_u8;
        ++index_u16;															// put into next coil register  from data table			
      }		
    }
    else {
      // perform full-byte coil reads
      while(number_of_full_coil_bytes_u8 > 0) {
        full_coil_byte_u8 = 0;
        number_of_coil_bits_u8 = 8;
        // process one full coil bye, one bit at a time
        while(number_of_coil_bits_u8 > 0) {
          if(block->start_of_data_pu16[index_u16] == 0x0000)
            full_coil_byte_u8 |= 0;			// coil is off, so add a 0 to the message
          else
            full_coil_byte_u8 |= 1 << (8-number_of_coil_bits_u8);			// coil is on,  so add a 1 to the message				  
          //full_coil_byte_u8 <<= 1;			// shift bull byte up by 1
          --number_of_coil_bits_u8;			// process next coil bit
          ++index_u16;						// grab next coil register  from data table
          --number_of_coil_registers_u16;
        }
        *data_buffer_pu8++ = full_coil_byte_u8;								// send complete byte to data packet
        --number_of_full_coil_bytes_u8;
      }
      // perform partial byte coil read
      full_coil_byte_u8 = 0;
      while(number_of_partial_coil_bits_u8 > 0) {
        if(block->start_of_data_pu16[index_u16] == 0x0000)
          full_coil_byte_u8 |= 0;			// coil is off, so add a 0 to the message
        else
          full_coil_byte_u8 |= 1 << (start_number_of_partial_coil_bits_u8-number_of_partial_coil_bits_u8);			// coil is on,  so add a 1 to the message
        partial_final_byte_u8 <<= 1;			// shift bull byte up by 1
        --number_of_partial_coil_bits_u8;	// process next coil bit
        ++index_u16;						// grab next coil register  from data table
      }
      //partial_final_byte_u8 <<= (8 - number_of_partial_coil_bits_u8);	// fill remaining byte with 0's 
      *data_buffer_pu8++ = full_coil_byte_u8;								// send complete byte to data packet			  
    }
    eStatus = MB_NO_ERROR;
  }
  return eStatus;
}

MBErrorCode ProcessMBDiscreteRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16, MBRegisterMode eMode) {
  MBErrorCode    eStatus = MB_ILLEGAL_ADDRESS;
  ModbusMapBlock *block = find_modbus_block(starting_address_u16, number_of_registers_u16);
  
  if (block) {
    //block_1 =(uint16_t) ( block->start_address_u16);
    //offset_1 = starting_address_u16;
    uint16_t index_u16 = starting_address_u16 - block->start_address_u16;
    
    while (number_of_registers_u16 > 0 ) {
      if (eMode == MB_WRITE_REGISTER) {
        block->start_of_data_pu16[index_u16] = ((uint16_t)*data_buffer_pu8++) << 8;
        block->start_of_data_pu16[index_u16] |= (uint16_t)*data_buffer_pu8++;
      }
      else {
        *data_buffer_pu8++ = (uint8_t)(block->start_of_data_pu16[index_u16] >> 8);
        *data_buffer_pu8++ = (uint8_t)(block->start_of_data_pu16[index_u16] & 0xff);
      }
      ++index_u16;
      --number_of_registers_u16;
    }
    eStatus = MB_NO_ERROR;
  }
  return eStatus;
}


//
// Given a modbus start address and number of registers,
// return the mapping block that contains those values, or 0 (null) if no match 
//
ModbusMapBlock* find_modbus_block(uint16_t desired_address_u16, uint16_t number_of_registers_u16) {
  uint16_t block_index_u16;
  for (block_index_u16 = 0; block_index_u16 < number_of_modbus_blocks_u16; ++block_index_u16)
  {
    ModbusMapBlock *block = masterBlocks[block_index_u16];
    if ((block->start_address_u16 <= desired_address_u16) && (desired_address_u16 + number_of_registers_u16 <= block->start_address_u16 + block->number_of_registers_u16))
      return block;
  }
  return 0;
}

// Compute the MODBUS RTU CRC
uint16_t calculateModbusCRC(uint8_t *buf_pu8, uint16_t length_u16)
{
  uint16_t crc_u16 = 0xFFFF;
  
  for (uint16_t pos_u16 = 0; pos_u16 < length_u16; pos_u16++) {
    crc_u16 ^= (uint16_t)buf_pu8[pos_u16];          // XOR byte into least sig. byte of crc
    
    for (uint8_t index_u8 = 8; index_u8 != 0; index_u8--) {    // Loop over each bit
      if ((crc_u16 & 0x0001) != 0) {      // If the LSB is set
        crc_u16 >>= 1;                    // Shift right and XOR 0xA001
        crc_u16 ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc_u16 >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc_u16;  
}

// TODO: Move Register Storage to a Universal location to be used by universal protocol, modbus, and bluetooth modules
//
void Modbus_UpdateStoredStatus(uint8_t status){
  modbusRegisters100s[MODBUS_100S_INDEX_STATUS] = status;
}
void Modbus_UpdateStoredFaults(uint16_t faults){
  modbusRegisters100s[MODBUS_100S_INDEX_FAULTS] = faults;
}
void Modbus_UpdateStoredBusVoltage(uint16_t bus_voltage){
  modbusRegisters100s[MODBUS_100S_INDEX_BUS_VOLTAGE] = bus_voltage;
}
//
void Modbus_UpdateStoredDirection(uint8_t direction){
  modbusRegisters100s[MODBUS_100S_INDEX_DIRECTION] = direction;
}
void Modbus_UpdateStoredMeasuredSpeed(int16_t measured_speed){
  modbusRegisters100s[MODBUS_100S_INDEX_MEASURED_SPEED] = measured_speed;
}
void Modbus_UpdateStoredTorque(int16_t torque){
  modbusRegisters100s[MODBUS_100S_INDEX_TORQUE] = torque;
}
//
void Modbus_UpdateStoredPower(int16_t power){
  modbusRegisters100s[MODBUS_100S_INDEX_POWER_WATTS] = power;
}
void Modbus_UpdateStoredTemperature(int16_t temperature){
  modbusRegisters100s[MODBUS_100S_INDEX_IPM_TEMPERATURE] = temperature;
}

// Bootloader Features (START)
// TODO: In this section, move to their own file
#define UTILITY_FUNCTION_ID_JUMP_TO_PARTITION 1 
#define UTILITY_SECURITY_CODE 99
#define PARTITION_ID_BOOTLOADER 1
void Utility_ExecuteOperation(uint8_t function_id, uint8_t function_parameter, uint8_t function_subparameter) {
  switch (function_id)
  {
    case UTILITY_FUNCTION_ID_JUMP_TO_PARTITION:
      { // function_parameter = partition id, function_subparameter = sercurity code
        if (function_subparameter ==  UTILITY_SECURITY_CODE) { // only execute if security code was entered
          if (function_parameter == PARTITION_ID_BOOTLOADER) {
            Modbus_JumpToBootloader();
          }
        }
        break;
      }
    default:
      {
        break;
      }

  }
}

void Modbus_WatchDogInitialize(void) { // REVIEW: This is not currently used, as watchdog is initialized in application.
    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);
    //LL_IWDG_SetWindow(IWDG, 4095);
    LL_IWDG_SetReloadCounter(IWDG, 4095);
    while (LL_IWDG_IsReady(IWDG) != 1) {
    }
//    LL_IWDG_ReloadCounter(IWDG);
    LL_IWDG_SetWindow(IWDG, 4095);
}
// !errorTemporary: Watchdog Timer (END)
void Modbus_JumpToBootloader(void) {
    // Reboot via Watchdog Reset: 
	// - This works, when bootloader starts at address 0x08000000
    Modbus_WatchDogInitialize();
    while (1) {}
}
// Bootloader Features (END)



// Fort Wayne MODBUS code examples
#if REMOVE_MODBUS == 0
#define BROADCAST (0)
/*   Module Constant Macros Definitions */
#define MAX_SLAVE_ADDRESS 247
#define MAX_COILS_MODBUS (2000)
#define MAX_COILS_QUANTITY (MAX_COILS_MODBUS)
#define MIN_COILS_QUANTITY (1)

#define MAX_DISCRETE_MODBUS (224) // Tx Buffer 32bytes so 28 bytes * 8 = 224 Registers
#define MAX_DISCRETE_QUANTITY (MAX_DISCRETE_MODBUS)
#define MIN_DISCRETE_QUANTITY (1)

#define MAX_INPUTS_MODBUS (14) // 125 -> As Per ModBus Standard // 124 // Do not increase this!! - buffer limitation

#define MAX_INPUTS_QUANTITY (MAX_INPUTS_MODBUS)
#define MIN_INPUTS_QUANTITY (1)

#define MAX_HOLDING_READ_MODBUS (14) // 125 -> As Per ModBus Standard // (124) // limited by USB stack limit
#define MAX_HOLDING_WRITE_MODBUS (14)
#define MAX_HOLDING_READ_QUANTITY (MAX_HOLDING_READ_MODBUS)
#define MAX_HOLDING_WRITE_QUANTITY (MAX_HOLDING_WRITE_MODBUS)
#define MIN_HOLDING_QUANTITY (1)

#define MAX_REG_ADDRESS (0xFFFF)
#define MIN_PDU_SIZE (5)

#define BYTE_COUNT_USED (1) // used to conditionally compile code to check for  byte count in Function Code 16

#endif // REMOVE_MODBUS == 0
