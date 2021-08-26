/**
  ***************************************************************************************************
  * @file    module_modbus_application_map.h 
  * @author  Regal Myron Mychal
  * @version V1.0
  * @date    15-Jun-2021
  * @brief   Macros for defining register map in applicatin micro
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MODBUS_APPLICATION_MAP_H_
#define _MODULE_MODBUS_APPLICATION_MAP_H_

/* Includes ------------------------------------------------------------------*/
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"
#include "module_analog_0_10v.h"
#include "module_analog_4_20ma.h"
#include "module_digital_inputs.h"
#include "module_digital_outputs.h"
#include "module_flash_blk_setting.h"
#include "module_motor_com.h"
#include "module_motor_demand_multiplexer.h"
#include "module_Motor_FW_Update.h"
#include "module_pwm_input.h"
#include "module_modbus.h"

extern	DigitalInputs_Control digitalInputs_Control;			// structure from digital inputs module
extern	AnalogVolts_Control analogVolts_Control;				// structure from analog inputs 0_to_10V module
extern	PwmInput_Control pwmInput_Control;						// structure from pwm inputsA module
extern  Modbus_RTU_Control modbus_RTU_control;					// structure from modbus module
extern	Analog_4_20ma_Control analog_4_20ma_Control_ptr;		// structure from analog inputs 4_to_20mA module
extern  MotorDemandMux_Control motorDemandMux_Control;			// structure from motor demand multiplexer module
extern  Motor_FW_Update_Control motor_FW_Update_Control;		// structure from motor firmware update module
extern	Flash_Blk_Setting_Control *flash_blk_setting_Control;	// structure from flash block setings module

// Template for module modbus address map structure
#define	MAX_COILS		(8)			// maximum number of modbus coils allowed for a single module
#define	MAX_DISCRETES	(32)		// maximum number of modbus discrete registers allowed for a single module
#define MAX_INPUTS		(64)		// maximum number of modbus input registers allowed for a single module
#define	MAX_HOLDINGS	(152)		// maximum bumber of modbus holding registers allowed for a single module

uint16_t	void_start_of_coils_pu16[1];
uint16_t	void_start_of_discretes_pu16[1];
uint16_t	void_start_of_inputs_pu16[1];
uint16_t	void_start_of_holdings_pu16[1];

// number of each type of modbus data element in the module
#define MODULE_NUMBER_OF_COILS			0
#define MODULE_NUMBER_OF_DISCRETES		0
#define MODULE_NUMBER_OF_INPUTS			0
#define MODULE_NUMBER_OF_HOLDINGS		0

// address for the start of each element type within the module
#define MODULE_MODBUS_GROUP					(0)
#define MODULE_MODBUS_GROUP_BASE_ADDRESS	(MODULE_MODBUS_GROUP<<8)
#define	MODULE_START_OF_COILS_ADDRESS		(MODULE_MODBUS_GROUP_BASE_ADDRESS + 0)
#define MODULE_START_OF_DISCRETES_ADDRESS	(MODULE_MODBUS_GROUP_BASE_ADDRESS + MAX_COILS)
#define MODULE_START_OF_INPUTS_ADDRESS		(MODULE_MODBUS_GROUP_BASE_ADDRESS + MAX_DISCRETES)
#define MODULE_START_OF_HOLDINGS_ADDRESS	(MODULE_MODBUS_GROUP_BASE_ADDRESS + MAX_INPUTS)
#define	MODULE_END_OF_GROUP					(MODULE_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_start_of_coils_pu16;			// first register in coil subgroup
uint16_t*	module_start_of_discretes_pu16;		// first register in discrete subgroup
uint16_t*	module_start_of_inputs_pu16;			// first register in inputs subgroup
uint16_t*	module_start_of_holdings_pu16;		// first register in holdings subgroup

// structure for analog input 0-10v
/*
struct AnalogVolts_Settings
{  
  uint16_t onVolts_u16;              // Demand turn ON volts
  uint16_t offVolts_u16;             // Demand trun OFF volts. Acts as Hysteresis
  uint16_t minVolts_u16;             // Min analog volts corresponding to minDemand_u16.
  uint16_t maxVolts_u16;             // Max acceptable analog volts corresponding to maxDemand_u16
  uint16_t maxAdcCounts_u16;         // Max output of ADC
  uint16_t debounceThresholdCounts_u16;  // Only use volts if change in analog is outside this threshold
  int16_t calibratonFactor_s16;     // Add/Delete this form measured for calibration
  uint16_t voltsHysteresis_u16;      // Hysteresis volts at the top and bottom end of demand
  uint16_t minDemand_u16;            // Min allowed demand
  uint16_t maxDemand_u16;            // Maximum allowed demand
  uint16_t minDemandHysteresis_u16;  // Lower end hysteresis if Min_Demand > Off_Volts demand
  uint16_t analogLowVolts_u16;       // Volts below which the bAnalog_Loss_Enable is set 
  uint16_t failSafeDemand_u16;       // Fail safe demand when loss of analog input is detected
  uint16_t analogPeriod_u16;         // Update rate in mSec for the analog voltage
  uint16_t digitalOnVolts_u16;       // Volts above which the input is considered as ON
  uint16_t digtialOffVolts_u16;      // Volts below which the input is considered as OFF 
  uint8_t  analogLowAlarmEnableCount_u8;  // analogPeriod*count delay before loss of analog low volts alarm is detected in mSec. 
  uint8_t  debounceLoopCount_u8;     // # of counts analog voltage should be above or below to consider it as a good value
  uint8_t  inputMode_u8;             // 0 = Analog input; 1 = digital input; 
  float    analogGain_f;             // Gain to convert ADC value to Volts
  uint8_t  is_enableAnalog;          // If "1" enable analog input for demand
  uint8_t  is_invertAnalog;          // If "1", 0V = Max demand 10V= 0 Demand 
  uint8_t  is_analogLowEnable;       // Enable loss of analog input
  uint8_t  is_analogFailSafeEnable;  // If "1" Switch to fail safe demand when loss of anlaog is detected
  uint8_t  is_analogMinDemandEnable; // If "True", min demand is maintaned when volts is below minVolts and above onVolts
  
};

// Live Analog Data
struct AnalogVolts_Data
{  
  uint16_t analogVoltsCounts_u16;     // Analog Volts in ADC counts
  uint16_t analogVolts_u16;           // Analog volts in xxyy, xx.yy volts
  uint16_t analogVoltsCountsStable_u16;  // Previous measured analog volts used for debounce check
  uint16_t analogDemand_u16;          // Calculated analog demand
  uint16_t analogVoltsPercent_u16;    // Analog Volts scalled 0 to 100%
  uint16_t analogDemandPercent_u16;   // Analog Demand scalled 0 to 100%
  uint8_t  analogErrorCode_u8;        // Error code  
  float    decreasingSlope_f;         // Slope used to calculate demand when analog is decreasing //SPA REVIEW
  float    increasingSlope_f;         // Slope used to calculate demand when analog is increasing //SPA REVIEW
  float    increasingIntercept_u16;   // Intercept used to calculate demand when analog is increasing
  float    decreasingIntercept_u16;   // Intercept used to calcualte demand when analog is decreasing
  uint8_t  is_analogLow;              // Set to "1" if loss of analog is detected
  uint8_t  is_decreasingAnalog;       // Set to "1" if analog voltage is decreasing
  uint8_t  is_lowerHysteresisEnable;  // Set to "1" if lower end hysteresis need to be enabled
  uint8_t  is_upperHysteresisEnable;  // Set to "1" if upper end hysteresis need to be enabled  
  uint8_t  is_demandOn;               // Set to "1" if demand goes from 0 to above min demand.
  uint8_t  is_digitalInputON;         // Set to "1" if the measured voltage is above "digitalOnVolts"
};
*/
// number of each type of modbus data element in the module
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_COILS			0
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_DISCRETES		0
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_INPUTS			(11) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_HOLDINGS		(14) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_ANALOG_INPUTS_0TO10_GROUP						(136)			// analog inputs 0-10V group 
#define MODULE_ANALOG_INPUTS_0TO10_BASE_ADDRESS					(MODULE_ANALOG_INPUTS_0TO10_GROUP<<8)
#define	MODULE_ANALOG_INPUTS_0TO10_START_OF_COILS_ADDRESS		(MODULE_ANALOG_INPUTS_0TO10_BASE_ADDRESS + 0)
#define MODULE_ANALOG_INPUTS_0TO10_START_OF_DISCRETES_ADDRESS	(MODULE_ANALOG_INPUTS_0TO10_BASE_ADDRESS + MAX_COILS)
#define MODULE_ANALOG_INPUTS_0TO10_START_OF_INPUTS_ADDRESS		(MODULE_ANALOG_INPUTS_0TO10_BASE_ADDRESS + MAX_DISCRETES)
#define MODULE_ANALOG_INPUTS_0TO10_START_OF_HOLDINGS_ADDRESS	(MODULE_ANALOG_INPUTS_0TO10_BASE_ADDRESS + MAX_INPUTS)
#define	MODULE_ANALOG_INPUTS_0TO10_END_OF_GROUP					(MODULE_ANALOG_INPUTS_0TO10_BASE_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_analog_inputs_0TO10_start_of_coils_pu16			= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_analog_inputs_0TO10_start_of_discretes_pu16		= (uint16_t*) &(void_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_analog_inputs_0TO10_start_of_inputs_pu16			= (uint16_t*) &(analogVolts_Control.analogVolts_Data.analogVoltsCounts_u16);			// first register in inputs subgroup
uint16_t*	module_analog_inputs_0TO10_start_of_holdings_pu16		= (uint16_t*) &(analogVolts_Control.analogVolts_Setting.onVolts_u16);		// first register in holdings subgroup

// structure for digital inputs
/*
struct DigitalInputs_Settings
{ 
  uint8_t  inputFunction[TOTAL_DIGITAL_INPUTS]; // Function of each digital input
  uint8_t  debounceCountLimit_u8; // # of counts digital input should be active high/low to consider it as active high/low
  uint16_t inputPolarity_u16;        // Each bit indicates the polarity of the input. 0= Normally Open, 1 = Normally closed. Bit 0 = Digital input 0 polarity.
  uint16_t inputEnable_u16;          // Each bit enables an individual digital input. Bit 0 = Digital input 0 enable.
  uint16_t demandPercent[MAX_DISCRETE_DEMANDS]; // Demand for each possible input combination
};

// Live Digital Inputs Data
struct DigitalInputs_Data
{ 
  uint16_t digitalInputStatus_u16;    // Each bit indicates status of digital inputs. Bit 0 = Digital input 0 status.
  uint16_t digitalInputStatusFiltered_u16; // Each bit indicates filtered status of digital inputs. Bit 0 = Digital input 0 status.
  uint32_t digitalInputsValue[TOTAL_DIGITAL_INPUTS]; // Value based on input functionality of digital input
  uint16_t discreteDemandPercent; // Demand when input mode is sepected as discrete demand input
  uint8_t  is_invertDirection;    // Flag that indicated that direction of rotation is inverted
  uint8_t  is_motorEnabled;       // Enable Motor
};
*/

// number of each type of modbus data element in the module
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_COILS			0
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_DISCRETES		0
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_INPUTS			(11) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_HOLDINGS		(14) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_DIGITAL_INPUTS_GROUP							(137)			// digital inputs group 
#define MODULE_DIGITAL_INPUTS_BASE_ADDRESS					(MODULE_DIGITAL_INPUTS_GROUP<<8)
#define	MODULE_DIGITAL_INPUTS_START_OF_COILS_ADDRESS		(MODULE_DIGITAL_INPUTS_BASE_ADDRESS + 0)
#define MODULE_DIGITAL_INPUTS_START_OF_DISCRETES_ADDRESS	(MODULE_DIGITAL_INPUTS_BASE_ADDRESS + MAX_COILS)
#define MODULE_DIGITAL_INPUTS_START_OF_INPUTS_ADDRESS		(MODULE_DIGITAL_INPUTS_BASE_ADDRESS + MAX_DISCRETES)
#define MODULE_DIGITAL_INPUTS_START_OF_HOLDINGS_ADDRESS		(MODULE_DIGITAL_INPUTS_BASE_ADDRESS + MAX_INPUTS)
#define	MODULE_DIGITAL_INPUTS_END_OF_GROUP					(MODULE_DIGITAL_INPUTS_BASE_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_digital_inputs_start_of_coils_pu16		= (uint16_t*) &(void_start_of_coils_pu16[0]);	
uint16_t*	module_digital_inputs_start_of_discretes_pu16	= (uint16_t*) &(void_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_digital_inputs_start_of_inputs_pu16		= (uint16_t*) &(digitalInputs_Control.digitalInputs_Data.digitalInputStatus_u16);		// first register in holdings subgroup
uint16_t*	module_digital_inputs_start_of_holdings_pu16		= (uint16_t*) &(digitalInputs_Control.digitalInputs_Setting.inputFunction[0]);			// first register in inputs subgroup

// structure for PWM input
/*
// Digital PWM settings
struct PWMInput_Settings
{  
  //uint8_t        is_pwmLowEnable;          // Enable loss of digital input
  uint8_t        is_pwmFailSafeEnable;      // If "1" Switch to fail safe demand when loss of anlaog is detected
  uint16_t       maxPWMFrequency_u16;       // Max readable PWM frequency, ignores duty cycle for anything past this
  uint16_t       minPWMFrequency_u16;       // Min readable PWM frequency, ignores duty cycle for anything before this
  uint8_t        is_invertDigitalPWM;       // If '1', 0% = Max demand and 100%= 0 Demand
  uint16_t       maxDutyCycle_u16;          // Maximum acceptable duty cycle         
  uint16_t       minDutyCycle_u16;          // Minimum acceptable duty cycle    
  uint16_t       minDemand_u16;             // Min allowed demand             
  uint16_t       maxDemand_u16;             // Maximum allowed demand   
  uint16_t       calibratonFactor_u16;      // Add/Delete this form measured for calibration 
  uint8_t        pwmLowAlarmEnableCount_u8; // pwmPeriod*count delay before loss of analog low volts alarm is detected in mSec. 
  uint16_t       pwmPeriod;                 // Time until calculated demand is transmitted to the main app (in ms) 
  uint16_t       pwmLossFrequency_u16;      // Frequency below which the bPWM_Loss_Enable is set     
  uint16_t       failSafeDemand_u16;        // Fail safe demand when loss of pwm input is detected  
  uint16_t       minDemandHysteresis_u16;   // Lower end hysteresis if Min_Demand > Off_Volts demand
  uint16_t       minTurnOnDutyCycle;      // Minimum duty cycle module takes to turn on and begin operation
  uint8_t        is_pwmLowEnable;          // Enable loss of digital input   spec 12
  uint16_t       pwmLowDutyCycle_u16;
  uint8_t        inputMode_u8;              // Enable alternate function of Digital Input
  uint16_t       dutyCycleHysteresis_u16;
  uint16_t       offDutyCycle_u16;
  uint16_t       onDutyCycle_u16;
  uint8_t        is_pwmMinDemandEnable;
};

// Live Digital Frequency Data
struct PWMInput_Data
{  
  float          inheritedPwmInputDutyCycle_f;       // Calculated duty cycle demand (calculation done in driver) averaged over 4 values
  //float          PWMInputDutyCycleAverageCalc_f;       // Calculated duty cycle demand (calculation done in driver) averaged over 4 values
  uint8_t        pwmInputDigitalInputState_u8;   // '1' = high state;'2' = low state
  float          pwmInputDemand_f;       // Calculated output demand from duty cycle
  uint16_t       inheritedPWMInputFrequency_u16;    // Measured PWM input frequency
  uint16_t       pwmInputDemandPercentage_u16;    // Output demand percentage given by module
  //uint16_t       pwmInputFrequencyAverageCalc_u16;    // Measured PWM input frequency
  float          pwmStableDutyCycle_f;    // Previous duty cycle readings
  uint8_t        is_decreasingPWM;        // Set to "1" if analog voltage is decreasing
  float          increasingIntercept_f;    // Intercept used to calculate demand when analog is increasing
  float          decreasingIntercept_f;    // Intercept used to calcualte demand when analog is decreasing
  float          decreasingSlope_f;          // Slope used to calculate demand when analog is decreasing //SPA REVIEW
  float          increasingSlope_f;          // Slope used to calculate demand when analog is increasing //SPA REVIEW
  uint8_t        pwmErrorCode_u8;            // Error code  
  uint8_t        is_pwmDutyCycleLow;                  // Set to "1" if loss of analog is detected
  uint8_t        is_pwmFrequencyLow;         // Set to "1" if loss of pwm input frequency is low
  uint8_t        is_demandOn;                // Set to "1" if demand goes from 0 to above min demand.
  uint8_t        is_lowerHysteresisEnable;   // Set to "1" if lower end hysteresis need to be enabled
  uint8_t        is_upperHysteresisEnable;   // Set to "1" if upper end hysteresis need to be enabled  
  uint8_t        is_digitalInputON;         // Enable digital input
  uint8_t        is_TurnOnDutyCycleOccured; // Determines when Turn On Duty Cycle has been achieved
  uint8_t        is_LossDutyCycleBeenExceeded; //Set to '1' if loss duty cycle has been exceeded and fail safe mode can now engage
};
*/

// number of each type of modbus data element in the module
#define MODULE_PWM_INPUT_NUMBER_OF_COILS			0
#define MODULE_PWM_INPUT_NUMBER_OF_DISCRETES		0
#define MODULE_PWM_INPUT_NUMBER_OF_INPUTS			(11) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_PWM_INPUT_NUMBER_OF_HOLDINGS			(14) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_PWM_INPUT_GROUP						(138)			// analog inputs 0-10V group 
#define MODULE_PWM_INPUT_BASE_ADDRESS				(MODULE_PWM_INPUT_GROUP<<8)
#define	MODULE_PWM_INPUT_START_OF_COILS_ADDRESS		(MODULE_PWM_INPUT_BASE_ADDRESS + 0)
#define MODULE_PWM_INPUT_START_OF_DISCRETES_ADDRESS	(MODULE_PWM_INPUT_BASE_ADDRESS + MAX_COILS)
#define MODULE_PWM_INPUT_START_OF_INPUTS_ADDRESS	(MODULE_PWM_INPUT_BASE_ADDRESS + MAX_DISCRETES)
#define MODULE_PWM_INPUT_START_OF_HOLDINGS_ADDRESS	(MODULE_PWM_INPUT_BASE_ADDRESS + MAX_INPUTS)
#define	MODULE_PWM_INPUT_END_OF_GROUP				(MODULE_PWM_INPUT_BASE_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_pwm_input_start_of_coils_pu16			= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_pwm_input_start_of_discretes_pu16		= (uint16_t*) &(void_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_pwm_input_start_of_inputs_pu16		= (uint16_t*) &(pwmInput_Control.pwmInput_Data.pwmInputDigitalInputState_u8);			// first register in inputs subgroup
uint16_t*	module_pwm_input_start_of_holdings_pu16		= (uint16_t*) &(pwmInput_Control.pwmInput_Settings.is_pwmFailSafeEnable);		// first register in holdings subgroup

// structure for MODBUS module
/*
// Digital PWM settings
struct PWMInput_Settings
{  
  //uint8_t        is_pwmLowEnable;          // Enable loss of digital input
  uint8_t        is_pwmFailSafeEnable;      // If "1" Switch to fail safe demand when loss of anlaog is detected
  uint16_t       maxPWMFrequency_u16;       // Max readable PWM frequency, ignores duty cycle for anything past this
  uint16_t       minPWMFrequency_u16;       // Min readable PWM frequency, ignores duty cycle for anything before this
  uint8_t        is_invertDigitalPWM;       // If '1', 0% = Max demand and 100%= 0 Demand
  uint16_t       maxDutyCycle_u16;          // Maximum acceptable duty cycle         
  uint16_t       minDutyCycle_u16;          // Minimum acceptable duty cycle    
  uint16_t       minDemand_u16;             // Min allowed demand             
  uint16_t       maxDemand_u16;             // Maximum allowed demand   
  uint16_t       calibratonFactor_u16;      // Add/Delete this form measured for calibration 
  uint8_t        pwmLowAlarmEnableCount_u8; // pwmPeriod*count delay before loss of analog low volts alarm is detected in mSec. 
  uint16_t       pwmPeriod;                 // Time until calculated demand is transmitted to the main app (in ms) 
  uint16_t       pwmLossFrequency_u16;      // Frequency below which the bPWM_Loss_Enable is set     
  uint16_t       failSafeDemand_u16;        // Fail safe demand when loss of pwm input is detected  
  uint16_t       minDemandHysteresis_u16;   // Lower end hysteresis if Min_Demand > Off_Volts demand
  uint16_t       minTurnOnDutyCycle;      // Minimum duty cycle module takes to turn on and begin operation
  uint8_t        is_pwmLowEnable;          // Enable loss of digital input   spec 12
  uint16_t       pwmLowDutyCycle_u16;
  uint8_t        inputMode_u8;              // Enable alternate function of Digital Input
  uint16_t       dutyCycleHysteresis_u16;
  uint16_t       offDutyCycle_u16;
  uint16_t       onDutyCycle_u16;
  uint8_t        is_pwmMinDemandEnable;
};

// Live Digital Frequency Data
struct PWMInput_Data
{  
  float          inheritedPwmInputDutyCycle_f;       // Calculated duty cycle demand (calculation done in driver) averaged over 4 values
  //float          PWMInputDutyCycleAverageCalc_f;       // Calculated duty cycle demand (calculation done in driver) averaged over 4 values
  uint8_t        pwmInputDigitalInputState_u8;   // '1' = high state;'2' = low state
  float          pwmInputDemand_f;       // Calculated output demand from duty cycle
  uint16_t       inheritedPWMInputFrequency_u16;    // Measured PWM input frequency
  uint16_t       pwmInputDemandPercentage_u16;    // Output demand percentage given by module
  //uint16_t       pwmInputFrequencyAverageCalc_u16;    // Measured PWM input frequency
  float          pwmStableDutyCycle_f;    // Previous duty cycle readings
  uint8_t        is_decreasingPWM;        // Set to "1" if analog voltage is decreasing
  float          increasingIntercept_f;    // Intercept used to calculate demand when analog is increasing
  float          decreasingIntercept_f;    // Intercept used to calcualte demand when analog is decreasing
  float          decreasingSlope_f;          // Slope used to calculate demand when analog is decreasing //SPA REVIEW
  float          increasingSlope_f;          // Slope used to calculate demand when analog is increasing //SPA REVIEW
  uint8_t        pwmErrorCode_u8;            // Error code  
  uint8_t        is_pwmDutyCycleLow;                  // Set to "1" if loss of analog is detected
  uint8_t        is_pwmFrequencyLow;         // Set to "1" if loss of pwm input frequency is low
  uint8_t        is_demandOn;                // Set to "1" if demand goes from 0 to above min demand.
  uint8_t        is_lowerHysteresisEnable;   // Set to "1" if lower end hysteresis need to be enabled
  uint8_t        is_upperHysteresisEnable;   // Set to "1" if upper end hysteresis need to be enabled  
  uint8_t        is_digitalInputON;         // Enable digital input
  uint8_t        is_TurnOnDutyCycleOccured; // Determines when Turn On Duty Cycle has been achieved
  uint8_t        is_LossDutyCycleBeenExceeded; //Set to '1' if loss duty cycle has been exceeded and fail safe mode can now engage
};
*/

// number of each type of modbus data element in the module
#define MODULE_MODBUS_RTU_NUMBER_OF_COILS			0
#define MODULE_MODBUS_RTU_NUMBER_OF_DISCRETES		0
#define MODULE_MODBUS_RTU_NUMBER_OF_INPUTS			(1) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_MODBUS_RTU_NUMBER_OF_HOLDINGS		(5) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_MODBUS_RTU_GROUP							(139)			// analog inputs 0-10V group 
#define MODULE_MODBUS_RTU_BASE_ADDRESS					(MODULE_MODBUS_RTU_GROUP<<8)
#define	MODULE_MODBUS_RTU_START_OF_COILS_ADDRESS		(MODULE_MODBUS_RTU_BASE_ADDRESS + 0)
#define MODULE_MODBUS_RTU_START_OF_DISCRETES_ADDRESS	(MODULE_MODBUS_RTU_BASE_ADDRESS + MAX_COILS)
#define MODULE_MODBUS_RTU_START_OF_INPUTS_ADDRESS		(MODULE_MODBUS_RTU_BASE_ADDRESS + MAX_DISCRETES)
#define MODULE_MODBUS_RTU_START_OF_HOLDINGS_ADDRESS		(MODULE_MODBUS_RTU_BASE_ADDRESS + MAX_INPUTS)
#define	MODULE_MODBUS_RTU_END_OF_GROUP					(MODULE_MODBUS_RTU_BASE_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_modbus_rtu_start_of_coils_pu16		= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_modbus_rtu_start_of_discretes_pu16	= (uint16_t*) &(void_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_modbus_rtu_start_of_inputs_pu16		= (uint16_t*) &(modbus_RTU_control.modbus_RTU_Data.messages_received_u16);			// first register in inputs subgroup
uint16_t*	module_modbus_rtu_start_of_holdings_pu16	= (uint16_t*) &(modbus_RTU_control.modbus_RTU_Settings.baudRate_u16);		// first register in holdings subgroup

// structure for analg input 4-20 mA
/*
struct Analog_4_20mA_Settings
{  
  uint16_t onVolts_u16;              // Demand turn ON volts
  uint16_t offVolts_u16;             // Demand trun OFF volts. Acts as Hysteresis
  uint16_t minVolts_u16;             // Min analog volts corresponding to minDemand_u16.
  uint16_t maxVolts_u16;             // Max acceptable analog volts corresponding to maxDemand_u16
  uint16_t maxAdcCounts_u16;         // Max output of ADC
  uint16_t debounceThresholdCounts_u16;   // Only use volts if change in analog is outside this threshold
  int16_t  calibratonFactor_s16;     // Add/Delete this form measured for calibration
  uint16_t voltsHysteresis_u16;      // Hysteresis volts at the top and bottom end of demand
  uint16_t minDemand_u16;            // Min allowed demand
  uint16_t maxDemand_u16;            // Maximum allowed demand
  uint16_t minDemandHysteresis_u16;  // Lower end hysteresis if Min_Demand > Off_Volts demand
  uint16_t analogLowVolts_u16;       // Volts below which the bAnalog_Loss_Enable is set 
  uint16_t failSafeDemand_u16;       // Fail safe demand when loss of analog input is detected
  uint16_t analogPeriod_u16;         // Update rate in mSec for the analog voltage
  uint16_t digitalOnVolts_u16;       // Volts above which the input is considered as ON
  uint16_t digtialOffVolts_u16;      // Volts below which the input is considered as OFF 
  uint8_t  analogLowAlarmEnableCount_u8;  // analogPeriod*count delay before loss of analog low volts alarm is detected in mSec. 
  uint8_t  debounceLoopCount_u8;     // # of counts analog voltage should be above or below to consider it as a good value
  uint8_t  inputMode_u8;             // 0 = Analog input; 1 = digital input; 
  float    analogGain_f;             // Gain to convert ADC value to Volts
  float    analogVoltsToAmpsGain_f;  // Gain to convert analog volts to mAmps
  uint8_t  is_enableAnalog;          // If "1" enable analog input for demand
  uint8_t  is_invertAnalog;          // If "1", 0V = Max demand 10V= 0 Demand 
  uint8_t  is_analogLowEnable;       // Enable loss of analog input
  uint8_t  is_analogFailSafeEnable;  // If "1" Switch to fail safe demand when loss of anlaog is detected
  uint8_t  is_analogMinDemandEnable; // If "True", min demand is maintaned when volts is below minVolts and above onVolts
  
};

// Live Analog Data
struct Analog_4_20mA_Data
{  
  uint16_t analogVoltsCounts_u16;     // Analog Volts in ADC counts
  uint16_t analogAmps_u16;            // Analog Amps in xxyy, xx.yy mAmps
  uint16_t analogVolts_u16;           // Analog volts in xxyy, xx.yy volts
  uint16_t analogVoltsCountsStable_u16;  // Previous measured analog volts used for debounce check
  uint16_t analogDemand_u16;          // Calculated analog demand
  uint16_t analogVoltsPercent_u16;    // Analog Volts scalled 0 to 100%
  uint16_t analogDemandPercent_u16;   // Analog Demand scalled 0 to 100%
  uint8_t  analogErrorCode_u8;        // Error code  
  float    decreasingSlope_f;         // Slope used to calculate demand when analog is decreasing //SPA REVIEW
  float    increasingSlope_f;         // Slope used to calculate demand when analog is increasing //SPA REVIEW
  float    increasingIntercept_u16;   // Intercept used to calculate demand when analog is increasing
  float    decreasingIntercept_u16;   // Intercept used to calcualte demand when analog is decreasing
  uint8_t  is_analogLow;              // Set to "1" if loss of analog is detected
  uint8_t  is_decreasingAnalog;       // Set to "1" if analog voltage is decreasing
  uint8_t  is_lowerHysteresisEnable;  // Set to "1" if lower end hysteresis need to be enabled
  uint8_t  is_upperHysteresisEnable;  // Set to "1" if upper end hysteresis need to be enabled  
  uint8_t  is_demandOn;               // Set to "1" if demand goes from 0 to above min demand.
  uint8_t  is_digitalInputON;         // Set to "1" if the measured voltage is above "digitalOnVolts"
};
*/
// number of each type of modbus data element in the module
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_COILS			0
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_DISCRETES		0
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_INPUTS			(11) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_HOLDINGS		(14) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_ANALOG_INPUTS_4TO20_GROUP						(140)			// analog inputs 0-10V group 
#define MODULE_ANALOG_INPUTS_4TO20_BASE_ADDRESS					(MODULE_ANALOG_INPUTS_4TO20_GROUP<<8)
#define	MODULE_ANALOG_INPUTS_4TO20_START_OF_COILS_ADDRESS		(MODULE_ANALOG_INPUTS_4TO20_BASE_ADDRESS + 0)
#define MODULE_ANALOG_INPUTS_4TO20_START_OF_DISCRETES_ADDRESS	(MODULE_ANALOG_INPUTS_4TO20_BASE_ADDRESS + MAX_COILS)
#define MODULE_ANALOG_INPUTS_4TO20_START_OF_INPUTS_ADDRESS		(MODULE_ANALOG_INPUTS_4TO20_BASE_ADDRESS + MAX_DISCRETES)
#define MODULE_ANALOG_INPUTS_4TO20_START_OF_HOLDINGS_ADDRESS	(MODULE_ANALOG_INPUTS_4TO20_BASE_ADDRESS + MAX_INPUTS)
#define	MODULE_ANALOG_INPUTS_4TO20_END_OF_GROUP					(MODULE_ANALOG_INPUTS_4TO20_BASE_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_analog_inputs_4TO20_start_of_coils_pu16			= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_analog_inputs_4TO20_start_of_discretes_pu16		= (uint16_t*) &(void_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_analog_inputs_4TO20_start_of_inputs_pu16			= (uint16_t*) &(analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCounts_u16);			// first register in inputs subgroup
uint16_t*	module_analog_inputs_4TO20_start_of_holdings_pu16		= (uint16_t*) &(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.onVolts_u16);		// first register in holdings subgroup

// structure for Demand Multiplexer
/*
// Live settings and data
struct MotorDemandMux_Settings {
  uint16_t demandSource_u16;          // User set demand source
  uint8_t control_Mode_u8;            // 00 = Speed; 01 = Torque;
  uint8_t analog0_10vPriority_u8;      // Demand priority. 1 being highest
  uint8_t analog4_20mAPriority_u8;  // Demand priority. 1 being highest
  uint8_t digitalInputsPriority_u8; // Demand priority. 1 being highest
  uint8_t modbusPriority_u8;         // Demand priority. 1 being highest
  uint8_t pwmInputPriority_u8;      // Demand priority. 1 being highest
 uint8_t maxPriority_u8;            // Demand priority. 1 being highest
};

struct MotorDemandMux_Data {
  uint16_t demandValue_u16;
  uint8_t currentDemandSource_u8;  // Demand source used for calculating demand based on priority
};

typedef struct {
    struct MotorDemandMux_Settings motorDemandMux_Settings;
    struct MotorDemandMux_Data motorDemandMux_Data;
} MotorDemandMux_Control;
*/

// number of each type of modbus data element in the module
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_COILS			0
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_DISCRETES		0
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_INPUTS			(1) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_HOLDINGS		(8) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_DEMAND_MULTIPLEXER_GROUP							(141)			// demand multiplexer group
#define MODULE_DEMAND_MULTIPLEXER_BASE_ADDRESS					(MODULE_DEMAND_MULTIPLEXER_GROUP<<8)

#define	MODULE_DEMAND_MULTIPLEXER_START_OF_COILS_ADDRESS		(MODULE_DEMAND_MULTIPLEXER_BASE_ADDRESS + 0)
#define MODULE_DEMAND_MULTIPLEXER_START_OF_DISCRETES_ADDRESS	(MODULE_DEMAND_MULTIPLEXER_BASE_ADDRESS + MAX_COILS)
#define MODULE_DEMAND_MULTIPLEXER_START_OF_INPUTS_ADDRESS		(MODULE_DEMAND_MULTIPLEXER_BASE_ADDRESS + MAX_DISCRETES)
#define MODULE_DEMAND_MULTIPLEXER_START_OF_HOLDINGS_ADDRESS		(MODULE_DEMAND_MULTIPLEXER_BASE_ADDRESS + MAX_INPUTS)
#define	MODULE_DEMAND_MULTIPLEXER_END_OF_GROUP					(MODULE_DEMAND_MULTIPLEXER_BASE_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_demand_multiplexer_start_of_coils_pu16			= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_demand_multiplexer_start_of_discretes_pu16		= (uint16_t*) &(void_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_demand_multiplexer_start_of_inputs_pu16			= (uint16_t*) &(motorDemandMux_Control.motorDemandMux_Data.demandValue_u16);			// first register in inputs subgroup
uint16_t*	module_demand_multiplexer_start_of_holdings_pu16		= (uint16_t*) &(motorDemandMux_Control.motorDemandMux_Settings.demandSource_u16);		// first register in holdings subgroup

// structure for Firmware update
/*
typedef struct{
  uint8_t FWCMD;
  
  //Firmware block number(256byte per block) in current buffer, Bootloader can handle max 256byte per write. 
  //Firmware write will end when this paramter contain a negative value(WrBufBlkNO < -1)
  int16_t WrBufBlkNO;  
  //When Rd_Wr_busy = 1, extrnal user should not alter the content of the read/write buffer, value of WrBufBlkNO
  uint8_t Rd_Wr_busy;
  
  int16_t RdBufBlkNO;
  uint8_t errorCode_u8;
}Motor_FW_Update_Control;
*/

// number of each type of modbus data element in the module
#define MODULE_FIRMWARE_UPDATE_NUMBER_OF_COILS			0
#define MODULE_FIRMWARE_UPDATE_NUMBER_OF_DISCRETES		0
#define MODULE_FIRMWARE_UPDATE_NUMBER_OF_INPUTS			(0) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_FIRMWARE_UPDATE_NUMBER_OF_HOLDINGS		(5) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_FIRMWARE_UPDATE_GROUP						(142)			// demand multiplexer group
#define MODULE_FIRMWARE_UPDATE_BASE_ADDRESS					(MODULE_FIRMWARE_UPDATE_GROUP<<8)

#define	MODULE_FIRMWARE_UPDATE_START_OF_COILS_ADDRESS		(MODULE_FIRMWARE_UPDATE_BASE_ADDRESS + 0)
#define MODULE_FIRMWARE_UPDATE_START_OF_DISCRETES_ADDRESS	(MODULE_FIRMWARE_UPDATE_BASE_ADDRESS + MAX_COILS)
#define MODULE_FIRMWARE_UPDATE_START_OF_INPUTS_ADDRESS		(MODULE_FIRMWARE_UPDATE_BASE_ADDRESS + MAX_DISCRETES)
#define MODULE_FIRMWARE_UPDATE_START_OF_HOLDINGS_ADDRESS	(MODULE_FIRMWARE_UPDATE_BASE_ADDRESS + MAX_INPUTS)
#define	MODULE_FIRMWARE_UPDATE_END_OF_GROUP					(MODULE_FIRMWARE_UPDATE_BASE_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_firmware_update_start_of_coils_pu16			= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_firmware_update_start_of_discretes_pu16		= (uint16_t*) &(void_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_firmware_update_start_of_inputs_pu16			= (uint16_t*) &(void_start_of_inputs_pu16[0]);			// first register in inputs subgroup
uint16_t*	module_firmware_update_start_of_holdings_pu16		= (uint16_t*) &(motor_FW_Update_Control.FWCMD);		// first register in holdings subgroup

// structure for Flash Block Settings
/*
typedef struct{
  uint8_t FWCMD;
  
  //Firmware block number(256byte per block) in current buffer, Bootloader can handle max 256byte per write. 
  //Firmware write will end when this paramter contain a negative value(WrBufBlkNO < -1)
  int16_t WrBufBlkNO;  
  //When Rd_Wr_busy = 1, extrnal user should not alter the content of the read/write buffer, value of WrBufBlkNO
  uint8_t Rd_Wr_busy;
  
  int16_t RdBufBlkNO;
  uint8_t errorCode_u8;
}Motor_FW_Update_Control;
*/

// number of each type of modbus data element in the module
#define MODULE_FLASH_BLOCK_NUMBER_OF_COILS			0
#define MODULE_FLASH_BLOCK_NUMBER_OF_DISCRETES		0
#define MODULE_FLASH_BLOCK_NUMBER_OF_INPUTS			(1) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_FLASH_BLOCK_NUMBER_OF_HOLDINGS		(0) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_FLASH_BLOCK_GROUP						(143)			// demand multiplexer group
#define MODULE_FLASH_BLOCK_BASE_ADDRESS					(MODULE_FLASH_BLOCK_GROUP<<8)

#define	MODULE_FLASH_BLOCK_START_OF_COILS_ADDRESS		(MODULE_FLASH_BLOCK_BASE_ADDRESS + 0)
#define MODULE_FLASH_BLOCK_START_OF_DISCRETES_ADDRESS	(MODULE_FLASH_BLOCK_BASE_ADDRESS + MAX_COILS)
#define MODULE_FLASH_BLOCK_START_OF_INPUTS_ADDRESS		(MODULE_FLASH_BLOCK_BASE_ADDRESS + MAX_DISCRETES)
#define MODULE_FLASH_BLOCK_START_OF_HOLDINGS_ADDRESS	(MODULE_FLASH_BLOCK_BASE_ADDRESS + MAX_INPUTS)
#define	MODULE_FLASH_BLOCK_END_OF_GROUP					(MODULE_FLASH_BLOCK_BASE_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_flash_block_start_of_coils_pu16			= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_flash_block_start_of_discretes_pu16		= (uint16_t*) &(void_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_flash_block_start_of_inputs_pu16			= (uint16_t*) &(void_start_of_inputs_pu16[0]);			// first register in inputs subgroup
uint16_t*	module_flash_block_start_of_holdings_pu16		= (uint16_t*) &(void_start_of_holdings_pu16[0]);			// first register in holdings subgroup

#endif /* _MODULE_MODBUS_APPLICATION_MAP_H_ */

