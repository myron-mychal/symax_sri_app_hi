/**
  *****************************************************************************
  * @file    module_analog_0_10v.h 
  * @author  Regal, Satya Akkina
  * @version V1.0
  * @date    17-Aug-2020
  * @brief   Header for Analog 0-10V input
  * @note    
  *****************************************************************************
  */

// Define to prevent recursive inclusion 
#ifndef _MODULE_ANALOG_4_20MA_H_
#define _MODULE_ANALOG_4_20MA_H_

#ifdef __cplusplus
extern "C" {
#endif
  
// Includes 
#include "driver_adc1.h" 

#include "structured_memory.h"
#include "scheduler.h"
  
extern Ram_Buf sharedMemArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];
extern Ram_Buf *analog_Input_Settings_StructMem_u32;
extern Ram_Buf *analog_Data_StructMem_u32;

#define ADC_12_BIT_FULL_SCALE  (4096)
//#define AnalogPeriod (50)  //Analog voltage Update time mSec
#define DEBOUNCE_COUNT (4) // # of sequential samples above debounce threshold to indicate a good reading

//******************* Analog 0-10V Control (inside shared memory) ************  
// Analog 0-10V settings
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

typedef struct{
 struct Analog_4_20mA_Settings analog_4_20mA_Setting ;
 struct Analog_4_20mA_Data analog_4_20mA_Data;  
}Analog_4_20ma_Control;


//******* end of Analog 0-10V Control (inside shared memory) ****************

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_ANALOG_4_20MA_H_ */

