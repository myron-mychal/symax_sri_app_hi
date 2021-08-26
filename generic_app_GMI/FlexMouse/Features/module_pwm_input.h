/**
  *****************************************************************************
  * @file    module_pwm.h 
  * @author  Regal, Logan Schaufler
  * @version V1.0
  * @date    23-Nov-2020
  * @brief   Header for Digital PWM input
  * @note    
  *****************************************************************************
  */

// Define to prevent recursive inclusion 
#ifndef _MODULE_PWM_INPUT_H_
#define _MODULE_PWM_INPUT_H_

#ifdef __cplusplus
extern "C" {
#endif
  
// Includes 
#include "driver_gpio.h"  // REVIEW: is this needed now that PWM in has been moved to it's own driver?
#include "driver_pwm_input.h"
    
#include "structured_memory.h"
#include "scheduler.h"
  
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
extern Ram_Buf *pwm_input_Settings_StructMem_u32;
extern Ram_Buf *pwm_input_Data_StructMem_u32;

#define DigitalPeriod 50 //PWM digital Update time mSec

//******************* Digital PWM Control (inside shared memory) ************  
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

typedef struct{
 struct PWMInput_Settings pwmInput_Settings;
 struct PWMInput_Data pwmInput_Data;  
}PwmInput_Control;

enum { // TODO: This enum had conflicts with 0-10V module, why can it see that module?
  PWM_IN_DISABLE_INPUT = 0,
  PWM_IN_PWM_MODE = 1,
  PWM_IN_DIGITAL_MODE = 2
};

//******* end of Digital PWM Control (inside shared memory) ****************

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_PWM_INPUT_H_ */

