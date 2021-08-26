/**
  ********************************************************************************************************************************
  * @file    module_motor_demand_multiplexer.h
  * @author  Justin Moon
  * @brief   Combine multiple motor demand sources into a single motor demand output.
  * @details This module is to be referenced by 'module_motor_com' to send a demand value to the motor control MCU.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_MOTOR_DEMAND_MUX_H_
#define _MODULE_MOTOR_DEMAND_MUX_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

  /* Harmonic injection variables*/
 extern int16_t  hi_amplitude_s16[8]; 		
 extern uint8_t  hi_angle_multiplier_hi_u8[8];
  extern uint16_t hi_angle_offset_u16[8]; 		
  extern uint8_t  hi_is_phase_inverted_u8[8]; 	
  extern uint16_t hi_min_speed_u16[8]; 		
  extern uint16_t hi_max_speed_u16[8]; 		
  extern uint8_t  hi_is_harmonic_injection_allowed_hi_u8;
  
  
  
  /* Harmonic injection variables*/
  
/* Setup -----------------------------------------------------------------------------------------------------------------------*/

#define MODULE_MOTOR_DEMAND_MUX_POLL_TIME 100 // Poll time for the module
enum MotorDemandSources {
    MOTOR_DEMAND_SOURCE_PRIORITY_SYSTEM, // Selects based on Priority Matrix
    MOTOR_DEMAND_SOURCE_ANALOG_0_10V,
    MOTOR_DEMAND_SOURCE_ANALOG_4_20MA,
    MOTOR_DEMAND_SOURCE_DIGITAL_INPUTS,
    MOTOR_DEMAND_SOURCE_PWM_INPUT,
    MOTOR_DEMAND_SOURCE_SERIAL_COMMAND, // Modbus
    // MOTOR_DEMAND_SOURCES_TOTAL must stay at the end
    MOTOR_DEMAND_SOURCES_TOTAL
};

#define DEFAULT_MOTOR_DEMAND_SOURCE MOTOR_DEMAND_SOURCE_ANALOG_0_10V

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

enum
{
  DEMAND_MODE_SPEED,
  DEMAND_MODE_TORQUE,
  DEMAND_MODE_AIRFLOW,
};

// externally accessible functions
void MotorDemandMux_ModbusUpdate(uint16_t commanded_speed, uint16_t commanded_demand, uint16_t commanded_start, uint16_t demand_source, uint16_t direction);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_MOTOR_DEMAND_MUX_H_ */
