/**
  ********************************************************************************************************************************
  * @file    module_usart2.h 
  * @author  Pamela Lee
  * @brief   Main driver module for USART2 Communication.
  * @details This module initializes the USART2 port and attaches the pre-selected fixed memory allocation to the module.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_USART2_H_
#define _MODULE_USART2_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
enum {
  BUS_VOLTS_CMD       = 0x40, // Read bus volts
  MOTOR_STATUS_CMD    = 0x41, // Motor status
  MOTOR_DIR_CMD       = 0x42, // Motor direction
  BULK_MONITORING_CMD   = 0x4D, // Many Settings
  HEART_BEAT_CMD      = 0x4E, // Heart beat
  ELECTRICAL_INFO_CMD = 0x4F, // Motor volts, phase current & power
  MEASURED_SPEED_CMD  = 0x60, // Measrued speed
  MEASURED_TORQUE_CMD = 0x61, // Measured Torque
  THERMO_MECHANICAL_INFO_CMD = 0x6F, // Measured torque & temperature
  SINGLE_REG_READ_CMD = 0x07, // Read single regiser
  //REG_STATUS_CMD      = 0x72, 
  MULT_DATA_REQ1_CMD  = 0xB0, // Read multiple commands 1
  MULT_DATA_REQ2_CMD  = 0xB1, // Read multiple commands 2
};



#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_USART2_H_ */