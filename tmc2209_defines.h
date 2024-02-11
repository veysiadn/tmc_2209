/**
******************************************************************************
* @file           : tmc2209_defines.h
* @author         : Veysi Adin (veysi.adin@outlook.com)
* @brief          : TMC2209 stepper driver register definitions.
*
* @version        : 0.9
* @date           : 2024-01-02
*
******************************************************************************
* @attention

* Copyright (c) 2022 Veysi Adin.
* All rights reserved.

* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.

******************************************************************************
*/

#ifndef INC_TMC2209_DEFINES_H_
#define INC_TMC2209_DEFINES_H_

/* Private includes ----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

#define REPLY_DELAY_MAX 15

#define BYTE_MAX_VALUE 0xFF
#define BITS_PER_BYTE  8

#define ECHO_DELAY_INC_MICROSECONDS 1
#define ECHO_DELAY_MAX_MICROSECONDS 4000

#define REPLY_DELAY_INC_MICROSECONDS 1
#define REPLY_DELAY_MAX_MICROSECONDS 10000

#define STEPPER_DRIVER_FEATURE_OFF 0
#define STEPPER_DRIVER_FEATURE_ON  1

#define WRITE_READ_REPLY_DATAGRAM_SIZE 8
#define DATA_SIZE                      4

#define SYNC                      0b101
#define RW_READ                   0
#define RW_WRITE                  1
#define READ_REPLY_SERIAL_ADDRESS 0b11111111

#define READ_REQUEST_DATAGRAM_SIZE 4

#define ADDRESS_GCONF 0x00

#define ADDRESS_GSTAT 0x01
#define ADDRESS_IFCNT 0x02

#define ADDRESS_REPLYDELAY 0x03

#define ADDRESS_IOIN 0x06

#define VERSION 0x21

#define ADDRESS_IHOLD_IRUN 0x10

#define PERCENT_MIN         0
#define PERCENT_MAX         100
#define CURRENT_SETTING_MIN 0
#define CURRENT_SETTING_MAX 31
#define HOLD_DELAY_MIN      0
#define HOLD_DELAY_MAX      15
#define IHOLD_DEFAULT       16
#define IRUN_DEFAULT        31
#define IHOLDDELAY_DEFAULT  1

#define ADDRESS_TPOWERDOWN 0x11
#define TPOWERDOWN_DEFAULT 20

#define ADDRESS_TSTEP 0x12

#define ADDRESS_TPWMTHRS 0x13
#define TPWMTHRS_DEFAULT 0

#define ADDRESS_VACTUAL            0x22
#define VACTUAL_DEFAULT            0
#define VACTUAL_STEP_DIR_INTERFACE 0

#define ADDRESS_TCOOLTHRS 0x14
#define TCOOLTHRS_DEFAULT 0
#define ADDRESS_SGTHRS    0x40
#define SGTHRS_DEFAULT    0
#define ADDRESS_SG_RESULT 0x41

#define ADDRESS_COOLCONF           0x42
#define COOLCONF_DEFAULT           0
#define SEIMIN_UPPER_CURRENT_LIMIT 20
#define SEIMIN_LOWER_SETTING       0
#define SEIMIN_UPPER_SETTING       1
#define SEMIN_OFF                  0
#define SEMIN_MIN                  1
#define SEMIN_MAX                  15
#define SEMAX_MIN                  0
#define SEMAX_MAX                  15

#define ADDRESS_MSCNT    0x6A
#define ADDRESS_MSCURACT 0x6B

#define ADDRESS_CHOPCONF       0x6C
#define CHOPPER_CONFIG_DEFAULT 0x10000053
#define TBL_DEFAULT            0b10
#define HEND_DEFAULT           0
#define HSTART_DEFAULT         5
#define TOFF_DEFAULT           3
#define TOFF_DISABLE           0
#define MRES_256               0b0000
#define MRES_128               0b0001
#define MRES_064               0b0010
#define MRES_032               0b0011
#define MRES_016               0b0100
#define MRES_008               0b0101
#define MRES_004               0b0110
#define MRES_002               0b0111
#define MRES_001               0b1000

#define MICROSTEPS_PER_STEP_MIN 1
#define MICROSTEPS_PER_STEP_MAX 256

#define ADDRESS_DRV_STATUS 0x6F

#define PWM_CONFIG_DEFAULT 0xC10D0024
#define PWM_OFFSET_MIN     0
#define PWM_OFFSET_MAX     255
#define PWM_OFFSET_DEFAULT 0x24
#define PWM_GRAD_MIN       0
#define PWM_GRAD_MAX       255
#define PWM_GRAD_DEFAULT   0x14

#define ADDRESS_PWM_SCALE 0x71

#define ADDRESS_PWM_AUTO 0x72

#define ADDRESS_PWMCONF 0x70

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

#endif /* INC_TMC2209_DEFINES_H_ */
