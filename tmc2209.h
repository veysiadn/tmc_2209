/**
******************************************************************************
* @file           : tmc2209_c.h
* @author         : Veysi Adin (veysi.adin@outlook.com)
* @brief          : This file contains the functions prototypes for the
* TMC2209 stepper driver. For detailed explanation of the functions please
* refer to the TMC2209 datasheet and  calcuLation spreadsheet.
*
* @note The calculation spreadsheet allows proper settings of parameters for the stepper driver.
* You can download the calculations spreadsheet from the following link:
* Reference for calculations:
* https://www.analog.com/media/en/engineering-tools/design-tools/TMC220x_TMC222x_Calculations.xlsx
* Take a look at the spreadsheet and the datasheet to understand the calculations.
* Datasheet : https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf
* Additionally if you want to use this driver you need to implement the
* following functions: Example implementation is available in README.md file.
* void tmc2209_set_hardware_enable_pin(tmc2209_stepper_driver_t *stepper_driver, uint8_t hardware_enable_pin);
* void tmc2209_enable(tmc2209_stepper_driver_t *stepper_driver);
* void tmc2209_disable(tmc2209_stepper_driver_t *stepper_driver);
* void tmc2209_write(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address, uint32_t data);
* uint32_t tmc2209_read(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address);

* @version        : 0.7
* @date           : 2023-12-04
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

#ifndef INC_TMC2209_C_H_
#define INC_TMC2209_C_H_

/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "tmc2209_defines.h"
/* Private defines -----------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef union
{
  struct
  {
    uint32_t semin : 4;
    uint32_t reserved_0 : 1;
    uint32_t seup : 2;
    uint32_t reserved_1 : 1;
    uint32_t semax : 4;
    uint32_t reserved_2 : 1;
    uint32_t sedn : 2;
    uint32_t seimin : 1;
    uint32_t reserved_3 : 16;
  };
  uint32_t bytes;
} cool_step_config_t;

typedef union
{
  struct
  {
    uint32_t toff : 4;
    uint32_t hstart : 3;
    uint32_t hend : 4;
    uint32_t reserved_0 : 4;
    uint32_t tbl : 2;
    uint32_t vsense : 1;
    uint32_t reserved_1 : 6;
    uint32_t mres : 4;
    uint32_t interpolation : 1;
    uint32_t double_edge : 1;
    uint32_t diss2g : 1;
    uint32_t diss2vs : 1;
  };
  uint32_t bytes;
} chopper_config_t;

typedef union
{
  struct
  {
    uint64_t sync : 4;
    uint64_t reserved : 4;
    uint64_t serial_address : 8;
    uint64_t register_address : 7;
    uint64_t rw : 1;
    uint64_t data : 32;
    uint64_t crc : 8;
  };
  uint64_t bytes;
} write_read_reply_datagram_t;

typedef union
{
  struct
  {
    uint32_t i_scale_analog : 1;
    uint32_t internal_rsense : 1;
    uint32_t enable_spread_cycle : 1;
    uint32_t shaft : 1;
    uint32_t index_otpw : 1;
    uint32_t index_step : 1;
    uint32_t pdn_disable : 1;
    uint32_t mstep_reg_select : 1;
    uint32_t multistep_filt : 1;
    uint32_t test_mode : 1;
    uint32_t reserved : 22;
  };
  uint32_t bytes;
} global_config_t;

typedef union
{
  struct
  {
    uint32_t pwm_offset : 8;
    uint32_t pwm_grad : 8;
    uint32_t pwm_freq : 2;
    uint32_t pwm_autoscale : 1;
    uint32_t pwm_autograd : 1;
    uint32_t freewheel : 2;
    uint32_t reserved : 2;
    uint32_t pwm_reg : 4;
    uint32_t pwm_lim : 4;
  };
  uint32_t bytes;
} pwm_config_t;

typedef union
{
  struct
  {
    uint32_t pwm_offset_auto : 8;
    uint32_t reserved_0 : 8;
    uint32_t pwm_gradient_auto : 8;
    uint32_t reserved_1 : 8;
  };
  uint32_t bytes;
} pwm_auto_t;

typedef struct
{
  uint32_t over_temperature_warning : 1;
  uint32_t over_temperature_shutdown : 1;
  uint32_t short_to_ground_a : 1;
  uint32_t short_to_ground_b : 1;
  uint32_t low_side_short_a : 1;
  uint32_t low_side_short_b : 1;
  uint32_t open_load_a : 1;
  uint32_t open_load_b : 1;
  uint32_t over_temperature_120c : 1;
  uint32_t over_temperature_143c : 1;
  uint32_t over_temperature_150c : 1;
  uint32_t over_temperature_157c : 1;
  uint32_t reserved0 : 4;
  uint32_t current_scaling : 5;
  uint32_t reserved1 : 9;
  uint32_t stealth_chop_mode : 1;
  uint32_t standstill : 1;
} tmc2209_status_t;

typedef struct
{
  bool     is_communicating;
  bool     is_setup;
  bool     software_enabled;
  uint16_t microsteps_per_step;
  bool     inverse_motor_direction_enabled;
  bool     stealth_chop_enabled;
  uint8_t  standstill_mode;
  uint8_t  irun_percent;
  uint8_t  irun_register_value;
  uint8_t  ihold_percent;
  uint8_t  ihold_register_value;
  uint8_t  iholddelay_percent;
  uint8_t  iholddelay_register_value;
  bool     automatic_current_scaling_enabled;
  bool     automatic_gradient_adaptation_enabled;
  uint8_t  pwm_offset;
  uint8_t  pwm_gradient;
  bool     cool_step_enabled;
  bool     analog_current_scaling_enabled;
  bool     internal_sense_resistors_enabled;
} tmc2209_settings_t;

typedef union
{
  struct
  {
    tmc2209_status_t status;
  };
  uint32_t bytes;
} tmc2209_drive_status_t;

typedef enum
{
  CURRENT_INCREMENT_1 = 0,
  CURRENT_INCREMENT_2 = 1,
  CURRENT_INCREMENT_4 = 2,
  CURRENT_INCREMENT_8 = 3,
} current_increment_t;

typedef enum
{
  MEASUREMENT_COUNT_32 = 0,
  MEASUREMENT_COUNT_8  = 1,
  MEASUREMENT_COUNT_2  = 2,
  MEASUREMENT_COUNT_1  = 3,
} measurement_count_t;

typedef union
{
  struct
  {
    uint32_t reset : 1;
    uint32_t drv_err : 1;
    uint32_t uv_cp : 1;
    uint32_t reserved : 29;
  };
  uint32_t bytes;
} tmc2209_global_status_t;

typedef union
{
  struct
  {
    uint32_t enn : 1;
    uint32_t reserved_0 : 1;
    uint32_t ms1 : 1;
    uint32_t ms2 : 1;
    uint32_t diag : 1;
    uint32_t reserved_1 : 1;
    uint32_t pdn_serial : 1;
    uint32_t step : 1;
    uint32_t spread_en : 1;
    uint32_t dir : 1;
    uint32_t reserved_2 : 14;
    uint32_t version : 8;
  };
  uint32_t bytes;
} tmc2209_input_t;

typedef union
{
  struct
  {
    uint32_t ihold : 5;
    uint32_t reserved_0 : 3;
    uint32_t irun : 5;
    uint32_t reserved_1 : 3;
    uint32_t iholddelay : 4;
    uint32_t reserved_2 : 12;
  };
  uint32_t bytes;
} tmc2209_driver_current_t;

typedef union
{
  struct
  {
    uint32_t sync : 4;
    uint32_t reserved : 4;
    uint32_t serial_address : 8;
    uint32_t register_address : 7;
    uint32_t rw : 1;
    uint32_t crc : 8;
  };
  uint32_t bytes;
} read_request_datagram_t;

typedef union
{
  struct
  {
    uint32_t reserved_0 : 8;
    uint32_t replydelay : 4;
    uint32_t reserved_1 : 20;
  };
  uint32_t bytes;
} tmc2209_reply_delay_t;

typedef union
{
  struct
  {
    uint32_t pwm_scale_sum : 8;
    uint32_t reserved_0 : 8;
    uint32_t pwm_scale_auto : 9;
    uint32_t reserved_1 : 7;
  };
  uint32_t bytes;
} tmc2209_pwm_scale_t;

typedef enum
{
  SERIAL_ADDRESS_0 = 0,
  SERIAL_ADDRESS_1 = 1,
  SERIAL_ADDRESS_2 = 2,
  SERIAL_ADDRESS_3 = 3,
} tmc2209_serial_address_t;

typedef enum
{
  TMC_NORMAL         = 0,
  TMC_FREEWHEELING   = 1,
  TMC_STRONG_BRAKING = 2,
  TMC_BRAKING        = 3,
} tmc2209_stand_still_mode_t;

typedef struct
{
  uint32_t                 serial_baud_rate_;
  uint8_t                  serial_address_;
  int16_t                  hardware_enable_pin_;
  uint8_t                  toff_;
  chopper_config_t         chopper_config_;
  pwm_config_t             pwm_config_;
  cool_step_config_t       cool_config_;
  bool                     cool_step_enabled_;
  tmc2209_driver_current_t driver_current_;
  global_config_t          global_config_;

} tmc2209_stepper_driver_t;
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Initialize the tmc2209_stepper_driver_t stepper driver with the given serial baud rate and address.
 * @note The address is determined by MS1 and MS2 pins. The address is 0 if both pins are low, 1 if MS1 is high and MS2 is low, 2 if MS1 is
 * low and MS2 is high, and 3 if both pins are high.
 *
 * @param stepper_driver Pointer to the tmc2209_stepper_driver_t struct.
 * @param serial_baud_rate The serial baud rate.
 * @param serial_address The serial address @see the note.
 */
void tmc2209_initialize(tmc2209_stepper_driver_t *stepper_driver, long serial_baud_rate, tmc2209_serial_address_t serial_address);

/**
 * @brief Initialize the tmc2209_stepper_driver_t stepper driver with default values.
 *
 * @param stepper_driver
 * @param serial_baud_rate
 * @param serial_address
 */
void tmc2209_setup(tmc2209_stepper_driver_t *stepper_driver, long serial_baud_rate, tmc2209_serial_address_t serial_address);

/**
 * @brief Set the hardware enable pin for the stepper driver.
 *
 * @param stepper_driver
 * @param hardware_enable_pin
 */
void tmc2209_set_hardware_enable_pin(tmc2209_stepper_driver_t *stepper_driver, uint8_t hardware_enable_pin) __attribute__((weak));
/**
 * @brief Enable the stepper driver, this enables motor by setting the ENN pin to LOW using HAL interface, also enables the driver using
 * serial interface.
 * @todo Convert this function to a function pointer so that interface will be implemented by the user.
 * @param stepper_driver
 */
void tmc2209_enable(tmc2209_stepper_driver_t *stepper_driver) __attribute__((weak));
/**
 * @brief Disable the stepper driver, this disables motor by setting the ENN pin to HIGH using HAL interface, also disables the driver using
 * serial interface.
 * @todo Convert this function to a function pointer so that interface will be implemented by the user.
 * @param stepper_driver
 */
void tmc2209_disable(tmc2209_stepper_driver_t *stepper_driver) __attribute__((weak));

/**
 * @brief Write the given data to the given register address.
 *
 * @param stepper_driver
 * @param register_address
 * @param data
 */
void tmc2209_write(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address, uint32_t data) __attribute__((weak));
/**
 * @brief Read the data from the given register address.
 *
 * @param stepper_driver
 * @param register_address
 * @return uint32_t
 */
uint32_t tmc2209_read(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address) __attribute__((weak));

/**
 * @brief Set the micro steps per step for the stepper driver.
 *
 * @param stepper_driver
 * @param microsteps_per_step The micro steps per step ranging from 1 to 256, value must be power of two.
 */
void set_micro_steps_per_step(tmc2209_stepper_driver_t *stepper_driver, uint16_t microsteps_per_step);
/**
 * @brief Helper function to convert the micro steps per step to the exponent value.
 *
 * @param stepper_driver
 * @param exponent
 */
void set_micro_steps_per_step_power_of_two(tmc2209_stepper_driver_t *stepper_driver, uint8_t exponent);
/**
 * @brief Set the run current for the stepper driver. This value is used to determine the current to be used while the motor is moving.
 * @note The current value will be the given percent of the maximum current. The maximum current is determined by the VREF voltage and
 * the sense resistor value.
 *
 * @param stepper_driver
 * @param percent 0-100 percent to be scaled to 0-31 range.
 */
void set_run_current_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent);
/**
 * @brief Set the hold current percentage in stand still mode for the stepper driver. This value is used to determine the current to be
 * used while the motor is not moving.
 *
 * @param stepper_driver
 * @param percent 0-100 percent to be scaled to 0-31 range.
 */
void set_hold_current_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent);
/**
 * @brief Set the hold delay for the stepper driver, this value controls the number of clock cycles for switching to hold current after the
 * motor stops moving. This function scales the given percent to 0-15 range. Depending on the difference between run current and hold
 * current, the hold delay value must be adjusted.
 * @note Set the hold delay to 0 if you want instant power down of the motor after the motor stops moving.
 *
 * @param stepper_driver
 * @param percent 0-100 percent to be scaled to 0-15 range.
 */
void set_hold_delay_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent);
/**
 * @brief Set the all current percentage values and hold delay percentage all at once for the stepper driver.
 *
 * @param stepper_driver
 * @param run_current_percent The run current percentage, 0-100 percent to be scaled to 0-31 range.
 * @param hold_current_percent The hold current percentage, 0-100 percent to be scaled to 0-31 range.
 * @param hold_delay_percent The hold delay percentage, 0-100 percent to be scaled to 0-15 range.
 */
void set_all_current_percent_values(tmc2209_stepper_driver_t *stepper_driver,
                                    uint8_t                   run_current_percent,
                                    uint8_t                   hold_current_percent,
                                    uint8_t                   hold_delay_percent);
/**
 * @brief Changes the direction of motor rotation to clockwise/counter-clockwise depending on gear orientation.
 * @note This change is done using UART interface.
 * @param stepper_driver
 */
void enable_inverse_motor_direction(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Changes the direction of motor rotation to counter-clockwise/clockwise depending on gear orientation.
 * @note This change is done using UART interface.
 * @param stepper_driver
 */
void disable_inverse_motor_direction(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Sets the stand still mode for the stepper driver.Stand still option when motor current setting is zero (I_HOLD=0).
 * Only available with StealthChop enabled. The freewheeling option makes the motor easy movable, while both coil short
 * options realize a passive brake.
 * @see tmc2209_stand_still_mode_t
 * @param stepper_driver
 * @param mode The stand still mode, possible values are TMC_NORMAL, TMC_FREEWHEELING, TMC_STRONG_BRAKING, TMC_BRAKING.
 */
void set_stand_still_mode(tmc2209_stepper_driver_t *stepper_driver, tmc2209_stand_still_mode_t mode);
/**
 * @brief Enables the automatic current scaling for the stepper driver. The driver measures the motor current during the chopper on time
 * and uses a proportional regulator to regulate PWM_SCALE_AUTO in order match the motor current to the target current.
 * PWM_REG is the proportionality coefficient for this regulator.
 * @see Datasheet Pg. 40, Section 6.3. Stealthchop current regulator section.
 * @param stepper_driver
 */
void enable_automatic_current_scaling(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Disables the automatic current scaling for the stepper driver.
 * @note User defined feed forward PWM amplitude will be used. The current settings IRUN and IHOLD are not enforced by regulation but scale
 * the PWM amplitude, only!The resulting PWM amplitude (limited to 0…255) is:
 * PWM_OFS * ((CS_ACTUAL+1) / 32) + PWM_GRAD * 256 / TSTEP
 *
 * @param stepper_driver
 */
void disable_automatic_current_scaling(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Enables the automatic gradient adaptation for the stepper driver. Automatic tuning (only with pwm_autoscale=1)
 * PWM_GRAD_AUTO is initialized with PWM_GRAD and becomes optimized automatically during motion.
 *
 * @param stepper_driver
 */
void enable_automatic_gradient_adaptation(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Disable the automatic gradient adaptation for the stepper driver.Fixed value for PWM_GRAD
 *(PWM_GRAD_AUTO = PWM_GRAD)
 * @see Datasheet Pg. 35 Section 5.5.2. PWMCONF register.
 * @param stepper_driver
 */
void disable_automatic_gradient_adaptation(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Set the pwm offset for the stepper driver. User defined PWM amplitude offset (0-255) related to full motor current (CS_ACTUAL=31)
 * in stand still. (Reset default=36)
 * When using automatic scaling (pwm_autoscale=1) the value is used for initialization, only. The autoscale function starts with
 * PWM_SCALE_AUTO=PWM_OFS and finds the required offset to yield the target current automatically.
 * @see Datasheet Pg. 36 Section 5.5.2. PWMCONF register.
 * @param stepper_driver
 * @param pwm_amplitude
 */
void set_pwm_offset(tmc2209_stepper_driver_t *stepper_driver, uint8_t pwm_amplitude);
/**
 * @brief Set the pwm gradient for the stepper driver. Velocity dependent gradient for PWM amplitude:
 * PWM_GRAD * 256 / TSTEP This value is added to PWM_AMPL to compensate for the velocity-dependent motor back-EMF. 0…((2^8)-1) * 2^18 tCLK
 * @param stepper_driver
 * @param pwm_amplitude
 */
void set_pwm_gradient(tmc2209_stepper_driver_t *stepper_driver, uint8_t pwm_amplitude);
/**
 * @brief Sets the delay time from stand still (stst) detection to motor current power
 * down. Time range is about 0 to 5.6 seconds.
 * @note Attention: A minimum setting of 2 is required to allow automatic tuning of StealthChop PWM_OFFS_AUTO.
 * @param stepper_driver
 * @param power_down_delay
 */
void set_power_down_delay(tmc2209_stepper_driver_t *stepper_driver, uint8_t power_down_delay);
/**
 * @brief Set the reply delay for uart communication with the stepper driver. SENDDELAY for read access (time until reply is sent) from the
 * stepper driver to the host. 0…15: 0…15 * 8 bit times.
 * @see Datasheet Pg. 24 Section 5.1. General configuration register.
 *
 * @param stepper_driver
 * @param delay
 */
void set_reply_delay(tmc2209_stepper_driver_t *stepper_driver, uint8_t delay);
/**
 * @brief Move the motor at the given velocity, assigned value allows moving the motor by UART control.
 * It gives the motor velocity in +-(2^23)-1 [μsteps / t]
 * @note 0 value is interpreted as normal operation. Driver reacts to STEP input. Additionally, the sign of the velocity value determines
 * the direction of the motor rotation.
 * @param stepper_driver
 * @param microsteps_per_period
 */
void move_at_velocity(tmc2209_stepper_driver_t *stepper_driver, int32_t microsteps_per_period);
/**
 * @brief Move the motor using Step/Dir interface pins - Legacy mode.
 *
 * @param stepper_driver
 */
void move_using_step_dir_interface(tmc2209_stepper_driver_t *stepper_driver);

/**
 * @brief Enable the stealth chop mode for the stepper driver, and disables the spread cycle mode.
 * In general, spreadCycle mode provides greater torque and greater positional accuracy than stealthChop mode. However, stealthChop mode may
 * produce significantly lower audible noise on some applications.
 * @note This mode works only with slow RPMs <800RPM.
 *
 * @param stepper_driver
 */
void enable_stealth_chop(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Disable the stealth chop mode and enable the spread cycle mode for the stepper driver.
 * In general, spreadCycle mode provides greater torque and greater positional accuracy than stealthChop mode. However, stealthChop mode may
 * produce significantly lower audible noise on some applications.
 * @param stepper_driver
 */
void disable_stealth_chop(tmc2209_stepper_driver_t *stepper_driver);

/**
 * @brief Set the stealth chop duration threshold for the stepper driver. StealthChop PWM mode is enabled, if configured
 * When the velocity exceeds the limit set by the threshold value, the driver switches to SpreadCycle mode.
 *
 * @param stepper_driver
 * @param duration_threshold
 */
void set_stealth_chop_duration_threshold(tmc2209_stepper_driver_t *stepper_driver, uint32_t duration_threshold);

/**
 * @brief Set the stall guard threshold for sensorless homing.Detection threshold for stall. The StallGuard value SG_RESULT becomes compared
 * to the double of this threshold. A stall is signaled with SG_RESULT ≤ SGTHRS*2
 * @note The stall guard threshold value must be found experimentally.
 *
 * @param stepper_driver
 * @param stall_guard_threshold
 */
void set_stall_guard_threshold(tmc2209_stepper_driver_t *stepper_driver, uint8_t stall_guard_threshold);
/**
 * @brief Set the pwm frequency for the stepper driver. PWM frequency selection. Use the lowest setting giving good results. The frequency
 *  measured at each of the chopper outputs is half of the effective chopper frequency fPWM.
 *
 * @param stepper_driver
 * @param pwm_freq 0-3 for 3-58.82kHz, 2-46.51kHz, 1-35.15kHz, 0-23.44kHz respectively.
 */
void set_pwm_frequency(tmc2209_stepper_driver_t *stepper_driver, uint8_t pwm_freq);
/**
 * @brief Enable the cool step mode for the stepper driver. This mode reduces the current when the motor is not moving and
 * adapts the current to the load. CoolStep is an automatic smart energy optimization for stepper motors based on the motor mechanical load.
 * @see Datasheet Pg. 60 Section 12.2 Setting up for CoolStep.
 * @param stepper_driver
 * @param lower_threshold SEMIN, 4-bit unsigned integer that sets a lower threshold. If SG_RESULT goes below this threshold, CoolStep
 * increases the current to both coils. The 4-bit SEMIN value is scaled by 32 to cover the lower half of the range of the 10-bit SG value.
 * @param upper_threshold SEMAX, 4-bit unsigned integer that controls an upper threshold. If SG is sampled equal to or above this threshold
 * enough times, CoolStep decreases the current to both coils. The upper threshold is (SEMIN + SEMAX + 1)*32.
 */
void enable_cool_step(tmc2209_stepper_driver_t *stepper_driver, uint8_t lower_threshold, uint8_t upper_threshold);

/**
 * @brief Disable the cool step mode for the stepper driver.
 *
 * @param stepper_driver
 */
void disable_cool_step(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Set the cool step current increment while adapting the current to the load.
 *
 * @param stepper_driver
 * @param current_increment
 */
void set_cool_step_current_increment(tmc2209_stepper_driver_t *stepper_driver, current_increment_t current_increment);
/**
 * @brief Set the cool step measurement count, this value determines the number of stall guard measurements to be taken before
 * adapting the current to the load.
 *
 * @param stepper_driver
 * @param measurement_count
 */
void set_cool_step_measurement_count(tmc2209_stepper_driver_t *stepper_driver, measurement_count_t measurement_count);

/**
 * @brief Set the cool step velocity threshold.Lower velocity threshold for switching on
 * CoolStep and stall output. Below this velocity CoolStep becomes disabled
 *
 * @param stepper_driver
 * @param duration_threshold
 */
void set_cool_step_velocity_threshold(tmc2209_stepper_driver_t *stepper_driver, uint32_t duration_threshold);
/**
 * @brief Enable analog current scaling for the stepper driver. This feature is used to scale the current using a potentiometer.
 * Basically telling the driver to use voltage supplied to VREF as current reference. If disabled the driver will use the internal
 * reference derived from the VDD voltage.
 *
 * @param stepper_driver
 */
void enable_analog_current_scaling(tmc2209_stepper_driver_t *stepper_driver);

/**
 * @brief Disable analog current scaling for the stepper driver. The driver will use the internal reference derived from the VDD voltage.
 *
 * @param stepper_driver
 */
void disable_analog_current_scaling(tmc2209_stepper_driver_t *stepper_driver);

/**
 * @brief Use external sense resistor for the stepper driver and notificy the driver about the usage. Since the driver also has an internal
 * sense resistor, by default the driver is using internal sense resistor.
 * @note Use VREF voltage to calculate the current.
 *
 * @param stepper_driver
 */
void use_external_resistor(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Use internal sense resistor for the stepper driver and notificy the driver about the usage.
 * Use current supplied into VREF as reference for internal sense resistor. VREF pin internally is driven to GND in this mode.
 *
 * @param stepper_driver
 */
void use_internal_sense_resistor(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the version for the stepper driver.
 * version: 0x21=first version of the IC Identical numbers mean full digital compatibility.
 *
 * @param stepper_driver
 * @return uint8_t
 */
uint8_t get_version(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Check if the stepper driver is communicating by checking the version register and comparing it with 0x21.
 *
 * @param stepper_driver
 * @return true
 * @return false
 */
bool is_communicating(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Check if the stepper driver is setup and communicating.
 *
 * @param stepper_driver
 * @return true
 * @return false
 */
bool is_setup_and_communicating(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Check if the stepper driver is communicating without setup. It is important to setup the stepper driver before using it.
 * Otherwise you might experience unexpected behavior.
 *
 * @param stepper_driver
 * @return true
 * @return false
 */
bool is_communicating_without_setup(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Check if the stepper driver is disabled.
 *
 * @param stepper_driver
 * @return true
 * @return false
 */
bool hardware_disabled(tmc2209_stepper_driver_t *stepper_driver);

/**
 * @brief Get the microstep per step value set by the user.
 *
 * @param stepper_driver
 * @return uint16_t The microstep per step value.
 */
uint16_t get_microstep_per_step(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the assigned settings for the stepper driver.
 *
 * @param stepper_driver
 * @return tmc2209_settings_t
 */
tmc2209_settings_t get_settings(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the status for the stepper driver.
 *
 * @param stepper_driver
 * @return tmc2209_status_t
 */
tmc2209_status_t get_status(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Interface transmission counter. This register becomes incremented with each successful UART interface write access. Read out to
 * check the serial transmission for lost data. Read accesses do not change the content. The counter wraps around from 255 to 0.
 *
 * @param stepper_driver
 * @return uint8_t
 */
uint8_t get_interface_transmission_counter(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the duration between two steps for the stepper driver. Actual measured time between two 1/256 microsteps derived from the step
 * input frequency in units of 1/fCLK. Measured value is (2^20)-1 in case of overflow or stand still. TSTEP always relates to 1/256 step,
 * independent of the actual MRES.
 *
 * @param stepper_driver
 * @return uint32_t
 */
uint32_t get_interstep_duration(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the stall guard result for sensorless homing. StallGuard result. SG_RESULT becomes updated with each fullstep, independent of
 * TCOOLTHRS and SGTHRS. This is the StallGuard4 result. A higher reading indicates less mechanical load. A lower reading indicates a higher
 * load and thus a higher load angle. Range 0-510.
 *
 * @param stepper_driver
 * @return uint16_t
 */
uint16_t get_stall_guard_result(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the pwm scale sum. Information about the motor state is available with automatic scaling by reading out PWM_SCALE_SUM. As this
 * parameter reflects the actual voltage required to drive the target current into the motor, it depends on several factors: motor load,
 * coil resistance, supply voltage, and current setting. Therefore, an evaluation of the PWM_SCALE_SUM value allows checking the motor
 * operation point.
 *
 * @param stepper_driver
 * @return uint8_t
 */
uint8_t get_pwm_scale_sum(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the pwm scale auto. Results of StealthChop amplitude regulator. These values can be used to monitor automatic PWM amplitude
 * scaling (255=max. voltage).
 *
 * @param stepper_driver
 * @return int16_t
 */
int16_t get_pwm_scale_auto(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the pwm offset auto value. This function gets the automatically determined offset value.
 * @note These automatically generated values can be read out in order to determine a default / power up setting for
 * PWM_GRAD and PWM_OFS.
 * @param stepper_driver
 * @return uint8_t
 */
uint8_t get_pwm_offset_auto(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the pwm gradient auto value. This function gets the automatically determined gradient value.
 * @note These automatically generated values can be read out in order to determine a default / power up setting for
 * PWM_GRAD and PWM_OFS.
 * @param stepper_driver
 * @return uint8_t
 */
uint8_t get_pwm_gradient_auto(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Get the microstep counter. Microstep counter. Indicates actual position in the microstep table for CUR_A. CUR_B uses an offset of
 * 256 into the table. Reading out MSCNT allows determination of the motor position within the electrical wave.
 *
 * @param stepper_driver
 * @return uint16_t
 */
uint16_t get_microstep_counter(tmc2209_stepper_driver_t *stepper_driver);

/*---------------------------------------------*/
/**
 * @brief Helper function to calculate the crc for the given datagram for writing to the stepper driver.
 *
 * @param stepper_driver
 * @param datagram
 * @param datagram_size
 * @return uint8_t
 */
uint8_t calculate_crc_write(tmc2209_stepper_driver_t *stepper_driver, write_read_reply_datagram_t *datagram, uint8_t datagram_size);

/**
 * @brief Helper function to calculate the crc for the given datagram for reading from the stepper driver.
 *
 * @param stepper_driver
 * @param datagram
 * @param datagram_size
 * @return uint8_t
 */
uint8_t calculate_crc_read(tmc2209_stepper_driver_t *stepper_driver, read_request_datagram_t *datagram, uint8_t datagram_size);

/**
 * @brief Set the operation mode to serial, this is used if you want to dynamically switch between step/dir and uart interface.
 *
 * @param stepper_driver
 * @param serial_address
 */
void set_operation_mode_to_serial(tmc2209_stepper_driver_t *stepper_driver, tmc2209_serial_address_t serial_address);

/**
 * @brief Set the registers to default values, the default values are factory settings.
 * @see Registers in the datasheet for checking default values.
 *
 * @param stepper_driver
 */
void set_registers_to_default(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Read and store all register for the stepper driver.
 *
 * @param stepper_driver
 */
void read_and_store_registers(tmc2209_stepper_driver_t *stepper_driver);

/**
 * @brief Check the operation mode for the stepper driver.
 *
 * @param stepper_driver
 * @return true
 * @return false
 */
bool serial_operation_mode(tmc2209_stepper_driver_t *stepper_driver);

/**
 * @brief Minimize the motor current to prevent overheating. This function sets the run and hold current values to the minumum, which is 0%.
 *
 * @param stepper_driver
 */
void minimize_motor_current(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Helper function to reverse data while reading/writing to the stepper driver.
 *
 * @param stepper_driver
 * @param data
 * @return uint32_t
 */
uint32_t reverse_data(tmc2209_stepper_driver_t *stepper_driver, uint32_t data);

/**
 * @brief Helper function to convert percent to current settings.
 *
 * @param stepper_driver
 * @param percent
 * @return uint8_t
 */
uint8_t percent_to_current_settings(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent);
/**
 * @brief Helper function to convert current settings to percent.
 *
 * @param stepper_driver
 * @param current_setting
 * @return uint8_t
 */
uint8_t current_setting_to_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t current_setting);
/**
 * @brief Helper function to convert percent to hold delay setting.
 *
 * @param stepper_driver
 * @param percent
 * @return uint8_t
 */
uint8_t percent_to_hold_delay_setting(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent);
/**
 * @brief Helper function to convert hold delay setting to percent.
 *
 * @param stepper_driver
 * @param hold_delay_setting
 * @return uint8_t
 */
uint8_t hold_delay_setting_to_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t hold_delay_setting);

/**
 * @brief Writes the data to the global configuration register.
 *
 * @param stepper_driver
 */
void write_stored_global_config(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Reads the global configuration register from the stepper driver.
 *
 * @param stepper_driver
 * @return uint32_t
 */
uint32_t read_global_config_bytes(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Writes the stored driver current to the stepper driver.
 *
 * @param stepper_driver
 */
void write_stored_driver_current(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Writes the stored pwm config to the stepper driver.
 *
 * @param stepper_driver
 */
void write_stored_chopper_config(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Reads  chopper config from the stepper driver.
 *
 * @param stepper_driver
 * @return uint32_t
 */
uint32_t read_chopper_config_bytes(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Writes the stored pwm config to the stepper driver.
 *
 * @param stepper_driver
 */
void write_stored_pwm_config(tmc2209_stepper_driver_t *stepper_driver);
/**
 * @brief Reads the pwm config from the stepper driver.
 *
 * @param stepper_driver
 * @return uint32_t
 */
uint32_t read_pwm_config_bytes(tmc2209_stepper_driver_t *stepper_driver);

#endif /* INC_TMC2209_C_H_ */
