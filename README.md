# TMC2209 Stepper Driver Driver

## Overview
This module is a driver for the TMC2209 step motor driver IC. It provides an interface for controlling step motors, includes UART based control and STEP/DIR based control (Legacy Mode). 
UART interface allows : 
+ Detailed diagnostics and thermal management
+ Passive braking and freewheeling for flexible, lowest power stop modes
+ More options for microstep resolution setting (fullstep to 256 microstep)
+ Software controlled motor current setting and more chopper options
+ Use StallGuard for sensorless homing and CoolStep for adaptive motor current and cool motor
+ This mode allows replacing all control lines like ENN, DIAG, INDEX, MS1, MS2, and analog current
setting VREF by a single interface line. 
+ This way, only three signals are required for full control: STEP,
DIR and PDN_UART. 
+ Even motion without external STEP pulses is provided by an internal programmable step pulse generator: Just set the desired motor velocity. However, no ramping is
provided by the TMC2209.

### HAL implementation

For porting the project to a new microcontroller, weak functions from [tmc2209.h](tmc2209.h) should be implemented.

#### Functions must be implemented: 

| Interface Functions                                                                                          | Description                                                     |
| :----------------------------------------------------------------------------------------------------------- | :-------------------------------------------------------------- |
| void tmc2209_set_hardware_enable_pin(tmc2209_stepper_driver_t *stepper_driver, uint8_t hardware_enable_pin); | Assigns hardware enable pin and disables it. (***Active Low***) |
| void tmc2209_disable(tmc2209_stepper_driver_t *stepper_driver);                                              | Disables the stepper driver.                                    |
| void tmc2209_enable(tmc2209_stepper_driver_t *stepper_driver);                                               | Enables the stepper driver                                      |
| void tmc2209_write(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address, uint32_t data);       | Write data to the given register address via UART interface     |
| uint32_t tmc2209_read(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address);                   | Read data from the given register address via UART interface    |
------------

### Some notes and Datasheet info.

- It is important to read the datasheet of the TMC2209 stepper driver IC before using this module. Datasheet can be found [here](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf).
- Additionally a spreadsheet for refernce calculations can be found [here](https://www.analog.com/media/en/engineering-tools/design-tools/TMC220x_TMC222x_Calculations.xlsx).

## License
This software is licensed under terms that can be found in the LICENSE file in the root directory of this software component. If no LICENSE file comes with this software, it is provided AS-IS.

## Example Interface Implementation

Note that example interface file can be found [here](tmc2209_if.c). Everything is commented out, you can use that file and modify it for your application.

```c

#include "tmc2209.h"
#include "tmc2209_defines.h"
#include "main.h"


extern UART_HandleTypeDef huart5;

void tmc2209_set_hardware_enable_pin(tmc2209_stepper_driver_t *stepper_driver, uint8_t hardware_enable_pin)
{
  stepper_driver->hardware_enable_pin_ = hardware_enable_pin;

  HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_SET);
}

void tmc2209_enable(tmc2209_stepper_driver_t *stepper_driver)
{
  if (stepper_driver->hardware_enable_pin_ >= 0)
  {
    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_RESET);
  }
  stepper_driver->toff_                = TOFF_DEFAULT;
  stepper_driver->chopper_config_.toff = TOFF_DEFAULT;
  write_stored_chopper_config(stepper_driver);
}

void tmc2209_disable(tmc2209_stepper_driver_t *stepper_driver)
{
  if (stepper_driver->hardware_enable_pin_ >= 0)
  {
    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_SET);
  }
  stepper_driver->chopper_config_.toff = TOFF_DISABLE;
  write_stored_chopper_config(stepper_driver);
}

void tmc2209_write(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address, uint32_t data)
{
  write_read_reply_datagram_t write_datagram;
  write_datagram.bytes            = 0;
  write_datagram.sync             = SYNC;
  write_datagram.serial_address   = stepper_driver->serial_address_;
  write_datagram.register_address = register_address;
  write_datagram.rw               = RW_WRITE;
  write_datagram.data             = reverse_data(stepper_driver, data);
  write_datagram.crc              = calculate_crc_write(stepper_driver, &write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

  uint8_t datagram_bytes[8];
  for (int i = 0; i < 8; i++)
  {
    datagram_bytes[i] = (write_datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
  }
  HAL_HalfDuplex_EnableTransmitter(&huart5);
  HAL_UART_Transmit(&huart5, datagram_bytes, WRITE_READ_REPLY_DATAGRAM_SIZE, 0XFFFF);
}

uint32_t tmc2209_read(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address)
{
  read_request_datagram_t read_request_datagram;
  read_request_datagram.bytes            = 0;
  read_request_datagram.sync             = SYNC;
  read_request_datagram.serial_address   = stepper_driver->serial_address_;
  read_request_datagram.register_address = register_address;
  read_request_datagram.rw               = RW_READ;
  read_request_datagram.crc              = calculate_crc_read(stepper_driver, &read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

  uint8_t datagram_bytes[WRITE_READ_REPLY_DATAGRAM_SIZE];
  for (int i = 0; i < READ_REQUEST_DATAGRAM_SIZE; ++i)
  {
    datagram_bytes[i] = (read_request_datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
  }
  HAL_HalfDuplex_EnableTransmitter(&huart5);
  HAL_UART_Transmit(&huart5, datagram_bytes, READ_REQUEST_DATAGRAM_SIZE, 0XFFFF);

  uint8_t                     byte       = 0;
  uint8_t                     byte_count = 0;
  write_read_reply_datagram_t read_reply_datagram;
  read_reply_datagram.bytes = 0;

  HAL_HalfDuplex_EnableReceiver(&huart5);
  for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i)
  {
    HAL_UART_Receive(&huart5, (uint8_t *)&byte, 1, 100);
    datagram_bytes[i] = byte;
  }
  for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i)
  {
    read_reply_datagram.bytes |= ((uint64_t)datagram_bytes[i] << (byte_count++ * BITS_PER_BYTE));
  }
  uint32_t reversed_data = reverse_data(stepper_driver, read_reply_datagram.data);
  uint8_t  crc           = calculate_crc_write(stepper_driver, &read_reply_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
  if (crc != read_reply_datagram.crc)
  {
  }
  return reversed_data;
}

```
## Example Usage
```c


#define INTERNAL_PWM_FREQUENCY_23KHZ 0 // Actual frequency is 23.44 kHz
#define INTERNAL_PWM_FREQUENCY_35KHZ 1 // Actual frequency is 35.15 kHz
#define INTERNAL_PWM_FREQUENCY_46KHZ 2 // Actual frequency is 46.51 kHz
#define INTERNAL_PWM_FREQUENCY_58KHZ 3 // Actual frequency is 58.82 kHz
#define STEPS_PER_REVOLUTION         48
#define STOP_VELOCITY                0

#define MICROSTEPS_PER_STEP     4
#define INITIAL_VELOCITY_RPM    500
#define MAX_TARGET_VELOCITY_RPM 3500
#define RUN_CURRENT_PERCENT     70
#define ACCELERATION            MICROSTEPS_PER_STEP * 100

const int32_t INITIAL_RUN_VELOCITY = INITIAL_VELOCITY_RPM * MICROSTEPS_PER_STEP;
const int32_t MAX_TARGET_VELOCITY  = MAX_TARGET_VELOCITY_RPM * MICROSTEPS_PER_STEP;



tmc2209_stepper_driver_t stepper_driver;
bool                     invert_direction = false;
extern TIM_HandleTypeDef htim3;

  tmc2209_setup(&stepper_driver, 115200, SERIAL_ADDRESS_0);
  tmc2209_set_hardware_enable_pin(&stepper_driver, MOT_EN_Pin);
  enable_cool_step(&stepper_driver, 1, 0);
  tmc2209_enable(&stepper_driver);
  set_micro_steps_per_step(&stepper_driver, MICROSTEPS_PER_STEP);
  set_pwm_frequency(&stepper_driver, INTERNAL_PWM_FREQUENCY_46KHZ);
  set_stand_still_mode(&stepper_driver, TMC_NORMAL);
  set_all_current_percent_values(&stepper_driver, RUN_CURRENT_PERCENT, 0, 0);
  enable_automatic_current_scaling(&stepper_driver);
  enable_stealth_chop(&stepper_driver);

  set_stealth_chop_duration_threshold(&stepper_driver, 9999999);
  while (1)
  {
    if (!HAL_GPIO_ReadPin(TEST_MOTOR_RIGHT_GPIO_Port, TEST_MOTOR_RIGHT_Pin))
    {

      tmc2209_enable(&stepper_driver);
      enable_inverse_motor_direction(&stepper_driver);

      current_velocity = tx_movement_ramp_up(MAX_TARGET_VELOCITY, ACCELERATION);
      while (!HAL_GPIO_ReadPin(TEST_MOTOR_RIGHT_GPIO_Port, TEST_MOTOR_RIGHT_Pin))
      {
        move_at_velocity(&stepper_driver, current_velocity);
        mstep_counter = get_microstep_counter(&stepper_driver);
        if (mstep_counter)
        {
        }
      }
    }
    else if (!HAL_GPIO_ReadPin(TEST_MOTOR_LEFT_GPIO_Port, TEST_MOTOR_LEFT_Pin))
    {
      tmc2209_enable(&stepper_driver);
      disable_inverse_motor_direction(&stepper_driver);

      current_velocity = tx_movement_ramp_up(MAX_TARGET_VELOCITY, ACCELERATION);
      while (!HAL_GPIO_ReadPin(TEST_MOTOR_LEFT_GPIO_Port, TEST_MOTOR_LEFT_Pin))
      {
        move_at_velocity(&stepper_driver, current_velocity);
      }
    }
    else
    {
      if (current_velocity > 0)
      {
        tx_movement_ramp_down(0, ACCELERATION);
        current_velocity = 0;
      }
      else
      {
        move_at_velocity(&stepper_driver, STOP_VELOCITY);
      }
      tmc2209_disable(&stepper_driver);
      HAL_Delay(1);
    }
  }
}

int32_t tx_movement_ramp_up(int32_t target_velocity, int32_t acceleration)
{
  move_at_velocity(&stepper_driver, INITIAL_RUN_VELOCITY);
  HAL_Delay(1);
  int32_t current_velocity = INITIAL_RUN_VELOCITY;
  while (current_velocity < target_velocity)
  {
    current_velocity += acceleration;
    move_at_velocity(&stepper_driver, current_velocity);
    HAL_Delay(1);
  }
  return current_velocity;
}

int32_t tx_movement_ramp_down(int32_t target_velocity, int32_t acceleration)
{
  move_at_velocity(&stepper_driver, MAX_TARGET_VELOCITY);
  HAL_Delay(1);
  int32_t current_velocity = MAX_TARGET_VELOCITY;
  while (current_velocity > target_velocity)
  {
    current_velocity -= acceleration;
    move_at_velocity(&stepper_driver, current_velocity);
    HAL_Delay(1);
  }
  return current_velocity;
}

```

## Author
Veysi Adin (veysi.adin@outlook.com)
