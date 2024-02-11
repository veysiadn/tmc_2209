#include "tmc2209.h"
// #include "main.h"

// extern UART_HandleTypeDef huart5;

void tmc2209_setup(tmc2209_stepper_driver_t *stepper_driver, long serial_baud_rate, tmc2209_serial_address_t serial_address)
{
  stepper_driver->serial_baud_rate_  = serial_baud_rate;
  stepper_driver->serial_address_    = serial_address;
  stepper_driver->cool_step_enabled_ = false;

  tmc2209_initialize(stepper_driver, serial_baud_rate, serial_address);
}

// unidirectional methods

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long constrain(long x, long a, long b)
{
  if (x < a)
  {
    return a;
  }
  else if (b < x)
  {
    return b;
  }
  else
  {
    return x;
  }
}

void set_micro_steps_per_step(tmc2209_stepper_driver_t *stepper_driver, uint16_t microsteps_per_step)
{
  uint16_t microsteps_per_step_shifted = constrain(microsteps_per_step, MICROSTEPS_PER_STEP_MIN, MICROSTEPS_PER_STEP_MAX);
  microsteps_per_step_shifted          = microsteps_per_step >> 1;
  uint16_t exponent                    = 0;
  while (microsteps_per_step_shifted > 0)
  {
    microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
    ++exponent;
  }
  set_micro_steps_per_step_power_of_two(stepper_driver, exponent);
}

void set_micro_steps_per_step_power_of_two(tmc2209_stepper_driver_t *stepper_driver, uint8_t exponent)
{
  switch (exponent)
  {
    case 0: {
      stepper_driver->chopper_config_.mres = MRES_001;
      break;
    }
    case 1: {
      stepper_driver->chopper_config_.mres = MRES_002;
      break;
    }
    case 2: {
      stepper_driver->chopper_config_.mres = MRES_004;
      break;
    }
    case 3: {
      stepper_driver->chopper_config_.mres = MRES_008;
      break;
    }
    case 4: {
      stepper_driver->chopper_config_.mres = MRES_016;
      break;
    }
    case 5: {
      stepper_driver->chopper_config_.mres = MRES_032;
      break;
    }
    case 6: {
      stepper_driver->chopper_config_.mres = MRES_064;
      break;
    }
    case 7: {
      stepper_driver->chopper_config_.mres = MRES_128;
      break;
    }
    case 8:
    default: {
      stepper_driver->chopper_config_.mres = MRES_256;
      break;
    }
  }
  write_stored_chopper_config(stepper_driver);
}

void set_run_current_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent)
{
  uint8_t run_current                  = percent_to_current_settings(stepper_driver, percent);
  stepper_driver->driver_current_.irun = run_current;
  write_stored_driver_current(stepper_driver);
}

void set_hold_current_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent)
{
  uint8_t hold_current = percent_to_current_settings(stepper_driver, percent);

  stepper_driver->driver_current_.ihold = hold_current;
  write_stored_driver_current(stepper_driver);
}

void set_hold_delay_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent)
{
  uint8_t hold_delay = percent_to_hold_delay_setting(stepper_driver, percent);

  stepper_driver->driver_current_.iholddelay = hold_delay;
  write_stored_driver_current(stepper_driver);
}

void set_all_current_percent_values(tmc2209_stepper_driver_t *stepper_driver,
                                    uint8_t                   run_current_percent,
                                    uint8_t                   hold_current_percent,
                                    uint8_t                   hold_delay_percent)
{
  uint8_t run_current  = percent_to_current_settings(stepper_driver, run_current_percent);
  uint8_t hold_current = percent_to_current_settings(stepper_driver, hold_current_percent);
  uint8_t hold_delay   = percent_to_hold_delay_setting(stepper_driver, hold_delay_percent);

  stepper_driver->driver_current_.irun       = run_current;
  stepper_driver->driver_current_.ihold      = hold_current;
  stepper_driver->driver_current_.iholddelay = hold_delay;
  write_stored_driver_current(stepper_driver);
}

void enable_inverse_motor_direction(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.shaft = 1;
  write_stored_global_config(stepper_driver);
}

void disable_inverse_motor_direction(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.shaft = 0;
  write_stored_global_config(stepper_driver);
}

void set_stand_still_mode(tmc2209_stepper_driver_t *stepper_driver, tmc2209_stand_still_mode_t mode)
{
  stepper_driver->pwm_config_.freewheel = mode;
  write_stored_pwm_config(stepper_driver);
}

void enable_automatic_current_scaling(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
  write_stored_pwm_config(stepper_driver);
}

void disable_automatic_current_scaling(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
  write_stored_pwm_config(stepper_driver);
}
/// @brief  Set the PWM frequency of the stepper driver.
/// 0 = 2/1024th of the internal clock frequency
/// 1 = 2/683th of the internal clock frequency
/// 2 = 2/512th of the internal clock frequency
/// 3 = 2/410th of the internal clock frequency
/// @param pwm_freq 0 - 3
void set_pwm_frequency(tmc2209_stepper_driver_t *stepper_driver, uint8_t pwm_freq)
{
  stepper_driver->pwm_config_.pwm_freq = pwm_freq;
  write_stored_pwm_config(stepper_driver);
}

void enable_automatic_gradient_adaptation(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
  write_stored_pwm_config(stepper_driver);
}

void disable_automatic_gradient_adaptation(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
  write_stored_pwm_config(stepper_driver);
}

void set_pwm_offset(tmc2209_stepper_driver_t *stepper_driver, uint8_t pwm_amplitude)
{
  stepper_driver->pwm_config_.pwm_offset = pwm_amplitude;
  write_stored_pwm_config(stepper_driver);
}

void set_pwm_gradient(tmc2209_stepper_driver_t *stepper_driver, uint8_t pwm_amplitude)
{
  stepper_driver->pwm_config_.pwm_grad = pwm_amplitude;
  write_stored_pwm_config(stepper_driver);
}

void set_power_down_delay(tmc2209_stepper_driver_t *stepper_driver, uint8_t power_down_delay)
{
  tmc2209_write(stepper_driver, ADDRESS_TPOWERDOWN, power_down_delay);
}

void set_reply_delay(tmc2209_stepper_driver_t *stepper_driver, uint8_t reply_delay)
{
  if (reply_delay > REPLY_DELAY_MAX)
  {
    reply_delay = REPLY_DELAY_MAX;
  }
  tmc2209_reply_delay_t reply_delay_data;
  reply_delay_data.bytes      = 0;
  reply_delay_data.replydelay = reply_delay;
  tmc2209_write(stepper_driver, ADDRESS_REPLYDELAY, reply_delay_data.bytes);
}

void move_at_velocity(tmc2209_stepper_driver_t *stepper_driver, int32_t microsteps_per_period)
{
  tmc2209_write(stepper_driver, ADDRESS_VACTUAL, microsteps_per_period);
}

void move_using_step_dir_interface(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_write(stepper_driver, ADDRESS_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
}

void enable_stealth_chop(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.enable_spread_cycle = 0;
  write_stored_global_config(stepper_driver);
}

void disable_stealth_chop(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.enable_spread_cycle = 1;
  write_stored_global_config(stepper_driver);
}

void set_cool_step_velocity_threshold(tmc2209_stepper_driver_t *stepper_driver, uint32_t duration_threshold)
{
  tmc2209_write(stepper_driver, ADDRESS_TCOOLTHRS, duration_threshold);
}

void set_stealth_chop_duration_threshold(tmc2209_stepper_driver_t *stepper_driver, uint32_t duration_threshold)
{
  tmc2209_write(stepper_driver, ADDRESS_TPWMTHRS, duration_threshold);
}

void set_stall_guard_threshold(tmc2209_stepper_driver_t *stepper_driver, uint8_t stall_guard_threshold)
{
  tmc2209_write(stepper_driver, ADDRESS_SGTHRS, stall_guard_threshold);
}

void enable_cool_step(tmc2209_stepper_driver_t *stepper_driver, uint8_t lower_threshold, uint8_t upper_threshold)
{
  lower_threshold                    = constrain(lower_threshold, SEMIN_MIN, SEMIN_MAX);
  stepper_driver->cool_config_.semin = lower_threshold;
  upper_threshold                    = constrain(upper_threshold, SEMAX_MIN, SEMAX_MAX);
  stepper_driver->cool_config_.semax = upper_threshold;
  tmc2209_write(stepper_driver, ADDRESS_COOLCONF, stepper_driver->cool_config_.bytes);
  stepper_driver->cool_step_enabled_ = true;
}

void disable_cool_step(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->cool_config_.semin = SEMIN_OFF;
  tmc2209_write(stepper_driver, ADDRESS_COOLCONF, stepper_driver->cool_config_.bytes);
  stepper_driver->cool_step_enabled_ = false;
}

void set_cool_step_current_increment(tmc2209_stepper_driver_t *stepper_driver, current_increment_t current_increment)
{
  stepper_driver->cool_config_.seup = current_increment;
  tmc2209_write(stepper_driver, ADDRESS_COOLCONF, stepper_driver->cool_config_.bytes);
}

void set_cool_step_measurement_count(tmc2209_stepper_driver_t *stepper_driver, measurement_count_t measurement_count)
{
  stepper_driver->cool_config_.sedn = measurement_count;
  tmc2209_write(stepper_driver, ADDRESS_COOLCONF, stepper_driver->cool_config_.bytes);
}

void enable_analog_current_scaling(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.i_scale_analog = 1;
  write_stored_global_config(stepper_driver);
}

void disable_analog_current_scaling(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.i_scale_analog = 0;
  write_stored_global_config(stepper_driver);
}

void use_external_resistor(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.internal_rsense = 0;
  write_stored_global_config(stepper_driver);
}

void use_internal_sense_resistor(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.internal_rsense = 1;
  write_stored_global_config(stepper_driver);
}

// bidirectional methods

uint8_t get_version(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_input_t input;
  input.bytes = tmc2209_read(stepper_driver, ADDRESS_IOIN);

  return input.version;
}

bool is_communicating(tmc2209_stepper_driver_t *stepper_driver)
{
  return (get_version(stepper_driver) == VERSION);
}

bool is_setup_and_communicating(tmc2209_stepper_driver_t *stepper_driver)
{
  return serial_operation_mode(stepper_driver);
}

bool is_communicating_without_setup(tmc2209_stepper_driver_t *stepper_driver)
{
  return (is_communicating(stepper_driver) && (!is_setup_and_communicating(stepper_driver)));
}

bool hardware_disabled(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_input_t input;
  input.bytes = tmc2209_read(stepper_driver, ADDRESS_IOIN);

  return input.enn;
}

uint16_t get_microstep_per_step(tmc2209_stepper_driver_t *stepper_driver)
{
  uint16_t microsteps_per_step_exponent;
  switch (stepper_driver->chopper_config_.mres)
  {
    case MRES_001: {
      microsteps_per_step_exponent = 0;
      break;
    }
    case MRES_002: {
      microsteps_per_step_exponent = 1;
      break;
    }
    case MRES_004: {
      microsteps_per_step_exponent = 2;
      break;
    }
    case MRES_008: {
      microsteps_per_step_exponent = 3;
      break;
    }
    case MRES_016: {
      microsteps_per_step_exponent = 4;
      break;
    }
    case MRES_032: {
      microsteps_per_step_exponent = 5;
      break;
    }
    case MRES_064: {
      microsteps_per_step_exponent = 6;
      break;
    }
    case MRES_128: {
      microsteps_per_step_exponent = 7;
      break;
    }
    case MRES_256:
    default: {
      microsteps_per_step_exponent = 8;
      break;
    }
  }
  return 1 << microsteps_per_step_exponent;
}

tmc2209_settings_t get_settings(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_settings_t settings;
  settings.is_communicating = is_communicating(stepper_driver);

  if (settings.is_communicating)
  {
    read_and_store_registers(stepper_driver);

    settings.is_setup                          = stepper_driver->global_config_.pdn_disable;
    settings.software_enabled                  = (stepper_driver->chopper_config_.toff > TOFF_DISABLE);
    settings.microsteps_per_step               = get_microstep_per_step(stepper_driver);
    settings.inverse_motor_direction_enabled   = stepper_driver->global_config_.shaft;
    settings.stealth_chop_enabled              = !stepper_driver->global_config_.enable_spread_cycle;
    settings.standstill_mode                   = stepper_driver->pwm_config_.freewheel;
    settings.irun_percent                      = current_setting_to_percent(stepper_driver, stepper_driver->driver_current_.irun);
    settings.irun_register_value               = stepper_driver->driver_current_.irun;
    settings.ihold_percent                     = current_setting_to_percent(stepper_driver, stepper_driver->driver_current_.ihold);
    settings.ihold_register_value              = stepper_driver->driver_current_.ihold;
    settings.iholddelay_percent                = hold_delay_setting_to_percent(stepper_driver, stepper_driver->driver_current_.iholddelay);
    settings.iholddelay_register_value         = stepper_driver->driver_current_.iholddelay;
    settings.automatic_current_scaling_enabled = stepper_driver->pwm_config_.pwm_autoscale;
    settings.automatic_gradient_adaptation_enabled = stepper_driver->pwm_config_.pwm_autograd;
    settings.pwm_offset                            = stepper_driver->pwm_config_.pwm_offset;
    settings.pwm_gradient                          = stepper_driver->pwm_config_.pwm_grad;
    settings.cool_step_enabled                     = stepper_driver->cool_step_enabled_;
    settings.analog_current_scaling_enabled        = stepper_driver->global_config_.i_scale_analog;
    settings.internal_sense_resistors_enabled      = stepper_driver->global_config_.internal_rsense;
  }
  else
  {
    settings.is_setup                              = false;
    settings.software_enabled                      = false;
    settings.microsteps_per_step                   = 0;
    settings.inverse_motor_direction_enabled       = false;
    settings.stealth_chop_enabled                  = false;
    settings.standstill_mode                       = stepper_driver->pwm_config_.freewheel;
    settings.irun_percent                          = 0;
    settings.irun_register_value                   = 0;
    settings.ihold_percent                         = 0;
    settings.ihold_register_value                  = 0;
    settings.iholddelay_percent                    = 0;
    settings.iholddelay_register_value             = 0;
    settings.automatic_current_scaling_enabled     = false;
    settings.automatic_gradient_adaptation_enabled = false;
    settings.pwm_offset                            = 0;
    settings.pwm_gradient                          = 0;
    settings.cool_step_enabled                     = false;
    settings.analog_current_scaling_enabled        = false;
    settings.internal_sense_resistors_enabled      = false;
  }

  return settings;
}

tmc2209_status_t get_status(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_drive_status_t drive_status;
  drive_status.bytes = 0;
  drive_status.bytes = tmc2209_read(stepper_driver, ADDRESS_DRV_STATUS);
  return drive_status.status;
}

uint8_t get_interface_transmission_counter(tmc2209_stepper_driver_t *stepper_driver)
{
  return tmc2209_read(stepper_driver, ADDRESS_IFCNT);
}

uint32_t get_interstep_duration(tmc2209_stepper_driver_t *stepper_driver)
{
  return tmc2209_read(stepper_driver, ADDRESS_TSTEP);
}

uint16_t get_stall_guard_result(tmc2209_stepper_driver_t *stepper_driver)
{
  return tmc2209_read(stepper_driver, ADDRESS_SG_RESULT);
}

uint8_t get_pwm_scale_sum(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_pwm_scale_t pwm_scale;
  pwm_scale.bytes = tmc2209_read(stepper_driver, ADDRESS_PWM_SCALE);

  return pwm_scale.pwm_scale_sum;
}

int16_t get_pwm_scale_auto(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_pwm_scale_t pwm_scale;
  pwm_scale.bytes = tmc2209_read(stepper_driver, ADDRESS_PWM_SCALE);

  return pwm_scale.pwm_scale_auto;
}

uint8_t get_pwm_offset_auto(tmc2209_stepper_driver_t *stepper_driver)
{
  pwm_auto_t pwm_auto;
  pwm_auto.bytes = tmc2209_read(stepper_driver, ADDRESS_PWM_AUTO);

  return pwm_auto.pwm_offset_auto;
}

uint8_t get_pwm_gradient_auto(tmc2209_stepper_driver_t *stepper_driver)
{
  pwm_auto_t pwm_auto;
  pwm_auto.bytes = tmc2209_read(stepper_driver, ADDRESS_PWM_AUTO);

  return pwm_auto.pwm_gradient_auto;
}

uint16_t get_microstep_counter(tmc2209_stepper_driver_t *stepper_driver)
{
  return tmc2209_read(stepper_driver, ADDRESS_MSCNT);
}

void tmc2209_initialize(tmc2209_stepper_driver_t *stepper_driver, long serial_baud_rate, tmc2209_serial_address_t serial_address)
{
  stepper_driver->serial_baud_rate_ = serial_baud_rate;

  set_operation_mode_to_serial(stepper_driver, serial_address);
  set_registers_to_default(stepper_driver);

  minimize_motor_current(stepper_driver);
  tmc2209_disable(stepper_driver);
  disable_automatic_current_scaling(stepper_driver);
  disable_automatic_gradient_adaptation(stepper_driver);
}

void set_operation_mode_to_serial(tmc2209_stepper_driver_t *stepper_driver, tmc2209_serial_address_t serial_address)
{
  stepper_driver->serial_address_ = serial_address;

  stepper_driver->global_config_.bytes            = 0;
  stepper_driver->global_config_.i_scale_analog   = 1;
  stepper_driver->global_config_.pdn_disable      = 1;
  stepper_driver->global_config_.mstep_reg_select = 1;
  stepper_driver->global_config_.multistep_filt   = 1;

  write_stored_global_config(stepper_driver);
}

void set_registers_to_default(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->driver_current_.bytes      = 0;
  stepper_driver->driver_current_.ihold      = IHOLD_DEFAULT;
  stepper_driver->driver_current_.irun       = IRUN_DEFAULT;
  stepper_driver->driver_current_.iholddelay = IHOLDDELAY_DEFAULT;
  tmc2209_write(stepper_driver, ADDRESS_IHOLD_IRUN, stepper_driver->driver_current_.bytes);

  stepper_driver->chopper_config_.bytes  = CHOPPER_CONFIG_DEFAULT;
  stepper_driver->chopper_config_.tbl    = TBL_DEFAULT;
  stepper_driver->chopper_config_.hend   = HEND_DEFAULT;
  stepper_driver->chopper_config_.hstart = HSTART_DEFAULT;
  stepper_driver->chopper_config_.toff   = TOFF_DEFAULT;
  tmc2209_write(stepper_driver, ADDRESS_CHOPCONF, stepper_driver->chopper_config_.bytes);

  stepper_driver->pwm_config_.bytes = PWM_CONFIG_DEFAULT;
  tmc2209_write(stepper_driver, ADDRESS_PWMCONF, stepper_driver->pwm_config_.bytes);

  stepper_driver->cool_config_.bytes = COOLCONF_DEFAULT;
  tmc2209_write(stepper_driver, ADDRESS_COOLCONF, stepper_driver->cool_config_.bytes);

  tmc2209_write(stepper_driver, ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
  tmc2209_write(stepper_driver, ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
  tmc2209_write(stepper_driver, ADDRESS_VACTUAL, VACTUAL_DEFAULT);
  tmc2209_write(stepper_driver, ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
  tmc2209_write(stepper_driver, ADDRESS_SGTHRS, SGTHRS_DEFAULT);
  tmc2209_write(stepper_driver, ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}

void read_and_store_registers(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.bytes  = read_global_config_bytes(stepper_driver);
  stepper_driver->chopper_config_.bytes = read_chopper_config_bytes(stepper_driver);
  stepper_driver->pwm_config_.bytes     = read_pwm_config_bytes(stepper_driver);
}

bool serial_operation_mode(tmc2209_stepper_driver_t *stepper_driver)
{
  global_config_t global_config;
  global_config.bytes = read_global_config_bytes(stepper_driver);

  return global_config.pdn_disable;
}

void minimize_motor_current(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->driver_current_.irun  = CURRENT_SETTING_MIN;
  stepper_driver->driver_current_.ihold = CURRENT_SETTING_MIN;
  write_stored_driver_current(stepper_driver);
}

uint32_t reverse_data(tmc2209_stepper_driver_t *stepper_driver, uint32_t data)
{
  uint32_t reversed_data = 0;
  uint8_t  right_shift;
  uint8_t  left_shift;
  for (uint8_t i = 0; i < DATA_SIZE; ++i)
  {
    right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
    left_shift  = i * BITS_PER_BYTE;
    reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
  }
  return reversed_data;
}

uint8_t calculate_crc_write(tmc2209_stepper_driver_t *stepper_driver, write_read_reply_datagram_t *datagram, uint8_t datagram_size)
{
  uint8_t crc = 0;
  uint8_t byte;
  uint8_t datagram_bytes[7];
  for (int i = 0; i < datagram_size - 1; i++)
  {
    datagram_bytes[i] = (datagram->bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
  }
  for (uint8_t i = 0; i < (datagram_size - 1); ++i)
  {
    byte = datagram_bytes[i];
    for (uint8_t j = 0; j < BITS_PER_BYTE; ++j)
    {
      if ((crc >> 7) ^ (byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = crc << 1;
      }
      byte = byte >> 1;
    }
  }
  return crc;
}

uint8_t calculate_crc_read(tmc2209_stepper_driver_t *stepper_driver, read_request_datagram_t *datagram, uint8_t datagram_size)
{
  uint8_t crc = 0;
  uint8_t byte;
  uint8_t datagram_bytes[4];
  for (int i = 0; i < (datagram_size - 1); i++)
  {
    datagram_bytes[i] = (datagram->bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
  }
  for (uint8_t i = 0; i < (datagram_size - 1); ++i)
  {
    byte = datagram_bytes[i];
    for (uint8_t j = 0; j < BITS_PER_BYTE; ++j)
    {
      if ((crc >> 7) ^ (byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = crc << 1;
      }
      byte = byte >> 1;
    }
  }
  return crc;
}

uint8_t percent_to_current_settings(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent)
{
  uint8_t constrained_percent = constrain(percent, PERCENT_MIN, PERCENT_MAX);
  uint8_t current_setting     = map(constrained_percent, PERCENT_MIN, PERCENT_MAX, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
  return current_setting;
}

uint8_t current_setting_to_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t current_setting)
{
  uint8_t percent = map(current_setting, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX, PERCENT_MIN, PERCENT_MAX);
  return percent;
}

uint8_t percent_to_hold_delay_setting(tmc2209_stepper_driver_t *stepper_driver, uint8_t percent)
{
  uint8_t constrained_percent = constrain(percent, PERCENT_MIN, PERCENT_MAX);
  uint8_t hold_delay_setting  = map(constrained_percent, PERCENT_MIN, PERCENT_MAX, HOLD_DELAY_MIN, HOLD_DELAY_MAX);
  return hold_delay_setting;
}

uint8_t hold_delay_setting_to_percent(tmc2209_stepper_driver_t *stepper_driver, uint8_t hold_delay_setting)
{
  uint8_t percent = map(hold_delay_setting, HOLD_DELAY_MIN, HOLD_DELAY_MAX, PERCENT_MIN, PERCENT_MAX);
  return percent;
}

void write_stored_global_config(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_write(stepper_driver, ADDRESS_GCONF, stepper_driver->global_config_.bytes);
}

uint32_t read_global_config_bytes(tmc2209_stepper_driver_t *stepper_driver)
{
  return tmc2209_read(stepper_driver, ADDRESS_GCONF);
}

void write_stored_driver_current(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_write(stepper_driver, ADDRESS_IHOLD_IRUN, stepper_driver->driver_current_.bytes);

  if (stepper_driver->driver_current_.irun >= SEIMIN_UPPER_CURRENT_LIMIT)
  {
    stepper_driver->cool_config_.seimin = SEIMIN_UPPER_SETTING;
  }
  else
  {
    stepper_driver->cool_config_.seimin = SEIMIN_LOWER_SETTING;
  }
  if (stepper_driver->cool_step_enabled_)
  {
    tmc2209_write(stepper_driver, ADDRESS_COOLCONF, stepper_driver->cool_config_.bytes);
  }
}

void write_stored_chopper_config(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_write(stepper_driver, ADDRESS_CHOPCONF, stepper_driver->chopper_config_.bytes);
}

uint32_t read_chopper_config_bytes(tmc2209_stepper_driver_t *stepper_driver)
{
  return tmc2209_read(stepper_driver, ADDRESS_CHOPCONF);
}

void write_stored_pwm_config(tmc2209_stepper_driver_t *stepper_driver)
{
  tmc2209_write(stepper_driver, ADDRESS_PWMCONF, stepper_driver->pwm_config_.bytes);
}

uint32_t read_pwm_config_bytes(tmc2209_stepper_driver_t *stepper_driver)
{
  return tmc2209_read(stepper_driver, ADDRESS_PWMCONF);
}
