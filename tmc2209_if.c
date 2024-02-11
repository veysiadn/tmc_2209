// /**
// ******************************************************************************
// * @file           : tmc2209_if.c
// * @author         : Veysi Adin (veysi.adin@outlook.com)
// * @brief          : TMC2209 interface implementation.
// *
// * @version        : 0.
// * @date           : 2024-01-03
// *
// ******************************************************************************
// * @attention
// *
// * Copyright (c) 2022 Veysi Adin.
// * All rights reserved.
// *
// * This software is licensed under terms that can be found in the LICENSE file
// * in the root directory of this software component.
// * If no LICENSE file comes with this software, it is provided AS-IS.
// *
// ******************************************************************************
// */

// /* Includes ------------------------------------------------------------------*/
// #include "tmc2209.h"
// #include "tmc2209_defines.h"
// #include "main.h"
// /* Private define ------------------------------------------------------------*/

// /* Private macro -------------------------------------------------------------*/

// /* Private typedef -----------------------------------------------------------*/

// /* Private variables ---------------------------------------------------------*/

// /* Private function prototypes -----------------------------------------------*/

// /* Public variables ----------------------------------------------------------*/

// /* Public function code ------------------------------------------------------*/

// /* Private function code -----------------------------------------------------*/

// extern UART_HandleTypeDef huart5;

// void tmc2209_set_hardware_enable_pin(tmc2209_stepper_driver_t *stepper_driver, uint8_t hardware_enable_pin)
// {
//   stepper_driver->hardware_enable_pin_ = hardware_enable_pin;

//   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_SET);
// }

// void tmc2209_enable(tmc2209_stepper_driver_t *stepper_driver)
// {
//   if (stepper_driver->hardware_enable_pin_ >= 0)
//   {
//     HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_RESET);
//   }
//   stepper_driver->toff_                = TOFF_DEFAULT;
//   stepper_driver->chopper_config_.toff = TOFF_DEFAULT;
//   write_stored_chopper_config(stepper_driver);
// }

// void tmc2209_disable(tmc2209_stepper_driver_t *stepper_driver)
// {
//   if (stepper_driver->hardware_enable_pin_ >= 0)
//   {
//     HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_SET);
//   }
//   stepper_driver->chopper_config_.toff = TOFF_DISABLE;
//   write_stored_chopper_config(stepper_driver);
// }

// void tmc2209_write(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address, uint32_t data)
// {
//   write_read_reply_datagram_t write_datagram;
//   write_datagram.bytes            = 0;
//   write_datagram.sync             = SYNC;
//   write_datagram.serial_address   = stepper_driver->serial_address_;
//   write_datagram.register_address = register_address;
//   write_datagram.rw               = RW_WRITE;
//   write_datagram.data             = reverse_data(stepper_driver, data);
//   write_datagram.crc              = calculate_crc_write(stepper_driver, &write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

//   uint8_t datagram_bytes[8];
//   for (int i = 0; i < 8; i++)
//   {
//     datagram_bytes[i] = (write_datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
//   }
//   HAL_HalfDuplex_EnableTransmitter(&huart5);
//   HAL_UART_Transmit(&huart5, datagram_bytes, WRITE_READ_REPLY_DATAGRAM_SIZE, 0XFFFF);
// }

// uint32_t tmc2209_read(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address)
// {
//   read_request_datagram_t read_request_datagram;
//   read_request_datagram.bytes            = 0;
//   read_request_datagram.sync             = SYNC;
//   read_request_datagram.serial_address   = stepper_driver->serial_address_;
//   read_request_datagram.register_address = register_address;
//   read_request_datagram.rw               = RW_READ;
//   read_request_datagram.crc              = calculate_crc_read(stepper_driver, &read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

//   uint8_t datagram_bytes[WRITE_READ_REPLY_DATAGRAM_SIZE];
//   for (int i = 0; i < READ_REQUEST_DATAGRAM_SIZE; ++i)
//   {
//     datagram_bytes[i] = (read_request_datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
//   }
//   HAL_HalfDuplex_EnableTransmitter(&huart5);
//   HAL_UART_Transmit(&huart5, datagram_bytes, READ_REQUEST_DATAGRAM_SIZE, 0XFFFF);

//   uint8_t                     byte       = 0;
//   uint8_t                     byte_count = 0;
//   write_read_reply_datagram_t read_reply_datagram;
//   read_reply_datagram.bytes = 0;

//   HAL_HalfDuplex_EnableReceiver(&huart5);
//   for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i)
//   {
//     HAL_UART_Receive(&huart5, (uint8_t *)&byte, 1, 100);
//     datagram_bytes[i] = byte;
//   }
//   for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i)
//   {
//     read_reply_datagram.bytes |= ((uint64_t)datagram_bytes[i] << (byte_count++ * BITS_PER_BYTE));
//   }
//   uint32_t reversed_data = reverse_data(stepper_driver, read_reply_datagram.data);
//   uint8_t  crc           = calculate_crc_write(stepper_driver, &read_reply_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
//   if (crc != read_reply_datagram.crc)
//   {
//   }
//   return reversed_data;
// }
