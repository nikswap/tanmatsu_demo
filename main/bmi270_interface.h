// SPDX-FileCopyrightText: 2025 Nicolai Electronics
// SPDX-License-Identifier: MIT

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include "bmi2.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

void bmi2_set_i2c_configuration(i2c_master_bus_handle_t bus, uint8_t address, SemaphoreHandle_t semaphore);

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr);
void                  bmi2_delay_us(uint32_t period, void* intf_ptr);
int8_t                bmi2_interface_init(struct bmi2_dev* bmi, uint8_t intf);
void                  bmi2_error_codes_print_result(int8_t rslt);

#ifdef __cplusplus
}
#endif
