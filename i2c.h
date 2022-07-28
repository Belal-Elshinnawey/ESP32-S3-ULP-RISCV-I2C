#pragma once
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"
#include <stdbool.h>

#ifndef WSI2C
#define WSI2C
void i2c_init(gpio_num_t sda,gpio_num_t scl);
void i2cSetFrequency(uint16_t freqKHz);
void i2cSetTimeout(uint16_t timeout_step_us, uint16_t timeout_count);
bool i2c_read_scl(void);
bool i2c_read_sda(void);
void i2c_clear_sda(void);
void i2c_clear_scl(void);
void i2c_get_ack(void);
void i2cWaitForHighScl(uint16_t timeoutCount, uint16_t timoutStepsUS);void i2cStop(void);
void i2cStop(void);
void i2cStart(void);
void i2cWriteBit(bool b);
bool i2cReadBit(void);
bool i2cWriteByte(uint8_t byte);
uint8_t i2cReadByte(bool nack);
uint8_t read_from_address(uint8_t sad, uint8_t sub);
void read_multiple_from_address(uint8_t sad, uint8_t sub, uint8_t length, uint8_t * buffer);
void write_to_address(uint8_t sad, uint8_t sub,uint8_t data);
void write_multiple_to_address(uint8_t sad, uint8_t sub,uint8_t * data, uint8_t length);
#endif