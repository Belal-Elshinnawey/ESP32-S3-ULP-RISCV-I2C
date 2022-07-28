#include "i2c.h"
#include <stdlib.h>
#include <stdbool.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

static uint16_t halfPeriodUs = 5; // freq = 100 kHz
static uint16_t timeout_step_us = 1;
static uint16_t timeout_count = 100;
static  bool started = false;
bool i2cTimeoutOccurred = 0;
static  bool internalTimeoutOccurred = 0;
static gpio_num_t sda = GPIO_NUM_2;
static gpio_num_t scl = GPIO_NUM_3;

void i2c_init(gpio_num_t sda,gpio_num_t scl){
    sda = sda;
    scl = scl;
    ulp_riscv_gpio_init(sda);
    ulp_riscv_gpio_output_enable(sda);
    ulp_riscv_gpio_set_output_mode(sda,RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_input_enable(sda);
    ulp_riscv_gpio_pulldown_disable(sda);

    ulp_riscv_gpio_init(scl);
    ulp_riscv_gpio_output_enable(scl);
    ulp_riscv_gpio_set_output_mode(scl,RTCIO_MODE_OUTPUT_OD );
    ulp_riscv_gpio_input_enable(scl);
    ulp_riscv_gpio_pulldown_disable(scl);
}
 void i2cSetFrequency(uint16_t freqKHz)
{
    // delayMicroseconds takes a uint8, so halfPeriodUs cannot be more than 255
    if (freqKHz < 2)
    {
        freqKHz = 2;
    }

    // force halfPeriodUs to round up so we don't use a higher frequency than what was chosen
    // TODO: implement a timing function with better resolution than delayMicroseconds to allow finer-grained frequency control?
    halfPeriodUs = (500 + freqKHz - 1) / freqKHz;
}


void i2cSetTimeout(uint16_t timeout_step_us, uint16_t timeout_count)
{
    timeout_step_us = timeout_step_us;
    timeout_count = timeout_count;
}

bool i2c_read_scl(void){
    return (bool) ulp_riscv_gpio_get_level(scl);
}

bool i2c_read_sda(void){
    return (bool) ulp_riscv_gpio_get_level(sda);
}

void i2c_clear_sda(void){
    ulp_riscv_gpio_output_level(sda,0);
}

void i2c_clear_scl(void){
    ulp_riscv_gpio_output_level(scl,0);
}

void i2c_set_sda(void){
    ulp_riscv_gpio_output_level(sda,1);
}

void i2c_set_scl(void){
    ulp_riscv_gpio_output_level(scl,1);
}
void i2c_get_ack(void){
    ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    ulp_riscv_gpio_output_disable(sda);
    ulp_riscv_gpio_input_enable(sda);
    ulp_riscv_gpio_pulldown_disable(sda);
    ulp_riscv_gpio_pullup(sda);
    ulp_riscv_gpio_output_disable(scl);
    ulp_riscv_gpio_input_enable(scl);
    ulp_riscv_gpio_pulldown_disable(scl);
    ulp_riscv_gpio_pullup(scl);

    ulp_riscv_delay_cycles(10*halfPeriodUs*ULP_RISCV_CYCLES_PER_US);

    ulp_riscv_gpio_output_enable(sda);
    ulp_riscv_gpio_set_output_mode(sda,RTCIO_MODE_OUTPUT_OD );
    ulp_riscv_gpio_output_enable(scl);
    ulp_riscv_gpio_set_output_mode(scl,RTCIO_MODE_OUTPUT_OD );
}

 void i2cWaitForHighScl(uint16_t timeoutCount, uint16_t timoutStepsUS)
{
    uint16_t count = 0;
    while (i2c_read_scl() == 0)
    {
        if (count > timeoutCount)
        {
            internalTimeoutOccurred = 1;
            i2cTimeoutOccurred = 1;
            started = 0;
            return;
        }
        ulp_riscv_delay_cycles(timoutStepsUS*ULP_RISCV_CYCLES_PER_US);
        count++;
    }
}

 void i2cWaitForLowScl(uint16_t timeoutCount, uint16_t timoutStepsUS)
{
    uint16_t count = 0;
    while (i2c_read_scl() == 1)
    {
        if (count > timeoutCount)
        {
            internalTimeoutOccurred = 1;
            i2cTimeoutOccurred = 1;
            started = 0;
            return;
        }
        ulp_riscv_delay_cycles(timoutStepsUS*ULP_RISCV_CYCLES_PER_US);
        count++;
    }
}

 /* Generate an I2C STOP condition (P):
    * SDA goes high while SCL is high
 */
void i2cStop(void)
{   
    ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    i2c_set_scl();
    ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    i2cWaitForHighScl(timeout_count, timeout_step_us);
    i2c_set_sda(); 
    ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    started = 0;
}

void i2cStart(void)
{   
    if (started){
        i2c_set_sda();
        ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
        i2c_set_scl();
        i2cWaitForHighScl(timeout_count, timeout_step_us);
        if (internalTimeoutOccurred) return;
        ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
        i2c_clear_sda();
        ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
        i2c_clear_scl();
        ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    } else {
        i2c_set_sda();
        i2c_set_scl();
        i2cWaitForHighScl(timeout_count, timeout_step_us);
        if (internalTimeoutOccurred) return;
        ulp_riscv_delay_cycles(100*1000*ULP_RISCV_CYCLES_PER_US);
        i2c_clear_sda();// drive SDA high while SCL is LOW
        ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
        i2c_clear_scl(); // drive SCL LOW
        ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    }
    // SCL is now LOW
    started = 1;
}

void i2cWriteBit(bool b)
{
    if (b)
    {
        i2c_set_sda(); // drive SDA go high
    }
    else
    {
        i2c_clear_sda(); // drive SDA low
    }
    ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    i2c_set_scl();
    i2cWaitForHighScl(timeout_count, timeout_step_us);
    if (internalTimeoutOccurred) return;
    ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    i2c_clear_scl();                    // drive SCL low
}

bool i2cReadBit(void)
{
    bool b;
    ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    i2c_set_scl();
    // Wait for SCL to go high
    i2cWaitForHighScl(1000, timeout_step_us);
    ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    b = i2c_read_sda();
    i2c_clear_scl();
    return b;
}

 bool i2cWriteByte(uint8_t byte)
{
    uint8_t i;
    bool nack;

    internalTimeoutOccurred = 0;

    for (i = 0; i < 8; i++)
    {
        i2cWriteBit(byte & 0x80);
        if (internalTimeoutOccurred) return 0;
        byte <<= 1;
    }
    nack = i2cReadBit();
    if (internalTimeoutOccurred) return 0;

    if (nack)
    {
        i2cStop();
        if (internalTimeoutOccurred) return 0;
    }
    return 1;
}

uint8_t i2cReadByte(bool nack)
{
    uint16_t byte = 0;
    uint8_t i;
    bool b;

    for (i = 0; i < 8; i++)
    {   
        b = i2cReadBit();
        byte = (byte << 1) | b;
    }
    ulp_riscv_delay_cycles(halfPeriodUs*ULP_RISCV_CYCLES_PER_US);
    i2cWriteBit(nack);
    return byte;
}

void write_to_address(uint8_t sad, uint8_t sub,uint8_t data){
    i2cStart();
    i2cWriteByte(((sad << 1)|0));
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cWriteByte(sub);
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cWriteByte(data);
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cStop();
    
}

void write_multiple_to_address(uint8_t sad, uint8_t sub,uint8_t * data, uint8_t length){
    uint8_t count = 0;
    i2cStart();
    i2cWriteByte(((sad << 1)|0));
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cWriteByte(sub);
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    while (count < length)
    {
        i2cWriteByte(data[count]);
        ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
        count++;
    }
    i2cStop();
}


uint8_t read_from_address(uint8_t sad, uint8_t sub){
    uint8_t value;
    i2cStart();
    i2cWriteByte((uint8_t)((sad << 1)|0));
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cWriteByte((uint8_t)sub);
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cStart();
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cWriteByte((uint8_t)((sad << 1)|1));
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    value = i2cReadByte(true);
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cStop();
    return value;
}

void read_multiple_from_address(uint8_t sad, uint8_t sub, uint8_t length, uint8_t * buffer){
    uint8_t value;
    uint8_t count = 0;
    i2cStart();
    i2cWriteByte((uint8_t)((sad << 1)|0));
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cWriteByte((uint8_t)sub);
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cStart();
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cWriteByte((uint8_t)((sad << 1)|1));
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    while(count < length-1){
        buffer[count] = i2cReadByte(false); 
        count++;
        ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    }
    buffer[length-1] = i2cReadByte(true);
    ulp_riscv_delay_cycles(100*ULP_RISCV_CYCLES_PER_US);
    i2cStop();
}