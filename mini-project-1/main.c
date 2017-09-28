#include <stdio.h>
#include <rtt_stdio.h>
#include "xtimer.h"
#include <string.h>
#include "periph/i2c.h"
#include "periph/gpio.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifndef SAMPLE_INTERVAL
#define SAMPLE_INTERVAL ( 1000000UL)
#endif

#define ACCEL_ADDRESS 0x4C
#define TEMP_ADDRESS 0x49

void sensor_shutdown(void) {

    //init devices
    i2c_acquire(I2C_0);
    i2c_init_master(I2C_0, I2C_SPEED_NORMAL);
    gpio_init(GPIO_PIN(0,28), GPIO_OUT);

    //accelerometer - standby 0x07=0x00
    i2c_write_reg(I2C_0,ACCEL_ADDRESS,0x07,0x00);    

    //light sensor
    //set the shutdown pin high
    gpio_write(GPIO_PIN(0, 28), 1); 

    //temperature sensor
    i2c_write_reg(I2C_0,TEMP_ADDRESS,0x01,0x01);    
}

int main(void) {
    uint16_t wakeup_count = 0;
    sensor_shutdown();

    while (1) {
		xtimer_usleep(SAMPLE_INTERVAL);
        printf("I am Alive! %u\n", wakeup_count++);
    }

    return 0;
}
