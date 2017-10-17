#include <stdio.h>
#include <rtt_stdio.h>
#include "xtimer.h"
#include <string.h>
#include "periph/i2c.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "periph/rtt.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define TEMP_ADDRESS 0x44
#define TMP006_CFG 0x02
#define RF233_STATE 0xC2

void blink_radio_state(void) {
    uint8_t out = spi_transfer_reg(SPI_DEV(0),GPIO_PIN(PB,31),0x81,0x00);
    if(out == 0x00) {
       gpio_write(GPIO_PIN(PA,19),1);
    }
    if(out == 0x08) {
       gpio_write(GPIO_PIN(PA,19),0);
    }
    if(out == 0x10) {
       gpio_write(GPIO_PIN(PA,19),1);
    }
}

void sensor_shutdown(void) {

    //init devices
    i2c_acquire(I2C_0);
    i2c_init_master(I2C_0, I2C_SPEED_NORMAL);

    //SPI Init
    spi_init(SPI_DEV(0));
    spi_acquire(SPI_DEV(0),GPIO_PIN(PB,31),SPI_MODE_0,SPI_CLK_1MHZ);

    //GPIO init
    gpio_init(GPIO_PIN(PA, 28), GPIO_OUT);
    gpio_init(GPIO_PIN(PA, 20), GPIO_OUT);
    gpio_init(GPIO_PIN(PB, 15), GPIO_OUT);
    gpio_init(GPIO_PIN(PB, 31), GPIO_OUT);

    //light sensor
    //set the shutdown pin high
    gpio_write(GPIO_PIN(0, 28), 1);

    //temperature sensor
    uint8_t dat[2] = {0x00,0x00};
    i2c_write_regs(I2C_0,TEMP_ADDRESS,TMP006_CFG,dat,2);

    //radio
    gpio_write(GPIO_PIN(PA,20), 0);
    gpio_write(GPIO_PIN(PB,15), 1);
    for(volatile uint32_t i = 0; i < 10000000; i++);

    //setup trx_off
    spi_transfer_reg(SPI_DEV(0),GPIO_PIN(PB,31),RF233_STATE,0x03);
    for(volatile uint32_t i = 0; i < 10000000; i++);

    //set prep deep sleep - This seems to make it higher power???
    //spi_transfer_reg(SPI_DEV(0),GPIO_PIN(PB,31),RF233_STATE,0x10);
    //for(volatile uint32_t i = 0; i < 10000000; i++);

    //enter deep sleep
    gpio_write(GPIO_PIN(PA,20), 1);
}

void peripheral_shutdown(void) {
    //this is the arm deepsleep register
    uint32_t* reg = (uint32_t*)0xE000ED10;
    *reg = 0x00000004;
}

void enable_rtc(void) {

    //essentially steal the rtt init function with different clock sources
  	RtcMode0 *rtcMode0 = &(RTC->MODE0);

    /* Turn on power manager for RTC */
    PM->APBAMASK.reg |= PM_APBAMASK_RTC;

    /* Setup clock GCLK2 with divider 1 */
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(1);
    while (GCLK->STATUS.bit.SYNCBUSY) {}

    /* Enable GCLK2 with OSCULP32K as source. Use divider without modification
     * and keep running in standby. */
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
                        GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_RUNSTDBY |
                        GCLK_GENCTRL_SRC_OSCULP32K;
    while (GCLK->STATUS.bit.SYNCBUSY) {}

    /* Connect GCLK2 to RTC */
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK2 |
                        GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_ID(RTC_GCLK_ID);
    while (GCLK->STATUS.bit.SYNCBUSY) {}

    /* Disable RTC */
    rtcMode0->CTRL.bit.ENABLE = 0;
    while (rtcMode0->STATUS.bit.SYNCBUSY) {}

    /* Reset RTC */
    rtcMode0->CTRL.bit.SWRST = 1;
    while (rtcMode0->STATUS.bit.SYNCBUSY || rtcMode0->CTRL.bit.SWRST) {}

    /* Configure as 32bit counter with no prescaler and no clear on match compare */
    rtcMode0->CTRL.reg = RTC_MODE0_CTRL_MODE_COUNT32 | RTC_MODE0_CTRL_PRESCALER_DIV1;
    while (rtcMode0->STATUS.bit.SYNCBUSY) {}

    /* Setup interrupt */
    NVIC_EnableIRQ(RTT_IRQ);

    /* Enable RTC */
    rtcMode0->CTRL.bit.ENABLE = 1;
    while (rtcMode0->STATUS.bit.SYNCBUSY) {}

    rtcMode0->COMP[0].reg = rtcMode0->COUNT.reg + 70000;
    while (rtcMode0->STATUS.bit.SYNCBUSY) {}

    /* Enable Compare Interrupt and clear flag */
    rtcMode0->INTFLAG.bit.CMP0 = 1;
    rtcMode0->INTENSET.bit.CMP0 = 1;
}

void isr_rtc(void)
{
    RtcMode0 *rtcMode0 = &(RTC->MODE0);
    uint8_t status = rtcMode0->INTFLAG.reg;

    if ( (status & RTC_MODE0_INTFLAG_CMP0)) {
        rtcMode0->INTFLAG.bit.CMP0 = 1;
    }

    if ( (status & RTC_MODE0_INTFLAG_OVF)) {
        rtcMode0->INTFLAG.bit.OVF = 1;
    }

    cortexm_isr_end();
}


int main(void) {
    gpio_init(GPIO_PIN(PA,19), GPIO_OUT);
    sensor_shutdown();

    uint8_t on = 0;
    while(1) {
        on ^= 1;
        gpio_write(GPIO_PIN(PA,19),on);
        enable_rtc();
        peripheral_shutdown();
        __WFI();
    }
}
