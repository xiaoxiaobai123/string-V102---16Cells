/**
 * @file AD7327.c
 * @brief This file contains the implementation of the protocols for AD7327 chips.
 *
 * @author Sam Gao
 *
 * @copyright 2012 Powin Corporation. All rights reserved.
 */
#include <stdint.h>
#include "BESS_3-0_AHM_device_lib_V_1-0.h"
#include "BESS_3-0_AHM_ad7327_V_1-0.h"
#include "BESS_3-0_AHM_gpio_V_1-0.h"

// Channels 0/1 are the current +/-
// Channels 2/3 are the voltage +/-

// This value will setup the converter for internal reference, powered on,
// taking continuous samples from channels 0 and 2 (over and over),
// outputting data in 2's complement form. We do this by setting the address
// to 2, the sequence to truncated (ending at the addr field 2, inclusive),
// and setting only the 0 and 2 channel bits of the sequence register. For
// pseudo-differential mode, addresses 0 and 1 both cause conversion of the
// ch0/1 pair, and addresses 2 and 3 both cause conversion of the 2/3 pair;
// therefore, we only sample addresses 0 and 2 to loop between the two inputs.
// The samples are actually taken only at the falling edge of CSn. To sample
// close together in time (for close-to-simultaneous v/i readings), we want
// to do two transfers back-to-back. The conversion happens in 1 cycle of fclk,
// which is 9 MHz; we only need to wait 75 ns between the conversions.


int32_t v_acc = 0;
int32_t i_acc = 0;
int16_t sample_count = 0;   // for both v and i, since they're always taken together


//do a SPI transaction with the AD7327. Return 16-bit signed data as result.
static uint16_t rw_dac(uint16_t data)
{
    GPIO_ResetBits(ADC_CSn);       // drop CSn
    SPI_I2S_SendData(SPI1, data);
    while (SPI1->SR & 0x80) ;      // wait for completion. This takes 16/9e6, or 1.77 us.
    // and, if it hangs, the watchdog will save us (todo: remember to enable it!!)
    GPIO_SetBits(ADC_CSn);       // raise CSn
    return SPI_I2S_ReceiveData(SPI1);
}

// Initialize the AD7327
void	AD7327_init( void )
{
    volatile uint32_t i;
//    uint16_t value;
    GPIO_SetBits(ADC_CSn);       // raise CSn
    for (i = 0; i < 100000; i++) ;     // time for chip to recognize rising cs and reset (?)
    rw_dac(GLOB_WR(AD_WR) |
           GLOB_REG(REG_RANGE1) |
           RNG_SEL(VRNG_2p5, 0) |
           RNG_SEL(VRNG_2p5, 1) |
           RNG_SEL(VRNG_2p5, 2) |
           RNG_SEL(VRNG_2p5, 3) |
           0);
    rw_dac(GLOB_WR(AD_WR) |
           GLOB_REG(REG_RANGE2) |
           RNG_SEL(VRNG_2p5, 0) |
           RNG_SEL(VRNG_2p5, 1) |
           RNG_SEL(VRNG_2p5, 2) |
           RNG_SEL(VRNG_2p5, 3) |
           0);

    rw_dac(GLOB_WR(AD_WR) |
           GLOB_REG(REG_SEQ) |
           SEQ_SEL(0) |
           SEQ_SEL(2) |
           0);
    rw_dac(GLOB_WR(AD_WR) |
           GLOB_REG(REG_CTL) |
           CTL_ADDR(7) |
           CTL_MODE(MODE_SINGLE_ENDED) |
           CTL_PM(PM_NORMAL) |
           CTL_CODING(CODING_2COMPL) |
           CTL_REF(REF_INT) |
           CTL_SEQ(SEQ_FULL) |
           0);
}


int sample_vi(void)
{
    uint16_t spi;
    uint8_t chan;
    int16_t value;
    uint8_t i;
//    volatile float v;

    rw_dac(GLOB_WR(AD_RD));   // do a primer read to read v or i, then a pair close together to get the other.
    // we're reading the most recent result, which is old, and tossing it. Then we initiate two more reads, the first
    // of which is the result of the primer, the second of which is the result of the first, and the result of the
    // second we throw away - it's what the next 'primer' will read.
    for (i = 0; i < 2; i++) {
        spi = rw_dac(GLOB_WR(AD_RD));
        chan = spi >> 13;
        value = spi << 3;        // left align 2s complement 13-bit result
        value >>= 3;			 // then sign-extend result
        switch (chan) {
            case 0:
                // current, channels 0/1
                i_acc += value;
//                v = value * 2.5 / 4095.0;
                break;
            case 2:
                // voltage, channels 2/3
                v_acc += value;
//                v = value * 2.5 / 4095.0;
                break;
        }
    }
    ++sample_count;
    return 0;
}
