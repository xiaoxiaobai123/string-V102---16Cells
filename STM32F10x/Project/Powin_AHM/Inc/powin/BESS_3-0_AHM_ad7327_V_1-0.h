/**
 * @file AD7327.h
 * @brief Register definitions for Analog Devices AD7237 bipolar ADC
 *
 * @author Pat Nystrom
 *
 * @copyright 2014 Powin Corporation. All rights reserved.
 */
#ifndef __AD7237_H
#define __AD7237_H

// Register definitions

//************************************************************
// GLOBAL BITS
// These are the top three bits of any SPI transfer
#define GLOB_WR(a) (a << 15)     // 1 == write, 0 == read
#define AD_WR 1
#define AD_RD 0
#define GLOB_REG(a) (a << 13)    // 2 bits register select
#define REG_CTL 0
#define REG_RANGE1 1
#define REG_RANGE2 2
#define REG_SEQ 3

//************************************************************
//CONTROL REGISTER DEFINITIONS
#define CTL_ADDR(a) (a << 10)   // 3 bits channel select
#define CTL_MODE(a) (a << 8)    // 2 bits mode
#define MODE_SINGLE_ENDED 0
#define MODE_PDIFF4 1
#define MODE_DIFF 2
#define MODE_PDIFF7 3
#define CTL_PM(a) (a << 6)      // 2 bits power mode
#define PM_SHUTDOWN 3
#define PM_AUTOSHUTDOWN 2
#define PM_AUTOSTBY 1
#define PM_NORMAL 0
#define CTL_CODING(a) (a << 5)  // data encoding
#define CODING_2COMPL 0
#define CODING_BIN    1
#define CTL_REF(a) (a << 4)     // reference select
#define REF_EXT 0
#define REF_INT 1
#define CTL_SEQ(a) (a << 2)     // 2 bits sequence operation
#define SEQ_OFF_CTL 0       // ctl register addr bits select channel
#define SEQ_FULL 1          // all channels selected in seq are run in ascending order
#define SEQ_TRUNC 2         // [0..ctl.addr] inclusive
// bottom two bits of the control register are not used and
// must always be written as 0

//************************************************************
// RANGE REGISTER DEFINITIONS
#define RNG_SEL(r, ch) (r << (ch * 2 + 6))
#define VRNG_BI10 0       // -10 .. +10
#define VRNG_5 1          // -5 .. +5
//    #define VRNG_2p5 2        // -2.5 .. +2.5
// 	#define VRNG_5 2          // -5 .. +5
#define VRNG_2p5 1
#define VRNG_10 3         // 0 .. +10V

//************************************************************
// SEQUENCE REGISTER DEFINITIONS
#define SEQ_SEL(ch) (1 << (12-(ch)))

int sample_vi(void);
void    AD7327_init( void );

extern int32_t v_acc;					// system voltage accumulator
extern int32_t i_acc;					// system current accumulator
extern int16_t sample_count;			// number of samples of current/voltage taken

#endif
