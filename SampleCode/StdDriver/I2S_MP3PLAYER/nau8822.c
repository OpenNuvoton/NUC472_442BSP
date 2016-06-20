/**************************************************************************//**
 * @file     nau8822.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/03/19 5:27p $
 * @brief    NUC472/NUC442 I2S Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"

#define NAU8822_ADDR    0x1A                /* NAU8822 Device ID */

void Delay(int count)
{
    volatile uint32_t i;
    for (i = 0; i < count ; i++);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of WAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{
    I2C_START(I2C3);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, NAU8822_ADDR<<1);
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_STOP(I2C3);
}

void WAU8822_ConfigSampleRate(uint32_t u32SampleRate)
{
    printf("[NAU8822] Configure Sampling Rate to %d\n", u32SampleRate);
    if((u32SampleRate % 8) == 0) {
        I2C_WriteWAU8822(36, 0x008);    //12.288Mhz
        I2C_WriteWAU8822(37, 0x00C);
        I2C_WriteWAU8822(38, 0x093);
        I2C_WriteWAU8822(39, 0x0E9);
    } else if(u32SampleRate == 44100) {
        I2C_WriteWAU8822(36, 0x007);    //11.2896Mhz, for 44.1k
        I2C_WriteWAU8822(37, 0x021);
        I2C_WriteWAU8822(38, 0x161);
        I2C_WriteWAU8822(39, 0x026);
    } else {
        I2C_WriteWAU8822(36, 0x00B);    //16.934Mhz
        I2C_WriteWAU8822(37, 0x011);
        I2C_WriteWAU8822(38, 0x153);
        I2C_WriteWAU8822(39, 0x1F0);
    }

    switch (u32SampleRate) {
    case 16000:
        I2C_WriteWAU8822(6, 0x1AD);   /* Divide by 6, 16K */
        I2C_WriteWAU8822(7, 0x006);   /* 16K for internal filter cofficients */
        break;

    case 32000:
        I2C_WriteWAU8822(6, 0x16D);    /* Divide by 3, 32K */
        I2C_WriteWAU8822(7, 0x002);    /* 32K for internal filter cofficients */
        break;

    case 44100:
    case 48000:
        I2C_WriteWAU8822(6, 0x14D);    /* Divide by 1, 48K */
        I2C_WriteWAU8822(7, 0x000);    /* 48K for internal filter cofficients */
        break;

    case 11025:
        I2C_WriteWAU8822(6, 0x1ED);    /* Divide by 12 */
        break;

    case 22050:
        I2C_WriteWAU8822(6, 0x1AD);    /* Divide by 6 */
        break;

    case 192000:
//      I2C_WriteWAU8822(6, 0x109);     //16bit/192k
        I2C_WriteWAU8822(6, 0x105);     //32bit/192k
        I2C_WriteWAU8822(72, 0x017);
        break;
    }
}

void WAU8822_ConfigBitNumber(uint32_t u32BitNum)
{
    switch(u32BitNum) {
    case 16:
        I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
        break;
    case 20:
        I2C_WriteWAU8822(4,  0x020);   /* 20-bit word length, I2S format, Stereo */
        break;
    case 24:
        I2C_WriteWAU8822(4,  0x040);   /* 24-bit word length, I2S format, Stereo */
        break;
    case 32:
        I2C_WriteWAU8822(4,  0x060);   /* 32-bit word length, I2S format, Stereo */
        break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  WAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void WAU8822_Setup()
{
    printf("\nConfigure WAU8822 ...");

    I2C_WriteWAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

    I2C_WriteWAU8822(1,  0x02F);
    I2C_WriteWAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteWAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */

    // offset: 0x4 => default, 24bit, I2S format, Stereo
    I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */

    I2C_WriteWAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteWAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteWAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteWAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteWAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteWAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteWAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteWAU8822(11, 0x1CF);   /* DAC left digital volume control */
    I2C_WriteWAU8822(12, 0x1CF);   /* DAC right digital volume control */

    I2C_WriteWAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteWAU8822(47, 0x050);   /* LLIN connected, and its Gain value */
    I2C_WriteWAU8822(48, 0x050);   /* RLIN connected, and its Gain value */
    I2C_WriteWAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteWAU8822(51, 0x001);   /* Right DAC connected to RMIX */

    printf("[OK]\n");
}
