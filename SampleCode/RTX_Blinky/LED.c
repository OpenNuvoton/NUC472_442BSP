/*----------------------------------------------------------------------------
 * Name:    LED.c
 * Purpose: low level LED functions
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009-2011 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "NUC472_442.h"                     /* NUC4xx definitions           */
#include "LED.h"

const unsigned long led_mask[] = { 1UL << 0, 1UL << 1, 1UL << 2, 1UL << 3,
                                   1UL << 4, 1UL << 5, 1UL << 6, 1UL << 7
                                 };


/*----------------------------------------------------------------------------
  initialize LED Pins
 *----------------------------------------------------------------------------*/
void LED_Init (void)
{

    M32(&PE->MODE) = 0x00005555;        /* LEDs on PORT E defined as Output   */
    PE->DOUT |= 0x000000FF;             /* switch off LEDs                    */
}

/*----------------------------------------------------------------------------
  Function that turns on requested LED
 *----------------------------------------------------------------------------*/
void LED_On (unsigned int num)
{

    if (num < LED_NUM)
    {
        PE->DOUT  &= ~led_mask[num];
    }
}

/*----------------------------------------------------------------------------
  Function that turns off requested LED
 *----------------------------------------------------------------------------*/
void LED_Off (unsigned int num)
{

    if (num < LED_NUM)
    {
        PE->DOUT  |=  led_mask[num];
    }
}

/*----------------------------------------------------------------------------
  Function that outputs value to LEDs
 *----------------------------------------------------------------------------*/
void LED_Out(unsigned int value)
{
    int i;

    for (i = 0; i < LED_NUM; i++)
    {
        if (value & (1<<i))
        {
            LED_On (i);
        }
        else
        {
            LED_Off(i);
        }
    }
}
