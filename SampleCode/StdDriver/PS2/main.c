/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 5 $
 * $Date: 14/05/30 6:01p $
 * @brief    Simulate the behavior of a PS/2 mouse by moving the cursor
 *           on the screen.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define DEVICE_ID                   0x00

#define PS2CMD_RESET                0xFF
#define PS2CMD_RESEND               0xFE
#define PS2CMD_SET_DEFAULTS         0xF6
#define PS2CMD_DISABLE_DATA_REPORT  0xF5
#define PS2CMD_ENABLE_DATA_REPORT   0xF4
#define PS2CMD_SET_SAMPLE_RATE      0xF3
#define PS2CMD_GET_DEVICE_ID        0xF2
#define PS2CMD_SET_REMOTE_MODE      0xF0
#define PS2CMD_SET_WARP_MODE        0xEE
#define PS2CMD_RESET_WARP_MODE      0xEC
#define PS2CMD_READ_DATA            0xEB
#define PS2CMD_SET_STREAM_MODE      0xEA
#define PS2CMD_STATUS_REQUEST       0xE9
#define PS2CMD_SET_RESOLUTION       0xE8
#define PS2CMD_SET_SCALLING2        0xE7
#define PS2CMD_SET_SCALLING1        0xE6

#define PS2MOD_RESET    0x0
#define PS2MOD_STREAM   0x1
#define PS2MOD_REMOTE   0x2
#define PS2MOD_WARP     0x3

void SYS_Init(void);
void UART0_Init(void);
void PS2_Init(void);

uint8_t g_opMode = PS2MOD_RESET;
uint8_t g_sampleRate = 0;
uint8_t g_resolution = 0;
uint8_t g_scalling = 0;
uint8_t g_dataReportEnable = 0;
uint32_t g_mouseData = 0;
uint8_t g_cmd[2] = {0};

uint32_t u32PS2ACK = 0xFA;
uint32_t u32PS2PASS = 0xAA;

uint32_t g_cnt = 0;
void SysTick_Handler(void)
{
    if(g_opMode == PS2MOD_STREAM && g_dataReportEnable)
    {
        if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
        {
            /* Calcuate cursor moving data */
            g_cnt++;
            if(g_cnt < 101)
                g_mouseData = (0x0 << 16) | (0x01 << 8) | 0x08;  // move right */
            else if(g_cnt < 201)
                g_mouseData = (0xFF << 16) | (0x00 << 8) | 0x28; // move down */
            else if(g_cnt < 301)
                g_mouseData = (0x0 << 16) | (0xFF << 8) | 0x18;  // move left */
            else if(g_cnt < 401)
                g_mouseData = (0x1 << 16) | (0x00 << 8) | 0x08;  // move up */
            else if(g_cnt >401)
                g_cnt = 1;

            /* Transmit data*/
            PS2_Write(&g_mouseData, 3);

            if((g_cnt & 0x0F) == 0)
                printf("Device->Host: Data report 0x%06x\n", g_mouseData);
        }
    }
}

void PS2D_IRQHandler(void)
{
    uint32_t u32RxData;
    uint32_t u32TxData;

    /* RXINT */
    if(PS2_GET_INT_FLAG(PS2_INTSTS_RXIF_Msk))
    {
        /* Clear PS2 Receive Interrupt flag */
        PS2_CLR_RX_INT_FLAG();

        /* Get Receive Data */
        u32RxData = PS2_Read();

        printf("\n u32RxData = 0x%x \n",u32RxData);

        if(g_cmd[0])
        {
            /* If g_cmd[0] is not 0, it should be in data phase */
            if(g_cmd[0] == PS2CMD_SET_SAMPLE_RATE)
            {
                printf("Host->Device: Set sample rate data %d\n", u32RxData);

                if(u32RxData < 10)   u32RxData = 10;
                if(u32RxData > 200) u32RxData = 200;
                g_sampleRate = u32RxData;
                g_cmd[0] = 0;

                /* Wait Tx ready */
                if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                {
                    printf("Device->Host: ACK\n");

                    PS2_Write(&u32PS2ACK, 1);
                }
                else
                {
                    printf("Something wrong!! Stop code!\n");

                    PS2_SET_CLK_LOW();
                    PS2_SET_DATA_HIGH();
                    PS2_ENABLE_OVERRIDE();

                    while(1);
                }

            }
            else if(g_cmd[0] == PS2CMD_SET_RESOLUTION)
            {
                printf("Host->Device: Set resolution data %d\n", u32RxData);

                if(u32RxData < 1) u32RxData = 1;
                if(u32RxData > 3) u32RxData = 3;
                g_resolution = (1 << u32RxData);
                g_cmd[0] = 0;

                /* Wait Tx ready */
                if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                {
                    PS2_Write(&u32PS2ACK, 1);

                    printf("Device->Host: ACK\n");
                }
                else
                {
                    printf("Something Wrong!! Stop code!\n");

                    PS2_SET_CLK_LOW();
                    PS2_SET_DATA_HIGH();
                    PS2_ENABLE_OVERRIDE();

                    while(1);
                }
            }
        }
        else
        {
            /* Only support PS2CMD_DISABLE_DATA_REPORT command when data report enabled */
            if((u32RxData == PS2CMD_DISABLE_DATA_REPORT) || (g_dataReportEnable == 0))
            {
                /* Process the command phase */
                if(u32RxData == PS2CMD_RESET)
                {
                    printf("Host->Device: Reset\n");

                    /* Reset command */
                    g_opMode = PS2MOD_RESET;
                    g_cmd[0] = 0;

                    /* Clear FIFO */
                    PS2_CLEAR_TX_FIFO();

                    /* Wait Tx ready */
                    if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                    {
                        u32TxData = ((DEVICE_ID<<16)|(u32PS2PASS<<8)|u32PS2ACK);
                        PS2_Write(&u32TxData, 3);

                        printf("Device->Host: ACK\n");
                    }

                }
                else if(u32RxData == PS2CMD_SET_SAMPLE_RATE)
                {
                    printf("Host->Device: Set sample rate\n");

                    /* Set sample rate */
                    g_cmd[0] = PS2CMD_SET_SAMPLE_RATE;

                    /* Wait Tx ready */
                    if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                    {
                        PS2_Write(&u32PS2ACK, 1);

                        printf("Device->Host: ACK\n");
                    }
                }
                else if(u32RxData == PS2CMD_GET_DEVICE_ID)
                {
                    printf("Host->Device: Get device ID\n");

                    g_cmd[0] = 0;

                    printf("(PS2->STATUS).TXEMPTY is (%0x)\n", (uint32_t)(((PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk) >> PS2_STATUS_TXEMPTY_Pos)));

                    /* Wait Tx ready */
                    if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                    {
                        u32TxData = ((DEVICE_ID << 8) | u32PS2ACK);
                        PS2_Write(&u32TxData, 2);

                        printf("Device->Host: ACK + Device ID(0x%x)\n", DEVICE_ID);
                    }
                }
                else if(u32RxData == PS2CMD_SET_SCALLING2)
                {
                    printf("Host->Device: Set scalling 2\n");

                    g_scalling = 2;
                    g_cmd[0] = 0;

                    /* Wait Tx ready */
                    if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                    {
                        PS2_Write(&u32PS2ACK, 1);

                        printf("Device->Host: ACK\n");
                    }
                }
                else if(u32RxData == PS2CMD_SET_SCALLING1)
                {
                    printf("Host->Device: Set scalling 1\n");

                    g_scalling = 1;
                    g_cmd[0] = 0;

                    /* Wait Tx ready */
                    if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                    {
                        PS2_Write(&u32PS2ACK, 1);

                        printf("Device->Host: ACK\n");
                    }
                }
                else if(u32RxData == PS2CMD_ENABLE_DATA_REPORT)
                {
                    printf("Host->Device: Enable data report\n");

                    g_dataReportEnable = 1;
                    g_cmd[0] = 0;

                    /* Set the timer for g_sampleRate */
                    /* The sample rate could be 10 ~ 200 samples/sec */
                    SysTick_Config(SystemCoreClock / g_sampleRate);

                    /* Wait Tx ready */
                    if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                    {
                        PS2_Write(&u32PS2ACK, 1);

                        printf("Device->Host: ACK\n");
                    }
                }
                else if(u32RxData == PS2CMD_DISABLE_DATA_REPORT)
                {
                    printf("Host->Device: Disable data report\n");

                    g_dataReportEnable = 0;
                    g_cmd[0] = 0;

                    SysTick->CTRL = 0;

                    /* Wait Tx ready */
                    if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                    {
                        PS2_Write(&u32PS2ACK, 1);

                        printf("Device->Host: ACK\n");
                    }
                }
                else if(u32RxData == PS2CMD_SET_RESOLUTION)
                {
                    printf("Host->Device: Set resolution\n");

                    g_cmd[0] = PS2CMD_SET_RESOLUTION;

                    /* Wait Tx ready */
                    if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                    {
                        PS2_Write(&u32PS2ACK, 1);

                        printf("Device->Host: ACK\n");
                    }
                }
                else if(u32RxData == PS2CMD_STATUS_REQUEST)
                {
                    printf("Host->Device: PS2CMD_STATUS_REQUEST\n");

                    g_cmd[0] = 0;

                    /* Wait Tx ready */
                    if(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk)
                    {
                        u32TxData = ((0x64<<24) | u32PS2ACK);
                        PS2_Write(&u32TxData, 4);

                        printf("Device->Host: ACK\n");
                    }
                }
            }
        }
    }

    /* TXINT */
    while(PS2_GET_INT_FLAG(PS2_INTSTS_TXIF_Msk))
    {
        PS2_CLR_TX_INT_FLAG();
    }

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HXT;

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_PS2CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PG multi-function pins for UART0 RXD, TXD */
    SYS->GPG_MFPL &= ~(SYS_GPG_MFPL_PG1MFP_Msk | SYS_GPG_MFPL_PG2MFP_Msk);
    SYS->GPG_MFPL |= (SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD);

    /* Set PF multi-function pins for PS2 PS2DAT and PS2CLK */
    SYS->GPG_MFPL &= ~(SYS_GPG_MFPL_PG3MFP_Msk | SYS_GPG_MFPL_PG4MFP_Msk);
    SYS->GPG_MFPL |= (SYS_GPG_MFPL_PG4MFP_PS2_DAT | SYS_GPG_MFPL_PG3MFP_PS2_CLK);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}

void PS2_Init(void)
{
    PS2_Open();

    PS2_EnableInt(PS2_CTL_RXIEN_Msk | PS2_CTL_TXIEN_Msk);
    NVIC_EnableIRQ(PS2D_IRQn);
}

int main(void)
{
    //uint32_t u32DeviceID;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init PS2 device */
    PS2_Init();

    g_cmd[0] = 0;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  PS2 Demo Code Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The demo code will show the cursor moving on the       |\n");
    printf("|    screen.                                                |\n");
    printf("+-----------------------------------------------------------+\n");

    while(1)
    {
        if(g_opMode == PS2MOD_RESET)
        {
            /* Reset to default configuration */
            g_sampleRate = 100;
            g_resolution = 4;
            g_scalling = 1;
            g_dataReportEnable = 0;

#if 0
            /* Do self test ... */

            /* Report 0xAA for successful self-test or 0xFC for error */

            /* Waiting for transmit */
            while(!(PS2_GET_STATUS() & PS2_STATUS_TXEMPTY_Msk))
            {
                printf("B4 PS2PASS CHECK!!\n");
            }

            PS2_Write(&u32PS2PASS, 1);

            /* Report device ID */
            while((PS2->STATUS & PS2_STATUS_TXEMPTY_Msk) == 0)
            {
                printf("B4 DEVICE_ID CHECK!!\n");
            }

            u32DeviceID = DEVICE_ID;
            PS2_Write(&u32DeviceID, 1);

            printf("Device->Host: DEVICE_ID!!\n");

#endif

            /* Enter Stream mode */
            g_opMode = PS2MOD_STREAM;
        }
    }
}




