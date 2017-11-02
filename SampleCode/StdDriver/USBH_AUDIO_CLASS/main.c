/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/10/31 10:28a $
 * @brief    This sample shows how to use USB Host Audio Class driver.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"
#include "usbh_core.h"
#include "usbh_uac.h"


//#define USBH_CLK_FROM_PLL
#define USBH_CLK_FROM_PLL2_DUAL_ROLE

#ifdef USBH_CLK_FROM_PLL
#define PLLCON_SETTING      0xc22e
#define PLL_CLOCK           48000000
#else
#define PLLCON_SETTING      0x4228
#define PLL_CLOCK           84000000
#endif

uint32_t PllClock         = PLL_CLOCK;

#define HCLK_HZ             PLL_CLOCK

#define POWER_UP_DELAY      20            /* 200 ms delay after port power on       */

uint8_t  au_in_buff[2048];

static volatile int  au_in_cnt, au_out_cnt;


static volatile int  g_tick_cnt = 0;

void SysTick_Handler(void)
{
    g_tick_cnt++;
}

uint32_t get_sys_ticks(void)
{
    return g_tick_cnt;
}

void enable_sys_ticks(int ticks_per_second)
{
    uint32_t  systick_load;
    /*
     *  Configure system tick to be 0.1 second count
     */
    systick_load = ((HCLK_HZ/(2 * ticks_per_second)) & SysTick_LOAD_RELOAD_Msk) - 1;
    /* system tick from HCLK/2 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | (3 << CLK_CLKSEL0_STCLKSEL_Pos  );
    SysTick->LOAD  = systick_load;
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Cortex-M0 System Interrupts */
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_HXT,CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USB Host clock                                                                                     */
    /*---------------------------------------------------------------------------------------------------------*/

#ifdef USBH_CLK_FROM_PLL

    // Configure OTG function as Host-Only
    SYS->USBPHY = 0x101;

    // Multi-function PB.0 OTG_5V status, PB.1 OTG_5V enable
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_USB0_OTG5V_ST | SYS_GPB_MFPL_PB1MFP_USB0_OTG5V_EN)) | 0x11;

    // Multi-function (PB.2,PB.3) for USB port 2 (D-,D+)
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_USB1_D_N | SYS_GPB_MFPL_PB3MFP_USB1_D_P)) | 0x3300;

    // Set PB.4 output high to enable USB power
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB4MFP_Msk;
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE4_Msk) | (0x1 << GPIO_MODE_MODE4_Pos);
    PB->DOUT |= 0x10;

    // Select USB Host clock source from PLL
    CLK->CLKSEL0 |= CLK_CLKSEL0_USBHSEL_Msk;

    // USB clock is HCLK divided by 1
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBHDIV_Msk) | (0 << CLK_CLKDIV0_USBHDIV_Pos);

    // Enable USB Host
    CLK->AHBCLK |= CLK_AHBCLK_USBHCKEN_Msk;

#endif

#ifdef USBH_CLK_FROM_PLL2_DUAL_ROLE

    SYS->USBPHY |= SYS_USBPHY_LDO33EN_Msk;

    // Enable OTG
    CLK->APBCLK0 |= CLK_APBCLK0_OTGCKEN_Msk;

    OTG->PHYCTL = (OTG->PHYCTL | OTG_PHYCTL_OTGPHYEN_Msk) & ~OTG_PHYCTL_IDDETEN_Msk;

    // Multi-function PB.0 OTG_5V status, PB.1 OTG_5V enable
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_USB0_OTG5V_ST | SYS_GPB_MFPL_PB1MFP_USB0_OTG5V_EN)) | 0x11;

    // Multi-function (PB.2,PB.3) for USB port 2 (D-,D+)
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_USB1_D_N | SYS_GPB_MFPL_PB3MFP_USB1_D_P)) | 0x3300;

    // Set PB.4 output high to enable USB power
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB4MFP_Msk;
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE4_Msk) | (0x1 << GPIO_MODE_MODE4_Pos);
    PB->DOUT |= 0x10;

    // Enable PLL2, 480 MHz / 2 / (1+4) => 48 MHz output
    CLK->PLL2CTL = (4 << CLK_PLL2CTL_PLL2DIV_Pos);

    // wait PLL2 stable...
    while (!(CLK->STATUS & CLK_STATUS_PLL2STB_Msk));

    // USB clock divided by 1
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBHDIV_Msk) | (0 << CLK_CLKDIV0_USBHDIV_Pos);

    // Select USB Host clock source from PLL2
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_USBHSEL_Msk;

    // Enable USB Host
    CLK->AHBCLK |= CLK_AHBCLK_USBHCKEN_Msk;

#endif

    /* Lock protected registers */
    SYS_LockReg();
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART0->LINE = 0x07;
    UART0->BAUD = 0x30000066;   /* 12MHz reference clock input, for 115200 */
}


/**
 *  @brief  Audio-in data callback function.
 *          UAC driver notify user that audio-in data has been moved into user audio-in buffer,
 *          which is provided by user application via UAC_InstallIsoInCbFun().
 *  @param[in] dev    Audio Class device
 *  @param[in] data   Available audio-in data, which is located in user audio-in buffer.
 *  @param[in] len    Length of available audio-in data started from <data>.
 *  @return   UAC driver does not check this return value.
 */
int audio_in_callback(UAC_DEV_T *dev, uint8_t *data, int len)
{
    au_in_cnt += len;
    //printf("I %x,%x\n", (int)data & 0xffff, len);   // UART send too many will cause ISO transfer time overrun

    // Add your code here to get audio-in data ...
    // For example, memcpy(audio_record_buffer, data, len);
    // . . .

    return 0;
}


/**
 *  @brief  Audio-out data callback function.
 *          UAC driver requests user to move audio-out data into the specified address. The audio-out
 *          data will then be send to UAC device via isochronous-out pipe.
 *  @param[in] dev    Audio Class device
 *  @param[in] data   Application should move audio-out data into this buffer.
 *  @param[in] len    Maximum length of audio-out data can be moved.
 *  @return   Actual length of audio data moved.
 */
int audio_out_callback(UAC_DEV_T *dev, uint8_t *data, int len)
{
    au_out_cnt += len;
    //printf("O %x,%x\n", (int)data & 0xffff, len);   // UART send too many will cause ISO transfer time overrun

    // Add your code here to put audio-out data ...
    // For example, memcpy(data, playback_buffer, actual_len);
    //              return actual_len;
    // . . .

    return 192;   // for 48000 stero Hz
}


void  uac_control_example(UAC_DEV_T *uac_dev)
{
    uint8_t    data[8];
    uint32_t   srate[4];
    uint32_t   val32;
    uint16_t   val16;
    uint8_t    val8;
    int        i, ret;

    printf("\nGet channel information ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get channel number information                             */
    /*-------------------------------------------------------------*/
    ret = UAC_GetChannelNumber(uac_dev, UAC_SPEAKER);
    if (ret < 0)
        printf("    Failed to get speaker's channel number.\n");
    else
        printf("    Speaker: %d\n", ret);

    ret = UAC_GetChannelNumber(uac_dev, UAC_MICROPHONE);
    if (ret < 0)
        printf("    Failed to get microphone's channel number.\n");
    else
        printf("    Microphone: %d\n", ret);

    printf("\nGet subframe bit resolution ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get audio subframe bit resolution information              */
    /*-------------------------------------------------------------*/
    ret = UAC_GetBitResolution(uac_dev, UAC_SPEAKER, &val8);
    if (ret < 0)
        printf("    Failed to get speaker's bit resoltion.\n");
    else {
        printf("    Speaker audio subframe size: %d bytes\n", val8);
        printf("    Speaker subframe bit resolution: %d\n", ret);
    }

    ret = UAC_GetBitResolution(uac_dev, UAC_MICROPHONE, &val8);
    if (ret < 0)
        printf("    Failed to get microphone's bit resoltion.\n");
    else {
        printf("    Microphone audio subframe size: %d bytes\n", val8);
        printf("    Microphone subframe bit resolution: %d\n", ret);
    }

    printf("\nGet sampling rate list ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get audio subframe bit resolution information              */
    /*-------------------------------------------------------------*/
    ret = UAC_GetSamplingRate(uac_dev, UAC_SPEAKER, (uint32_t *)&srate[0], 4, &val8);
    if (ret < 0)
        printf("    Failed to get speaker's sampling rate.\n");
    else {
        if (val8 == 0)
            printf("    Speaker sampling rate range: %d ~ %d Hz\n", srate[0], srate[1]);
        else {
            for (i = 0; i < val8; i++)
                printf("    Speaker sampling rate: %d\n", srate[i]);
        }
    }

    ret = UAC_GetSamplingRate(uac_dev, UAC_MICROPHONE, (uint32_t *)&srate[0], 4, &val8);
    if (ret < 0)
        printf("    Failed to get microphone's sampling rate.\n");
    else {
        if (val8 == 0)
            printf("    Microphone sampling rate range: %d ~ %d Hz\n", srate[0], srate[1]);
        else {
            for (i = 0; i < val8; i++)
                printf("    Microphone sampling rate: %d\n", srate[i]);
        }
    }

    printf("\nSpeaker mute control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's speaker.            */
    /*-------------------------------------------------------------*/
    if (UAC_MuteControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, data) == UAC_RET_OK) {
        printf("    Speaker mute state is %d.\n", data[0]);
    } else
        printf("    Failed to get speaker mute state!\n");

    printf("\nSpeaker L(F) volume control ===>\n");

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK) {
        printf("    Speaker L(F) volume is 0x%x.\n", val16);
    } else
        printf("    Failed to get seaker L(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK) {
        printf("    Speaker L(F) minimum volume is 0x%x.\n", val16);
    } else
        printf("    Failed to get speaker L(F) minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK) {
        printf("    Speaker L(F) maximum volume is 0x%x.\n", val16);
    } else
        printf("    Failed to get speaker L(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker left channel.             */
    /*--------------------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK) {
        printf("    Speaker L(F) volume resolution is 0x%x.\n", val16);
    } else
        printf("    Failed to get speaker L(F) volume resolution!\n");


    printf("\nSpeaker R(F) volume control ===>\n");

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK) {
        printf("    Speaker R(F) volume is 0x%x.\n", val16);
    } else
        printf("    Failed to get speaker R(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK) {
        printf("    Speaker R(F) minimum volume is 0x%x.\n", val16);
    } else
        printf("    Failed to get speaker R(F) minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK) {
        printf("    Speaker R(F) maximum volume is 0x%x.\n", val16);
    } else
        printf("    Failed to get speaker R(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker right channel.            */
    /*--------------------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK) {
        printf("    Speaker R(F) volume resolution is 0x%x.\n", val16);
    } else
        printf("    Failed to get speaker R(F) volume resolution!\n");

    printf("\nMicrophone mute control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's speaker.            */
    /*-------------------------------------------------------------*/
    if (UAC_MuteControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, data) == UAC_RET_OK) {
        printf("    Microphone mute state is %d.\n", data[0]);
    } else
        printf("    Failed to get microphone mute state!\n");

    printf("\nMicrophone volume control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, &val16) == UAC_RET_OK) {
        printf("    Microphone current volume is 0x%x.\n", val16);
    } else
        printf("    Failed to get microphone current volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_MIN, UAC_CH_MASTER, &val16) == UAC_RET_OK) {
        printf("    Microphone minimum volume is 0x%x.\n", val16);
    } else
        printf("    Failed to get microphone minimum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_MAX, UAC_CH_MASTER, &val16) == UAC_RET_OK) {
        printf("    Microphone maximum volume is 0x%x.\n", val16);
    } else
        printf("    Failed to get microphone maximum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get resolution of UAC device's microphone volume value.    */
    /*-------------------------------------------------------------*/
    if (UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_RES, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK) {
        printf("    Microphone volume resolution is 0x%x.\n", val16);
    } else
        printf("    Failed to get microphone volume resolution!\n");

    printf("\nMicrophone automatic gain control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's speaker.            */
    /*-------------------------------------------------------------*/
    if (UAC_AutoGainControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, data) == UAC_RET_OK) {
        printf("    Microphone auto gain is %s.\n", data[0] ? "ON" : "OFF");
    } else
        printf("    Failed to get microphone auto-gain state!\n");

    printf("\nSampling rate control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's speaker.   */
    /*-------------------------------------------------------------*/
    if (UAC_SamplingRateControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &val32) == UAC_RET_OK) {
        printf("    Speaker's current sampling rate is %d.\n", val32);
    } else
        printf("    Failed to get speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's speaker.       */
    /*-------------------------------------------------------------*/
    val32 = 48000;
    if (UAC_SamplingRateControl(uac_dev, UAC_SPEAKER, UAC_SET_CUR, &val32) == UAC_RET_OK) {
        printf("    Speaker's current sampling rate is %d.\n", val32);
    } else
        printf("    Failed to set speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's microphone.*/
    /*-------------------------------------------------------------*/
    if (UAC_SamplingRateControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, &val32) == UAC_RET_OK) {
        printf("    Microphone's current sampling rate is %d.\n", val32);
    } else
        printf("    Failed to get microphone's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's microphone.    */
    /*-------------------------------------------------------------*/
    val32 = 48000;
    if (UAC_SamplingRateControl(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, &val32) == UAC_RET_OK) {
        printf("    Microphone's current sampling rate is %d.\n", val32);
    } else
        printf("    Failed to set microphone's current sampling rate!\n");
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    int           t0;
    UAC_DEV_T     *uac_dev;

    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+----------------------------------------------+\n");
    printf("|                                              |\n");
    printf("|     USB Host Audio Class sample program      |\n");
    printf("|                                              |\n");
    printf("+----------------------------------------------+\n");

    enable_sys_ticks(100);

    USBH_Open();

#ifdef USBH_CLK_FROM_PLL2_DUAL_ROLE
    // Configure OTG function as Dual Role
    SYS_UnlockReg();
    SYS->USBPHY = 0x102;
#endif

    t0 = get_sys_ticks();
    while (get_sys_ticks() - t0 < POWER_UP_DELAY) ;

    UAC_Init();

    printf("Wait until any Audio Class devices connected...\n");
    while (1) {
        USBH_ProcessHubEvents();             /* USB Host port detect polling and management */

        uac_dev = UAC_GetDeviceList();
        if (uac_dev != NULL)
            break;
    }

    uac_control_example(uac_dev);

    if (UAC_InstallIsoInCbFun(uac_dev, au_in_buff, 2048, audio_in_callback) != UAC_RET_OK) {
        printf("Failed to install audio-in callback function!\n");
        goto err_out;
    }

    if (UAC_InstallIsoOutCbFun(uac_dev, audio_out_callback) != UAC_RET_OK) {
        printf("Failed to install audio-out callback function!\n");
        goto err_out;
    }

    while (1) {
        au_in_cnt = 0;
        au_out_cnt = 0;

        printf("\nStart audio output stream...\n");
        UAC_StartIsoOutPipe(uac_dev);

        printf("\nStart audio input stream...\n");
        UAC_StartIsoInPipe(uac_dev);

        while (au_in_cnt < 64*1024);

        UAC_StopIsoInPipe(uac_dev);
        printf("64 KB bytes audio data received.\n");
        printf("Audio input stream stopped.\n");

        while (au_out_cnt < 64*1024) ;

        UAC_StopIsoOutPipe(uac_dev);
        printf("64 KB bytes audio data send.\n");
        printf("Audio output stream stopped.\n");

        getchar();
    }

err_out:
    printf("\nFailed!\n");
    while (1);
}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
