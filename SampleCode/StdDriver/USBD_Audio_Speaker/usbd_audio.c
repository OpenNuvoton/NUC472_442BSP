/******************************************************************************
 * @file     usbd_audio.c
 * @brief    NuMicro series USBD driver Sample file
 * @version  1.0.0
 * @date     23, Sep, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC472_442.h"
#include "usbd_audio.h"

/*--------------------------------------------------------------------------*/
static volatile uint8_t bPlayVolumeLAdjust = FALSE;
static volatile uint8_t bPlayVolumeRAdjust = FALSE;
static volatile uint8_t bIsI2CIdle = TRUE;

/*--------------------------------------------------------------------------*/
/* Global variables for Audio class */
uint8_t  g_usbd_PlayMute = 0x01;
int16_t  g_usbd_PlayVolumeL = 0x1000;
int16_t  g_usbd_PlayVolumeR = 0x1000;
int16_t  g_usbd_PlayMaxVolume = 0x7FFF;
int16_t  g_usbd_PlayMinVolume = 0x8000;
int16_t  g_usbd_PlayResVolume = 0x400;
uint32_t g_usbd_SampleRate = PLAY_RATE;

volatile uint8_t u8PlayEn = 0;
volatile uint8_t u8AudioPlaying=0;
volatile uint8_t u8DataCntInBuffer=0;
volatile uint8_t u8PDMATxIdx=0;

volatile uint8_t g_usbd_rxflag=0;

/* Player Buffer and its pointer */
#ifdef __ICCARM__
#pragma data_alignment=4
uint32_t PcmPlayBuff[PDMA_TXBUFFER_CNT][BUFF_LEN] = {0};
uint32_t PcmPlayBuffLen[PDMA_TXBUFFER_CNT] = {0};
#else
uint32_t PcmPlayBuff[PDMA_TXBUFFER_CNT][BUFF_LEN] __attribute__((aligned(4))) = {0};
uint32_t PcmPlayBuffLen[PDMA_TXBUFFER_CNT] __attribute__((aligned(4))) = {0};
#endif

/* Player Buffer and its pointer */
volatile uint32_t u32BufPlayIdx = 0;
volatile uint32_t u32PlayBufPos=0;

/*--------------------------------------------------------------------------*/
/**
 * @brief       USBD Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the USBD ISR
 */
void USBD_IRQHandler(void)
{
    __IO uint32_t IrqStL, IrqSt;

    IrqStL = USBD->GINTSTS & USBD->GINTEN;    /* get interrupt status */

    if (!IrqStL)    return;

    /* USB Bus interrupt */
    if (IrqStL & USBD_GINTSTS_USBIF_Msk)
    {
        IrqSt = USBD->BUSINTSTS & USBD->BUSINTEN;

        if (IrqSt & USBD_BUSINTSTS_SOFIF_Msk)
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_SOFIF_Msk);

        if (IrqSt & USBD_BUSINTSTS_RSTIF_Msk)
        {
            USBD_SwReset();
            USBD_ResetDMA();

            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            USBD_SET_ADDR(0);
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_RESUMEIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_RSTIF_Msk);
            USBD_CLR_CEP_INT_FLAG(0x1ffc);
        }

        if (IrqSt & USBD_BUSINTSTS_RESUMEIF_Msk)
        {
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_RESUMEIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_SUSPENDIF_Msk)
        {
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk | USBD_BUSINTEN_RESUMEIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_SUSPENDIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_HISPDIF_Msk)
        {
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_HISPDIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_DMADONEIF_Msk)
        {
            g_usbd_DmaDone = 1;
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_DMADONEIF_Msk);

            if (!(USBD->DMACTL & USBD_DMACTL_DMARD_Msk))
            {
                USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk);
            }

            if (USBD->DMACTL & USBD_DMACTL_DMARD_Msk)
            {
                if (g_usbd_ShortPacket == 1)
                {
                    USBD->EP[EPA].EPRSPCTL = USB_EP_RSPCTL_SHORTTXEN;    // packet end
                    g_usbd_ShortPacket = 0;
                }
            }
        }

        if (IrqSt & USBD_BUSINTSTS_PHYCLKVLDIF_Msk)
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_PHYCLKVLDIF_Msk);

        if (IrqSt & USBD_BUSINTSTS_VBUSDETIF_Msk)
        {
            if (USBD_IS_ATTACHED())
            {
                /* USB Plug In */
                USBD_ENABLE_USB();
            }
            else
            {
                /* USB Un-plug */
                USBD_DISABLE_USB();
            }
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_VBUSDETIF_Msk);
        }
    }

    /* Control endpoint interrupt */
    if (IrqStL & USBD_GINTSTS_CEPIF_Msk)
    {
        IrqSt = USBD->CEPINTSTS & USBD->CEPINTEN;

        if (IrqSt & USBD_CEPINTSTS_SETUPTKIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_SETUPTKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_SETUPPKIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_SETUPPKIF_Msk);
            USBD_ProcessSetupPacket();
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_OUTTKIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_OUTTKIF_Msk);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_INTKIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
            if (!(IrqSt & USBD_CEPINTSTS_STSDONEIF_Msk))
            {
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_TXPKIEN_Msk);
                USBD_CtrlIn();
            }
            else
            {
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_TXPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
            }
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_PINGIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_PINGIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_TXPKIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
            USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
            if (g_usbd_CtrlInSize)
            {
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
            }
            else
            {
                if (g_usbd_CtrlZero == 1)
                    USBD_SET_CEP_STATE(USB_CEPCTL_ZEROLEN);
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
            }
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_RXPKIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_RXPKIF_Msk);
            USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_NAKIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_NAKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_STALLIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STALLIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_ERRIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_ERRIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_STSDONEIF_Msk)
        {
            USBD_UpdateDeviceState();
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_BUFFULLIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_BUFFULLIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_BUFEMPTYIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_BUFEMPTYIF_Msk);
            return;
        }
    }

    /* Non-control endpoint interrupt */
    if (IrqStL & USBD_GINTSTS_EPAIF_Msk)
    {
        /* Isochronous in */
        IrqSt = USBD->EP[EPA].EPINTSTS & USBD->EP[EPA].EPINTEN;

        USBD_ENABLE_EP_INT(EPA, 0);
        USBD_CLR_EP_INT_FLAG(EPA, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPBIF_Msk)
    {
        /* Isochronous out */
        IrqSt = USBD->EP[EPB].EPINTSTS & USBD->EP[EPB].EPINTEN;

        EPB_Handler();
        USBD_CLR_EP_INT_FLAG(EPB, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPCIF_Msk)
    {
        IrqSt = USBD->EP[EPC].EPINTSTS & USBD->EP[EPC].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPC, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPDIF_Msk)
    {
        IrqSt = USBD->EP[EPD].EPINTSTS & USBD->EP[EPD].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPD, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPEIF_Msk)
    {
        IrqSt = USBD->EP[EPE].EPINTSTS & USBD->EP[EPE].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPE, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPFIF_Msk)
    {
        IrqSt = USBD->EP[EPF].EPINTSTS & USBD->EP[EPF].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPF, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPGIF_Msk)
    {
        IrqSt = USBD->EP[EPG].EPINTSTS & USBD->EP[EPG].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPG, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPHIF_Msk)
    {
        IrqSt = USBD->EP[EPH].EPINTSTS & USBD->EP[EPH].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPH, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPIIF_Msk)
    {
        IrqSt = USBD->EP[EPI].EPINTSTS & USBD->EP[EPI].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPI, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPJIF_Msk)
    {
        IrqSt = USBD->EP[EPJ].EPINTSTS & USBD->EP[EPJ].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPJ, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPKIF_Msk)
    {
        IrqSt = USBD->EP[EPK].EPINTSTS & USBD->EP[EPK].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPK, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPLIF_Msk)
    {
        IrqSt = USBD->EP[EPL].EPINTSTS & USBD->EP[EPL].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPL, IrqSt);
    }
}

/**
 * @brief       EPB Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EPB event
 */
/* Play */
void EPB_Handler(void)
{
    if (USBD->EP[EPB].EPINTSTS & USBD_EPINTSTS_RXPKIF_Msk)
    {
        g_usbd_rxflag = 1;
    }
}

/*--------------------------------------------------------------------------*/
/**
 * @brief       UAC Class Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure endpoints for UAC class
 */
void UAC_Init(void)
{
    /* Configure USB controller */
    USBD->OPER = 0; /* Full Speed */
    /* Enable USB BUS, CEP and EPA , EPB global interrupt */
    USBD_ENABLE_USB_INT(USBD_GINTEN_USBIE_Msk|USBD_GINTEN_CEPIE_Msk|USBD_GINTEN_EPBIE_Msk);
    /* Enable BUS interrupt */
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_RESUMEIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);
    /* Reset Address to 0 */
    USBD_SET_ADDR(0);

    /*****************************************************/
    /* Control endpoint */
    USBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_BUF_LEN);
    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);

    /*****************************************************/
    /* EPB ==> ISO OUT endpoint, address 2 */
    USBD_SetEpBufAddr(EPB, EPB_BUF_BASE, EPB_BUF_LEN);
#if ((PLAY_RATE == 44100) || (PLAY_RATE == 22050))
    USBD_SET_MAX_PAYLOAD(EPB, EPB_MAX_PKT_SIZE+4);
#else
    USBD_SET_MAX_PAYLOAD(EPB, EPB_MAX_PKT_SIZE);
#endif
    USBD_ConfigEp(EPB, ISO_OUT_EP_NUM, USB_EP_CFG_TYPE_ISO, USB_EP_CFG_DIR_OUT);
    USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk|USBD_EPINTEN_SHORTRXIEN_Msk);
}


/**
 * @brief       UAC class request
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process UAC class requests
 */
void UAC_ClassRequest(void)
{
    if (gUsbCmd.bmRequestType & 0x80)   /* request data transfer direction */
    {
        /* Get Feature Unit Control Request */
        // Device to host
        switch (gUsbCmd.bRequest)
        {
        /* Get current setting attribute */
        case UAC_GET_CUR:
        {
            switch ((gUsbCmd.wValue & 0xff00) >> 8)
            {
            /* Mute Control */
            case MUTE_CONTROL:
            {
                if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
                    USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMute, 1);

                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                break;
            }
            /* Volume Control */
            case VOLUME_CONTROL:
            {
                if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
                {
                    if((gUsbCmd.wValue & 0xff) == 1)
                        USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayVolumeL, 2);
                    else
                        USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayVolumeR, 2);
                }
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                break;
            }
            default:
            {
                /* Setup error, stall the device */
                USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
            }
            }
            break;
        }

        /* Get Minimum setting attribute */
        case UAC_GET_MIN:
        {
            switch ((gUsbCmd.wValue & 0xff00) >> 8)
            {
            /* Volume Control */
            case VOLUME_CONTROL:
            {
                if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
                    USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMinVolume, 2);

                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                break;
            }
            default:
                /* STALL control pipe */
                USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
            }
            break;
        }

        /* Get Maximum setting attribute */
        case UAC_GET_MAX:
        {
            switch ((gUsbCmd.wValue & 0xff00) >> 8)
            {
            /* Volume Control */
            case VOLUME_CONTROL:
            {
                if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
                    USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMaxVolume, 2);
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                break;
            }
            default:
                /* STALL control pipe */
                USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
            }
            break;
        }

        /* Get Resolution attribute */
        case UAC_GET_RES:
        {
            switch ((gUsbCmd.wValue & 0xff00) >> 8)
            {
            /* Volume Control */
            case VOLUME_CONTROL:
            {
                if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
                    USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayResVolume, 2);
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                break;
            }
            default:
                /* STALL control pipe */
                USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
            }
            break;
        }

        default:
        {
            /* Setup error, stall the device */
            USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
        }
        }
    }
    else
    {
        // Host to device
        switch (gUsbCmd.bRequest)
        {
        /* Set Current setting attribute */
        case UAC_SET_CUR:
        {
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_OUTTKIEN_Msk | USBD_CEPINTEN_RXPKIEN_Msk);
            if ((gUsbCmd.bmRequestType & 0x0f) == 0x2)      /* request to endpoint */
            {
                USBD_CtrlOut((uint8_t *)&g_usbd_SampleRate, gUsbCmd.wLength);

                /* Status stage */
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
            }
            else     /* request to interface */
            {
                switch ((gUsbCmd.wValue & 0xff00) >> 8)
                {
                /* Mute Control */
                case MUTE_CONTROL:
                    if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
                        USBD_CtrlOut((uint8_t *)&g_usbd_PlayMute, gUsbCmd.wLength);

                    /* Status stage */
                    USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                    USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
                    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
                    break;

                /* Volume Control */
                case VOLUME_CONTROL:
                    if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
                    {
                        if (((gUsbCmd.wValue >> 8) & 0xff) == 1)
                        {
                            USBD_CtrlOut((uint8_t *)&g_usbd_PlayVolumeL, gUsbCmd.wLength);
                            if(g_usbd_PlayVolumeL & 0x8000)
                                g_usbd_PlayVolumeL = (g_usbd_PlayVolumeL & 0x7FFF) >> 8;
                            else
                                g_usbd_PlayVolumeL = (g_usbd_PlayVolumeL >> 7);
                            bPlayVolumeLAdjust = TRUE;
                        }
                        else
                        {
                            USBD_CtrlOut((uint8_t *)&g_usbd_PlayVolumeR, gUsbCmd.wLength);
                            if(g_usbd_PlayVolumeR & 0x8000)
                                g_usbd_PlayVolumeR = (g_usbd_PlayVolumeR & 0x7FFF) >> 8;
                            else
                                g_usbd_PlayVolumeR = (g_usbd_PlayVolumeR >> 7);
                            bPlayVolumeRAdjust = TRUE;
                        }
                    }
                    /* Status stage */
                    USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                    USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
                    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
                    break;

                default:
                    /* STALL control pipe */
                    USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
                    break;
                }
            }
            break;
        }

        default:
        {
            /* Setup error, stall the device */
            USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
            break;
        }
        }
    }
}

/**
 * @brief       Set Interface standard request
 *
 * @param[in]   u32AltInterface Interface
 *
 * @return      None
 *
 * @details     This function is used to set UAC Class relative setting
 */
void UAC_SetInterface(uint32_t u32AltInterface)
{
    if ((gUsbCmd.wIndex & 0xff) == 1)
    {
        /* Audio Iso OUT interface */
        if (u32AltInterface == 1)   /* start play */
            UAC_DeviceEnable();
        else
            UAC_DeviceDisable();    /* stop play */
    }
}

/*******************************************************************/
/* For I2C transfer */
__IO uint32_t EndFlag0 = 0;
uint8_t Device_Addr0 = 0x1A;                /* WAU8822 Device ID */
uint8_t Tx_Data0[2];
uint8_t DataCnt0;

typedef enum
{
    E_RS_NONE,          // no re-sampling
    E_RS_UP,            // up sampling
    E_RS_DOWN           // down sampling
} RESAMPLE_STATE_T;

static void Delay(uint32_t t)
{
    volatile int32_t delay;

    delay = t;

    while(delay-- >= 0);
}

void RecoveryFromArbLost(void)
{
    I2C3->CTL &= ~I2C_CTL_I2CEN_Msk;
    I2C3->CTL |= I2C_CTL_I2CEN_Msk;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of WAU8822 with I2C3                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{
    bIsI2CIdle = FALSE;
restart:
    I2C_START(I2C3);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, 0x1A<<1);
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);
    if(I2C_GET_STATUS(I2C3) == 0x38)
    {
        RecoveryFromArbLost();
        goto restart;
    }
    else if(I2C_GET_STATUS(I2C3) != 0x18)
        goto stop;

    I2C_SET_DATA(I2C3, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);
    if(I2C_GET_STATUS(I2C3) == 0x38)
    {
        RecoveryFromArbLost();
        goto restart;
    }
    else if(I2C_GET_STATUS(I2C3) != 0x28)
        goto stop;

    I2C_SET_DATA(I2C3, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);
    if(I2C_GET_STATUS(I2C3) == 0x38)
    {
        RecoveryFromArbLost();
        goto restart;
    }
    else if(I2C_GET_STATUS(I2C3) != 0x28)
        goto stop;

stop:
    I2C_STOP(I2C3);
    while(I2C3->CTL & I2C_CTL_STO_Msk);

    bIsI2CIdle = TRUE;
    EndFlag0 = 1;
}

static void ATOM_I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{
    if(!bIsI2CIdle)
        while (EndFlag0 == 0);

    I2C_WriteWAU8822(u8addr, u16data);
}

uint32_t u32OldSampleRate=0;

/* config play sampling rate */
void WAU8822_ConfigSampleRate(uint32_t u32SampleRate)
{
    if(u32SampleRate == u32OldSampleRate)
        return;

    u32OldSampleRate = u32SampleRate;

    printf("[NAU8822] Configure Sampling Rate to %d\n", u32SampleRate);

    if((u32SampleRate % 8) == 0)
    {
        I2C_WriteWAU8822(36, 0x008);    //12.288Mhz
        I2C_WriteWAU8822(37, 0x00C);
        I2C_WriteWAU8822(38, 0x093);
        I2C_WriteWAU8822(39, 0x0E9);
    }
    else
    {
        I2C_WriteWAU8822(36, 0x007);    //11.2896Mhz
        I2C_WriteWAU8822(37, 0x021);
        I2C_WriteWAU8822(38, 0x161);
        I2C_WriteWAU8822(39, 0x026);
    }

    switch (u32SampleRate)
    {
    case 8000:
        I2C_WriteWAU8822(6, 0x1ED);   /* Divide by 12, 8K */
        I2C_WriteWAU8822(7, 0x00A);   /* 8K for internal filter coefficients */
        break;

    case 11025:
        I2C_WriteWAU8822(6, 0x1CD);    /* Divide by 8, 11.025K */
        break;

    case 22050:
        I2C_WriteWAU8822(6, 0x18D);    /* Divide by 4, 22.050K */
        break;

    case 16000:
        I2C_WriteWAU8822(6, 0x1AD);   /* Divide by 6, 16K */
        I2C_WriteWAU8822(7, 0x006);   /* 16K for internal filter coefficients */
        break;

    case 32000:
        I2C_WriteWAU8822(6, 0x16D);    /* Divide by 3, 32K */
        I2C_WriteWAU8822(7, 0x002);    /* 32K for internal filter coefficients */
        break;

    case 44100:
    case 48000:
        I2C_WriteWAU8822(6, 0x14D);    /* Divide by 2, 48K */
        I2C_WriteWAU8822(7, 0x000);    /* 48K for internal filter coefficients */
        break;
    }
}

void WAU8822_Setup(void)
{
    I2C_WriteWAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

    I2C_WriteWAU8822(1,  0x02F);
    I2C_WriteWAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteWAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteWAU8822(5,  0x000);   /* Commanding control and loop back mode (all disable) */
    I2C_WriteWAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteWAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteWAU8822(15, 0x1FF);   /* ADC left digital volume control */
    I2C_WriteWAU8822(16, 0x1FF);   /* ADC right digital volume control */
    I2C_WriteWAU8822(47, 0x106);   /* LAUX connected, and its Gain value */
    I2C_WriteWAU8822(48, 0x106);   /* RAUX connected, and its Gain value */
    I2C_WriteWAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteWAU8822(51, 0x001);   /* Right DAC connected to RMIX */
}

/**
  * @brief  UAC_DeviceEnable. To enable the device to play or record audio data.
  * @param  None.
  * @retval None.
  */
void UAC_DeviceEnable(void)
{
    /* Enable play hardware */
    u8PlayEn = 1;

    /* enable sound output on the LB board */
    PA4 = 0;

    TIMER_Start(TIMER0);
}


/**
  * @brief  UAC_DeviceDisable. To disable the device to play or record audio data.
  * @param  None.
  * @retval None.
  */
void UAC_DeviceDisable(void)
{
    /* Disable play hardware/stop play */
    u8PlayEn = 0;

    /* Disable I2S Tx function */
    I2S_DISABLE_TXDMA(I2S1);
    I2S_DISABLE_TX(I2S1);

    /* Disable PDMA channel */
    PDMA->CHCTL &= ~(PDMA_I2S_TX_CH<<1);
    printf("Stop Play ...\n");

    /* Reset some variables */
    u32BufPlayIdx = 0;
    u32PlayBufPos = 0;
    u8PDMATxIdx = 0;
    u8AudioPlaying = 0;
    u8DataCntInBuffer = 0;

    /* flush PCM buffer */
    memset(PcmPlayBuff, 0, sizeof(PcmPlayBuff));

    TIMER0->CTL |= TIMER_CTL_RSTCNT_Msk;
}

/**
  * @brief  GetPlayData, To get data from ISO OUT to play buffer.
  * @param  None.
  * @retval None.
  */
void UAC_GetPlayData(void)
{
    volatile uint32_t u32len, tmp;

    /* if buffer has enough data, play it!! */
    if (!u8AudioPlaying && (u8DataCntInBuffer >= (PDMA_TXBUFFER_CNT/2+1)))
    {
        AudioStartPlay(g_usbd_SampleRate);
        u8AudioPlaying = 1;
    }

    if (USBD->DMACTL & USBD_DMACTL_DMAEN_Msk)
        return;

    // get data from FIFO
    u32len = USBD->EP[EPB].EPDATCNT & 0xffff;
    tmp = u32len >> 2;  /* byte to word */

    /* Ring buffer check */
    if ((u32PlayBufPos + tmp) > BUFF_LEN)
    {
        PcmPlayBuffLen[u32BufPlayIdx] = u32PlayBufPos;
        u32PlayBufPos = 0;
        u32BufPlayIdx ++;

        /* change buffer index */
        if(u32BufPlayIdx >= PDMA_TXBUFFER_CNT)
            u32BufPlayIdx=0;

        /* increase data count in buffer */
        u8DataCntInBuffer ++;
    }

    /* active usbd DMA to read data from FIFO and then send to I2S */
    USBD_SET_DMA_WRITE(ISO_OUT_EP_NUM);
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);
    USBD_SET_DMA_ADDR((uint32_t)&PcmPlayBuff[u32BufPlayIdx][u32PlayBufPos]);
    USBD_SET_DMA_LEN(u32len);
    g_usbd_DmaDone = 0;
    USBD_ENABLE_DMA();

    /* wait usbd dma complete */
    while(1)
    {
        if (g_usbd_DmaDone)
            break;

        if (!USBD_IS_ATTACHED())
            break;
    }

    u32PlayBufPos += tmp;
    g_usbd_rxflag = 0;
}

/* adjust codec PLL */
void AdjustCodecPll(RESAMPLE_STATE_T r)
{
    static uint16_t tb0[3][3] = {{0x00C, 0x093, 0x0E9}, // 8.192
        {0x00E, 0x1D2, 0x1E3},  // * 1.005 = 8.233
        {0x009, 0x153, 0x1EF}
    }; // * .995 = 8.151
    static uint16_t tb1[3][3] = {{0x021, 0x131, 0x026}, // 7.526
        {0x024, 0x010, 0x0C5},  // * 1.005 = 7.563
        {0x01F, 0x076, 0x191}
    }; // * .995 = 7.488
    static RESAMPLE_STATE_T current = E_RS_NONE;
    int i, s;

    if(r == current)
        return;
    else
        current = r;
    switch(r)
    {
    case E_RS_UP:
        s = 1;
        break;
    case E_RS_DOWN:
        s = 2;
        break;
    case E_RS_NONE:
    default:
        s = 0;
    }

    if((g_usbd_SampleRate % 8) == 0)
    {
        for(i=0; i<3; i++)
            ATOM_I2C_WriteWAU8822(37+i, tb0[s][i]);
    }
    else
    {
        for(i=0; i<3; i++)
            ATOM_I2C_WriteWAU8822(37+i, tb1[s][i]);
    }
}

void AudioStartPlay(uint32_t u32SampleRate)
{
    UAC_DeviceEnable();

    /* Configure TX PDMA SG table */
    PDMA_WriteTxSGTable();

    /* Configure WAU8822 to specific sample rate */
    WAU8822_ConfigSampleRate(u32SampleRate);

    /* Enable I2S Tx function */
    I2S_ENABLE_TXDMA(I2S1);
    I2S_ENABLE_TX(I2S1);

    /* Enable PDMA channel */
    PDMA->CHCTL |= (PDMA_I2S_TX_CH<<1);
    printf("Start Play ...\n");

    // workaround for PDMA suspend
    PDMA->DSCT[PDMA_I2S_TX_CH].CTL = 0;
    PDMA->DSCT[PDMA_I2S_TX_CH].CTL = 2;
}

//======================================================
void TMR0_IRQHandler(void)
{
    TIMER_ClearIntFlag(TIMER0);

    if(u8PlayEn)
    {
        if((u8DataCntInBuffer >= (PDMA_TXBUFFER_CNT/2)) && (u8DataCntInBuffer <= (PDMA_TXBUFFER_CNT/2+1)))
            AdjustCodecPll(E_RS_NONE);
        else if(u8DataCntInBuffer >= (PDMA_TXBUFFER_CNT-2))
            AdjustCodecPll(E_RS_UP);
        else
            AdjustCodecPll(E_RS_DOWN);
    }
}




