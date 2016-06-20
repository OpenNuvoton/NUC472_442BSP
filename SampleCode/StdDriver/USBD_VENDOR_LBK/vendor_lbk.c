/**************************************************************************//**
 * @file     vendor_lbk.c
 * @version  V1.00
 * $Date: 14/11/17 5:48p $
 * @brief    NUC472/NUC442 USBD driver Sample file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC472_442.h"
#include "vendor_lbk.h"

/* Define DMA Maximum Transfer length */
#define USBD_MAX_DMA_LEN    0x1000


static uint32_t  g_CtrlLbkBuff[16];
static uint32_t  g_IntLbkBuff[16];
static uint32_t  g_IsoLbkBuff[256];

uint8_t volatile g_u8EPAReady = 0;

uint8_t volatile g_u8LbkStart = 0;
uint8_t volatile g_IsoOutRequest = 0;
uint8_t volatile g_IsoInTxDone = 1;
uint8_t volatile g_BulkOutRequest = 0;
uint8_t volatile g_BulkInTxDone = 1;

static void LBK_ActiveDMA(uint32_t u32Addr, uint32_t u32Len);


/* Interrupt OUT handler */
void EPB_Handler(void)
{
    int       i, len;

    len = USBD->EP[EPB].EPDATCNT & 0xffff;

    if (len != EPB_MAX_PKT_SIZE)
        printf("Int-OUT error, rx len=%d\n", len);

    for (i = 0; i < len/4; i++)
        g_IntLbkBuff[i] = USBD->EP[EPB].EPDAT;

    if (g_u8EPAReady) {
        g_u8EPAReady = 0;

        /* loop to interrup in pipe */
        for (i = 0; i < EPA_MAX_PKT_SIZE/4; i++)
            USBD->EP[EPA].EPDAT = g_IntLbkBuff[i];
        USBD->EP[EPA].EPRSPCTL = USB_EP_RSPCTL_SHORTTXEN;
        USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_INTKIEN_Msk);
    } else {
        // Not an error. USB Host did not get the last interrupt-in packet.
    }
}


/* Isochronous IN handler */
void EPC_Handler(void)
{
    int   i;

    for (i = 0; i < EPC_MAX_PKT_SIZE/4; i++)
        USBD->EP[EPC].EPDAT = g_IsoLbkBuff[i];
}


void USBD_IRQHandler(void)
{
    __IO uint32_t IrqStL, IrqSt;

    IrqStL = USBD->GINTSTS & USBD->GINTEN;    /* get interrupt status */

    if (!IrqStL)    return;

    /* USB interrupt */
    if (IrqStL & USBD_GINTSTS_USBIF_Msk) {
        IrqSt = USBD->BUSINTSTS & USBD->BUSINTEN;

        if (IrqSt & USBD_BUSINTSTS_SOFIF_Msk)
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_SOFIF_Msk);

        if (IrqSt & USBD_BUSINTSTS_RSTIF_Msk) {
            USBD_SwReset();
            g_u8LbkStart = 0;

            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            USBD_SET_ADDR(0);
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_RESUMEIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_RSTIF_Msk);
            USBD_CLR_CEP_INT_FLAG(0x1ffc);
        }

        if (IrqSt & USBD_BUSINTSTS_RESUMEIF_Msk) {
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_RESUMEIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_SUSPENDIF_Msk) {
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk | USBD_BUSINTEN_RESUMEIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_SUSPENDIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_HISPDIF_Msk) {
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_HISPDIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_DMADONEIF_Msk) {
            g_usbd_DmaDone = 1;
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_DMADONEIF_Msk);

            if (USBD->DMACTL & USBD_DMACTL_DMARD_Msk) {
                if (g_usbd_ShortPacket == 1) {
                    USBD->EP[EPA].EPRSPCTL = USBD->EP[EPA].EPRSPCTL & 0x10 | USB_EP_RSPCTL_SHORTTXEN;    // packet end
                    g_usbd_ShortPacket = 0;
                }
            }
        }

        if (IrqSt & USBD_BUSINTSTS_PHYCLKVLDIF_Msk)
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_PHYCLKVLDIF_Msk);

        if (IrqSt & USBD_BUSINTSTS_VBUSDETIF_Msk) {
            if (USBD_IS_ATTACHED()) {
                /* USB Plug In */
                USBD_ENABLE_USB();
            } else {
                /* USB Un-plug */
                USBD_DISABLE_USB();
            }
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_VBUSDETIF_Msk);
        }
    }

    if (IrqStL & USBD_GINTSTS_CEPIF_Msk) {
        IrqSt = USBD->CEPINTSTS & USBD->CEPINTEN;

        //printf("IrqSt = 0x%x\n", IrqSt);

        if (IrqSt & USBD_CEPINTSTS_SETUPTKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_SETUPTKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_SETUPPKIF_Msk) {
            g_u8LbkStart = 1;
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_SETUPPKIF_Msk);
            USBD_ProcessSetupPacket();
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_OUTTKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_OUTTKIF_Msk);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_INTKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
            if (!(IrqSt & USBD_CEPINTSTS_STSDONEIF_Msk)) {
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_TXPKIEN_Msk);
                USBD_CtrlIn();
            } else {
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_TXPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
            }
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_PINGIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_PINGIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_TXPKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
            USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
            if (g_usbd_CtrlInSize) {
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
            } else {
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
            }
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_RXPKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_RXPKIF_Msk);
            USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_NAKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_NAKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_STALLIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STALLIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_ERRIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_ERRIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_STSDONEIF_Msk) {
            USBD_UpdateDeviceState();
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_BUFFULLIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_BUFFULLIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_BUFEMPTYIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_BUFEMPTYIF_Msk);
            return;
        }
    }

    /* Interrupt in */
    if (IrqStL & USBD_GINTSTS_EPAIF_Msk) {
        IrqSt = USBD->EP[EPA].EPINTSTS & USBD->EP[EPA].EPINTEN;
        g_u8EPAReady = 1;
        USBD_ENABLE_EP_INT(EPA, 0);
        USBD_CLR_EP_INT_FLAG(EPA, IrqSt);
    }

    /* Interrupt out */
    if (IrqStL & USBD_GINTSTS_EPBIF_Msk) {
        IrqSt = USBD->EP[EPB].EPINTSTS & USBD->EP[EPB].EPINTEN;
        EPB_Handler();
        USBD_CLR_EP_INT_FLAG(EPB, IrqSt);
    }

    /* Isochronous in */
    if (IrqStL & USBD_GINTSTS_EPCIF_Msk) {
        IrqSt = USBD->EP[EPC].EPINTSTS & USBD->EP[EPC].EPINTEN;
        g_IsoInTxDone = 1;
        USBD_ENABLE_EP_INT(EPC, 0);
        USBD_CLR_EP_INT_FLAG(EPC, IrqSt);
    }

    /* Isochronous out */
    if (IrqStL & USBD_GINTSTS_EPDIF_Msk) {
        IrqSt = USBD->EP[EPD].EPINTSTS & USBD->EP[EPD].EPINTEN;
        g_IsoOutRequest = 1;
        USBD_ENABLE_EP_INT(EPD, 0);
        USBD_CLR_EP_INT_FLAG(EPD, IrqSt);
    }

    /* Bulk in */
    if (IrqStL & USBD_GINTSTS_EPEIF_Msk) {
        IrqSt = USBD->EP[EPE].EPINTSTS & USBD->EP[EPE].EPINTEN;
        if (USBD_GET_EP_INT_FLAG(EPE) & USBD_EPINTSTS_BUFEMPTYIF_Msk) {
            g_BulkInTxDone = 1;
            USBD_ENABLE_EP_INT(EPE, 0);
        } else {
            USBD_ENABLE_EP_INT(EPE, USBD_EPINTEN_INTKIEN_Msk);
        }
        USBD_CLR_EP_INT_FLAG(EPE, IrqSt);
    }

    /* Bulk out */
    if (IrqStL & USBD_GINTSTS_EPFIF_Msk) {
        IrqSt = USBD->EP[EPF].EPINTSTS & USBD->EP[EPF].EPINTEN;
        g_BulkOutRequest = 1;
        USBD_ENABLE_EP_INT(EPF, 0);
        USBD_CLR_EP_INT_FLAG(EPF, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPGIF_Msk) {
        IrqSt = USBD->EP[EPG].EPINTSTS & USBD->EP[EPG].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPG, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPHIF_Msk) {
        IrqSt = USBD->EP[EPH].EPINTSTS & USBD->EP[EPH].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPH, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPIIF_Msk) {
        IrqSt = USBD->EP[EPI].EPINTSTS & USBD->EP[EPI].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPI, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPJIF_Msk) {
        IrqSt = USBD->EP[EPJ].EPINTSTS & USBD->EP[EPJ].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPJ, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPKIF_Msk) {
        IrqSt = USBD->EP[EPK].EPINTSTS & USBD->EP[EPK].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPK, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPLIF_Msk) {
        IrqSt = USBD->EP[EPL].EPINTSTS & USBD->EP[EPL].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPL, IrqSt);
    }
}


/*--------------------------------------------------------------------------*/
/**
  * @brief  Initialize USB device
  * @param  None.
  * @retval None.
  */
void LBK_Init(void)
{
    /* Configure USB controller */
    /* Enable USB BUS, CEP and EPA global interrupt */
    USBD_ENABLE_USB_INT(USBD_GINTEN_USBIE_Msk|USBD_GINTEN_CEPIE_Msk|
                        USBD_GINTEN_EPAIE_Msk|USBD_GINTEN_EPBIE_Msk|
                        USBD_GINTEN_EPCIE_Msk|USBD_GINTEN_EPDIE_Msk|
                        USBD_GINTEN_EPEIE_Msk|USBD_GINTEN_EPFIE_Msk);
    /* Enable BUS interrupt */
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_RESUMEIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);
    /* Reset Address to 0 */
    USBD_SET_ADDR(0);

    /*****************************************************/
    /* Control endpoint */
    USBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_BUF_LEN);
    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);

    /* EPA ==> Interrupt IN endpoint */
    USBD_SetEpBufAddr(EPA, EPA_BUF_BASE, EPA_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPA, EPA_MAX_PKT_SIZE);
    USBD_ConfigEp(EPA, INT_IN_EP_NUM, USB_EP_CFG_TYPE_INT, USB_EP_CFG_DIR_IN);

    /* EPB ==> Interrupt OUT endpoint */
    USBD_SetEpBufAddr(EPB, EPB_BUF_BASE, EPB_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPB, EPB_MAX_PKT_SIZE);
    USBD_ConfigEp(EPB, INT_OUT_EP_NUM, USB_EP_CFG_TYPE_INT, USB_EP_CFG_DIR_OUT);
    USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk|USBD_EPINTEN_SHORTRXIEN_Msk);

    /* EPC ==> Isochronous IN endpoint */
    USBD_SetEpBufAddr(EPC, EPC_BUF_BASE, EPC_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPC, EPC_MAX_PKT_SIZE);
    USBD_ConfigEp(EPC, ISO_IN_EP_NUM, USB_EP_CFG_TYPE_ISO, USB_EP_CFG_DIR_IN);
    USBD_ENABLE_EP_INT(EPC, USBD_EPINTEN_TXPKIEN_Msk);
    //USBD->EPCRSPCTL &= ~0x6;

    /* EPD ==> Isochronous OUT endpoint */
    USBD_SetEpBufAddr(EPD, EPD_BUF_BASE, EPD_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPD, EPD_MAX_PKT_SIZE);
    USBD_ConfigEp(EPD, ISO_OUT_EP_NUM, USB_EP_CFG_TYPE_ISO, USB_EP_CFG_DIR_OUT);
    USBD_ENABLE_EP_INT(EPD, USBD_EPINTEN_RXPKIEN_Msk|USBD_EPINTEN_SHORTRXIEN_Msk);

    /* EPE ==> Bulk IN endpoint */
    USBD_SetEpBufAddr(EPE, EPE_BUF_BASE, EPE_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPE, EPE_MAX_PKT_SIZE);
    USBD_ConfigEp(EPE, BULK_IN_EP_NUM, USB_EP_CFG_TYPE_BULK, USB_EP_CFG_DIR_IN);
    //USBD_ENABLE_EP_INT(EPE, USBD_EPINTEN_TXPKIEN_Msk);

    /* EPF ==> Bulk OUT endpoint */
    USBD_SetEpBufAddr(EPF, EPF_BUF_BASE, EPF_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPF, EPF_MAX_PKT_SIZE);
    USBD_ConfigEp(EPF, BULK_OUT_EP_NUM, USB_EP_CFG_TYPE_BULK, USB_EP_CFG_DIR_OUT);
    USBD_ENABLE_EP_INT(EPF, USBD_EPINTEN_RXPKIEN_Msk|USBD_EPINTEN_SHORTRXIEN_Msk);

    g_u8EPAReady = 1;
    g_IsoOutRequest = 0;
    g_IsoInTxDone = 1;
    g_BulkOutRequest = 0;
    g_BulkInTxDone = 1;
}

void Vendor_ClassRequest(void)
{
    if (gUsbCmd.bmRequestType & 0x80) {
        // Device to host
        switch (gUsbCmd.bRequest) {
        case REQ_GET_DATA:
            USBD_PrepareCtrlIn((uint8_t *)g_CtrlLbkBuff, CEP_MAX_PKT_SIZE);
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
            break;

        default:
            /* Setup error, stall the device */
            USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
            break;
        }
    } else {
        // Host to device
        switch (gUsbCmd.bRequest) {
        case REQ_SET_DATA:
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_OUTTKIEN_Msk | USBD_CEPINTEN_RXPKIEN_Msk);
            USBD_CtrlOut((uint8_t *)g_CtrlLbkBuff, gUsbCmd.wLength);

            /* Status stage */
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
            USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
            break;

        default:
            /* Setup error, stall the device */
            USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
            break;
        }
    }
}


static void LBK_ActiveDMA(uint32_t u32Addr, uint32_t u32Len)
{
    /* Enable BUS interrupt */
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);

    USBD_SET_DMA_ADDR(u32Addr);
    USBD_SET_DMA_LEN(u32Len);
    g_usbd_DmaDone = 0;
    USBD_ENABLE_DMA();

    while(g_u8LbkStart) {
        if (g_usbd_DmaDone)
            break;

        if (!USBD_IS_ATTACHED())
            break;
    }
}

int  LBK_HasIsoOutReq(void)
{
    return g_IsoOutRequest;
}

void LBK_IsoOut(uint32_t u32Addr, uint32_t u32Len)
{
    //printf("[%d]\n", USBD->EPDDATCNT);

    u32Len = USBD->EP[EPD].EPDATCNT & 0xffff;

    /* isochronous out, dma write  */
    USBD_SET_DMA_WRITE(ISO_OUT_EP_NUM);
    g_usbd_ShortPacket = 0;

    if (u32Len)
        LBK_ActiveDMA(u32Addr, u32Len);

    g_IsoOutRequest = 0;
    USBD_ENABLE_EP_INT(EPD, USBD_EPINTEN_RXPKIEN_Msk);
}

int  LBK_IsoInXferDone(void)
{
    return g_IsoInTxDone;
}

void LBK_IsoInPushData(uint32_t u32Addr, uint32_t u32Len)
{
    /* isochronous in, dma read */
    USBD_SET_DMA_READ(ISO_IN_EP_NUM);

    LBK_ActiveDMA(u32Addr, u32Len);

    // re-enable Isochronous-In token interrupt
    g_IsoInTxDone = 0;
    USBD_ENABLE_EP_INT(EPC, USBD_EPINTEN_TXPKIEN_Msk);
}

int  LBK_HasBulkOutReq(void)
{
    return g_BulkOutRequest;
}

void LBK_BulkOut(uint32_t u32Addr, uint32_t u32Len)
{
    uint32_t u32Loop;
    uint32_t i;

    /* bulk out, dma write, epnum = 2 */
    USBD_SET_DMA_WRITE(BULK_OUT_EP_NUM);
    g_usbd_ShortPacket = 0;

    u32Loop = u32Len / USBD_MAX_DMA_LEN;
    for (i = 0; i < u32Loop; i++) {
        LBK_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, USBD_MAX_DMA_LEN);
    }

    u32Loop = u32Len % USBD_MAX_DMA_LEN;
    if (u32Loop) {
        LBK_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, u32Loop);
    }

    g_BulkOutRequest = 0;
    USBD_ENABLE_EP_INT(EPF, USBD_EPINTEN_RXPKIEN_Msk|USBD_EPINTEN_SHORTRXIEN_Msk);
}


int  LBK_BulkInXferDone(void)
{
    return g_BulkInTxDone;
}

void LBK_BulkInPushData(uint32_t u32Addr, uint32_t u32Len)
{
#if 1
    uint32_t u32Loop;
    uint32_t i;

    /* bulk out, dma write, epnum = 2 */
    USBD_SET_DMA_READ(BULK_IN_EP_NUM);
    g_usbd_ShortPacket = 0;

    u32Loop = u32Len / USBD_MAX_DMA_LEN;
    for (i = 0; i < u32Loop; i++) {
        LBK_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, USBD_MAX_DMA_LEN);
    }

    u32Loop = u32Len % USBD_MAX_DMA_LEN;
    if (u32Loop) {
        LBK_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, u32Loop);
    }
#else
    uint32_t u32Loop;
    uint32_t i, addr, count;

    /* bulk in, dma read  */
    USBD_SET_DMA_READ(BULK_IN_EP_NUM);

    u32Loop = u32Len / USBD_MAX_DMA_LEN;
    for (i=0; i<u32Loop; i++) {
        USBD_ENABLE_EP_INT(EPE, USBD_EPINTEN_TXPKIEN_Msk);
        g_usbd_ShortPacket = 0;
        while(1) {
            if (USBD_GET_EP_INT_FLAG(EPE) & USBD_EPINTSTS_BUFEMPTYIF_Msk) {
                LBK_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, USBD_MAX_DMA_LEN);
                break;
            }
        }
    }

    addr = u32Addr + i * USBD_MAX_DMA_LEN;
    u32Loop = u32Len % USBD_MAX_DMA_LEN;
    if (u32Loop) {
        count = u32Loop / EPE_MAX_PKT_SIZE;
        if (count) {
            USBD_ENABLE_EP_INT(EPE, USBD_EPINTEN_TXPKIEN_Msk);
            g_usbd_ShortPacket = 1;
            while(1) {
                if (USBD_GET_EP_INT_FLAG(EPE) & USBD_EPINTSTS_BUFEMPTYIF_Msk) {
                    LBK_ActiveDMA(addr, count * EPE_MAX_PKT_SIZE);
                    break;
                }
            }
            addr += (count * EPE_MAX_PKT_SIZE);
        }
        count = u32Loop % EPE_MAX_PKT_SIZE;
        if (count) {
            USBD_ENABLE_EP_INT(EPE, USBD_EPINTEN_TXPKIEN_Msk);
            g_usbd_ShortPacket = 1;
            while(1) {
                if (USBD_GET_EP_INT_FLAG(EPE) & USBD_EPINTSTS_BUFEMPTYIF_Msk) {
                    LBK_ActiveDMA(addr, count);
                    break;
                }
            }
        }
    }
#endif
    // re-enable Bulk-In token interrupt
    g_BulkInTxDone = 0;
    USBD_ENABLE_EP_INT(EPE, USBD_EPINTEN_BUFEMPTYIEN_Msk);
}

