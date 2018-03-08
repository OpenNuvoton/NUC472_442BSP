/******************************************************************************
 * @file     zero.c
 * @brief    NUC472/NUC442 USBD driver Sample file
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/11/17 5:34p $
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC472_442.h"
#include "zero.h"

/*--------------------------------------------------------------------------*/
/* Global variables for Control Pipe */

/* USB flow control variables */

uint32_t g_u32EpAMaxPacketSize;
uint32_t g_u32EpBMaxPacketSize;
uint8_t volatile g_u8ZeroOutShortPacket = 0;

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t g_u8ZeroBase[512];
#else
uint8_t g_u8ZeroBase[512] __attribute__((aligned(4)));
#endif



void USBD_IRQHandler(void)
{
    __IO uint32_t IrqStL, IrqSt;

    IrqStL = USBD->GINTSTS & USBD->GINTEN;    /* get interrupt status */

    if (!IrqStL)    return;

    /* USB interrupt */
    if (IrqStL & USBD_GINTSTS_USBIF_Msk)
    {

        IrqSt = USBD->BUSINTSTS & USBD->BUSINTEN;

        if (IrqSt & USBD_BUSINTSTS_SOFIF_Msk)
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_SOFIF_Msk);

        if (IrqSt & USBD_BUSINTSTS_RSTIF_Msk)
        {
            USBD_SwReset();
            USBD_ResetDMA();
            USBD->EP[EPA].EPRSPCTL = USBD_EPRSPCTL_FLUSH_Msk;
            USBD->EP[EPB].EPRSPCTL = USBD_EPRSPCTL_FLUSH_Msk;

            if (USBD->OPER & 0x04)  /* high speed */
                Zero_InitForHighSpeed();
            else                    /* full speed */
                Zero_InitForFullSpeed();
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

            if (!(USBD->DMACTL & USBD_DMACTL_DMARD_Msk))  //OUT token
            {
                USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk);
            }

            if (USBD->DMACTL & USBD_DMACTL_DMARD_Msk)  //IN token
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

    /* bulk in */
    if (IrqStL & USBD_GINTSTS_EPAIF_Msk)
    {
        IrqSt = USBD->EP[EPA].EPINTSTS & USBD->EP[EPA].EPINTEN;
        USBD_ENABLE_EP_INT(EPA, 0);
        USBD_CLR_EP_INT_FLAG(EPA, IrqSt);
    }
    /* bulk out */
    if (IrqStL & USBD_GINTSTS_EPBIF_Msk)
    {
        IrqSt = USBD->EP[EPB].EPINTSTS & USBD->EP[EPB].EPINTEN;
        if (IrqSt & USBD_EPINTSTS_SHORTRXIF_Msk)
        {
            g_u8ZeroOutShortPacket = 0;
        }
        USBD_ENABLE_EP_INT(EPB, 0);
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

void Zero_InitForHighSpeed(void)
{
    /*****************************************************/
    /* EPA ==> Bulk IN endpoint, address 2 */
    USBD_SetEpBufAddr(EPA, EPA_BUF_BASE, EPA_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPA, EPA_MAX_PKT_SIZE);
    USBD_ConfigEp(EPA, BULK_IN_EP_NUM, USB_EP_CFG_TYPE_BULK, USB_EP_CFG_DIR_IN);

    /* EPB ==> Bulk OUT endpoint, address 1 */
    USBD_SetEpBufAddr(EPB, EPB_BUF_BASE, EPB_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPB, EPB_MAX_PKT_SIZE);
    USBD_ConfigEp(EPB, BULK_OUT_EP_NUM, USB_EP_CFG_TYPE_BULK, USB_EP_CFG_DIR_OUT);
    USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk);

    g_u32EpAMaxPacketSize = EPA_MAX_PKT_SIZE;
    g_u32EpBMaxPacketSize = EPB_MAX_PKT_SIZE;
}

void Zero_InitForFullSpeed(void)
{
    /*****************************************************/
    /* EPA ==> Bulk IN endpoint, address 2 */
    USBD_SetEpBufAddr(EPA, EPA_BUF_BASE, EPA_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPA, EPA_OTHER_MAX_PKT_SIZE);
    USBD_ConfigEp(EPA, BULK_IN_EP_NUM, USB_EP_CFG_TYPE_BULK, USB_EP_CFG_DIR_IN);

    /* EPB ==> Bulk OUT endpoint, address 1 */
    USBD_SetEpBufAddr(EPB, EPB_BUF_BASE, EPB_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPB, EPB_OTHER_MAX_PKT_SIZE);
    USBD_ConfigEp(EPB, BULK_OUT_EP_NUM, USB_EP_CFG_TYPE_BULK, USB_EP_CFG_DIR_OUT);
    USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk);

    g_u32EpAMaxPacketSize = EPA_OTHER_MAX_PKT_SIZE;
    g_u32EpBMaxPacketSize = EPB_OTHER_MAX_PKT_SIZE;
}

void Zero_Init(void)
{
    /* Configure USB controller */
    /* Enable USB BUS, CEP and EPA , EPB global interrupt */
    USBD_ENABLE_USB_INT(USBD_GINTEN_USBIE_Msk|USBD_GINTEN_CEPIE_Msk|USBD_GINTEN_EPAIE_Msk|USBD_GINTEN_EPBIE_Msk);
    /* Enable BUS interrupt */
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_RESUMEIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);
    /* Reset Address to 0 */
    USBD_SET_ADDR(0);

    /*****************************************************/
    /* Control endpoint */
    USBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_BUF_LEN);
    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);

    Zero_InitForHighSpeed();
}

/*zero bulk out by blocking mode*/
void Zero_BulkOut_B(uint32_t u32Addr, uint32_t u32Len)
{
    uint32_t u32Loop;
    uint32_t i;

    /* bulk out, dma write, epnum = 1 */
    USBD_SET_DMA_WRITE(BULK_OUT_EP_NUM);

    u32Loop = u32Len / USBD_MAX_DMA_LEN;
    for (i=0; i<u32Loop; i++)
    {
        Zero_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, USBD_MAX_DMA_LEN, 1);
    }

    u32Loop = u32Len % USBD_MAX_DMA_LEN;
    if (u32Loop)
    {
        Zero_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, u32Loop, 1);
    }
}

/*zero bulk in by blocking mode*/
void Zero_BulkIn_B(uint32_t u32Addr, uint32_t u32Len)
{
    uint32_t u32Loop;
    uint32_t i, addr, count;

    /* bulk in, dma read, epnum = 2 */
    USBD_SET_DMA_READ(BULK_IN_EP_NUM);

    u32Loop = u32Len / USBD_MAX_DMA_LEN;
    for (i=0; i<u32Loop; i++)
    {
        USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_TXPKIEN_Msk);
        g_usbd_ShortPacket = 0;
        while(1)
        {
            if (USBD_GET_EP_INT_FLAG(EPA) & USBD_EPINTSTS_BUFEMPTYIF_Msk)
            {
                Zero_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, USBD_MAX_DMA_LEN, 1);
                break;
            }
        }
    }

    addr = u32Addr + i * USBD_MAX_DMA_LEN;
    u32Loop = u32Len % USBD_MAX_DMA_LEN;
    if (u32Loop)
    {
        count = u32Loop / g_u32EpAMaxPacketSize;
        if (count)
        {
            USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_TXPKIEN_Msk);
            g_usbd_ShortPacket = 0;
            while(1)
            {
                if (USBD_GET_EP_INT_FLAG(EPA) & USBD_EPINTSTS_BUFEMPTYIF_Msk)
                {
                    Zero_ActiveDMA(addr, count * g_u32EpAMaxPacketSize, 1);
                    break;
                }
            }
            addr += (count * g_u32EpAMaxPacketSize);
        }
        count = u32Loop % g_u32EpAMaxPacketSize;
        if (count)
        {
            USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_TXPKIEN_Msk);
            g_usbd_ShortPacket = 1;
            while(1)
            {
                if (USBD_GET_EP_INT_FLAG(EPA) & USBD_EPINTSTS_BUFEMPTYIF_Msk)
                {
                    Zero_ActiveDMA(addr, count, 1);
                    break;
                }
            }
        }
    }
}

/*zero bulk out by non_blocking mode*/
/*
    FALSE : error
    TRUE  : start DMA
*/
int Zero_BulkOut(uint32_t u32Addr, uint32_t u32Len)
{

    /* bulk out, dma write, epnum = 1 */
    USBD_SET_DMA_WRITE(BULK_OUT_EP_NUM);

    if(u32Len > USBD_MAX_DMA_LEN)
        return FALSE;

    Zero_ActiveDMA(u32Addr, u32Len, 0);

    return TRUE;
}

/*zero bulk in by non_blocking mode*/
/*
    the lenght u32Len must be multiple of EPA_MAX_PKT_SIZE
    if u32Len less than EPA_MAX_PKT_SIZE, shortPacket should be true
*/

int Zero_BulkIn(uint32_t u32Addr, uint32_t u32Len, uint32_t shortPacket)
{

    /* bulk in, dma read, epnum = 2 */
    USBD_SET_DMA_READ(BULK_IN_EP_NUM);

    /*u32Len must be multiple of EPA_MAX_PKT_SIZE*/
    if((u32Len > USBD_MAX_DMA_LEN) ||
            ((u32Len>g_u32EpAMaxPacketSize)&&(u32Len%g_u32EpAMaxPacketSize)))
        return FALSE;

    if ((USBD_GET_EP_INT_FLAG(EPA) & USBD_EPINTSTS_BUFEMPTYIF_Msk) == 0)//USB SRAM not empty
        return FALSE;

    USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_TXPKIEN_Msk);

    if(shortPacket)
    {
        USBD->EP[EPA].EPRSPCTL = USB_EP_RSPCTL_SHORTTXEN;    // short packet
    }

    Zero_ActiveDMA(u32Addr, u32Len, 0);

    return TRUE;

}


void Zero_ReceiveShort(uint32_t u32Buf, int *len, int wait)
{
    uint32_t volatile i, receive_len, loop;
    uint8_t *ptr = (uint8_t *)u32Buf;

    /* bulk out, dma write, epnum = 2 */
    USBD_SET_DMA_WRITE(BULK_OUT_EP_NUM);
    /* disable buffer */
    g_u8ZeroOutShortPacket = 1;
    USBD->EP[EPB].EPRSPCTL = USBD_EPRSPCTL_DISBUF_Msk;
    USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_SHORTRXIEN_Msk);//enable RX short packet interrupt

    /* Enable BUS interrupt */
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);

    USBD_SET_DMA_ADDR(u32Buf);
    USBD_SET_DMA_LEN(g_u32EpBMaxPacketSize);
    loop = g_u32EpBMaxPacketSize/32;

    g_usbd_DmaDone = 0;
    USBD_ENABLE_DMA();

    while(wait)
    {
        if (g_u8ZeroOutShortPacket == 0)
            break;

        if (!USBD_IS_ATTACHED())
            break;
    }

    if(wait)
    {
        /* get data from FIFO */
        receive_len = USBD->EP[EPB].EPDATCNT & 0xffff;
        loop = loop - (USBD->EP[EPB].EPDATCNT >> 16)& 0x7FFF;//loops that DMA has transfered
        ptr += (loop * 32);//each loop is 32B
        for (i=0; i<receive_len; i++)
            *(ptr+i) = USBD->EP[EPB].EPDAT_BYTE;
        // reset DMA
        USBD->DMACTL = 0x80;
        USBD->DMACTL = 0x00;

        USBD->EP[EPB].EPRSPCTL = 0;

        *len = receive_len;
    }
}

void Zero_MainProcess(void)
{
    int len;
    Zero_ReceiveShort((uint32_t)g_u8ZeroBase, &len, 1);

    Zero_BulkOut((uint32_t)g_u8ZeroBase,64);
    while(g_usbd_DmaDone == 0);
    if(*((uint8_t *)(g_u8ZeroBase))==0x31)
    {
        *((uint8_t *)(g_u8ZeroBase)) = 0x1;
        *((uint8_t *)(g_u8ZeroBase+1)) = 0x2;
        *((uint8_t *)(g_u8ZeroBase+2)) = 0x3;
        *((uint8_t *)(g_u8ZeroBase+3)) = 0x4;

        Zero_BulkIn((uint32_t)g_u8ZeroBase, 5, 1);
        while(g_usbd_DmaDone == 0);
    }

}

void Zero_ActiveDMA(uint32_t u32Addr, uint32_t u32Len, int wait)
{
    /* Enable BUS interrupt */
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);

    USBD_SET_DMA_ADDR(u32Addr);
    USBD_SET_DMA_LEN(u32Len);
    g_usbd_DmaDone = 0;
    USBD_ENABLE_DMA();
    while(wait)
    {
        if (g_usbd_DmaDone)
            break;

        if (!USBD_IS_ATTACHED())
            break;
    }
}

