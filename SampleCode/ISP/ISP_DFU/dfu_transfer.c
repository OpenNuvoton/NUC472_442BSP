/******************************************************************************//**
 * @file     dfu_transfer.c
 * @version  V1.00
 * @brief    NUC472_442 series USBD DFU transfer sample file
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"
#include "dfu_transfer.h"
#include "fmc_user.h"

extern uint32_t g_apromSize;
#define APROM_BLOCK_NUM         ((g_apromSize/TRANSFER_SIZE)-1)

uint32_t command_Count = 0;
uint8_t manifest_state = MANIFEST_COMPLETE;
dfu_status_struct dfu_status;
s_prog_struct prog_struct __attribute__((aligned(4))) = {{0}, 0, 0, APP_LOADED_ADDR};

void USBD_IRQHandler(void)
{
    __IO uint32_t IrqStL, IrqSt;
    IrqStL = USBD->GINTSTS & USBD->GINTEN;    /* get interrupt status */

    if (!IrqStL)
    {
        return;
    }

    /* USB interrupt */
    if (IrqStL & USBD_GINTSTS_USBIF_Msk)
    {
        IrqSt = USBD->BUSINTSTS & USBD->BUSINTEN;

        if (IrqSt & USBD_BUSINTSTS_RSTIF_Msk)
        {
            USBD_SwReset();
            USBD_ResetDMA();
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            USBD_SET_ADDR(0);
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk | USBD_BUSINTEN_RESUMEIEN_Msk | USBD_BUSINTEN_SUSPENDIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_RSTIF_Msk);
            USBD_CLR_CEP_INT_FLAG(0x1ffc);
        }

        if (IrqSt & USBD_BUSINTSTS_RESUMEIF_Msk)
        {
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk | USBD_BUSINTEN_SUSPENDIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_RESUMEIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_SUSPENDIF_Msk)
        {
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk | USBD_BUSINTEN_RESUMEIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_SUSPENDIF_Msk);
        }

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

        if (IrqSt & USBD_CEPINTSTS_SETUPPKIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_SETUPPKIF_Msk);
            USBD_ProcessSetupPacket();
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
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_TXPKIEN_Msk | USBD_CEPINTEN_STSDONEIEN_Msk);
            }
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
                {
                    USBD_SET_CEP_STATE(USB_CEPCTL_ZEROLEN);
                }
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk | USBD_CEPINTEN_STSDONEIEN_Msk);
            }

            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_RXPKIF_Msk)
        {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_RXPKIF_Msk);
            USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk | USBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_STSDONEIF_Msk)
        {
            USBD_UpdateDeviceState();
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            return;
        }
    }
}


/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */

void DFU_Init(void)
{
    /* Configure USB controller */
    /* Enable USB BUS, CEP and EPA global interrupt */
    USBD_ENABLE_USB_INT(USBD_GINTEN_USBIE_Msk | USBD_GINTEN_CEPIE_Msk);
    /* Enable BUS interrupt */
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk | USBD_BUSINTEN_RESUMEIEN_Msk | USBD_BUSINTEN_RSTIEN_Msk | USBD_BUSINTEN_VBUSDETIEN_Msk);
    /* Reset Address to 0 */
    USBD_SET_ADDR(0);
    /*****************************************************/
    /* Control endpoint */
    USBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_BUF_LEN);
    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk | USBD_CEPINTEN_STSDONEIEN_Msk);

    /* restore device default state */
    dfu_status.bStatus = STATUS_OK;
    dfu_status.bState = STATE_dfuIDLE;

    prog_struct.block_num = 0;
    prog_struct.data_len = 0;

}


void DFU_ClassRequest(void)
{
    uint32_t u32Len;
    uint8_t *address;

    if (gUsbCmd.bmRequestType & 0x80) 
    { /* request data transfer direction */
        // Device to host
        switch (gUsbCmd.bRequest)
        {
            case DFU_GETSTATUS: 
            {
                if (dfu_status.bState == STATE_dfuDNLOAD_SYNC)
                {
                    command_Count++;

                    if (command_Count == 5)
                    {
                        dfu_status.bState = STATE_dfuDNLOAD_IDLE;

                        FMC_Proc(FMC_ISPCMD_PROGRAM, prog_struct.block_num * TRANSFER_SIZE, prog_struct.block_num * TRANSFER_SIZE + prog_struct.data_len, (uint32_t *)prog_struct.buf);
                        //dfu_status.bStatus = STATUS_errWRITE;
                        command_Count = 0;
                    }
                }

                if (dfu_status.bState == STATE_dfuDNLOAD_IDLE)
                {
                    command_Count++;

                    if (command_Count == 5)
                    {
                        dfu_status.bState = STATE_dfuMANIFEST_SYNC;
                        command_Count = 0;
                    }
                }

                if (dfu_status.bState == STATE_dfuMANIFEST_SYNC)
                {
                    command_Count++;

                    if (command_Count == 5)
                    {
                        dfu_status.bState = STATE_dfuIDLE;
                        command_Count = 0;
                    }
                }
                USBD_PrepareCtrlIn((uint8_t *)&dfu_status.bStatus, 6);
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                break;
            }
            case DFU_GETSTATE:
            {
                USBD_PrepareCtrlIn((uint8_t *)&dfu_status.bState, 1);
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                break;
            }
            case DFU_UPLOAD:
            {
                if (dfu_status.bState == STATE_dfuIDLE || dfu_status.bState == STATE_dfuUPLOAD_IDLE)
                {
                    if (gUsbCmd.wLength <= 0)
                    {
                        dfu_status.bState = STATE_dfuIDLE;
                        return;
                    }
                    else
                    {
                        if (dfu_status.bState == STATE_dfuIDLE)
                        {
                            dfu_status.bState = STATE_dfuUPLOAD_IDLE;
                        }

                        if (gUsbCmd.wValue > APROM_BLOCK_NUM)
                        {
                            dfu_status.bState = STATE_dfuIDLE;
                            USBD->CEPCTL |= USBD_CEPCTL_ZEROLEN_Msk;
                            while (USBD->CEPCTL & USBD_CEPCTL_ZEROLEN_Msk);
                            /* Status stage */
                            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                            USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
                            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
                            break;
                        }

                        FMC_Proc(FMC_ISPCMD_READ, gUsbCmd.wValue * TRANSFER_SIZE, (gUsbCmd.wValue * TRANSFER_SIZE) + gUsbCmd.wLength, (uint32_t *)prog_struct.buf);
                        USBD_PrepareCtrlIn((uint8_t *)prog_struct.buf, gUsbCmd.wLength);
                        USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                        USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
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
    else
    {
        // Host to device
        switch (gUsbCmd.bRequest)
        {
            case DFU_DETACH:
            {
                switch (dfu_status.bState)
                {
                    case STATE_dfuIDLE:
                    case STATE_dfuDNLOAD_SYNC:
                    case STATE_dfuDNLOAD_IDLE:
                    case STATE_dfuMANIFEST_SYNC:
                    case STATE_dfuUPLOAD_IDLE:
                        dfu_status.bStatus = STATUS_OK;
                        dfu_status.bState = STATE_dfuIDLE;
                        dfu_status.iString = 0; /* iString */
                        prog_struct.block_num = 0;
                        prog_struct.data_len = 0;
                        break;

                    default:
                        break;
                }
            }

            case DFU_DNLOAD:
            {
                switch (dfu_status.bState)
                {
                    case STATE_dfuIDLE:
                    case STATE_dfuDNLOAD_IDLE:
                        if (gUsbCmd.wLength > 0)
                        {
                            /* update the global length and block number */
                            prog_struct.block_num = gUsbCmd.wValue;
                            prog_struct.data_len = gUsbCmd.wLength;
                            dfu_status.bState = STATE_dfuDNLOAD_SYNC;
                        }
                        else
                        {
                            manifest_state = MANIFEST_IN_PROGRESS;
                            dfu_status.bState = STATE_dfuMANIFEST_SYNC;
                        }

                        /* enable EP0 prepare receive the buffer */
                        u32Len = gUsbCmd.wLength;
                        address = prog_struct.buf;
                        while(u32Len)
                        {
                            if(u32Len > 64)
                            {
                                USBD_CtrlOut((uint8_t *)address, 64);
                                address += 64;
                                u32Len -= 64;
                            }
                            else
                            {
                                USBD_CtrlOut((uint8_t *)address, u32Len);
                                u32Len = 0;
                            }
                        }
                        /* Status stage */
                        USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                        USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
                        USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
                        break;
                }
                break;
            }

            case DFU_CLRSTATUS:
            {
                //  if (STATE_dfuERROR == dfu_status.bState) {
                dfu_status.bStatus = STATUS_OK;
                dfu_status.bState = STATE_dfuIDLE;
                // } //else {
                /* state Error */
                // dfu_status.bStatus = STATUS_errUNKNOWN;
                // dfu_status.bState = STATE_dfuERROR;
                // }
                dfu_status.iString = 0; /* iString: index = 0 */
                break;
            }

            case DFU_ABORT:
            {
                switch (dfu_status.bState)
                {
                    case STATE_dfuIDLE:
                    case STATE_dfuDNLOAD_SYNC:
                    case STATE_dfuDNLOAD_IDLE:
                    case STATE_dfuMANIFEST_SYNC:
                    case STATE_dfuUPLOAD_IDLE:
                        dfu_status.bStatus = STATUS_OK;
                        dfu_status.bState = STATE_dfuIDLE;
                        dfu_status.iString = 0; /* iString: index = 0 */

                        prog_struct.block_num = 0;
                        prog_struct.data_len = 0;
                        break;

                    default:
                        break;
                }
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
