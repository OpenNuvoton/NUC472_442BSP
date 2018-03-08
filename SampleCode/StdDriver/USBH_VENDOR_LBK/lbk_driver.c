/**************************************************************************//**
 * @file     lbk_driver.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/10/06 11:22a $
 * @brief    NUC472/NUC442 MCU USB Host vendor class driver
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "NUC472_442.h"
#include "usbh_core.h"
#include "lbk_driver.h"

#define LBK_DBGMSG        printf

static LBKDEV_T  g_lbk_dev;


/*
 *  If a USB device is connected, USB core library will enable it and read descriptors
 *  from it. If the USB device vendor ID and product ID matches with LBK device,
 *  lbk_probe will be called.
 */
static int  lbk_probe(USB_DEV_T *dev, USB_IF_DESC_T *ifd, const USB_DEV_ID_T *id)
{
    int         ifnum;
    int         i;

    LBK_DBGMSG("lbk_probe - dev=0x%x\n", (int)dev);

    if (g_lbk_dev.connected)
    {
        LBK_DBGMSG("There's an LBK device connected. The other LBK device will be ignored.\n");
        return -1;
    }

    memset((uint8_t *)&g_lbk_dev, 0, sizeof(g_lbk_dev));

    g_lbk_dev.dev = dev;
    ifnum = ifd->bInterfaceNumber;
    g_lbk_dev.ifnum = ifnum;

    for (i = 0; i < dev->ep_list_cnt; i++)
    {
        if (dev->ep_list[i].ifnum != ifnum)
            continue;

        switch (dev->ep_list[i].bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
        {
        case USB_ENDPOINT_XFER_INT:
            if (dev->ep_list[i].bEndpointAddress & USB_ENDPOINT_DIR_MASK)
                g_lbk_dev.int_in = &dev->ep_list[i];
            else
                g_lbk_dev.int_out = &dev->ep_list[i];
            break;

        case USB_ENDPOINT_XFER_ISOC:
            if (dev->ep_list[i].bEndpointAddress & USB_ENDPOINT_DIR_MASK)
                g_lbk_dev.iso_in = &dev->ep_list[i];
            else
                g_lbk_dev.iso_out = &dev->ep_list[i];
            break;

        case USB_ENDPOINT_XFER_BULK:
            if (dev->ep_list[i].bEndpointAddress & USB_ENDPOINT_DIR_MASK)
                g_lbk_dev.bulk_in = &dev->ep_list[i];
            else
                g_lbk_dev.bulk_out = &dev->ep_list[i];
            break;
        }
    }

    if (!g_lbk_dev.int_in || !g_lbk_dev.int_out || !g_lbk_dev.iso_in ||
            !g_lbk_dev.iso_out || !g_lbk_dev.bulk_in || !g_lbk_dev.bulk_out)
    {
        LBK_DBGMSG("Expected endpoints not found!\n");
        return -1;
    }

    g_lbk_dev.connected = 1;
    return 0;
}


static void  lbk_disconnect(USB_DEV_T *dev)
{
    int   i;

    g_lbk_dev.connected = 0;

    LBK_DBGMSG("LBK device disconnected!\n");

    if (g_lbk_dev.urb_int_in)
    {
        USBH_UnlinkUrb(g_lbk_dev.urb_int_in);
        USBH_FreeUrb(g_lbk_dev.urb_int_in);
    }

    if (g_lbk_dev.urb_int_out)
    {
        USBH_UnlinkUrb(g_lbk_dev.urb_int_out);
        USBH_FreeUrb(g_lbk_dev.urb_int_out);
    }

    for (i = 0; i < ISO_URB_COUNT; i++)
    {
        if (g_lbk_dev.urb_iso_in[i])
        {
            USBH_UnlinkUrb(g_lbk_dev.urb_iso_in[i]);
            USBH_FreeUrb(g_lbk_dev.urb_iso_in[i]);
            g_lbk_dev.urb_iso_in[i] = NULL;
        }

        if (g_lbk_dev.urb_iso_out[i])
        {
            USBH_UnlinkUrb(g_lbk_dev.urb_iso_out[i]);
            USBH_FreeUrb(g_lbk_dev.urb_iso_out[i]);
            g_lbk_dev.urb_iso_out[i] = NULL;
        }
    }
}


static USB_DEV_ID_T  lbk_id_table[] =
{
    USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT,     /* match_flags */
    LBKDEV_VID, LBKDEV_PID,
    0, 0, 0, 0, 0, 0, 0, 0, 0
};


static USB_DRIVER_T  lbk_driver =
{
    "vendor class LBK driver",
    lbk_probe,
    lbk_disconnect,
    lbk_id_table,
    NULL,                       /* suspend */
    NULL,                       /* resume */
    {NULL,NULL}                 /* driver_list */
};



/******************************************************************************************/
/*                                                                                        */
/*   Vendor class LBK driver Export functions                                              */
/*                                                                                        */
/******************************************************************************************/


/**
 *  @brief  This function executes vendor command REQ_SET_DATA. This command will send 64 bytes
 *          data to LBK device via a Control-Out transfer.
 *  @param[in] buff       A buffer stored the 64 bytes data to be send.
 *  @retval   0           Success
 *  @retval   Otherwise   Error
 */
int LBK_CtrlSetData(uint8_t *buff)
{
    USB_DEV_T *dev;

    if (!g_lbk_dev.connected)
        return -1;

    dev = g_lbk_dev.dev;

    if (USBH_SendCtrlMsg(dev, usb_sndctrlpipe(dev, 0),
                         REQ_SET_DATA,     /* bRequest: specific request */
                         0x40,             /* bmRequestType: Host to device, Vendor */
                         0,                /* wValue: not used in this command */
                         0,                /* wValue: not used in this command */
                         buff,
                         64,               /* wLength: this command always send 64 bytes */
                         1000) != 64)      /* return value must be 64, the actual transferred data length */
    {
        LBK_DBGMSG("REQ_SET_DATA command failed!\n");
        return -1;
    }
    return 0;
}


/**
 *  @brief  This function executes vendor command REQ_GET_DATA. This command will read 64 bytes
 *          data from LBK device via a Control-In transfer.
 *  @param[in] buff       The data buff to store received 64 bytes data.
 *  @retval   0           Success
 *  @retval   Otherwise   Error
 */
int LBK_CtrlGetData(uint8_t *buff)
{
    USB_DEV_T *dev;

    if (!g_lbk_dev.connected)
        return -1;

    dev = g_lbk_dev.dev;

    if (USBH_SendCtrlMsg(dev, usb_rcvctrlpipe(dev, 0),
                         REQ_GET_DATA,     /* bRequest: specific request */
                         0xC0,             /* bmRequestType: Device to host, Vendor */
                         0,                /* wValue: not used in this command */
                         0,                /* wValue: not used in this command */
                         buff,
                         64,               /* wLength: this command always send 64 bytes */
                         1000) != 64)      /* return value must be 64, the actual transferred data length */
    {
        LBK_DBGMSG("REQ_GET_DATA command failed!\n");
        return -1;
    }
    return 0;
}

static volatile int  bulk_out_done = 1;
static void  bulk_out_irq(URB_T *urb)
{
    bulk_out_done = 1;
}


/**
 *  @brief  This function executes Bulk-Out transfer on LBK device.
 *  @param[in] buff       A buffer stored the data to be send.
 *  @param[in] len        Transferred data length.
 *  @retval   0           Success
 *  @retval   Otherwise   Error
 */
int LBK_BulkWriteData(uint8_t *buff, int len)
{
    USB_DEV_T  *dev;
    EP_INFO_T  *ep_info;
    URB_T      *urb;
    int        pipe;
    int        timeout, ret;

    if (!g_lbk_dev.connected)
        return -1;

    dev = g_lbk_dev.dev;
    ep_info = g_lbk_dev.bulk_out;

    urb = USBH_AllocUrb();
    if (urb == NULL)
    {
        LBK_DBGMSG("No free URB!\n");
        return -1;
    }

    pipe = usb_sndbulkpipe(dev, ep_info->bEndpointAddress);

    FILL_BULK_URB(urb, dev, pipe, buff, len, bulk_out_irq, NULL);

    bulk_out_done = 0;

    ret = USBH_SubmitUrb(urb);
    if (ret < 0)
    {
        LBK_DBGMSG("Failed to submit Bulk out transfer!\n");
        goto err_out;
    }

    for (timeout = 0x1000000 ; (timeout > 0) && (bulk_out_done == 0) ; timeout--);

    if (timeout <= 0)
    {
        USBH_UnlinkUrb(urb);
        LBK_DBGMSG("Bulk out xfer time-out!\n");
        goto err_out;
    }

    USBH_FreeUrb(urb);
    return 0;

err_out:
    USBH_FreeUrb(urb);
    return -1;
}

static volatile int  bulk_in_done = 1;
static void  bulk_in_irq(URB_T *urb)
{
    bulk_in_done = 1;
}


/**
 *  @brief  This function executes Bulk-In transfer on LBK device.
 *  @param[in] buff       A buffer stored the data to be send.
 *  @param[in] len        Transferred data length.
 *  @retval   0           Success
 *  @retval   Otherwise   Error
 */
int LBK_BulkReadData(uint8_t *buff, int len)
{
    USB_DEV_T  *dev;
    EP_INFO_T  *ep_info;
    URB_T      *urb;
    int        pipe;
    int        timeout, ret;

    if (!g_lbk_dev.connected)
        return -1;

    dev = g_lbk_dev.dev;
    ep_info = g_lbk_dev.bulk_in;

    urb = USBH_AllocUrb();
    if (urb == NULL)
    {
        LBK_DBGMSG("No free URB!\n");
        return -1;
    }

    pipe = usb_rcvbulkpipe(dev, ep_info->bEndpointAddress);

    FILL_BULK_URB(urb, dev, pipe, buff, len, bulk_in_irq, NULL);

    bulk_in_done = 0;

    ret = USBH_SubmitUrb(urb);
    if (ret < 0)
    {
        LBK_DBGMSG("Failed to submit Bulk in transfer!\n");
        goto err_out;
    }

    for (timeout = 0x1000000 ; (timeout > 0) && (bulk_in_done == 0) ; timeout--);

    if (timeout <= 0)
    {
        USBH_UnlinkUrb(urb);
        LBK_DBGMSG("Bulk in xfer time-out!\n");
        goto err_out;
    }

    USBH_FreeUrb(urb);
    return 0;

err_out:
    USBH_FreeUrb(urb);
    return -1;
}

static void  int_in_irq(URB_T *urb)
{
    if (urb->status)
    {
        LBK_DBGMSG("int_in_irq - has error: 0x%x\n", urb->status);
        return;
    }
    if (g_lbk_dev.int_in_func)
    {
        g_lbk_dev.int_in_func(urb->transfer_buffer, &urb->actual_length);
    }
}

/**
 *  @brief  Install Interrupt-In callback function
 *  @param[in] func       The interrupt in data delivery callback function.
 */
void LBK_InstallIntInFunc(LBK_CB_FUN *func)
{
    g_lbk_dev.int_in_func = func;
}

/**
 *  @brief    Start the LBK Interrupt-In transfer.
 *  @retval   0           Success
 *  @retval   Otherwise   failed
 */
int LBK_StartIntInPipe()
{
    USB_DEV_T  *dev;
    EP_INFO_T  *ep_info;
    URB_T      *urb;
    int        pipe, maxp, ret;

    if (g_lbk_dev.urb_int_in)
        return 0;    // Interrupt in pipe should has been started.

    if (!g_lbk_dev.int_in_func)
    {
        LBK_DBGMSG("LBK_StartIntInPipe - Must install an Interrupt-In callback first!\n");
        return -1;
    }

    urb = USBH_AllocUrb();
    if (urb == NULL)
    {
        LBK_DBGMSG("LBK_StartIntInPipe - failed to allocated URB!\n");
        return -1;
    }

    dev = g_lbk_dev.dev;
    ep_info = g_lbk_dev.int_in;

    pipe = usb_rcvintpipe(dev, ep_info->bEndpointAddress);
    maxp = usb_maxpacket(dev, pipe, usb_pipeout(pipe));

    LBK_DBGMSG("Int-In endpoint 0x%x maxpksz: %d.\n", ep_info->bEndpointAddress, maxp);

    FILL_INT_URB(urb, dev, pipe, &g_lbk_dev.int_in_buff[0], maxp,
                 int_in_irq, &g_lbk_dev, ep_info->bInterval);

    g_lbk_dev.urb_int_in = urb;

    ret = USBH_SubmitUrb(urb);
    if (ret)
    {
        LBK_DBGMSG("LBK_StartIntInPipe - failed to submit interrupt in request (%d)", ret);
        USBH_FreeUrb(urb);
        g_lbk_dev.urb_int_in = NULL;
        return -1;
    }
    return 0;
}


/**
 *  @brief    Stop the LBK Interrupt-In transfer.
 *  @retval   0           Success
 *  @retval   Otherwise   failed
 */
void LBK_StopIntInPipe()
{
    if (!g_lbk_dev.connected || !g_lbk_dev.urb_int_in)
        return;

    USBH_UnlinkUrb(g_lbk_dev.urb_int_in);
    USBH_FreeUrb(g_lbk_dev.urb_int_in);
}


static void  int_out_irq(URB_T *urb)
{
    if (urb->status)
    {
        LBK_DBGMSG("int_out_irq - has error: 0x%x\n", urb->status);
        return;
    }
    if (g_lbk_dev.int_out_func)
    {
        urb->transfer_buffer_length = 8;
        g_lbk_dev.int_out_func(urb->transfer_buffer, &urb->transfer_buffer_length);
    }
}

/**
 *  @brief  Install Interrupt-Out callback function
 *  @param[in] func       The interrupt out data request callback function.
 */
void LBK_InstallIntOutFunc(LBK_CB_FUN *func)
{
    g_lbk_dev.int_out_func = func;
}

/**
 *  @brief    Start the LBK Interrupt-Out transfer.
 *  @retval   0           Success
 *  @retval   Otherwise   failed
 */
int LBK_StartIntOutPipe()
{
    USB_DEV_T  *dev;
    EP_INFO_T  *ep_info;
    URB_T      *urb;
    int        pipe, maxp, ret;

    if (g_lbk_dev.urb_int_out)
        return 0;    // Interrupt out pipe should has been started.

    if (!g_lbk_dev.int_out_func)
    {
        LBK_DBGMSG("LBK_StartIntOutPipe - Must install an Interrupt-Out callback first!\n");
        return -1;
    }

    urb = USBH_AllocUrb();
    if (urb == NULL)
    {
        LBK_DBGMSG("LBK_StartIntOutPipe - failed to allocated URB!\n");
        return -1;
    }

    dev = g_lbk_dev.dev;
    ep_info = g_lbk_dev.int_out;

    pipe = usb_sndintpipe(dev, ep_info->bEndpointAddress);
    maxp = usb_maxpacket(dev, pipe, usb_pipeout(pipe));

    LBK_DBGMSG("Int-Out endpoint 0x%x maxpksz: %d.\n", ep_info->bEndpointAddress, maxp);

    FILL_INT_URB(urb, dev, pipe, g_lbk_dev.int_out_buff, maxp,
                 int_out_irq, &g_lbk_dev, ep_info->bInterval);

    g_lbk_dev.int_out_func(urb->transfer_buffer, &urb->transfer_buffer_length);

    g_lbk_dev.urb_int_out = urb;

    ret = USBH_SubmitUrb(urb);
    if (ret)
    {
        LBK_DBGMSG("LBK_StartIntOutPipe - failed to submit interrupt in request (%d)", ret);
        USBH_FreeUrb(urb);
        g_lbk_dev.urb_int_out = NULL;
        return -1;
    }
    return 0;
}

/**
 *  @brief    Stop the LBK Interrupt-Out transfer.
 *  @retval   0           Success
 *  @retval   Otherwise   failed
 */
void LBK_StopIntOutPipe()
{
    if (!g_lbk_dev.connected || !g_lbk_dev.urb_int_out)
        return;

    USBH_UnlinkUrb(g_lbk_dev.urb_int_out);
    USBH_FreeUrb(g_lbk_dev.urb_int_out);
}

/**
 *  @brief  Install Isochronous-In callback function
 *  @param[in] func       The isochronous in data delivery callback function.
 */
void LBK_InstallIsoInFunc(LBK_CB_ISO_FUN *func)
{
    g_lbk_dev.iso_in_func = func;
}

static void  iso_in_irq(URB_T *urb)
{
    int   j, k, status;

    //printf("Iso in - SF=%d, EC=%d\n", urb->start_frame, urb->error_count);

    if (urb->status)
        printf("Iso out error, status=%d\n", urb->status);

    urb->status = 0;
    urb->transfer_flags = USB_ISO_ASAP;
    urb->transfer_buffer_length = ISO_MAX_PKSZ * FRAMES_PER_DESC;
    for (j=k=0; j < FRAMES_PER_DESC; j++, k += ISO_MAX_PKSZ)
    {
        g_lbk_dev.iso_in_func((uint8_t *)urb->transfer_buffer + k, urb->iso_frame_desc[j].actual_length);
        urb->iso_frame_desc[j].status = 0;
        urb->iso_frame_desc[j].actual_length = 0;
        urb->iso_frame_desc[j].offset = k;
        urb->iso_frame_desc[j].length = ISO_MAX_PKSZ;
    }

    if ((!g_lbk_dev.connected) || g_lbk_dev.stop_iso_in)
        return;

    /* Submit URB */
    status = USBH_SubmitUrb(urb);
    if (status)
        printf("usbvideo_IsocIrq: USB_SubmitUrb ret %d\n", status);

    return;
}

/**
 *  @brief    Start the LBK Isochronous-In transfer.
 *  @retval   0           Success
 *  @retval   Otherwise   failed
 */
int LBK_StartIsoInPipe()
{
    USB_DEV_T  *dev;
    EP_INFO_T  *ep_info;
    URB_T      *urb;
    int        i, j, k, ret;

    if (g_lbk_dev.urb_iso_in[0])
        return 0;    // Interrupt in pipe should has been started.

    if (!g_lbk_dev.iso_in_func)
    {
        LBK_DBGMSG("LBK_StartIsoInPipe - Must install an Isochronous-In callback first!\n");
        return -1;
    }

    g_lbk_dev.stop_iso_in = 0;

    for (i = 0; i < ISO_URB_COUNT; i++)
    {
        urb = USBH_AllocUrb();
        if (urb == NULL)
        {
            LBK_DBGMSG("LBK_StartIsoInPipe - failed to allocated URB!\n");
            goto err_out;
        }

        dev = g_lbk_dev.dev;
        ep_info = g_lbk_dev.iso_in;

        urb->dev = g_lbk_dev.dev;
        urb->context = &g_lbk_dev;
        urb->pipe = usb_rcvisocpipe(dev, ep_info->bEndpointAddress);
        urb->transfer_flags = USB_ISO_ASAP;
        urb->complete = iso_in_irq;
        urb->interval = ep_info->bInterval;
        urb->number_of_packets = FRAMES_PER_DESC;
        urb->transfer_buffer = &(g_lbk_dev.iso_in_buff[i][0]);
        urb->transfer_buffer_length = ISO_MAX_PKSZ * FRAMES_PER_DESC;

        for (j=k=0; j < FRAMES_PER_DESC; j++, k += ISO_MAX_PKSZ)
        {
            urb->iso_frame_desc[j].status = 0;
            urb->iso_frame_desc[j].offset = k;
            urb->iso_frame_desc[j].length = ISO_MAX_PKSZ;
        }

        g_lbk_dev.urb_iso_in[i] = urb;

        ret = USBH_SubmitUrb(urb);
        if (ret)
        {
            LBK_DBGMSG("LBK_StartIsoInPipe - failed to submit interrupt in request (%d)", ret);
            USBH_FreeUrb(urb);
            goto err_out;
        }
    }
    return 0;

err_out:
    for (i = 0; i < ISO_URB_COUNT; i++)
    {
        if (g_lbk_dev.urb_iso_in[i])
        {
            USBH_UnlinkUrb(g_lbk_dev.urb_iso_in[i]);
            USBH_FreeUrb(g_lbk_dev.urb_iso_in[i]);
            g_lbk_dev.urb_iso_in[i] = NULL;
        }
    }
    return -1;
}

/**
 *  @brief    Stop the LBK Isochronous-In transfer.
 *  @retval   0           Success
 *  @retval   Otherwise   failed
 */
void LBK_StopIsoInPipe()
{
    if (!g_lbk_dev.connected || !g_lbk_dev.urb_iso_in[0])
        return;

    g_lbk_dev.stop_iso_in = 1;
}


/**
 *  @brief  Install Isochronous-Out callback function
 *  @param[in] func       The isochronous out data request callback function.
 */
void LBK_InstallIsoOutFunc(LBK_CB_ISO_FUN *func)
{
    g_lbk_dev.iso_out_func = func;
}

static void  iso_out_irq(URB_T *urb)
{
    int   j, k, status;

    //printf("Iso Out - SF=%d, EC=%d\n", urb->start_frame, urb->error_count);

    if (urb->status)
        printf("Iso out error, status=%d\n", urb->status);

    urb->status = 0;
    urb->transfer_flags = USB_ISO_ASAP;
    urb->transfer_buffer_length = ISO_MAX_PKSZ * FRAMES_PER_DESC;
    for (j=k=0; j < FRAMES_PER_DESC; j++, k += ISO_MAX_PKSZ)
    {
        urb->iso_frame_desc[j].status = 0;
        urb->iso_frame_desc[j].actual_length = 0;
        urb->iso_frame_desc[j].offset = k;
        urb->iso_frame_desc[j].length = ISO_MAX_PKSZ;
        g_lbk_dev.iso_out_func((uint8_t *)urb->transfer_buffer + k, urb->iso_frame_desc[j].length);
    }


    if ((!g_lbk_dev.connected) || g_lbk_dev.stop_iso_out)
        return;

    /* Submit URB */
    status = USBH_SubmitUrb(urb);
    if (status)
        printf("usbvideo_IsocIrq: USB_SubmitUrb ret %d\n", status);

    return;
}

/**
 *  @brief    Start the LBK Isochronous-Out transfer.
 *  @retval   0           Success
 *  @retval   Otherwise   failed
 */
int LBK_StartIsoOutPipe()
{
    USB_DEV_T  *dev;
    EP_INFO_T  *ep_info;
    URB_T      *urb;
    int        i, j, k, ret;

    if (g_lbk_dev.urb_iso_out[0])
        return 0;    // Isochronous out pipe should has been started.

    if (!g_lbk_dev.iso_out_func)
    {
        LBK_DBGMSG("LBK_StartIsoOutPipe - Must install an Isochronous-Out callback first!\n");
        return -1;
    }

    g_lbk_dev.stop_iso_out = 0;

    for (i = 0; i < ISO_URB_COUNT; i++)
    {
        urb = USBH_AllocUrb();
        if (urb == NULL)
        {
            LBK_DBGMSG("LBK_StartIsoOutPipe - failed to allocated URB!\n");
            goto err_out;
        }

        dev = g_lbk_dev.dev;
        ep_info = g_lbk_dev.iso_out;

        urb->dev = g_lbk_dev.dev;
        urb->context = &g_lbk_dev;
        urb->pipe = usb_sndisocpipe(dev, ep_info->bEndpointAddress);
        urb->transfer_flags = USB_ISO_ASAP;
        urb->complete = iso_out_irq;
        urb->interval = ep_info->bInterval;
        urb->number_of_packets = FRAMES_PER_DESC;
        urb->transfer_buffer = &(g_lbk_dev.iso_out_buff[i][0]);
        urb->transfer_buffer_length = ISO_MAX_PKSZ * FRAMES_PER_DESC;

        for (j=k=0; j < FRAMES_PER_DESC; j++, k += ISO_MAX_PKSZ)
        {
            urb->iso_frame_desc[j].offset = k;
            urb->iso_frame_desc[j].length = ISO_MAX_PKSZ;
            g_lbk_dev.iso_out_func((uint8_t *)urb->transfer_buffer + k, urb->iso_frame_desc[j].length);
        }

        g_lbk_dev.urb_iso_out[i] = urb;

        ret = USBH_SubmitUrb(urb);
        if (ret)
        {
            LBK_DBGMSG("LBK_StartIsoOutPipe - failed to submit interrupt in request (%d)", ret);
            USBH_FreeUrb(urb);
            goto err_out;
        }
    }
    return 0;

err_out:
    for (i = 0; i < ISO_URB_COUNT; i++)
    {
        if (g_lbk_dev.urb_iso_out[i])
        {
            USBH_UnlinkUrb(g_lbk_dev.urb_iso_out[i]);
            USBH_FreeUrb(g_lbk_dev.urb_iso_out[i]);
            g_lbk_dev.urb_iso_out[i] = NULL;
        }
    }
    return -1;
}

/**
 *  @brief    Stop the LBK Isochronous-Out transfer.
 *  @retval   0           Success
 *  @retval   Otherwise   failed
 */
void LBK_StopIsoOutPipe()
{
    if (!g_lbk_dev.connected || !g_lbk_dev.urb_iso_out[0])
        return;

    g_lbk_dev.stop_iso_out = 1;
}

int  LBK_IsConnected(void)
{
    return g_lbk_dev.connected;
}

void LBK_Init(void)
{
    memset((uint8_t *)&g_lbk_dev, 0, sizeof(g_lbk_dev));
    USBH_RegisterDriver(&lbk_driver);
}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

