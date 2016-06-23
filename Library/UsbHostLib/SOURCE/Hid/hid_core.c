/**************************************************************************//**
 * @file     hid_core.c
 * @version  V1.00
 * $Revision: 16 $
 * $Date: 16/05/17 1:27p $ 
 * @brief    NUC400 series MCU USB Host HID library core
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/   

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "NUC400Series.h"
#include "usbh_core.h"

#include "usbh_hid.h"
#include "hid.h"

/// @cond HIDDEN_SYMBOLS

#define STATIC_MEMORY

#ifdef STATIC_MEMORY
static HIDParser  g_hid_parser_buff;
static int        g_hid_parser_used = 0;
static HIDData    g_hid_data_buff;
static int        g_hid_data_used = 0;
static HIDInterface g_hid_intf_buff;
static int        g_hid_intf_used = 0;
#endif



int usb_control_msg(int dev_handle, int requesttype, int request, 
                    int value, int index, char *bytes, int size, int timeout)
{
    HID_DEV_T       *hid_dev;
    USB_DEV_T       *udev;
    
    hid_dev = find_hid_deivce_by_handle(dev_handle);
    if (hid_dev == NULL)
        return -1;
        
    udev = hid_dev->udev;
        
    return USBH_SendCtrlMsg(udev, usb_rcvctrlpipe(udev, 0),
            request, requesttype, value, index, bytes, size, timeout);
}

void hid_reset_HIDInterface(HIDInterface*   const   hidif)
{
    hidif->dev_handle = 0;
    hidif->interface = -1;
    hidif->id[0] = '\0';
    hidif->hid_data = NULL;
    hidif->hid_parser = NULL;
}

HIDInterface*  hid_new_HIDInterface()
{
#ifdef STATIC_MEMORY    
    HIDInterface*   ret;
    
    if (g_hid_intf_used)
    {
        HID_DBGMSG("hid_new_HIDInterface mallo failed! %d\n", sizeof(HIDInterface));
        return NULL;
    }
    else
    {
        g_hid_intf_used = 1;
        ret = &g_hid_intf_buff;
    }
#else
    HIDInterface*   ret =   (HIDInterface*)malloc(sizeof(HIDInterface));

    if (!ret)   
    {
        HID_DBGMSG("hid_new_HIDInterface mallo failed!\n");
        return NULL;
    }
#endif

    hid_reset_HIDInterface(ret);
    return ret;
}


void hid_delete_HIDInterface(HIDInterface** ixs)
{
    if (!ixs || !*ixs) 
    {
        return;
    }
#ifdef STATIC_MEMORY    
    g_hid_intf_used = 0;
#else
    free(*ixs);
#endif  
    *ixs = 0;
}


int hid_is_opened(HIDInterface * hidif)
{
  if (!hidif) HID_DBGMSG("attempt to query open status of NULL HIDInterface.");
  return hidif && hidif->dev_handle;
}


/*! TODO: This code does not seem to properly retrieve descriptors for devices
 * with multiple interfaces. We probably need to parse each interface a little
 * more to determine which endpoints we want to talk to with usb_control_msg
 * (EP1IN can't be right for everything).
 */
static hid_return hid_prepare_hid_descriptor(HIDInterface* const hidif)
{
    /* TODO: BUFLEN seems to depend on the device, so we need to do something
     * about the following.
     */
#define BUFLEN          9
    uint8_t     buffer[BUFLEN];
    int         len;
    
    len = usb_control_msg(hidif->dev_handle,
            USB_ENDPOINT_IN+1,
            USB_REQ_GET_DESCRIPTOR,
            (USB_DT_HID << 8) + 0, hidif->interface,
            (char*)buffer, BUFLEN,
            USB_TIMEOUT);

    if (len < 0) {
        HID_DBGMSG("failed to get HID descriptor.\n");
        return HID_RET_NOT_HID_DEVICE;
    }

    if (len < BUFLEN) {
        HID_DBGMSG("HID descriptor is too short; expected: %d bytes; got: %d bytes.\n", BUFLEN, len);
        return HID_RET_HID_DESC_SHORT;
    }

    /* TODO:
     * the constants 7 and 8 should be exported.
     */
    hidif->hid_parser->ReportDescSize = buffer[7] | (buffer[8] << 8);

    HID_DBGMSG("successfully initialised HID descriptor %d bytes.\n",
            hidif->hid_parser->ReportDescSize);

    return HID_RET_SUCCESS;
}


static hid_return hid_prepare_report_descriptor(HIDInterface* const hidif)
{
    int     len;
    
    if (hidif->hid_parser->ReportDescSize > REPORT_DSC_SIZE) 
    {
        HID_DBGMSG("report descriptor size exceeds maximum size: %d > %d.\n", hidif->hid_parser->ReportDescSize, REPORT_DSC_SIZE);
        return HID_RET_REPORT_DESC_LONG;
    }

    len = usb_control_msg(hidif->dev_handle,
            USB_ENDPOINT_IN+1,
            USB_REQ_GET_DESCRIPTOR,
            (USB_DT_REPORT << 8) + 0, hidif->interface,
            (char*)hidif->hid_parser->ReportDesc, hidif->hid_parser->ReportDescSize,
            USB_TIMEOUT);

    if (len < 0) {
        HID_DBGMSG("failed to get report descriptor\n");
        return HID_RET_FAIL_GET_REPORT;
    }

    if (len < hidif->hid_parser->ReportDescSize) {
        HID_DBGMSG("HID report descriptor is too short expected: %d bytes; got: %d bytes.\n", hidif->hid_parser->ReportDescSize, len);
        return HID_RET_REPORT_DESC_SHORT;
    }
    HID_DBGMSG("successfully initialised report descriptor.");
    return HID_RET_SUCCESS;
}


hid_return hid_init_parser(HIDInterface* const hidif)
{
#ifdef STATIC_MEMORY    

    if (g_hid_data_used)
    {
        HID_DBGMSG("failed to allocate memory for HIDData %d\n", sizeof(HIDData));
        return HID_RET_FAIL_ALLOC;
    }
    else
    {
        hidif->hid_data = &g_hid_data_buff;
        g_hid_data_used = 1;
    }

    if (g_hid_parser_used)
    {
        HID_DBGMSG("failed to allocate memory for HIDParser %d\n", sizeof(HIDParser));
        return HID_RET_FAIL_ALLOC;
    }
    else
    {
        g_hid_parser_used = 1;
        hidif->hid_parser = &g_hid_parser_buff;
    }
#else
    hidif->hid_data = (HIDData*)malloc(sizeof(HIDData));
    if (!hidif->hid_data) 
    {
        HID_DBGMSG("failed to allocate memory for HIDData\n");
        return HID_RET_FAIL_ALLOC;
    }

    hidif->hid_parser = (HIDParser*)malloc(sizeof(HIDParser));
    if (!hidif->hid_parser) 
    {
        HID_DBGMSG("failed to allocate memory for HIDParser %d\n", sizeof(HIDParser));
        return HID_RET_FAIL_ALLOC;
    }
#endif  
    return HID_RET_SUCCESS;
}


hid_return hid_prepare_interface(HIDInterface* const hidif)
{
    hid_return  ret;
    
    ret = hid_init_parser(hidif);
    if (ret != HID_RET_SUCCESS) 
    {
        USBH_HidClose((int32_t)hidif);
        return ret;
    }

    ret = hid_prepare_hid_descriptor(hidif);
    if (ret != HID_RET_SUCCESS) 
    {
        USBH_HidClose((int32_t)hidif);
        return ret;
    }

    ret = hid_prepare_report_descriptor(hidif);
    if (ret != HID_RET_SUCCESS) {
    USBH_HidClose((int32_t)hidif);
    return ret;
    }

    ret = hid_prepare_parser(hidif);
    if (ret != HID_RET_SUCCESS) {
    USBH_HidClose((int32_t)hidif);
    return ret;
    }
    return HID_RET_SUCCESS;
}

/// @endcond HIDDEN_SYMBOLS

/*!@brief Send a control message to retrieve an entire input report
 *
 * To use an interrupt endpoint instead of EP0, use hid_interrupt_read().
 *
 * @param[in] i32Hanlde  HID interface hanlde
 * @param[in] path       Path to input item (to find Report ID)
 * @param[in] depth      See hid_find_object()
 * @param[out] buffer    Result is stored here
 * @param[in] size       How many bytes to fetch
 */
int32_t USBH_HidGetInputReport(int32_t i32Hanlde, int path[], uint32_t depth, char* buffer, uint32_t size)
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;
    int  len;
    
    //ASSERT(hid_is_initialised());
    //ASSERT(hid_is_opened(hidif));
    ///ASSERT(buffer);

    if (!buffer) return HID_RET_INVALID_PARAMETER;

    if (!hid_is_opened(hidif)) {
        HID_DBGMSG("the device has not been opened.\n");
        return HID_RET_DEVICE_NOT_OPENED;
    }

    HID_DBGMSG("looking up report ID...\n");
    hidif->hid_data->Type = ITEM_INPUT;
    hidif->hid_data->ReportID = 0;

    hid_find_object(hidif, path, depth);

    HID_DBGMSG("retrieving report ID 0x%02x (length: %d)...\n", 
                hidif->hid_data->ReportID, size);

    len = usb_control_msg(hidif->dev_handle,
            USB_ENDPOINT_IN + USB_TYPE_CLASS + USB_RECIP_INTERFACE,
            HID_REPORT_GET,
            hidif->hid_data->ReportID + (HID_RT_INPUT << 8),
            hidif->interface,
            buffer, size, USB_TIMEOUT);

    if (len < 0) {
        HID_DBGMSG("failed to retrieve report!\n");
        return HID_RET_FAIL_GET_REPORT;
    }

    if (len != (signed)size) {
        HID_DBGMSG("failed to retrieve complete report; "
                "requested: %d bytes, got: %d bytes.\n", size, len);
        return HID_RET_FAIL_GET_REPORT;
    }

    HID_DBGMSG("successfully retrieved report.\n");
    return HID_RET_SUCCESS;
}


/*!@brief Send an entire output report to the device
 *
 * This routine uses a control message to send the report. To use an interrupt
 * endpoint, use hid_interrupt_write().
 *
 * @param[in] i32Hanlde  HID interface handle
 * @param[in] path       Path to an output item (to find Report ID)
 * @param[in] depth      See hid_find_object()
 * @param[in] buffer     Output Report
 * @param[in] size       How many bytes to send
 */
int32_t USBH_HidSetOutputReport(int32_t i32Hanlde, int path[],
                                    uint32_t depth, char *buffer, uint32_t size)
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;
    int   len;
    
    //ASSERT(hid_is_initialised());
    //ASSERT(hid_is_opened(hidif));
    //ASSERT(buffer);

    if (!buffer) return HID_RET_INVALID_PARAMETER;

    if (!hid_is_opened(hidif)) {
        HID_DBGMSG("the device has not been opened.\n");
        return HID_RET_DEVICE_NOT_OPENED;
    }

    HID_DBGMSG("looking up report ID...\n");
    hidif->hid_data->Type = ITEM_OUTPUT;
    hidif->hid_data->ReportID = 0;

    hid_find_object(hidif, path, depth);

    HID_DBGMSG("sending report ID 0x%02x (length: %d)...\n", hidif->hid_data->ReportID, size);

    len = usb_control_msg(hidif->dev_handle,
            USB_ENDPOINT_OUT + USB_TYPE_CLASS + USB_RECIP_INTERFACE,
            HID_REPORT_SET,
            hidif->hid_data->ReportID + (HID_RT_OUTPUT << 8),
            hidif->interface,
            (char*)buffer, size, USB_TIMEOUT);

    if (len < 0) {
        HID_DBGMSG("failed to send report.\n");
        return HID_RET_FAIL_SET_REPORT;
    }

    if (len != (signed)size) 
    {
        HID_DBGMSG("failed to send complete report; "
                "requested: %d bytes, sent: %d bytes.\n", size, len);
        return HID_RET_FAIL_SET_REPORT;
    }

    HID_DBGMSG("successfully sent report.\n");
    return HID_RET_SUCCESS;
}


/*!@brief Send a control message to retrieve an entire feature report
 *
 * To use an interrupt endpoint instead of EP0, use hid_interrupt_read().
 *
 * @param[in] i32Hanlde  HID interface handle
 * @param[in] path       Path to input item (to find Report ID)
 * @param[in] depth      See hid_find_object()
 * @param[out] buffer    Result is stored here
 * @param[in] size       How many bytes to fetch
 */
int32_t USBH_HidGetFeatureReport(int32_t i32Hanlde, int path[],
                                    uint32_t depth, char *buffer, uint32_t size)
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;
    int   len;
    
    //ASSERT(hid_is_initialised());
    //ASSERT(hid_is_opened(hidif));
    //ASSERT(buffer);

    if (!buffer) return HID_RET_INVALID_PARAMETER;

    if (!hid_is_opened(hidif)) {
        HID_DBGMSG("the device has not been opened.\n");
        return HID_RET_DEVICE_NOT_OPENED;
    }

    HID_DBGMSG("looking up report ID...\n");
    hidif->hid_data->Type = ITEM_FEATURE;
    hidif->hid_data->ReportID = 0;

    hid_find_object(hidif, path, depth);

    HID_DBGMSG("retrieving report ID 0x%02x (length: %d)...\n", hidif->hid_data->ReportID, size);

    len = usb_control_msg(hidif->dev_handle,
            USB_ENDPOINT_IN + USB_TYPE_CLASS + USB_RECIP_INTERFACE,
            HID_REPORT_GET,
            hidif->hid_data->ReportID + (HID_RT_FEATURE << 8),
            hidif->interface,
            buffer, size, USB_TIMEOUT);

    if (len < 0) {
        HID_DBGMSG("failed to retrieve report!\n");
        return HID_RET_FAIL_GET_REPORT;
    }

    if (len != (signed)size) {
        HID_DBGMSG("failed to retrieve complete report; "
                "requested: %d bytes, got: %d bytes.", size, len);
        return HID_RET_FAIL_GET_REPORT;
    }

    HID_DBGMSG("successfully retrieved report.\n");
    return HID_RET_SUCCESS;
}


/*!@brief Send an entire feature report to the device
 *
 * This routine uses a control message to send the report. To use an interrupt
 * endpoint, use hid_interrupt_write().
 *
 * @param[in] i32Hanlde  HID interface handle
 * @param[in] path       Path to an output item (to find Report ID)
 * @param[in] depth      See hid_find_object()
 * @param[in] buffer     Output Report
 * @param[in] size       How many bytes to send
 */
int32_t USBH_HidSetFeatureReport(int32_t i32Hanlde, int path[],
                                    uint32_t depth, char * buffer, uint32_t size)
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;
    int   len;
    
    //ASSERT(hid_is_initialised());
    //ASSERT(hid_is_opened(hidif));
    //ASSERT(buffer);

    if (!buffer) return HID_RET_INVALID_PARAMETER;

    if (!hid_is_opened(hidif)) {
        HID_DBGMSG("the device has not been opened.\n");
        return HID_RET_DEVICE_NOT_OPENED;
    }

    HID_DBGMSG("looking up report ID...\n");
    hidif->hid_data->Type = ITEM_FEATURE;
    hidif->hid_data->ReportID = 0;

    hid_find_object(hidif, path, depth);

    HID_DBGMSG("sending report ID 0x%02x (length: %d)...\n",  hidif->hid_data->ReportID, size);

    len = usb_control_msg(hidif->dev_handle,
            USB_ENDPOINT_OUT + USB_TYPE_CLASS + USB_RECIP_INTERFACE,
            HID_REPORT_SET,
            hidif->hid_data->ReportID + (HID_RT_FEATURE << 8),
            hidif->interface,
            (char*)buffer, size, USB_TIMEOUT);

    if (len < 0) {
        HID_DBGMSG("failed to send report.\n");
        return HID_RET_FAIL_SET_REPORT;
    }

    if (len != (signed)size) {
        HID_DBGMSG("failed to send complete report; "
                "requested: %d bytes, sent: %d bytes.\n", size, len);
        return HID_RET_FAIL_SET_REPORT;
    }

    HID_DBGMSG("successfully sent report.\n");
    return HID_RET_SUCCESS;
}


/*!
 * Extract data from a report stored in Buf.
 * Use Value, Offset, Size and LogMax of pData.
 * @return Response in pData->Value.
 *
 * TODO: Fix this "+8" business if there is only one report ID
 */
void GetValue(uint8_t *Buf, HIDData* pData)
{
    int     Bit = pData->Offset+8; /* First byte of report indicate report ID */
    int     Weight = 0;
    int     State;

    pData->Value = 0;

    while (Weight < pData->Size)
    {
        State = Buf[Bit>>3] & (1 << (Bit % 8));
        if (State)
        {
            pData->Value += (1 << Weight);
        }
        Weight++;
        Bit++;
    }
/*  if(pData->Value > pData->LogMax)
        pData->Value=FormatValue(pData->Value, (uchar)((pData->Size-1)/8+1));
*/
    if (pData->Value > pData->LogMax)
        pData->Value |= ~pData->LogMax;
}

/// @cond HIDDEN_SYMBOLS

hid_return hid_extract_value(HIDInterface* hidif, uint8_t *buffer, double *value)
{
    if (!hid_is_opened(hidif)) {
        HID_DBGMSG("cannot extract value from unopened HIDinterface.\n");
        return HID_RET_DEVICE_NOT_OPENED;
    }
    //ASSERT(hidif->hid_parser);
    //ASSERT(hidif->hid_data);

    if (!buffer) {
        HID_DBGMSG("cannot extract value into NULL raw buffer.\n");
        return HID_RET_INVALID_PARAMETER;
    }

    if (!value) {
        HID_DBGMSG("cannot extract value into NULL value buffer.\n");
        return HID_RET_INVALID_PARAMETER;
    }
    
    /* Extract the data value */
    GetValue(buffer, hidif->hid_data);

    /* FIXME: unit conversion and exponent?! */
    *value = hidif->hid_data->Value;
    
    return HID_RET_SUCCESS;
}

/// @endcond HIDDEN_SYMBOLS

/**
  * @brief  Get size of a report
 *  @param[in] i32Hanlde   HID interface handle
 *  @param[in] reportID    Report ID
 *  @param[in] reportType  Report type
 *  @param[out] size       the return size of report.
 *  @retval   0:  Success; Otherwise: failed.
 */
int32_t USBH_HidGetReportSize(int32_t i32Hanlde, uint32_t reportID, 
                                    uint32_t reportType, uint32_t *size)
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;

    if (!hid_is_opened(hidif)) {
        HID_DBGMSG("cannot get report size of unopened HIDinterface.\n");
        return HID_RET_DEVICE_NOT_OPENED;
    }
    //ASSERT(hidif->hid_parser);
    //ASSERT(hidif->hid_data);
    
    if (!size) {
        HID_DBGMSG("cannot read report size into NULL size buffer.\n");
        return HID_RET_INVALID_PARAMETER;
    }
    
    /* FIXME: GetReportOffset has to be rewritten! */
    *size = *GetReportOffset(hidif->hid_parser, reportID, reportType);
    
    return HID_RET_SUCCESS;
}

/*!@brief Retrieve a numeric input item
 *
 * @param[in] i32Hanlde  HID interface handle
 * @param[in] path       Path to input item
 * @param[in] depth      See hid_find_object()
 * @param[out] value     Result from hid_extract_value()
 *
 * TODO: Handle exponent and unit conversion (separate library?)
 */
int32_t USBH_HidGetItemValue(int32_t i32Hanlde, int path[], uint32_t depth, double *value)
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;
    uint32_t    size;
    uint8_t     buffer[32]; /* TODO: Dynamically allocate the item buffer */
    int   len;
    
    //ASSERT(hid_is_initialised());
    //ASSERT(hid_is_opened(hidif));

    if (!hid_is_opened(hidif)) {
        HID_DBGMSG("the device has not been opened.\n");
        return HID_RET_DEVICE_NOT_OPENED;
    }

    HID_DBGMSG("retrieving report...\n");
    hidif->hid_data->Type = ITEM_FEATURE;
    hidif->hid_data->ReportID = 0;

    /* TODO: i think this and the buffer stuff should be passed in */
    hid_find_object(hidif, path, depth);
    USBH_HidGetReportSize((int32_t)hidif, hidif->hid_data->ReportID,
            hidif->hid_data->Type, &size);

    //ASSERT(size <= 32); /* remove when buffer situation is fixed. */

    len = usb_control_msg(hidif->dev_handle,
            USB_ENDPOINT_IN + USB_TYPE_CLASS + USB_RECIP_INTERFACE,
            HID_REPORT_GET,
            hidif->hid_data->ReportID + (HID_RT_FEATURE << 8),
            hidif->interface,
            (char*)buffer, size, USB_TIMEOUT);

    if (len < 0) {
        HID_DBGMSG("failed to retrieve report.\n");
        return HID_RET_FAIL_GET_REPORT;
    }

    if ((unsigned)len != size) {
        HID_DBGMSG("failed to retrieve complete report; "
                "requested: %d bytes, got: %d bytes.\n", size, len);
        return HID_RET_FAIL_GET_REPORT;
    }

    if (hid_extract_value(hidif, buffer, value) != HID_RET_SUCCESS) {
        return HID_RET_FAIL_GET_REPORT;
    }

    HID_DBGMSG("successfully retrieved report.\n");
    return HID_RET_SUCCESS;
}


/*!@brief Execute a Set_Idle request on an Interrupt In pipe
 *
 * This is used to tell a device not to send reports unless something has
 * changed (duration = 0), or unless a minimum time interval has passed.
 *
 * @param[in] i32Hanlde  HID interface handle
 * @param[in] duration   0 for indefinite, otherwise in increments of 4 ms (to 1020 ms)
 * @param[in] report_id  0 for all reports, otherwise a Report ID
 */
int32_t USBH_HidSetIdle(int32_t i32Hanlde, uint32_t duration, uint32_t report_id) 
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;
    int   len;
    
    if (duration > 255) {
        HID_DBGMSG("duration must be in the range [0,255]\n");
        return HID_RET_INVALID_PARAMETER;
    }

    if (report_id > 255) {
        HID_DBGMSG("Report ID must be in the range [0,255]\n");
        return HID_RET_INVALID_PARAMETER;
    }

    len = usb_control_msg(hidif->dev_handle,
            USB_TYPE_CLASS + USB_RECIP_INTERFACE,
            HID_SET_IDLE,
            report_id + ((duration & 0xff) << 8),
            hidif->interface,
            NULL, 0, USB_TIMEOUT);

    if (len != 0) {
        HID_DBGMSG("failed to Set_Idle!\n");
        return HID_RET_FAIL_GET_REPORT;
    }

    return HID_RET_SUCCESS;
}


/* 
 * HID INT-in complete function 
 */
static void  hid_read_irq(URB_T *urb)
{
    HIDInterface  *hidif = (HIDInterface *)urb->context;
    HID_DEV_T   *hid_dev;

    //HID_DBGMSG("hid_read_irq. %d\n", urb->actual_length);

    if (!hid_is_opened(hidif)) 
    {
        HID_DBGMSG("hid_read_irq - the device have been closed!\n");
        return;
    }

    hid_dev = find_hid_deivce_by_handle(hidif->dev_handle);
    if (hid_dev == NULL)
        return;
    
    if (urb->status)
    {
        HID_DBGMSG("hid_read_irq - has error: 0x%x\n", urb->status);
        return; 
    }

    if (hid_dev->read_func && urb->actual_length)
        hid_dev->read_func(urb->transfer_buffer, urb->actual_length);
}


/**
 * @brief  Start purge the USB interrupt in transfer.
 *  @param[in] i32Hanlde   HID interface handle.
 *  @param[in] endpoint    The endpoint address of interrupt in pipe.
 *  @param[in] func        The interrupt in data receiver callback function.
 *  @retval   0:  Success; Otherwise: failed.
 */
int32_t USBH_HidStartIntReadPipe(int32_t i32Hanlde, 
                                        uint16_t endpoint, HID_INT_READ_FUNC *func)
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;
    HID_DEV_T   *hid_dev;
    EP_INFO_T   *ep_info;
    URB_T       *urb;
    uint32_t    pipe;
    USB_DEV_T   *udev;
    int         maxp, ret;

    if (!func)
        return HID_RET_INVALID_PARAMETER;
    
    if (!hid_is_opened(hidif)) {
        HID_DBGMSG("the device has not been opened.\n");
        return HID_RET_DEVICE_NOT_OPENED;
    }
    
    hid_dev = find_hid_deivce_by_handle(hidif->dev_handle);
    if (hid_dev == NULL)
        return HID_RET_DEVICE_NOT_FOUND;
    udev = hid_dev->udev;
        
    if (hid_dev->urbin)
        return HID_RET_OUT_OF_SPACE;
        
    ep_info = hid_get_ep_info(hid_dev->udev, hid_dev->ifnum, endpoint);
    if (ep_info == NULL)
    {
        HID_DBGMSG("Assigned endpoint address 0x%x not found in this device!\n", endpoint);
        return HID_RET_INVALID_PARAMETER;
    }
    
    urb = USBH_AllocUrb();
    if (urb == NULL)
    {
        HID_DBGMSG("Failed to allocated URB!\n");
        return HID_RET_OUT_OF_SPACE;
    }   
    
    pipe = usb_rcvintpipe(udev, endpoint);
    maxp = usb_maxpacket(udev, pipe, usb_pipeout(pipe));
    
    HID_DBGMSG("Endpoint 0x%x maximum packet size is %d.\n", endpoint, maxp);
    
    FILL_INT_URB(urb, udev, pipe, &hid_dev->inbuf[0], maxp, hid_read_irq,
                hidif, ep_info->bInterval);

    hid_dev->urbin = urb;
    hid_dev->read_func = func;

    ret = USBH_SubmitUrb(urb);
    if (ret) 
    {
        HID_DBGMSG("Error - failed to submit interrupt read request (%d)", ret);
        USBH_FreeUrb(urb);
        hid_dev->urbin = NULL;
        return HID_RET_IO_ERR;
    }
    
    return HID_RET_SUCCESS;
}


/* 
 * HID INT-out complete function 
 */
static void  hid_write_irq(URB_T *urb)
{
    HIDInterface  *hidif = (HIDInterface *)urb->context;
    HID_DEV_T     *hid_dev;

    //HID_DBGMSG("hid_read_irq. %d\n", urb->actual_length);

    if (!hid_is_opened(hidif)) 
    {
        HID_DBGMSG("hid_read_irq - the device have been closed, terminate it.\n");
        return;
    }

    hid_dev = find_hid_deivce_by_handle(hidif->dev_handle);
    if (hid_dev == NULL)
        return;
    
    if (urb->status)
    {
        HID_DBGMSG("hid_read_irq - has error: 0x%x\n", urb->status);
        return; 
    }

    if (hid_dev->write_func)
        hid_dev->write_func((uint8_t **)&urb->transfer_buffer, &urb->transfer_buffer_length);
}


/**
  * @brief  Start purge the USB interrupt out transfer.
 *  @param[in] i32Hanlde   HID interface handle.
 *  @param[in] endpoint    The endpoint address of interrupt out pipe.
 *  @param[in] func        The interrupt in data transfer callback function.
 *  @retval   0:  Success; Otherwise: failed.
 */
int32_t USBH_HidStartIntWritePipe(int32_t i32Hanlde, 
                                        uint16_t endpoint, HID_INT_WRITE_FUNC *func)
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;
    HID_DEV_T   *hid_dev;
    EP_INFO_T   *ep_info;
    URB_T       *urb;
    uint32_t    pipe;
    USB_DEV_T   *udev;
    int         maxp, ret;
    
    if (!func)
        return HID_RET_INVALID_PARAMETER;
    
    if (!hid_is_opened(hidif)) {
        HID_DBGMSG("the device has not been opened.\n");
        return HID_RET_DEVICE_NOT_OPENED;
    }
    
    hid_dev = find_hid_deivce_by_handle(hidif->dev_handle);
    if (hid_dev == NULL)
        return HID_RET_DEVICE_NOT_FOUND;
    udev = hid_dev->udev;
        
    if (hid_dev->urbout)
        return HID_RET_OUT_OF_SPACE;
        
    ep_info = hid_get_ep_info(hid_dev->udev, hid_dev->ifnum, endpoint);
    if (ep_info == NULL)
    {
        HID_DBGMSG("Assigned endpoint address 0x%x not found in this device!\n", endpoint);
        return HID_RET_INVALID_PARAMETER;
    }
    
    urb = USBH_AllocUrb();
    if (urb == NULL)
    {
        HID_DBGMSG("Failed to allocated URB!\n");
        return HID_RET_OUT_OF_SPACE;
    }   
    
    pipe = usb_sndintpipe(udev, endpoint);
    maxp = usb_maxpacket(udev, pipe, usb_pipeout(pipe));
    
    HID_DBGMSG("Endpoint 0x%x maximum packet size is %d.\n", endpoint, maxp);
    
    FILL_INT_URB(urb, udev, pipe, NULL, maxp, hid_write_irq,
                                            hidif, ep_info->bInterval);

    func((uint8_t **)&urb->transfer_buffer, &urb->transfer_buffer_length);

    hid_dev->urbout = urb;
    hid_dev->write_func = func;

    ret = USBH_SubmitUrb(urb);
    if (ret) 
    {
        HID_DBGMSG("Error - failed to submit interrupt read request (%d)", ret);
        USBH_FreeUrb(urb);
        hid_dev->urbout = NULL;
        return HID_RET_IO_ERR;
    }
    
    return HID_RET_SUCCESS;
}

/// @cond HIDDEN_SYMBOLS

const char *hid_strerror(hid_return ret)
{
    switch(ret) {
        case HID_RET_SUCCESS:
            return "libhid: success";
        case HID_RET_INVALID_PARAMETER:
            return "libhid: invalid parameter";
        case HID_RET_NOT_INITIALISED:
            return "libhid: not initialized; call hid_init() first";
        case HID_RET_ALREADY_INITIALISED:
            return "libhid: hid_init() already called";
        case HID_RET_FAIL_FIND_BUSSES:
            return "libhid: failed to find any USB busses";
        case HID_RET_FAIL_FIND_DEVICES:
            return "libhid: failed to find any USB devices";
        case HID_RET_FAIL_OPEN_DEVICE:
            return "libhid: failed to open device";
        case HID_RET_DEVICE_NOT_FOUND:
            return "libhid: device not found";
        case HID_RET_DEVICE_NOT_OPENED:
            return "libhid: device not yet opened";
        case HID_RET_DEVICE_ALREADY_OPENED:
            return "libhid: device already opened";
        case HID_RET_FAIL_CLOSE_DEVICE:
            return "libhid: could not close device";
        case HID_RET_FAIL_CLAIM_IFACE:
            return "libhid: failed to claim interface; is another driver using it?";
        case HID_RET_FAIL_DETACH_DRIVER:
            return "libhid: failed to detach kernel driver";
        case HID_RET_NOT_HID_DEVICE:
            return "libhid: not recognized as a HID device";
        case HID_RET_HID_DESC_SHORT:
            return "libhid: HID interface descriptor too short";
        case HID_RET_REPORT_DESC_SHORT:
            return "libhid: HID report descriptor too short";
        case HID_RET_REPORT_DESC_LONG:
            return "libhid: HID report descriptor too long";
        case HID_RET_FAIL_ALLOC:
            return "libhid: failed to allocate memory";
        case HID_RET_OUT_OF_SPACE:
            return "libhid: no space left in buffer";
        case HID_RET_FAIL_SET_REPORT:
            return "libhid: failed to set report";
        case HID_RET_FAIL_GET_REPORT:
            return "libhid: failed to get report";
        case HID_RET_FAIL_INT_READ:
            return "libhid: interrupt read failed";
        case HID_RET_NOT_FOUND:
            return "libhid: not found";
        case HID_RET_TIMEOUT:
            return "libhid: timeout";
    }
    return "libhid: unknown error";
}

/// @endcond HIDDEN_SYMBOLS


/**
  * @brief  Probe and open a HID device.
 *  @param[in] interface   The interface number of HID device.
 *  @param[in] matcher     The HID device search conditions.
 *  @param[in] handle      The return handle of HID device if matched.
 *  @retval   0:  Success; Otherwise: failed.
 */
int32_t USBH_HidOpen(int interface, HIDInterfaceMatcher *matcher, int32_t * handle)
{
    HIDInterface  * hidif;
    hid_return ret;
    
    if (!matcher)   {
        return HID_RET_INVALID_PARAMETER;
    }

    hidif = hid_new_HIDInterface();
    if (hidif == NULL) {
        return HID_RET_FAIL_ALLOC;
    }

    hidif->interface = interface;
    
    ret = usbh_hid_find_device(hidif, matcher);
    if (ret != HID_RET_SUCCESS)
    {
        hid_delete_HIDInterface(&hidif);
        return ret;
    }

    //HID_DBGMSG("claiming  USB device %s.", hidif->id);
    //if (usb_claim_interface(hidif->dev_handle, interface) <   0) {
    //  WARNING("failed to claim USB device %s.",   hidif->id);
    //  USBH_HidClose((int32_t)hidif);
    //  return HID_RET_FAIL_CLAIM_IFACE;
    //}
    //NOTICE("successfully claimed USB device   %s.",   hidif->id);

    ret = hid_prepare_interface(hidif);
    if (ret != HID_RET_SUCCESS) 
    {
        hid_delete_HIDInterface(&hidif);
        return ret;
    }
    
    *handle = (int32_t)hidif;

    return HID_RET_SUCCESS;
}

/**
 * @brief  Close a HID device. 
 *  @retval   0:  Success; Otherwise: failed.
 */

int32_t  USBH_HidClose(int32_t i32Hanlde)
{
    HIDInterface * hidif = (HIDInterface *)i32Hanlde;
    
    if (hidif->hid_parser) ResetParser(hidif->hid_parser);
        
#ifdef STATIC_MEMORY    
    g_hid_parser_used = 0;
    g_hid_data_used = 0;
#else
    if (hidif->hid_parser) free(hidif->hid_parser);
    if (hidif->hid_data) free(hidif->hid_data);
#endif
        
    hid_delete_HIDInterface(&hidif);
    return HID_RET_SUCCESS;
}


