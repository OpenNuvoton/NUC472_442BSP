/**************************************************************************//**
 * @file     descriptors.c
 * @version  V1.00
 * $Date: 14/11/17 5:41p $
 * @brief    NUC472/NUC442 USBD driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DESCRIPTORS_C__
#define __DESCRIPTORS_C__

/*!<Includes */
#include "NUC472_442.h"
#include "hid_transfer.h"

/*!<USB HID Report Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t HID_DeviceReportDescriptor[] =
#else
uint8_t HID_DeviceReportDescriptor[]  __attribute__((aligned(4))) =
#endif
{
    0x06, 0x06, 0xFF,       /* USAGE_PAGE (Vendor Defined)*/
    0x09, 0x01,             /* USAGE (0x01)*/
    0xA1, 0x01,             /* COLLECTION (Application)*/
    0x15, 0x00,             /* LOGICAL_MINIMUM (0)*/
    0x26, 0xFF, 0x00,       /* LOGICAL_MAXIMUM (255)*/
    0x75, 0x08,             /* REPORT_SIZE (8)*/
    0x96, 0x00, 0x02,       /* REPORT_COUNT*/
    0x09, 0x01,
    0x81, 0x02,             /* INPUT (Data,Var,Abs)*/
    0x96, 0x00, 0x02,       /* REPORT_COUNT*/
    0x09, 0x01,
    0x91, 0x02,             /* OUTPUT (Data,Var,Abs)*/
    0x95, 0x08,             /* REPORT_COUNT (8) */
    0x09, 0x01,
    0xB1, 0x02,             /* FEATURE */
    0xC0,                   /* END_COLLECTION*/
};


/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8DeviceDescriptor[] =
#else
uint8_t gu8DeviceDescriptor[] __attribute__((aligned(4))) =
#endif
{
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    CEP_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    ((USBD_VID & 0xFF00) >> 8),
    /* idProduct */
    USBD_PID & 0x00FF,
    ((USBD_PID & 0xFF00) >> 8),
    0x00, 0x00,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x00,           /* iSerialNumber - no serial */
    0x01            /* bNumConfigurations */
};

/*!<USB Qualifier Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8QualifierDescriptor[] =
#else
uint8_t gu8QualifierDescriptor[] __attribute__((aligned(4))) =
#endif
{
    LEN_QUALIFIER,  /* bLength */
    DESC_QUALIFIER, /* bDescriptorType */
    0x10, 0x01,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    CEP_OTHER_MAX_PKT_SIZE, /* bMaxPacketSize0 */
    0x01,           /* bNumConfigurations */
    0x00
};

/*!<USB Configure Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8ConfigDescriptor[] =
#else
uint8_t gu8ConfigDescriptor[] __attribute__((aligned(4))) =
#endif
{
    LEN_CONFIG,     /* bLength */
    DESC_CONFIG,    /* bDescriptorType */
    /* wTotalLength */
    (LEN_CONFIG+LEN_INTERFACE+LEN_HID+LEN_ENDPOINT*2) & 0x00FF,
    (((LEN_CONFIG+LEN_INTERFACE+LEN_HID+LEN_ENDPOINT*2) & 0xFF00) >> 8),
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5),/* bmAttributes */
    USBD_MAX_POWER,         /* MaxPower */

    /* I/F descr: HID */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x00,           /* bInterfaceSubClass */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* HID Descriptor */
    LEN_HID,        /* Size of this descriptor in UINT8s. */
    DESC_HID,       /* HID descriptor type. */
    0x10, 0x01,     /* HID Class Spec. release number. */
    0x00,           /* H/W target country. */
    0x01,           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_DeviceReportDescriptor) & 0x00FF,
    ((sizeof(HID_DeviceReportDescriptor) & 0xFF00) >> 8),

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EPA_MAX_PKT_SIZE & 0x00FF,
    ((EPA_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL,        /* bInterval */

    /* EP Descriptor: interrupt out. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_OUT_EP_NUM | EP_OUTPUT),   /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EPB_MAX_PKT_SIZE & 0x00FF,
    ((EPB_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL     /* bInterval */
};

/*!<USB Other Speed Configure Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8OtherConfigDescriptor[] =
#else
uint8_t gu8OtherConfigDescriptor[] __attribute__((aligned(4))) =
#endif
{
    LEN_CONFIG,     /* bLength */
    DESC_CONFIG,    /* bDescriptorType */
    /* wTotalLength */
    (LEN_CONFIG+LEN_INTERFACE+LEN_HID+LEN_ENDPOINT*2) & 0x00FF,
    (((LEN_CONFIG+LEN_INTERFACE+LEN_HID+LEN_ENDPOINT*2) & 0xFF00) >> 8),
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5),/* bmAttributes */
    USBD_MAX_POWER,         /* MaxPower */

    /* I/F descr: HID */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x00,           /* bInterfaceSubClass */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* HID Descriptor */
    LEN_HID,        /* Size of this descriptor in UINT8s. */
    DESC_HID,       /* HID descriptor type. */
    0x10, 0x01,     /* HID Class Spec. release number. */
    0x00,           /* H/W target country. */
    0x01,           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_DeviceReportDescriptor) & 0x00FF,
    ((sizeof(HID_DeviceReportDescriptor) & 0xFF00) >> 8),

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EPA_OTHER_MAX_PKT_SIZE & 0x00FF,
    ((EPA_OTHER_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL,        /* bInterval */

    /* EP Descriptor: interrupt out. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_OUT_EP_NUM | EP_OUTPUT),   /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EPB_OTHER_MAX_PKT_SIZE & 0x00FF,
    ((EPB_OTHER_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL     /* bInterval */
};


/*!<USB Language String Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8StringLang[4] =
#else
uint8_t gu8StringLang[4] __attribute__((aligned(4))) =
#endif
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8VendorStringDesc[] =
#else
uint8_t gu8VendorStringDesc[] __attribute__((aligned(4))) =
#endif
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8ProductStringDesc[] =
#else
uint8_t gu8ProductStringDesc[] __attribute__((aligned(4))) =
#endif
{
    26,
    DESC_STRING,
    'H', 0, 'I', 0, 'D', 0, ' ', 0, 'T', 0, 'r', 0, 'a', 0, 'n', 0, 's', 0, 'f', 0, 'e', 0, 'r', 0
};

uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    NULL,
};

uint8_t *gu8UsbHidReport[3] =
{
    HID_DeviceReportDescriptor,
    NULL,
    NULL,
};

uint32_t gu32UsbHidReportLen[3] =
{
    sizeof(HID_DeviceReportDescriptor),
    0,
    0,
};

S_USBD_INFO_T gsInfo =
{
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    gu8QualifierDescriptor,
    gu8OtherConfigDescriptor,
    gu8UsbHidReport,
    gu32UsbHidReportLen,
};


#endif  /* __DESCRIPTORS_C__ */
