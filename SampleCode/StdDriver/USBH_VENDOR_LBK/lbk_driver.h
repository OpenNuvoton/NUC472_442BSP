/******************************************************************************
 * @file     vendor_lbk.h
 * @brief    NUC472/NUC442 USB Host vendor class driver header file
 * @version  2.0.0
 * $Date: 14/10/06 11:22a $
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBH_LBK_H__
#define __USBH_LBK_H__

#include "usbh_core.h"


/* Define the vendor id and product id */
#define LBKDEV_VID                     0x0416
#define LBKDEV_PID                     0xFFFD

/*!<Define Vendor Class Specific Request */
#define REQ_SET_DATA                   0x01
#define REQ_GET_DATA                   0x12

#define ISO_URB_COUNT                  2
#define FRAMES_PER_DESC                4
#define ISO_MAX_PKSZ                   128

typedef void (LBK_CB_FUN)(uint8_t *data_buff, int *data_len);
typedef void (LBK_CB_ISO_FUN)(uint8_t *data_buff, int data_len);

typedef struct lbk_dev_t {
    int          connected;
    int          ifnum;
    USB_DEV_T    *dev;
    EP_INFO_T    *int_in;
    EP_INFO_T    *int_out;
    EP_INFO_T    *iso_in;
    EP_INFO_T    *iso_out;
    EP_INFO_T    *bulk_in;
    EP_INFO_T    *bulk_out;
    URB_T        *urb_int_in;
    URB_T        *urb_int_out;
    URB_T        *urb_iso_in[ISO_URB_COUNT];
    URB_T        *urb_iso_out[ISO_URB_COUNT];
    LBK_CB_FUN   *int_in_func;
    LBK_CB_FUN   *int_out_func;
    LBK_CB_ISO_FUN   *iso_in_func;
    LBK_CB_ISO_FUN   *iso_out_func;
    uint8_t      stop_iso_in;
    uint8_t      stop_iso_out;
    uint8_t      int_in_buff[8];
    uint8_t      int_out_buff[8];
    uint8_t      iso_in_buff[ISO_URB_COUNT][ISO_MAX_PKSZ * FRAMES_PER_DESC];
    uint8_t      iso_out_buff[ISO_URB_COUNT][ISO_MAX_PKSZ * FRAMES_PER_DESC];
}  LBKDEV_T;


/*-------------------------------------------------------------*/

void LBK_Init(void);
int  LBK_IsConnected(void);

int  LBK_CtrlSetData(uint8_t *buff);
int  LBK_CtrlGetData(uint8_t *buff);

void LBK_InstallIntInFunc(LBK_CB_FUN *func);
int  LBK_StartIntInPipe(void);
void LBK_StopIntInPipe(void);

void LBK_InstallIntOutFunc(LBK_CB_FUN *func);
int  LBK_StartIntOutPipe(void);
void LBK_StopIntOutPipe(void);

void LBK_InstallIsoInFunc(LBK_CB_ISO_FUN *func);
int  LBK_StartIsoInPipe(void);
void LBK_StopIsoInPipe(void);

void LBK_InstallIsoOutFunc(LBK_CB_ISO_FUN *func);
int  LBK_StartIsoOutPipe(void);
void LBK_StopIsoOutPipe(void);

int  LBK_BulkWriteData(uint8_t *buff, int len);
int  LBK_BulkReadData(uint8_t *buff, int len);


#endif  /* __USBH_LBK_H_ */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
