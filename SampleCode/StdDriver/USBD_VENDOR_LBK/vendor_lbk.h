/******************************************************************************
 * @file     vendor_lbk.h
 * @brief    NUC472/NUC442 USB driver header file
 * @version  2.0.0
 * $Date: 14/10/06 11:23a $
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_LBK_H__
#define __USBD_LBK_H__

/* Define the vendor id and product id */
#define USBD_VID                       0x0416
#define USBD_PID                       0xFFFD

/*!<Define Vendor Class Specific Request */
#define REQ_SET_DATA                   0x01
#define REQ_GET_DATA                   0x12

#define ISO_BUFF_SIZE                  128
#define BULK_BUFF_SIZE                 512

/*-------------------------------------------------------------*/

/* Define endpoint number */
#define NUMBER_OF_EP                   6
#define INT_IN_EP_NUM                  0x01
#define INT_OUT_EP_NUM                 0x02
#define ISO_IN_EP_NUM                  0x03
#define ISO_OUT_EP_NUM                 0x04
#define BULK_IN_EP_NUM                 0x05
#define BULK_OUT_EP_NUM                0x06

/* Define endpoints' maximum packet size */
#define CEP_MAX_PKT_SIZE               64
#define EPA_MAX_PKT_SIZE               8
#define EPB_MAX_PKT_SIZE               8
#define EPC_MAX_PKT_SIZE               128     // Iso-in
#define EPD_MAX_PKT_SIZE               128     // Iso-out
#define EPE_MAX_PKT_SIZE               64
#define EPF_MAX_PKT_SIZE               64

/* Define other configuration endpoints' maximum packet size */
#define CEP_OTHER_MAX_PKT_SIZE         64
#define EPA_OTHER_MAX_PKT_SIZE         8
#define EPB_OTHER_MAX_PKT_SIZE         8
#define EPC_OTHER_MAX_PKT_SIZE         128     // Iso-in
#define EPD_OTHER_MAX_PKT_SIZE         128     // Iso-out
#define EPE_OTHER_MAX_PKT_SIZE         64
#define EPF_OTHER_MAX_PKT_SIZE         64

/* Define endpoint buffers */
#define CEP_BUF_BASE                   0
#define CEP_BUF_LEN                    CEP_MAX_PKT_SIZE
#define EPA_BUF_BASE                   (CEP_BUF_BASE + CEP_BUF_LEN)
#define EPA_BUF_LEN                    EPA_MAX_PKT_SIZE
#define EPB_BUF_BASE                   (EPA_BUF_BASE + EPA_BUF_LEN)
#define EPB_BUF_LEN                    EPB_MAX_PKT_SIZE
#define EPC_BUF_BASE                   (EPB_BUF_BASE + EPB_BUF_LEN)
#define EPC_BUF_LEN                    ISO_BUFF_SIZE
#define EPD_BUF_BASE                   (EPC_BUF_BASE + EPC_BUF_LEN)
#define EPD_BUF_LEN                    ISO_BUFF_SIZE
#define EPE_BUF_BASE                   (EPD_BUF_BASE + EPD_BUF_LEN)
#define EPE_BUF_LEN                    BULK_BUFF_SIZE
#define EPF_BUF_BASE                   (EPE_BUF_BASE + EPE_BUF_LEN)
#define EPF_BUF_LEN                    BULK_BUFF_SIZE

/* Define Descriptor information */
#define INT_IN_INTERVAL                4
#define INT_OUT_INTERVAL               4
#define ISO_IN_INTERVAL                1
#define ISO_OUT_INTERVAL               1
#define USBD_SELF_POWERED              0
#define USBD_REMOTE_WAKEUP             0
#define USBD_MAX_POWER                 50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

#define LEN_CONFIG_AND_SUBORDINATE      (LEN_CONFIG+LEN_INTERFACE+(LEN_ENDPOINT*NUMBER_OF_EP))


/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
void LBK_Init(void);
void Vendor_ClassRequest(void);
void LBK_BulkOut(uint32_t u32Addr, uint32_t u32Len);
int  LBK_HasBulkOutReq(void);
void LBK_BulkInPushData(uint32_t u32Addr, uint32_t u32Len);
int  LBK_BulkInXferDone(void);
void LBK_IsoOut(uint32_t u32Addr, uint32_t u32Len);
int  LBK_HasIsoOutReq(void);
void LBK_IsoInPushData(uint32_t u32Addr, uint32_t u32Len);
int  LBK_IsoInXferDone(void);

#endif  /* __USBD_LBK_H_ */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
