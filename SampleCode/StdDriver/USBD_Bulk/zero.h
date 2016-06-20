/******************************************************************************
 * @file     Zero.h
 * @brief    NUC472/NUC442 USB driver header file
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/10/02 4:28p $
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_BULK_H__
#define __USBD_BULK_H__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0x3102

/* Define DMA Maximum Transfer length */
#define USBD_MAX_DMA_LEN    0x1000

/* Define EP maximum packet size */
#define CEP_MAX_PKT_SIZE        64
#define CEP_OTHER_MAX_PKT_SIZE  64
#define EPA_MAX_PKT_SIZE        512
#define EPA_OTHER_MAX_PKT_SIZE  64
#define EPB_MAX_PKT_SIZE        512
#define EPB_OTHER_MAX_PKT_SIZE  64

#define CEP_BUF_BASE    0
#define CEP_BUF_LEN     CEP_MAX_PKT_SIZE
#define EPA_BUF_BASE    0x200
#define EPA_BUF_LEN     EPA_MAX_PKT_SIZE
#define EPB_BUF_BASE    0x400
#define EPB_BUF_LEN     EPB_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define BULK_IN_EP_NUM      0x02
#define BULK_OUT_EP_NUM     0x01

/* Define Descriptor information */
#define USBD_SELF_POWERED               0
#define USBD_REMOTE_WAKEUP              0
#define USBD_MAX_POWER                  50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */


static __INLINE uint32_t get_be32(uint8_t *buf)
{
    return ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) |
           ((uint32_t) buf[2] << 8) | ((uint32_t) buf[3]);
}


/*-------------------------------------------------------------*/
void Zero_Init(void);
void Zero_InitForHighSpeed(void);
void Zero_InitForFullSpeed(void);
void Zero_ClassRequest(void);
void Zero_MainProcess(void);
void Zero_ReceiveShort(uint32_t u32Buf, int *len, int wait);

void Zero_ActiveDMA(uint32_t u32Addr, uint32_t u32Len, int wait);
int Zero_BulkOut(uint32_t u32Addr, uint32_t u32Len);
int Zero_BulkIn(uint32_t u32Addr, uint32_t u32Len, uint32_t shortPacket);

#endif  /* __USBD_BULK_H_ */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
