/******************************************************************************
 * @file     usbd_audio.h
 * @brief    NuMicro series USB driver header file
 * @version  1.0.0
 * @date     22, Sep, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_UAC_H__
#define __USBD_UAC_H__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0x1284

#define UAC_SPEAKER     1

/*!<Define Audio information */
#define PLAY_RATE       44100   /* The audo play sampling rate. The setting is 16KHz */
#define PLAY_CHANNELS   2
#define PLAY_BIT_RATE   0x10    /* 16-bit data rate */

#define PLAY_FEATURE_UNITID     0x06

#define BUFF_LEN    441

/* Define Descriptor information */
#if(PLAY_CHANNELS == 1)
#define PLAY_CH_CFG     1
#endif
#if(PLAY_CHANNELS == 2)
#define PLAY_CH_CFG     3
#endif

#define PLAY_RATE_LO    (PLAY_RATE & 0xFF)
#define PLAY_RATE_MD    ((PLAY_RATE >> 8) & 0xFF)
#define PLAY_RATE_HI    ((PLAY_RATE >> 16) & 0xFF)

/***************************************************/
/*      Audio Class-Specific Request Codes         */
/***************************************************/
/*!<Define Audio Class Specific Request */
#define UAC_REQUEST_CODE_UNDEFINED  0x00
#define UAC_SET_CUR                 0x01
#define UAC_GET_CUR                 0x81
#define UAC_SET_MIN                 0x02
#define UAC_GET_MIN                 0x82
#define UAC_SET_MAX                 0x03
#define UAC_GET_MAX                 0x83
#define UAC_SET_RES                 0x04
#define UAC_GET_RES                 0x84
#define UAC_SET_MEM                 0x05
#define UAC_GET_MEM                 0x85
#define UAC_GET_STAT                0xFF

#define MUTE_CONTROL                0x01
#define VOLUME_CONTROL              0x02

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define CEP_MAX_PKT_SIZE        64
#define CEP_OTHER_MAX_PKT_SIZE  64
#define EPB_MAX_PKT_SIZE            (PLAY_RATE*PLAY_CHANNELS*2/1000)
#define EPB_OTHER_MAX_PKT_SIZE      (PLAY_RATE*PLAY_CHANNELS*2/1000)

#define CEP_BUF_BASE    0
#define CEP_BUF_LEN     CEP_MAX_PKT_SIZE
#define EPB_BUF_BASE    0x200
#define EPB_BUF_LEN     1024

/* Define the interrupt In EP number */
#define ISO_OUT_EP_NUM   0x02

#define PDMA_TXBUFFER_CNT     7
#define PDMA_I2S_TX_CH  1
/*-------------------------------------------------------------*/
extern uint32_t g_usbd_UsbAudioState;

void UAC_DeviceEnable(void);
void UAC_DeviceDisable(void);
void UAC_GetPlayData(void);

void AudioStartPlay(uint32_t u32SampleRate);
/*-------------------------------------------------------------*/
void UAC_Init(void);
void UAC_ClassRequest(void);
void UAC_SetInterface(uint32_t u32AltInterface);

void EPB_Handler(void);

void WAU8822_Setup(void);
void timer_init(void);

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t endsrc;
    uint32_t enddest;
    uint32_t offset;
} DMA_DESC_T;

extern volatile uint8_t g_usbd_rxflag;
extern void PDMA_ResetTxSGTable(uint8_t id);
extern void PDMA_Init(void);
extern void PDMA_WriteTxSGTable(void);
#endif  /* __USBD_UAC_H_ */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
