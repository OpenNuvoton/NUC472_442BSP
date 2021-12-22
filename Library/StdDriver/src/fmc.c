/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.10
 * $Revision: 20 $
 * $Date: 15/11/20 9:55a $
 * @brief    NUC472/NUC442 FMC driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

//* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "NUC472_442.h"


/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_FMC_Driver FMC Driver
  @{
*/


/** @addtogroup NUC472_442_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

int32_t  g_FMC_i32ErrCode;

/**
  * @brief Disable FMC ISP function.
  * @return None
  */
void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief    Erase a page. The page size is 2048 bytes.
  * @param[in]  u32PageAddr   Flash page address. Must be a 2048-byte aligned address.
  * @return   Success or not.
  * @retval   0    Success
  * @retval   -1   Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Erase failed or erase time-out
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    int32_t   tout;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_ERASE;
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        return -1;
    }
    return 0;
}


/**
  * @brief Get the current boot source.
  * @return The current boot source.
  * @retval   0  Is boot from APROM.
  * @retval   1  Is boot from LDROM.
  */
int32_t FMC_GetBootSource (void)
{
    if (FMC->ISPCTL & FMC_ISPCTL_BS_Msk)
        return 1;
    else
        return 0;
}


/**
  * @brief Enable FMC ISP function
  * @return None
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief Execute ISP command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return  The word data read from specified flash address.
  *          Return 0xFFFFFFFF if read failed.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_Read(uint32_t u32Addr)
{
    int32_t  tout;

    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_READ;
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return FMC->ISPDAT;
}


/**
  * @brief Execute ISP 64-bits read command to read two words from flash.
  * @param[in] u32Addr Flash word address. Must be a double word aligned address.
  * @param[out] u32Data0 The first word read from flash.
  * @param[out] u32Data1 The second word read from flash.
  * @return   0   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
int32_t FMC_Read_64(uint32_t u32Addr, uint32_t *u32Data0, uint32_t *u32Data1)
{
    int32_t  tout;

    FMC->ISPCMD = FMC_ISPCMD_READ_64;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0x0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_READ;
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    *u32Data0 = FMC->MPDAT0;
    *u32Data1 = FMC->MPDAT1;
    return 0;
}


/**
  * @brief    Read company ID.
  * @param    None
  * @return   The company ID (32-bit). 0xFFFFFFFF means read failed.
  * @details  The company ID of Nuvoton is fixed to be 0xDA
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadCID(void)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_CID;
    FMC->ISPADDR = 0x0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
        {
            if (FMC->ISPDAT != 0xDA)
                g_FMC_i32ErrCode = -1;
            return FMC->ISPDAT;
        }
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}


/**
  * @brief    Read product ID.
  * @param    None
  * @return   The product ID (32-bit). 0xFFFFFFFF means read failed.
  * @details  This function is used to read product ID.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadPID(void)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_PID;
    FMC->ISPADDR = 0x04;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}


/**
  * @brief    This function reads one of the four UCID.
  * @param[in]   u32Index Index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  * @return     The UCID of specified index. 0xFFFFFFFF means read failed.
  * @details    This function is used to read unique chip ID (UCID).
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = (0x04 * u32Index) + 0x10;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}


/**
  * @brief    This function reads one of the three UID.
  * @param[in]  u32Index Index of the UID to read. u32Index must be 0, 1, or 2.
  * @return      The 32-bit unique ID data of specified UID index. 0xFFFFFFFF means read failed.
  * @details     To read out 96-bit Unique ID.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadUID(uint32_t u32Index)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = 0x04 * u32Index;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}


/**
  * @brief    Get the base address of Data Flash if enabled.
  * @return   Base address of Data Flash
  */
uint32_t FMC_ReadDataFlashBaseAddr(void)
{
    return FMC->DFBA;
}


/**
  * @brief    This function will force re-map assigned flash page to CPU address 0x0.
  * @param[in]  u32PageAddr Address of the page to be mapped to CPU address 0x0.
  * @retval      0   Success
  * @retval      -1  Failed
  *
  * @note        Global error code g_FMC_i32ErrCode
  *              -1  Command time-out
  */
int32_t FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    int32_t  tout = FMC_TIMEOUT_WRITE;

    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))   /* Waiting for ISP Done */
            return 0;
    }
    g_FMC_i32ErrCode = -1;
    return -1;
}


/**
  * @brief    Obtain the current vector page address setting.
  * @return   The vector page address.
  */
uint32_t FMC_GetVectorPageAddr(void)
{
    return (FMC->ISPSTS & 0x0FFFFF00ul);
}


/**
  * @brief Execute ISP command to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[out] u32Data The word data to be programmed.
  * @return None
  * @return   0   Success
  * @return   -1  Program Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t  tout;

    FMC->ISPCMD = FMC_ISPCMD_WRITE;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_WRITE;
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}


/**
  * @brief  Execute ISP 64-bits write command to program two words to flash.
  * @param[in] u32Addr   Destination address. It must be double word aligned.
  * @param[in] u32Data0  First word data to be written.
  * @param[in] u32Data1  Second word data to be written.
  * @return   0   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t FMC_Write_64(uint32_t u32Addr, uint32_t u32Data0, uint32_t u32Data1)
{
    int32_t  tout;

    FMC->ISPCMD = FMC_ISPCMD_WRITE_64;
    FMC->ISPADDR = u32Addr;
    FMC->MPDAT0 = u32Data0;
    FMC->MPDAT1 = u32Data1;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_WRITE;
    while ((tout-- > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}


/**
  * @brief        CRC8 Calculator
  * @param[in]    au32Data   CRC8 input data words.
  * @param[in]    i32Count   Number of words in au32Data[].
  * @return       The CRC8 value.
  */
uint32_t FMC_CRC8(uint32_t au32Data[], int i32Count)
{
    int         i32ByteIdx;
    uint8_t     i, u8Cnt, u8InData;
    uint8_t     au8CRC[4] = { 0xff, 0xff, 0xff, 0xff };

    for (i32ByteIdx = 0; i32ByteIdx < 4; i32ByteIdx++)
    {
        for (u8Cnt = 0; u8Cnt < i32Count; u8Cnt++)
        {
            for (i = 0x80; i != 0; i /= 2)
            {
                if ((au8CRC[i32ByteIdx] & 0x80)!=0)
                {
                    au8CRC[i32ByteIdx] *= 2;
                    au8CRC[i32ByteIdx] ^= 7;
                }
                else
                    au8CRC[i32ByteIdx] *= 2;

                u8InData = (au32Data[u8Cnt] >> (i32ByteIdx * 8)) & 0xff;

                if ((u8InData & i) != 0)
                    au8CRC[i32ByteIdx]^=0x7;
            }
        }
    }
    return (au8CRC[0] | au8CRC[1] << 8 | au8CRC[2] << 16 | au8CRC[3] << 24);
}


/**
  * @brief    Read the User Configuration words.
  * @param[out] u32Config: The word array to store words read from flash.
  * @param[in]  u32Count: Maximum length of u32Config.
  * @return  Success or not.
  * @retval   0    Success
  * @retval   -1   User Configuration CRC check error
  */
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count)
{
    int         i;

    for (i = 0; i < u32Count; i++)
    {
        u32Config[i] = FMC_Read(FMC_CONFIG_BASE + i*4);
    }

    if (FMC->ISPSTS & FMC_ISPSTS_CFGCRCF_Msk)
        return -1;

    return 0;
}


/**
  * @brief  Write User Configuration
  * @param[in] u32Config  The word array to store data. MUST be a four word array.
  * @param[in] u32Count   MUST be 4.
  * @return  Success or not.
  * @retval 0    Success
  * @retval -1   Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           < 0  Errors caused by erase/program/read failed or time-out
  */
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count)
{
    uint32_t  i, u32CRC, u32Data;

    if (u32Count != 4)
        return -1;

    if (FMC_Erase(FMC_CONFIG_BASE) != 0)
        return -1;

    u32CRC = FMC_CRC8(u32Config, 3);

    for (i = 0; i < 4; i++)
    {
        if (FMC_Write(FMC_CONFIG_BASE + i * 4, (i < 3) ? u32Config[i] : u32CRC) != 0)
            return -1;
    }

    for (i = 0; i < 4; i++)
    {
        u32Data = FMC_Read(FMC_CONFIG_BASE + i * 4);

        if (u32Data != ((i < 3) ? u32Config[i] : u32CRC))
            return -1;
    }
    return 0;
}


/*@}*/ /* end of group NUC472_442_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_FMC_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


