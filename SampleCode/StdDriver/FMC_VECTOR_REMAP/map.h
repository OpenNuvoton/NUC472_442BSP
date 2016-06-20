/**************************************************************************//**
 * @file     map.h
 * @version  V1.00
 * $Revision 2 $
 * $Date: 13/11/10 9:33a $
 * @brief    FMC VECMAP sample program header file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __MAP_H__
#define __MAP_H__

#define USER_AP_MAX_SIZE        (96*1024)

#define ISP_CODE_BASE           (96*1024)
#define ISP_CODE_MAX_SIZE       (32*1024)

#define USER_AP_ENTRY           FMC_APROM_BASE
#define ISP_CODE_ENTRY          ISP_CODE_BASE
#define LD_BOOT_CODE_ENTRY      FMC_LDROM_BASE


typedef void (FUNC_PTR)(void);

extern uint32_t ApImage1Base, ApImage1Limit;
extern uint32_t loaderImage1Base, loaderImage1Limit;
extern uint32_t loaderImage2Base, loaderImage2Limit;


#endif  // __MAP_H__

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

