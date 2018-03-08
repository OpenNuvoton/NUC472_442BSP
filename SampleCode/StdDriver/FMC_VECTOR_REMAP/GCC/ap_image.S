;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

    .syntax unified
    .arch   armv7-m

    .globl  loaderImage1Base
    .globl  loaderImage1Limit
    .globl  loaderImage2Base
    .globl  loaderImage2Limit

    .align  4

    .text

loaderImage1Base:
    .incbin  "..\\..\\fmc_ld_boot.bin"
loaderImage1Limit:
    .space   4

loaderImage2Base:
    .incbin  "..\\..\\fmc_isp.bin"
loaderImage2Limit:
    .space   4

    .end

