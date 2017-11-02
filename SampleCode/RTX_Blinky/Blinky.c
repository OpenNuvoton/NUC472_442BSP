/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    BLINKY.C
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2006-2010 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "cmsis_os.h"
#include "NUC472_442.h"                   /* NUC4xx definitions                */
#include "LED.h"

/* Thread IDs */
osThreadId t_phaseA;                        /* assigned task id of task: phase_a */
osThreadId t_phaseB;                        /* assigned task id of task: phase_b */
osThreadId t_phaseC;                        /* assigned task id of task: phase_c */
osThreadId t_phaseD;                        /* assigned task id of task: phase_d */
osThreadId t_clock;                         /* assigned task id of task: clock   */

/* Forward reference */
void phaseA (void const *argument);
void phaseB (void const *argument);
void phaseC (void const *argument);
void phaseD (void const *argument);
void clock  (void const *argument);

osThreadDef(phaseA, osPriorityNormal, 1, 0);
osThreadDef(phaseB, osPriorityNormal, 1, 0);
osThreadDef(phaseC, osPriorityNormal, 1, 0);
osThreadDef(phaseD, osPriorityNormal, 1, 0);
osThreadDef(clock, osPriorityNormal, 1, 0);

#define LED_A      0
#define LED_B      1
#define LED_C      2
#define LED_D      3
#define LED_CLK    7


/*----------------------------------------------------------------------------
  switch LED on
 *---------------------------------------------------------------------------*/
void LED_on (unsigned char led) {
  LED_On (led);
}

/*----------------------------------------------------------------------------
  switch LED off
 *---------------------------------------------------------------------------*/
void LED_off (unsigned char led) {
  LED_Off(led);
}


/*----------------------------------------------------------------------------
  Function 'signal_func' called from multiple tasks
 *---------------------------------------------------------------------------*/
void signal_func (osThreadId thread)  {
  osSignalSet (t_clock, 0x0100);         /* send event signal to clock task  */
  osDelay (500);                      /* delay 50 clock ticks             */
  osSignalSet (t_clock, 0x0100);         /* send event signal to clock task  */
  osDelay (500);                      /* delay 50 clock ticks             */
  osSignalSet (thread, 0x0001);            /* send event to task 'task'        */
  osDelay (500);                      /* delay 50 clock ticks             */
}

/*----------------------------------------------------------------------------
  Task 1 'phaseA': Phase A output
 *---------------------------------------------------------------------------*/
void phaseA (void const *argument) {
  for (;;) {
    osSignalWait(0x0001, 0xffff);    /* wait for an event flag 0x0001    */
    LED_on (LED_A);
    signal_func (t_phaseB);              /* call common signal function      */
    LED_off(LED_A);
  }
}

/*----------------------------------------------------------------------------
  Task 2 'phaseB': Phase B output
 *---------------------------------------------------------------------------*/
void phaseB (void const *argument) {
  for (;;) {
    osSignalWait (0x0001, 0xffff);    /* wait for an event flag 0x0001    */
    LED_on (LED_B);
    signal_func (t_phaseC);              /* call common signal function      */
    LED_off(LED_B);
  }
}

/*----------------------------------------------------------------------------
  Task 3 'phaseC': Phase C output
 *---------------------------------------------------------------------------*/
void phaseC (void const *argument) {
  for (;;) {
    osSignalWait (0x0001, 0xffff);    /* wait for an event flag 0x0001    */
    LED_on (LED_C);
    signal_func (t_phaseD);              /* call common signal function      */
    LED_off(LED_C);
  }
}

/*----------------------------------------------------------------------------
  Task 4 'phaseD': Phase D output
 *---------------------------------------------------------------------------*/
void phaseD (void const *argument) {
  for (;;) {
    osSignalWait (0x0001, 0xffff);    /* wait for an event flag 0x0001    */
    LED_on (LED_D);
    signal_func (t_phaseA);              /* call common signal function      */
    LED_off(LED_D);
  }
}

/*----------------------------------------------------------------------------
  Task 5 'clock': Signal Clock
 *---------------------------------------------------------------------------*/
void clock (void const *argument) {
  for (;;) {
    osSignalWait (0x0100, 0xffff);    /* wait for an event flag 0x0100    */
    LED_on (LED_CLK);
    osDelay (80);                     /* delay 8 clock ticks              */
    LED_off(LED_CLK);
  }
}

void UART_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);
}

/*----------------------------------------------------------------------------
  Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {

    UART_Init();
    printf("4 LED Threads sample code...\n");
    LED_Init ();                              /* Initialize the LEDs           */
    osKernelInitialize ();

    t_phaseA = osThreadCreate(osThread(phaseA), NULL);  /* start task phaseA   */
    t_phaseB = osThreadCreate(osThread(phaseB), NULL);  /* start task phaseB   */
    t_phaseC = osThreadCreate(osThread(phaseC), NULL);  /* start task phaseC   */
    t_phaseD = osThreadCreate(osThread(phaseD), NULL);  /* start task phaseD   */
    t_clock  = osThreadCreate(osThread(clock), NULL);   /* start task clock    */
    osSignalSet(t_phaseA, 0x0001);         /* send signal event to task phaseA */

    osKernelStart ();
}
