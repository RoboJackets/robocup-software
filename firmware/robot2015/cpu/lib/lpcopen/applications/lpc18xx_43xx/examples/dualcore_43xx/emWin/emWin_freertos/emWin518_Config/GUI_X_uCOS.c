/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2012  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.18 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software has been licensed to  NXP Semiconductors USA, Inc.  whose
registered  office  is  situated  at  1109 McKay Dr, M/S 76, San Jose, 
CA 95131, USA  solely for  the  purposes  of  creating  libraries  for 
NXPs M0, M3/M4 and  ARM7/9 processor-based  devices,  sublicensed  and
distributed under the terms and conditions of the NXP End User License
Agreement.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : GUI_X_uCOS.C
Purpose     : Config / System dependent externals for GUI
---------------------------END-OF-HEADER------------------------------
*/

#include "GUI.h"

/* uCOS-iii include files */
#include "os.h"

#include "debug_frmwrk.h"

/*********************************************************************
*
*       Global data
*/
static struct os_mutex xQueueMutex;
static struct os_sem xSemaTxDone;

/*********************************************************************
*
*      Timing:
*                 GUI_X_GetTime()
*                 GUI_X_Delay(int)

  Some timing dependent routines require a GetTime
  and delay function. Default time unit (tick), normally is
1 ms.
*/

int GUI_X_GetTime(void)
{
	OS_ERR err;
	return ((int) OSTimeGet(&err));
}

void GUI_X_Delay(int ms)
{
	OS_ERR err;
	OSTimeDlyHMSM( 0, 0, (ms / 1000), (ms % 1000), OS_OPT_TIME_HMSM_STRICT, &err);
}

/*********************************************************************
*
*       GUI_X_Init()
*
* Note:
*     GUI_X_Init() is called from GUI_Init is a possibility to init
*     some hardware which needs to be up and running before the GUI.
*     If not required, leave this routine blank.
*/

void GUI_X_Init(void) {
}


/*********************************************************************
*
*       GUI_X_ExecIdle
*
* Note:
*  Called if WM is in idle state
*/

void GUI_X_ExecIdle(void) {}

/*********************************************************************
*
*      Multitasking:
*
*                 GUI_X_InitOS()
*                 GUI_X_GetTaskId()
*                 GUI_X_Lock()
*                 GUI_X_Unlock()
*
* Note:
*   The following routines are required only if emWin is used in a
*   true multi task environment, which means you have more than one
*   thread using the emWin API.
*   In this case the
*                       #define GUI_OS 1
*  needs to be in GUIConf.h
*/

/* Init OS */
void GUI_X_InitOS(void)
{
	OS_ERR err;
	
	/* Create Mutex lock */
	OSMutexCreate(&xQueueMutex, "QueueMutex", &err);
	//configASSERT (err == OS_ERR_NONE);

	/* Queue Semaphore */ 
	OSSemCreate( &xSemaTxDone, "TxDoneSem", 0, &err);
	//configASSERT ( err == OS_ERR_NONE );
}

void GUI_X_Unlock(void)
{
    OS_ERR os_err;
    OSMutexPost(&xQueueMutex, OS_OPT_POST_NONE, &os_err);
}

void GUI_X_Lock(void)
{
    CPU_TS ts;
    OS_ERR os_err;
        
    OSMutexPend(&xQueueMutex, 0, OS_OPT_PEND_BLOCKING, &ts, &os_err);
}

/* Get Task handle */
U32  GUI_X_GetTaskId(void) 
{	  
	return ((U32) OSTCBCurPtr);
}

void GUI_X_WaitEvent (void) 
{
	CPU_TS ts;
	OS_ERR os_err;
        
    OSSemPend(&xSemaTxDone, 0, OS_OPT_PEND_BLOCKING, &ts, &os_err);
}


void GUI_X_SignalEvent (void) 
{
    OS_ERR	ucErr;
    OSSemPost(&xSemaTxDone, OS_OPT_POST_ALL, &ucErr);
}

/*********************************************************************
*
*      Logging: OS dependent

Note:
  Logging is used in higher debug levels only. The typical target
  build does not use logging and does therefor not require any of
  the logging routines below. For a release build without logging
  the routines below may be eliminated to save some space.
  (If the linker is not function aware and eliminates unreferenced
  functions automatically)

*/

void GUI_X_Log     (const char *s) { DEBUGOUT(s); }
void GUI_X_Warn    (const char *s) { DEBUGOUT(s); }
void GUI_X_ErrorOut(const char *s) { DEBUGOUT(s); }

/*************************** End of file ****************************/
