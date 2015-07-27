           Micriµm
 1290 Weston Road, Suite 306
       Weston,  FL 33326

        www.micrium.com
        +1 954 217 2036


µC/OS-III, The Real-Time Kernel


           READ ME



-----------------------------------------------------------------------------------------------------------
Release V3.03.00 (2012/02/14):
-----------------------------------------------------------------------------------------------------------
  1) Added Thread Local Storage (TLS) support
     See chapter 20 of the User's Manual

  2) Computation of CPU usage has now a resolution of 1/100th of a percent

  3) Added OSTaskRegGetID() to assign task register IDs dynamically

  4) Now able to Suspend and Resume tasks from ISRs

  5) Added option 'OS_OPT_TASK_NO_TLS' to OSTaskCreate() to specify that a task will not require TLS

  6) Added and changed some error codes

  7) Removed OSMsgPoolExtend() since it was not documented and not used.

  8) Added global OSStatTaskCPUUsageMax to keep track of peak CPU usage.  
     This value is reset by OSStatReset()
     
  9) Added per task .CPUUsageMax which tracks the peak CPU usage of each task.
   

-----------------------------------------------------------------------------------------------------------
Release V3.02.00 (2011/08/01):
-----------------------------------------------------------------------------------------------------------
  1) Corrected pointer to integer cast in OSMemCreate.

  2) Corrected invalid typecast on constant initialization on os_cfg_app.c.

  3) Added a "return" statement after each invocation of the OS_SAFETY_CRITICAL_EXCEPTION() macro.

  4) Fixed OS_TmrLink() while adding Tmr object into middle of spoke linked-list.

  5) Corrected OS_TmrResetPeak() reset maximum number of entries (NbrEntriesMax).

  6) Added note to clarify use of OS_OPT_LINK_DLY option on OS_TmrLink() at OSTmrStart().

  7) Adjusted order of operation on Tick Wheel during insert (OS_TickListInsert) to first set link pointers
     on new object, then add it to the linked-list.

  8) Corrected OS_OPT_POST_NO_SCHED option for OSFlagPost and OSFlagPendAbort.

  9) Initialize Interrupt Queue Handler Task before any other task to prevent usage of OSIntQNbrEntries
     uninitialized.

 10) Corrected use of disabled variable 'OSCfg_ISRStk'.

 11) Changed default OS_CYCLES to 32-bits.

 12) Corrected per task CPU usage statistics computation.

 13) Adjusted version to new format Vx.yy.zz.

 14) Removed OS_TMR_TICK datatype; converted OSTmrTickCtr to OS_TICK to be consistent with other OS_TMR
     structure fields.

 15) Prevent OSSchedLock/Unlock() to be called from ISR.

 16) Re-arranged order of data structure members on OS_TCB to keep non-optional items at the beginning of 
     the structure.

 17) Changed error codes to enum.

 18) Converted OS object types to use CPU_TYPE_CREATE.

 19) Reworked check to not allow to create multiple tasks at idle task priority.

 20) Corrected alignment check on OSMemCreate() to handle cases where sizeof(void *) == 1.

 21) Added changes to priority handling to allow word addressable architectures.

 22) Adjusted copyright in file headers for source available distribution.



-----------------------------------------------------------------------------------------------------------
Release V3.01.2 (2010/05/14):
-----------------------------------------------------------------------------------------------------------
  1) Added error checking for time stamp configuration #defines.

  2) Removal of some MISRA C 2004 errors.

  3) Corrected OS_MEM structure definition for field FreeListPtr.

  4) Moved OSInitHook() to beginning of OSInit() to be consistent with documentation section about
     differences between uC/OS-II and uC/OS-III port functions.

  5) Removed duplicate call to OS_TickListRemove() in OS_TickListUpdate().

  6) Corrected OS_TmrLink() where number of entries in spoke was incremented twice for a particular
     case, and timer previous pointer was alwyas being cleared before exit the function.

  7) Corrected OSTaskChangePrio() to update task priority if task is pending on Task Queue, Task Sem,
     or Flag.

  8) Corrected OSTaskDel() TCB clean up where it could potentially be skipped if context switch happened
     before OSSched().

  9) Corrected sections of code conditional on OS_CFG_Q_EN & OS_CFG_TASK_Q_EN.

 10) Corrected DbgListRemove procedures where it did not clear the DbgPrevPtr of the first object when
     the head of the list was removed.

 11) Removed sections where TimeQuantaCtr was incorrectly reset.

 12) Corrected NULL pointer de-reference on OSMutexDel() with OS_OPT_DEL_ALWAYS option when Mutex created
     and deleted right away.

 13) Added OSStatResetFlag into OS_StatTask to force reset of the computed statistics.

 14) Added OSIntQMaxNbrEntries into computed statistics.

 15) Reordered DbgListAdd/Remove to prevent a still linked object from being cleared.

 16) Corrected DbgList removal in OSTmrDel().

 17) Added critical section in OSMemCreate().

 18) Removed access to uninitialized kernel objects (p_obj->Type).

 19) Corrected port's OSTaskSwHook where OSSchedLockTimeMaxCur was not always cleared between context switch.

 20) Corrected NULL pointer de-reference of p_tcb on OS_SchedRoundRobin(), if no TCB in the ready list.

 21) Adjusted check for nesting interrupts on OSSched() by removing conditional compilation based
     on OS_CFG_CALLED_FROM_ISR_CHK_EN.

 22) Adjusted formatting & function descriptions.



-----------------------------------------------------------------------------------------------------------
Release V3.01.1 (2010/01/11):
-----------------------------------------------------------------------------------------------------------
 1) Added PERIODIC and MATCH modes to OSTimeDlyHMSM().


 2) Improved the performance of the scheduler lock time measurement.


 3) Added OS_CFG_TS_EN in OS_CFG.H which is used to enable/disable time stamping.


 4) OSTaskStkChk() now returns the number of free and used 'ELEMENTS' instead of 'BYTES'.
    This is done for consistency.

 5) Fixed a bug with OS_PendListRemove1() which removes a task from a wait list.


 6) Fixed a bug when pend-aborting a object where a task has multi-pended on the same object 
    multiple times.


 7) Fixed a bug when posting to an event flag group where there are no task(s) pending on the
    event flag group.



-----------------------------------------------------------------------------------------------------------
Release V3.01.0 (2009/12/07):
-----------------------------------------------------------------------------------------------------------
 1) The API for OSTaskCreate() changed.    The seventh argument changed from:

       CPU_STK       *p_stk_limit;

    to

       CPU_STK_SIZE   stk_limit;

    This argument now represents the number of CPU_STK elements left before the stack is empty.  
    This is used when the CPU supports stack limit checking.


 2) As shown above, OS_STK_SIZE has been changed to CPU_STK_SIZE and thus, this data type is declared in uC/CPU
    instead of uC/OS-III.


 3) We removed OS_AppInitHookPtr because it was impossible to initialize this pointer prior to dereferncing it.
    OSInit() set the pointer to NULL and didn't allow the user to change its value before using it.


 4) Added a new operating mode to OSTimeDly(), i.e. OS_OPT_TIME_PERIODIC


 5) Added the function OSSafetyCriticalStart() which, when called will prevent further kernel objects from being created.
    In other words, after calling this function, you will no longer be allowed to create tasks, semaphores, mutexes, etc.
    In some safety critical systems, it's not allowed to create kernel objects once initialization has completed.


 6) Fixed a bug when an object was pend aborted when using OSPendMulti().


 7) Fixed a bug when an object was deleted when using OSPendMulti().


 8) Replaced:

       for (;;)

    to

       while (DEF_ON)

    for tasks to represent that the loop runs while power is applied.


 9) Fixed a bug in OSPendMulti().  
    The scheduler was locked during a critical region that should have been protected by disabling/enabling interrupts.


10) The timestamp is now read when a task is created to determine when the task starts.
    This is used to compute the per task CPU usage.


11) Statistics are reset after determining CPU usage capacity.


12) Changed the copyright notice.
