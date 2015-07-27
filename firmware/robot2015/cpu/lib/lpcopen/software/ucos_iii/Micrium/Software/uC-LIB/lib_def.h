/*
*********************************************************************************************************
*                                                uC/LIB
*                                        CUSTOM LIBRARY MODULES
*
*                          (c) Copyright 2004-2011; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               uC/LIB is provided in source form to registered licensees ONLY.  It is 
*               illegal to distribute this source code to any third party unless you receive 
*               written permission by an authorized Micrium representative.  Knowledge of 
*               the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest 
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                     CORE CUSTOM LIBRARY MODULE
*
* Filename      : lib_def.h
* Version       : V1.35.00
* Programmer(s) : ITJ
*********************************************************************************************************
* Note(s)       : (1) NO compiler-supplied standard library functions are used in library or product software.
*
*                     (a) ALL standard library functions are implemented in the custom library modules :
*
*                         (1) \<Custom Library Directory>\lib_*.*
*
*                         (2) \<Custom Library Directory>\Ports\<cpu>\<compiler>\lib*_a.*
*
*                               where
*                                       <Custom Library Directory>      directory path for custom library software
*                                       <cpu>                           directory name for specific processor (CPU)
*                                       <compiler>                      directory name for specific compiler
*
*                     (b) Product-specific library functions are implemented in individual products.
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MODULE
*********************************************************************************************************
*/

#ifndef  LIB_DEF_MODULE_PRESENT
#define  LIB_DEF_MODULE_PRESENT


/*$PAGE*/
/*
*********************************************************************************************************
*                                CUSTOM LIBRARY MODULE VERSION NUMBER
*
* Note(s) : (1) (a) The custom library module software version is denoted as follows :
*
*                       Vx.yy.zz
*
*                           where
*                                   V               denotes 'Version' label
*                                   x               denotes     major software version revision number
*                                   yy              denotes     minor software version revision number
*                                   zz              denotes sub-minor software version revision number
*
*               (b) The software version label #define is formatted as follows :
*
*                       ver = x.yyzz * 100 * 100
*
*                           where
*                                   ver             denotes software version number scaled as an integer value
*                                   x.yyzz          denotes software version number, where the unscaled integer 
*                                                       portion denotes the major version number & the unscaled 
*                                                       fractional portion denotes the (concatenated) minor 
*                                                       version numbers
*********************************************************************************************************
*/

#define  LIB_VERSION                                   13500u   /* See Note #1.                                         */


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*
* Note(s) : (1) The custom library software files are located in the following directories :
*
*               (a) \<Custom Library Directory>\lib_*.*
*
*                       where
*                               <Custom Library Directory>      directory path for custom library software
*
*           (2) CPU-configuration  software files are located in the following directories :
*
*               (a) \<CPU-Compiler Directory>\cpu_*.*
*               (b) \<CPU-Compiler Directory>\<cpu>\<compiler>\cpu*.*
*
*                       where
*                               <CPU-Compiler Directory>        directory path for common CPU-compiler software
*                               <cpu>                           directory name for specific processor (CPU)
*                               <compiler>                      directory name for specific compiler
*
*           (3) Compiler MUST be configured to include as additional include path directories :
*
*               (a) '\<Custom Library Directory>\' directory                            See Note #1a
*
*               (b) (1) '\<CPU-Compiler Directory>\'                  directory         See Note #2a
*                   (2) '\<CPU-Compiler Directory>\<cpu>\<compiler>\' directory         See Note #2b
*********************************************************************************************************
*/

#include  <cpu.h>


/*$PAGE*/
/*
*********************************************************************************************************
*                                          STANDARD DEFINES
*********************************************************************************************************
*/

                                                                /* ----------------- BOOLEAN DEFINES ------------------ */
#define  DEF_FALSE                                         0
#define  DEF_TRUE                                          1

#define  DEF_NO                                            0
#define  DEF_YES                                           1

#define  DEF_DISABLED                                      0
#define  DEF_ENABLED                                       1

#define  DEF_INACTIVE                                      0
#define  DEF_ACTIVE                                        1

#define  DEF_INVALID                                       0
#define  DEF_VALID                                         1

#define  DEF_OFF                                           0
#define  DEF_ON                                            1

#define  DEF_CLR                                           0
#define  DEF_SET                                           1

#define  DEF_FAIL                                          0
#define  DEF_OK                                            1


                                                                /* ------------------- BIT DEFINES -------------------- */
#define  DEF_BIT_NONE                                   0x00uL

#define  DEF_BIT_00                                     0x01uL
#define  DEF_BIT_01                                     0x02uL
#define  DEF_BIT_02                                     0x04uL
#define  DEF_BIT_03                                     0x08uL
#define  DEF_BIT_04                                     0x10uL
#define  DEF_BIT_05                                     0x20uL
#define  DEF_BIT_06                                     0x40uL
#define  DEF_BIT_07                                     0x80uL

#define  DEF_BIT_08                                   0x0100uL
#define  DEF_BIT_09                                   0x0200uL
#define  DEF_BIT_10                                   0x0400uL
#define  DEF_BIT_11                                   0x0800uL
#define  DEF_BIT_12                                   0x1000uL
#define  DEF_BIT_13                                   0x2000uL
#define  DEF_BIT_14                                   0x4000uL
#define  DEF_BIT_15                                   0x8000uL

#define  DEF_BIT_16                               0x00010000uL
#define  DEF_BIT_17                               0x00020000uL
#define  DEF_BIT_18                               0x00040000uL
#define  DEF_BIT_19                               0x00080000uL
#define  DEF_BIT_20                               0x00100000uL
#define  DEF_BIT_21                               0x00200000uL
#define  DEF_BIT_22                               0x00400000uL
#define  DEF_BIT_23                               0x00800000uL

#define  DEF_BIT_24                               0x01000000uL
#define  DEF_BIT_25                               0x02000000uL
#define  DEF_BIT_26                               0x04000000uL
#define  DEF_BIT_27                               0x08000000uL
#define  DEF_BIT_28                               0x10000000uL
#define  DEF_BIT_29                               0x20000000uL
#define  DEF_BIT_30                               0x40000000uL
#define  DEF_BIT_31                               0x80000000uL
/*$PAGE*/
#define  DEF_BIT_32                       0x0000000100000000uL
#define  DEF_BIT_33                       0x0000000200000000uL
#define  DEF_BIT_34                       0x0000000400000000uL
#define  DEF_BIT_35                       0x0000000800000000uL
#define  DEF_BIT_36                       0x0000001000000000uL
#define  DEF_BIT_37                       0x0000002000000000uL
#define  DEF_BIT_38                       0x0000004000000000uL
#define  DEF_BIT_39                       0x0000008000000000uL

#define  DEF_BIT_40                       0x0000010000000000uL
#define  DEF_BIT_41                       0x0000020000000000uL
#define  DEF_BIT_42                       0x0000040000000000uL
#define  DEF_BIT_43                       0x0000080000000000uL
#define  DEF_BIT_44                       0x0000100000000000uL
#define  DEF_BIT_45                       0x0000200000000000uL
#define  DEF_BIT_46                       0x0000400000000000uL
#define  DEF_BIT_47                       0x0000800000000000uL

#define  DEF_BIT_48                       0x0001000000000000uL
#define  DEF_BIT_49                       0x0002000000000000uL
#define  DEF_BIT_50                       0x0004000000000000uL
#define  DEF_BIT_51                       0x0008000000000000uL
#define  DEF_BIT_52                       0x0010000000000000uL
#define  DEF_BIT_53                       0x0020000000000000uL
#define  DEF_BIT_54                       0x0040000000000000uL
#define  DEF_BIT_55                       0x0080000000000000uL

#define  DEF_BIT_56                       0x0100000000000000uL
#define  DEF_BIT_57                       0x0200000000000000uL
#define  DEF_BIT_58                       0x0400000000000000uL
#define  DEF_BIT_59                       0x0800000000000000uL
#define  DEF_BIT_60                       0x1000000000000000uL
#define  DEF_BIT_61                       0x2000000000000000uL
#define  DEF_BIT_62                       0x4000000000000000uL
#define  DEF_BIT_63                       0x8000000000000000uL


                                                                /* ------------------ OCTET DEFINES ------------------- */
#define  DEF_OCTET_NBR_BITS                                8
#define  DEF_OCTET_MASK                                 0xFFuL

#define  DEF_NIBBLE_NBR_BITS                               4
#define  DEF_NIBBLE_MASK                                0x0FuL


                                                                /* --------------- NUMBER BASE DEFINES ---------------- */
#define  DEF_NBR_BASE_BIN                                  2
#define  DEF_NBR_BASE_OCT                                  8
#define  DEF_NBR_BASE_DEC                                 10
#define  DEF_NBR_BASE_HEX                                 16


/*$PAGE*/
                                                                /* ----------------- INTEGER DEFINES ------------------ */
#define  DEF_INT_08_NBR_BITS                               8L
#define  DEF_INT_08_MASK                                0xFFuL

#define  DEF_INT_08U_MIN_VAL                               0u
#define  DEF_INT_08U_MAX_VAL                             255u

#define  DEF_INT_08S_MIN_VAL_ONES_CPL                  (-127)
#define  DEF_INT_08S_MAX_VAL_ONES_CPL                    127

#define  DEF_INT_08S_MIN_VAL                            (DEF_INT_08S_MIN_VAL_ONES_CPL - 1)
#define  DEF_INT_08S_MAX_VAL                             DEF_INT_08S_MAX_VAL_ONES_CPL

#define  DEF_INT_08U_NBR_DIG_MIN                           1
#define  DEF_INT_08U_NBR_DIG_MAX                           3

#define  DEF_INT_08S_NBR_DIG_MIN                           3
#define  DEF_INT_08S_NBR_DIG_MAX                           3



#define  DEF_INT_16_NBR_BITS                              16L
#define  DEF_INT_16_MASK                              0xFFFFuL

#define  DEF_INT_16U_MIN_VAL                               0uL
#define  DEF_INT_16U_MAX_VAL                           65535uL

#define  DEF_INT_16S_MIN_VAL_ONES_CPL                (-32767L)
#define  DEF_INT_16S_MAX_VAL_ONES_CPL                  32767L

#define  DEF_INT_16S_MIN_VAL                            (DEF_INT_16S_MIN_VAL_ONES_CPL - 1)
#define  DEF_INT_16S_MAX_VAL                             DEF_INT_16S_MAX_VAL_ONES_CPL

#define  DEF_INT_16U_NBR_DIG_MIN                           1
#define  DEF_INT_16U_NBR_DIG_MAX                           5

#define  DEF_INT_16S_NBR_DIG_MIN                           5
#define  DEF_INT_16S_NBR_DIG_MAX                           5



#define  DEF_INT_32_NBR_BITS                              32L
#define  DEF_INT_32_MASK                          0xFFFFFFFFuL

#define  DEF_INT_32U_MIN_VAL                               0uL
#define  DEF_INT_32U_MAX_VAL                      4294967295uL

#define  DEF_INT_32S_MIN_VAL_ONES_CPL           (-2147483647L)
#define  DEF_INT_32S_MAX_VAL_ONES_CPL             2147483647L

#define  DEF_INT_32S_MIN_VAL                            (DEF_INT_32S_MIN_VAL_ONES_CPL - 1)
#define  DEF_INT_32S_MAX_VAL                             DEF_INT_32S_MAX_VAL_ONES_CPL

#define  DEF_INT_32U_NBR_DIG_MIN                           1
#define  DEF_INT_32U_NBR_DIG_MAX                          10

#define  DEF_INT_32S_NBR_DIG_MIN                          10
#define  DEF_INT_32S_NBR_DIG_MAX                          10



#define  DEF_INT_64_NBR_BITS                              64L
#define  DEF_INT_64_MASK                  0xFFFFFFFFFFFFFFFFuL

#define  DEF_INT_64U_MIN_VAL                               0uL
#define  DEF_INT_64U_MAX_VAL            18446744073709551615uL

#define  DEF_INT_64S_MIN_VAL_ONES_CPL  (-9223372036854775807L)
#define  DEF_INT_64S_MAX_VAL_ONES_CPL    9223372036854775807L

#define  DEF_INT_64S_MIN_VAL                            (DEF_INT_64S_MIN_VAL_ONES_CPL - 1)
#define  DEF_INT_64S_MAX_VAL                             DEF_INT_64S_MAX_VAL_ONES_CPL

#define  DEF_INT_64U_NBR_DIG_MIN                           1
#define  DEF_INT_64U_NBR_DIG_MAX                          20

#define  DEF_INT_64S_NBR_DIG_MIN                          19
#define  DEF_INT_64S_NBR_DIG_MAX                          19



/*$PAGE*/
                                                                /* --------------- CPU INTEGER DEFINES ---------------- */
#define  DEF_INT_CPU_NBR_BITS                           (CPU_CFG_DATA_SIZE * DEF_OCTET_NBR_BITS)


#if     (DEF_INT_CPU_NBR_BITS == DEF_INT_08_NBR_BITS)


#define  DEF_INT_CPU_MASK                                DEF_INT_08_MASK

#define  DEF_INT_CPU_U_MIN_VAL                           DEF_INT_08U_MIN_VAL
#define  DEF_INT_CPU_U_MAX_VAL                           DEF_INT_08U_MAX_VAL

#define  DEF_INT_CPU_S_MIN_VAL                           DEF_INT_08S_MIN_VAL
#define  DEF_INT_CPU_S_MAX_VAL                           DEF_INT_08S_MAX_VAL

#define  DEF_INT_CPU_S_MIN_VAL_ONES_CPL                  DEF_INT_08S_MIN_VAL_ONES_CPL
#define  DEF_INT_CPU_S_MAX_VAL_ONES_CPL                  DEF_INT_08S_MAX_VAL_ONES_CPL



#elif   (DEF_INT_CPU_NBR_BITS == DEF_INT_16_NBR_BITS)


#define  DEF_INT_CPU_MASK                                DEF_INT_16_MASK

#define  DEF_INT_CPU_U_MIN_VAL                           DEF_INT_16U_MIN_VAL
#define  DEF_INT_CPU_U_MAX_VAL                           DEF_INT_16U_MAX_VAL

#define  DEF_INT_CPU_S_MIN_VAL                           DEF_INT_16S_MIN_VAL
#define  DEF_INT_CPU_S_MAX_VAL                           DEF_INT_16S_MAX_VAL

#define  DEF_INT_CPU_S_MIN_VAL_ONES_CPL                  DEF_INT_16S_MIN_VAL_ONES_CPL
#define  DEF_INT_CPU_S_MAX_VAL_ONES_CPL                  DEF_INT_16S_MAX_VAL_ONES_CPL



#elif   (DEF_INT_CPU_NBR_BITS == DEF_INT_32_NBR_BITS)


#define  DEF_INT_CPU_MASK                                DEF_INT_32_MASK

#define  DEF_INT_CPU_U_MIN_VAL                           DEF_INT_32U_MIN_VAL
#define  DEF_INT_CPU_U_MAX_VAL                           DEF_INT_32U_MAX_VAL

#define  DEF_INT_CPU_S_MIN_VAL                           DEF_INT_32S_MIN_VAL
#define  DEF_INT_CPU_S_MAX_VAL                           DEF_INT_32S_MAX_VAL

#define  DEF_INT_CPU_S_MIN_VAL_ONES_CPL                  DEF_INT_32S_MIN_VAL_ONES_CPL
#define  DEF_INT_CPU_S_MAX_VAL_ONES_CPL                  DEF_INT_32S_MAX_VAL_ONES_CPL



#elif   (DEF_INT_CPU_NBR_BITS == DEF_INT_64_NBR_BITS)


#define  DEF_INT_CPU_MASK                                DEF_INT_64_MASK

#define  DEF_INT_CPU_U_MIN_VAL                           DEF_INT_64U_MIN_VAL
#define  DEF_INT_CPU_U_MAX_VAL                           DEF_INT_64U_MAX_VAL

#define  DEF_INT_CPU_S_MIN_VAL                           DEF_INT_64S_MIN_VAL
#define  DEF_INT_CPU_S_MAX_VAL                           DEF_INT_64S_MAX_VAL

#define  DEF_INT_CPU_S_MIN_VAL_ONES_CPL                  DEF_INT_64S_MIN_VAL_ONES_CPL
#define  DEF_INT_CPU_S_MAX_VAL_ONES_CPL                  DEF_INT_64S_MAX_VAL_ONES_CPL



#else

#error  "CPU_CFG_DATA_SIZE  illegally #defined in 'cpu.h'      "
#error  "                   [See 'cpu.h  CONFIGURATION ERRORS']"

#endif



/*$PAGE*/
                                                                /* ------------------- TIME DEFINES ------------------- */
#define  DEF_TIME_NBR_DAY_PER_WK                           7uL
#define  DEF_TIME_NBR_DAY_PER_YR                         365uL
#define  DEF_TIME_NBR_DAY_PER_YR_LEAP                    366uL

#define  DEF_TIME_NBR_HR_PER_DAY                          24uL
#define  DEF_TIME_NBR_HR_PER_WK                         (DEF_TIME_NBR_HR_PER_DAY  * DEF_TIME_NBR_DAY_PER_WK     )
#define  DEF_TIME_NBR_HR_PER_YR                         (DEF_TIME_NBR_HR_PER_DAY  * DEF_TIME_NBR_DAY_PER_YR     )
#define  DEF_TIME_NBR_HR_PER_YR_LEAP                    (DEF_TIME_NBR_HR_PER_DAY  * DEF_TIME_NBR_DAY_PER_YR_LEAP)

#define  DEF_TIME_NBR_MIN_PER_HR                          60uL
#define  DEF_TIME_NBR_MIN_PER_DAY                       (DEF_TIME_NBR_MIN_PER_HR  * DEF_TIME_NBR_HR_PER_DAY     )
#define  DEF_TIME_NBR_MIN_PER_WK                        (DEF_TIME_NBR_MIN_PER_DAY * DEF_TIME_NBR_DAY_PER_WK     )
#define  DEF_TIME_NBR_MIN_PER_YR                        (DEF_TIME_NBR_MIN_PER_DAY * DEF_TIME_NBR_DAY_PER_YR     )
#define  DEF_TIME_NBR_MIN_PER_YR_LEAP                   (DEF_TIME_NBR_MIN_PER_DAY * DEF_TIME_NBR_DAY_PER_YR_LEAP)

#define  DEF_TIME_NBR_SEC_PER_MIN                         60uL
#define  DEF_TIME_NBR_SEC_PER_HR                        (DEF_TIME_NBR_SEC_PER_MIN * DEF_TIME_NBR_MIN_PER_HR     )
#define  DEF_TIME_NBR_SEC_PER_DAY                       (DEF_TIME_NBR_SEC_PER_HR  * DEF_TIME_NBR_HR_PER_DAY     )
#define  DEF_TIME_NBR_SEC_PER_WK                        (DEF_TIME_NBR_SEC_PER_DAY * DEF_TIME_NBR_DAY_PER_WK     )
#define  DEF_TIME_NBR_SEC_PER_YR                        (DEF_TIME_NBR_SEC_PER_DAY * DEF_TIME_NBR_DAY_PER_YR     )
#define  DEF_TIME_NBR_SEC_PER_YR_LEAP                   (DEF_TIME_NBR_SEC_PER_DAY * DEF_TIME_NBR_DAY_PER_YR_LEAP)

#define  DEF_TIME_NBR_mS_PER_SEC                        1000uL
#define  DEF_TIME_NBR_uS_PER_SEC                     1000000uL
#define  DEF_TIME_NBR_nS_PER_SEC                  1000000000uL


/*$PAGE*/
/*
*********************************************************************************************************
*                                             ERROR CODES
*
* Note(s) : (1) All generic library error codes are #define'd in 'lib_def.h';
*               Any module-specific error codes are #define'd in library module header files.
*********************************************************************************************************
*/

#define  LIB_ERR_NONE                                      0u


/*$PAGE*/
/*
*********************************************************************************************************
*                                             DATA TYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                    LIBRARY ERROR CODES DATA TYPE
*********************************************************************************************************
*/

typedef  CPU_INT16U  LIB_ERR;


/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/


/*$PAGE*/
/*
*********************************************************************************************************
*                                             BIT MACRO'S
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              DEF_BIT()
*
* Description : Create bit mask with single, specified bit set.
*
* Argument(s) : bit         Bit number of bit to set.
*
* Return(s)   : Bit mask with single, specified bit set.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) (a) 'bit' values that overflow the target CPU &/or compiler environment (e.g. negative 
*                       or greater-than-CPU-data-size values) MAY generate compiler warnings &/or errors.
*
*                   (b) To avoid overflowing any target CPU &/or compiler's integer data type, unsigned 
*                       bit constant '1' is suffixed with 'L'ong integer modifier.
*
*                       This may still be insufficient for CPUs &/or compilers that support 'long long' 
*                       integer data types, in which case 'LL' integer modifier should be suffixed.  
*                       However, since almost all 16- & 32-bit CPUs & compilers support 'long' integer 
*                       data types but many may NOT support 'long long' integer data types, only 'long' 
*                       integer data types & modifiers are supported.
*********************************************************************************************************
*/

#define  DEF_BIT(bit)                           (1uL << (bit))


/*$PAGE*/
/*
*********************************************************************************************************
*                                           DEF_BIT_MASK()
*
* Description : Shift a bit mask.
*
* Argument(s) : bit_mask    Bit mask to shift.
*
*               bit_shift   Number of bit positions to left-shift bit mask.
*
* Return(s)   : Shifted bit mask.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) 'bit_shift' values that overflow the target CPU &/or compiler environment (e.g. negative
*                   or greater-than-CPU-data-size values) MAY generate compiler warnings &/or errors.
*********************************************************************************************************
*/

#define  DEF_BIT_MASK(bit_mask, bit_shift)             ((bit_mask) << (bit_shift))


/*
*********************************************************************************************************
*                                           DEF_BIT_FIELD()
*
* Description : Create & shift a contiguous bit field.
*
* Argument(s) : bit_field   Number of contiguous bits to set in the bit field.
*
*               bit_shift   Number of bit positions   to left-shift bit field.
*
* Return(s)   : Shifted bit field.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) (a) 'bit_field'/'bit_shift' values that overflow the target CPU &/or compiler 
*                       environment (e.g. negative or greater-than-CPU-data-size values) MAY generate 
*                       compiler warnings &/or errors.
*
*                   (b) To avoid overflowing any target CPU &/or compiler's integer data type, unsigned 
*                       bit constant '1' is suffixed with 'L'ong integer modifier.
*
*                       This may still be insufficient for CPUs &/or compilers that support 'long long' 
*                       integer data types, in which case 'LL' integer modifier should be suffixed.  
*                       However, since almost all 16- & 32-bit CPUs & compilers support 'long' integer 
*                       data types but many may NOT support 'long long' integer data types, only 'long' 
*                       integer data types & modifiers are supported.
*********************************************************************************************************
*/

#define  DEF_BIT_FIELD(bit_field, bit_shift)         ((((bit_field) >= DEF_INT_CPU_NBR_BITS) ? (DEF_INT_CPU_U_MAX_VAL)     \
                                                                                             : (DEF_BIT(bit_field) - 1uL)) \
                                                                                                    << (bit_shift))


/*$PAGE*/
/*
*********************************************************************************************************
*                                            DEF_BIT_SET()
*
* Description : Set specified bit(s) in a value.
*
* Argument(s) : val         Value to modify by setting specified bit(s).
*
*               mask        Mask of bits to set.
*
* Return(s)   : Modified value with specified bit(s) set.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_SET(val, mask)                        ((val) |=  (mask))


/*
*********************************************************************************************************
*                                            DEF_BIT_CLR()
*
* Description : Clear specified bit(s) in a value.
*
* Argument(s) : val         Value to modify by clearing specified bit(s).
*
*               mask        Mask of bits to clear.
*
* Return(s)   : Modified value with specified bit(s) clear.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_CLR(val, mask)                        ((val) &= ~(mask))


/*$PAGE*/
/*
*********************************************************************************************************
*                                          DEF_BIT_IS_SET()
*
* Description : Determine if specified bit(s) in a value are set.
*
* Argument(s) : val         Value to check for specified bit(s) set.
*
*               mask        Mask of bits to check if set.
*
* Return(s)   : DEF_YES, if ALL specified bit(s) are     set in value.
*
*               DEF_NO,  if ALL specified bit(s) are NOT set in value.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_IS_SET(val, mask)                   ((((val) & (mask)) == (mask)) ? (DEF_YES) : (DEF_NO ))


/*
*********************************************************************************************************
*                                          DEF_BIT_IS_CLR()
*
* Description : Determine if specified bit(s) in a value are clear.
*
* Argument(s) : val         Value to check for specified bit(s) clear.
*
*               mask        Mask of bits to check if clear.
*
* Return(s)   : DEF_YES, if ALL specified bit(s) are     clear in value.
*
*               DEF_NO,  if ALL specified bit(s) are NOT clear in value.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_IS_CLR(val, mask)                    (((val) & (mask))            ? (DEF_NO ) : (DEF_YES))


/*$PAGE*/
/*
*********************************************************************************************************
*                                        DEF_BIT_IS_SET_ANY()
*
* Description : Determine if any specified bit(s) in a value are set.
*
* Argument(s) : val         Value to check for specified bit(s) set.
*
*               mask        Mask of bits to check if set.
*
* Return(s)   : DEF_YES, if ANY specified bit(s) are     set in value.
*
*               DEF_NO,  if ALL specified bit(s) are NOT set in value.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_IS_SET_ANY(val, mask)                (((val) & (mask))            ? (DEF_YES) : (DEF_NO ))


/*
*********************************************************************************************************
*                                        DEF_BIT_IS_CLR_ANY()
*
* Description : Determine if any specified bit(s) in a value are clear.
*
* Argument(s) : val         Value to check for specified bit(s) clear.
*
*               mask        Mask of bits to check if clear.
*
* Return(s)   : DEF_YES, if ANY specified bit(s) are     clear in value.
*
*               DEF_NO,  if ALL specified bit(s) are NOT clear in value.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_IS_CLR_ANY(val, mask)               ((((val) & (mask)) != (mask)) ? (DEF_YES) : (DEF_NO ))


/*$PAGE*/
/*
*********************************************************************************************************
*                                          DEF_CHK_VAL_MIN()
*
* Description : Validate a value as greater than or equal to a specified minimum value.
*
* Argument(s) : val        Value to validate.
*
*               val_min    Minimum value to test.
*
* Return(s)   : DEF_OK,    Value is greater than or equal to minimum value.
*
*               DEF_FAIL,  otherwise.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) DEF_CHK_VAL_MIN() avoids directly comparing any two values if only one of the values 
*                   is negative since the negative value might be incorrectly promoted to an arbitrary 
*                   unsigned value if the other value to compare is unsigned.
*
*               (2) Validation of values is limited to the range supported by the compiler &/or target 
*                   environment.  All other values that underflow/overflow the supported range will 
*                   modulo/wrap into the supported range as arbitrary signed or unsigned values.
*
*                   Therefore, any values that underflow the most negative signed value or overflow 
*                   the most positive unsigned value supported by the compiler &/or target environment 
*                   cannot be validated :
*
*                           (    N-1       N     ]
*                           ( -(2   )  ,  2  - 1 ]
*                           (                    ]
*
*                               where
*                                       N       Number of data word bits supported by the compiler 
*                                                   &/or target environment
*
*                   (a) Note that the most negative value, -2^(N-1), is NOT included in the supported 
*                       range since many compilers do NOT always correctly handle this value.
*********************************************************************************************************
*/

#define  DEF_CHK_VAL_MIN(val, val_min)            (((!(((val)     >= 0) && ((val_min) < 0))) && \
                                                     ((((val_min) >= 0) && ((val)     < 0))  || \
                                                       ((val) < (val_min)))) ? DEF_FAIL : DEF_OK)
                                            

/*$PAGE*/
/*
*********************************************************************************************************
*                                          DEF_CHK_VAL_MAX()
*
* Description : Validate a value as less than or equal to a specified maximum value.
*
* Argument(s) : val        Value to validate.
*
*               val_max    Maximum value to test.
*
* Return(s)   : DEF_OK,    Value is less than or equal to maximum value.
*
*               DEF_FAIL,  otherwise.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) DEF_CHK_VAL_MAX() avoids directly comparing any two values if only one of the values 
*                   is negative since the negative value might be incorrectly promoted to an arbitrary 
*                   unsigned value if the other value to compare is unsigned.
*
*               (2) Validation of values is limited to the range supported by the compiler &/or target 
*                   environment.  All other values that underflow/overflow the supported range will 
*                   modulo/wrap into the supported range as arbitrary signed or unsigned values.
*
*                   Therefore, any values that underflow the most negative signed value or overflow 
*                   the most positive unsigned value supported by the compiler &/or target environment 
*                   cannot be validated :
*
*                           (    N-1       N     ]
*                           ( -(2   )  ,  2  - 1 ]
*                           (                    ]
*
*                               where
*                                       N       Number of data word bits supported by the compiler 
*                                                   &/or target environment
*
*                   (a) Note that the most negative value, -2^(N-1), is NOT included in the supported 
*                       range since many compilers do NOT always correctly handle this value.
*********************************************************************************************************
*/

#define  DEF_CHK_VAL_MAX(val, val_max)            (((!(((val_max) >= 0) && ((val)     < 0))) && \
                                                     ((((val)     >= 0) && ((val_max) < 0))  || \
                                                       ((val) > (val_max)))) ? DEF_FAIL : DEF_OK)
                                                

/*$PAGE*/
/*
*********************************************************************************************************
*                                            DEF_CHK_VAL()
*
* Description : Validate a value as greater than or equal to a specified minimum value & less than or 
*                   equal to a specified maximum value.
*
* Argument(s) : val        Value to validate.
*
*               val_min    Minimum value to test.
*
*               val_max    Maximum value to test.
*
* Return(s)   : DEF_OK,    Value is greater than or equal to minimum value AND 
*                                   less    than or equal to maximum value.
*
*               DEF_FAIL,  otherwise.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) DEF_CHK_VAL() avoids directly comparing any two values if only one of the values 
*                   is negative since the negative value might be incorrectly promoted to an arbitrary 
*                   unsigned value if the other value to compare is unsigned.
*
*               (2) Validation of values is limited to the range supported by the compiler &/or target 
*                   environment.  All other values that underflow/overflow the supported range will 
*                   modulo/wrap into the supported range as arbitrary signed or unsigned values.
*
*                   Therefore, any values that underflow the most negative signed value or overflow 
*                   the most positive unsigned value supported by the compiler &/or target environment 
*                   cannot be validated :
*
*                           (    N-1       N     ]
*                           ( -(2   )  ,  2  - 1 ]
*                           (                    ]
*
*                               where
*                                       N       Number of data word bits supported by the compiler 
*                                                   &/or target environment
*
*                   (a) Note that the most negative value, -2^(N-1), is NOT included in the supported 
*                       range since many compilers do NOT always correctly handle this value.
*
*               (3) DEF_CHK_VAL() does NOT validate that maximum value ('val_max') is greater than or 
*                   equal to the minimum value ('val_min').
*********************************************************************************************************
*/

#define  DEF_CHK_VAL(val, val_min, val_max)          (((DEF_CHK_VAL_MIN(val, val_min) == DEF_FAIL) ||                  \
                                                       (DEF_CHK_VAL_MAX(val, val_max) == DEF_FAIL)) ? DEF_FAIL : DEF_OK)


/*$PAGE*/
/*
*********************************************************************************************************
*                                            MATH MACRO'S
*
* Note(s) : (1) Ideally, ALL mathematical macro's & functions SHOULD be defined in the custom mathematics 
*               library ('lib_math.*').  #### However, to maintain backwards compatibility with previously-
*               released modules, mathematical macro & function definitions should only be moved to the 
*               custom mathematics library once all previously-released modules are updated to include the 
*               custom mathematics library.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              DEF_MIN()
*
* Description : Determine the minimum of two values.
*
* Argument(s) : a           First  value.
*
*               b           Second value.
*
* Return(s)   : Minimum of the two values.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_MIN(a, b)                                  (((a) < (b)) ? (a) : (b))


/*
*********************************************************************************************************
*                                              DEF_MAX()
*
* Description : Determine the maximum of two values.
*
* Argument(s) : a           First  value.
*
*               b           Second value.
*
* Return(s)   : Maximum of the two values.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_MAX(a, b)                                  (((a) > (b)) ? (a) : (b))


/*$PAGE*/
/*
*********************************************************************************************************
*                                              DEF_ABS()
*
* Description : Determine the absolute value of a value.
*
* Argument(s) : a           Value to calculate absolute value.
*
* Return(s)   : Absolute value of the value.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_ABS(a)                                     (((a) < 0) ? (-(a)) : (a))


/*$PAGE*/
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                        CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif                                                          /* End of lib def module include.                       */

