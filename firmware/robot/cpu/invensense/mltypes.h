/*******************************************************************************
 *
 * $Id: mltypes.h 1572 2010-02-09 20:41:05Z kpowell $
 *
 *******************************************************************************/

/*******************************************************************************
 * Copyright (c) 2009 InvenSense Corporation, All Rights Reserved.
 *******************************************************************************/

/*******************************************************************************
    File Name:    MLTypes.h

    Description:  Data types for the InvenSense Motion Library.

********************************************************************************/

#ifndef MLTYPES_H
#define MLTYPES_H

#include <stdint.h>
typedef uint_fast8_t tMLError;

/* - ML Types. - */

typedef long long          MLS64;
typedef int                MLS32;
typedef unsigned int       MLU32;
typedef short              MLS16;
typedef unsigned short     MLU16;
typedef char               MLS8;
typedef unsigned char      MLU8;
typedef unsigned char      MLBOOL;
typedef double             MLDBL;
typedef float              MLFLT;

#ifdef LINUX
typedef unsigned int       HANDLE;
#endif

/* - ML Defines. - */

#ifndef NULL
#define NULL 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* - ML Errors. - */

#define ML_SUCCESS                       (0)
#define ML_ERROR                         (1)
#define ML_ERROR_INVALID_PARAMETER       (2)
#define ML_ERROR_FEATURE_NOT_ENABLED     (3)
#define ML_ERROR_FEATURE_NOT_IMPLEMENTED (4)
#define ML_ERROR_DMP_NOT_STARTED         (6)
#define ML_ERROR_DMP_STARTED             (7)
#define ML_ERROR_NOT_OPENED              (8)
#define ML_ERROR_OPENED                  (9)
#define ML_ERROR_INVALID_MODULE         (10)

#endif /* MLTYPES_H */

