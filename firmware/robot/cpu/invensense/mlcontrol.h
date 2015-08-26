/*******************************************************************************
 * Copyright (c) 2009-2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/

/*******************************************************************************/
/** @defgroup ML_CONTROL

    The Control processes gyroscopes and accelerometers to provide control
    signals that can be used in user interfaces to manipulate objects such as
    documents, images, cursors, menus, etc.

    @{
        @file mlcontrol.h
        @brief Header file for the Control Library.
*/
/******************************************************************************/
#ifndef MLCONTROL_H
#define MLCONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

/* ------------ */
/* - Defines. - */
/* ------------ */

/*******************************************************************************/
/* Control Signals. */
/*******************************************************************************/

#define ML_CONTROL_1 0x0001
#define ML_CONTROL_2 0x0002
#define ML_CONTROL_3 0x0004
#define ML_CONTROL_4 0x0008

/*******************************************************************************/
/* Control Functions. */
/*******************************************************************************/

#define ML_GRID \
    0x0001  // Indicates that the user will be controlling a system that
            //   has discrete steps, such as icons, menu entries, pixels, etc.
#define ML_SMOOTH \
    0x0002  // Indicates that noise from unintentional motion should be filtered
            // out.
#define ML_DEAD_ZONE \
    0x0004  // Indicates that a dead zone should be used, below which sensor
            // data is set to zero.
#define ML_HYSTERESIS \
    0x0008  // Indicates that, when ML_GRID is selected, hysteresis should be
            // used to prevent
//   the control signal from switching rapidly across elements of the grid.</dd>

/*******************************************************************************/
/* Integral reset options. */
/*******************************************************************************/

#define ML_NO_RESET 0x0000
#define ML_RESET 0x0001

/*******************************************************************************/
/* Data select options. */
/*******************************************************************************/

#define ML_CTRL_SIGNAL 0x0000
#define ML_CTRL_GRID_NUM 0x0001

/*******************************************************************************/
/* Control Axis. */
/*******************************************************************************/
#define ML_CTRL_PITCH 0x0000  // (ML_PITCH >> 1)
#define ML_CTRL_ROLL 0x0001   // (ML_ROLL  >> 1)
#define ML_CTRL_YAW 0x0002    // (ML_YAW   >> 1)

/*******************************************************************************/
/* tMLCTRLParams structure default values.                                   */
/*******************************************************************************/

#define MLCTRL_SENSITIVITY_0_DEFAULT 128
#define MLCTRL_SENSITIVITY_1_DEFAULT 128
#define MLCTRL_SENSITIVITY_2_DEFAULT 128
#define MLCTRL_SENSITIVITY_3_DEFAULT 128
#define MLCTRL_FUNCTIONS_DEFAULT 0
#define MLCTRL_CONTROL_SIGNALS_DEFAULT 0
#define MLCTRL_PARAMETER_ARRAY_0_DEFAULT 0
#define MLCTRL_PARAMETER_ARRAY_1_DEFAULT 0
#define MLCTRL_PARAMETER_ARRAY_2_DEFAULT 0
#define MLCTRL_PARAMETER_ARRAY_3_DEFAULT 0
#define MLCTRL_PARAMETER_AXIS_0_DEFAULT 0
#define MLCTRL_PARAMETER_AXIS_1_DEFAULT 0
#define MLCTRL_PARAMETER_AXIS_2_DEFAULT 0
#define MLCTRL_PARAMETER_AXIS_3_DEFAULT 0
#define MLCTRL_GRID_THRESHOLD_0_DEFAULT 1
#define MLCTRL_GRID_THRESHOLD_1_DEFAULT 1
#define MLCTRL_GRID_THRESHOLD_2_DEFAULT 1
#define MLCTRL_GRID_THRESHOLD_3_DEFAULT 1
#define MLCTRL_GRID_MAXIMUM_0_DEFAULT 0
#define MLCTRL_GRID_MAXIMUM_1_DEFAULT 0
#define MLCTRL_GRID_MAXIMUM_2_DEFAULT 0
#define MLCTRL_GRID_MAXIMUM_3_DEFAULT 0
#define MLCTRL_GRID_CALLBACK_DEFAULT 0

/* --------------- */
/* - Structures. - */
/* --------------- */

/**************************************************************************/
/* Control Parameters Structure.                                          */
/**************************************************************************/

typedef struct {
    unsigned short
        sensitivity[4];        // Sensitivity of control signal 1, 2, 3, and 4.
    unsigned short functions;  // Indicates what functions will be used. Can be
                               // a bitwise OR of ML_GRID, ML_SMOOT,
                               // ML_DEAD_ZONE, and ML_HYSTERISIS.
    unsigned short controlSignals;  // Used to register which control signals
                                    // will be processed. Can be a control
                                    // signal or bitwise OR of multiple control
                                    // signals.
    unsigned short parameterArray[4];  // Indicates which parameter array is
                                       // being assigned to a control signal.
                                       // Must be one of ML_GYROS,
                                       // ML_ANGULAR_VELOCITY.
    unsigned short parameterAxis[4];   // Indicates which axis of the parameter
                                       // array will be used. Must be ML_ROLL,
                                       // ML_PITCH, or ML_YAW.
    int gridThreshold[4];  // Threshold of the control signal at which the grid
                           // number will be incremented or decremented.
    int gridMaximum[4];    // Maximum grid number for the control signal.
    void (*gridCallback)(  // User defined callback that will trigger when the
                           // grid location changes.
        unsigned short controlSignal,  // Indicates which control signal crossed
                                       // a grid threshold. Must be one of
                                       // ML_CONTROL_1, ML_CONTROL_2,
                                       // ML_CONTROL_3 or ML_CONTROL_4.
        int* gridNum,  // An array of four numbers representing the grid number
                       // for each control signal.
        int* gridChange  // An array of four numbers representing the change in
                         // grid number for each control signal.
        );
} tMLCTRLParams,      // new type definition
    MLCTRL_Params_t;  // background-compatible type definition

typedef struct {
    int gridNum[4];      // Current grid number for each control signal.
    int controlInt[4];   // Current data for each control signal.
    int lastGridNum[4];  // Previous grid number
    unsigned char controlDir[4];  // Direction of control signal
    int gridChange[4];            // Change in grid number

    int mlGridNumDMP[4];
    int gridNumOffset[4];
    int prevDMPGridNum[4];

} tMLCTRLXData,      // new type definition
    MLCTRLX_Data_t;  // background-compatible type definition

/* --------------------- */
/* - Function p-types. - */
/* --------------------- */

/**************************************************************************/
/* ML Control Functions.                                                  */
/**************************************************************************/

unsigned short MLCtrlGetParams(tMLCTRLParams* params);
unsigned short MLCtrlSetParams(tMLCTRLParams* params);

/*API for handling control signals*/
tMLError MLSetControlSensitivity(unsigned short controlSignal, int sensitivity);
tMLError MLSetControlFunc(unsigned short function);
tMLError MLGetControlSignal(unsigned short controlSignal, unsigned short reset,
                            int* data);
tMLError MLGetGridNum(unsigned short controlSignal, unsigned short reset,
                      int* data);
tMLError MLSetGridThresh(unsigned short controlSignal, int threshold);
tMLError MLSetGridMax(unsigned short controlSignal, int maximum);
tMLError MLSetGridCallback(void (*func)(unsigned short controlSignal,
                                        int* gridNum, int* gridChange));
tMLError MLSetControlSignals(unsigned short controlSignals);
tMLError MLSetControlData(unsigned short controlSignal,
                          unsigned short parameterArray,
                          unsigned short parameterNum);
tMLError MLGetControlData(int* controlSignal, int* gridNum, int* gridChange);

#ifdef __cplusplus
}
#endif

#endif /* MLCONTROL_H */
