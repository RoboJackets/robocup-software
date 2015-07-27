#****************************************************************************
# blinky.inc
#
# Project: SCT Application Example for LPC4300
#
# Description:
#   This include file is used for the SCT state machine code generator.
#----------------------------------------------------------------------------
# Software that is described herein is for illustrative purposes only
# which provides customers with programming information regarding the
# products. This software is supplied "AS IS" without any warranties.
# NXP Semiconductors assumes no responsibility or liability for the
# use of the software, conveys no license or title under any patent,
# copyright, or mask work right to the product. NXP Semiconductors
# reserves the right to make changes in the software without
# notification. NXP Semiconductors also make no representation or
# warranty that such application will be suitable for the specified
# use without further testing or modification.
#****************************************************************************

# Define the capabilities of the SCT block in LPC4300
STATES 32;
OUTPUTS 16;
INPUTS 8;
EVENTS 16;


# Assign input/output names to physical I/O numbers
# Note: don't use INPUT 2 on the LPC4300 Test Chip (RaptorTC2)

ASSIGN INPUT DOWN  6;
ASSIGN INPUT RESET 0;

ASSIGN OUTPUT LED1 3;
ASSIGN OUTPUT LED2 2;
ASSIGN OUTPUT LED3 4;
ASSIGN OUTPUT LED4 5;
