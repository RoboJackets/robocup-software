#****************************************************************************
# $Id::                                                                     $
#
# Project: SCT Application Example for LPC4300TC
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

# Define the capabilities of the SCT block
STATES 32;
OUTPUTS 16;
INPUTS 8;
EVENTS 16;

# Assign input/output names to physical I/O numbers

#inputs
ASSIGN INPUT CROSS_REQUEST 5;

#outputs
ASSIGN OUTPUT GREEN_N 	2;
ASSIGN OUTPUT BLUE_N 		4;
ASSIGN OUTPUT RED_N 		3;

# used internally
ASSIGN OUTPUT REQUEST 	5;
ASSIGN OUTPUT AUTO_CROSS 	6;


