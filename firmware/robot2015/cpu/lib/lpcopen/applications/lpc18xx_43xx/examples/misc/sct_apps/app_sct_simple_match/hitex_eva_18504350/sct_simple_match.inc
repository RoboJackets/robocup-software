#****************************************************************************
# $Id::                                                                     $
#
# Project: SCT Application Example for LPC1800TC
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

# this output is used for internal purposes only
ASSIGN OUTPUT JUMP_SIGNAL	6;

ASSIGN OUTPUT CTOUT0 2;
ASSIGN OUTPUT CTOUT1 3;
ASSIGN OUTPUT CTOUT2 4;
ASSIGN OUTPUT CTOUT3 5;


