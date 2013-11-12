#pragma once

/* Baud rate */
#define COMM_BAUD_RATE		   	9600

// play commands
#define COMM_STOP		       	'S'
#define COMM_START		       	's'
#define COMM_HALT	           	'H'
#define COMM_READY	           	' '

// game flow commands
#define COMM_FIRST_HALF		   	'1'
#define COMM_HALF_TIME		   	'h'
#define COMM_SECOND_HALF	   	'2'
#define COMM_OVER_TIME1		   	'o'
#define COMM_OVER_TIME2		   	'O'
#define COMM_PENALTY_SHOOTOUT  	'a'

/* timeout commands */
#define COMM_TIMEOUT_YELLOW	   	't'
#define COMM_TIMEOUT_BLUE	   	'T'
#define COMM_TIMEOUT_END       	'z'

#define COMM_CANCEL            	'c'

// goal status
#define COMM_GOAL_YELLOW	   	'g'
#define COMM_GOAL_BLUE		   	'G'
#define COMM_SUBGOAL_YELLOW	   	'd'
#define COMM_SUBGOAL_BLUE	   	'D'

// penalty signals
#define COMM_YELLOWCARD_YELLOW 	'y'
#define COMM_YELLOWCARD_BLUE   	'Y'

#define COMM_REDCARD_YELLOW    	'r'
#define COMM_REDCARD_BLUE      	'R'


/* game flow commands */
#define COMM_RESTART		  	'n'

#define COMM_KICKOFF_YELLOW	  	'k'
#define COMM_KICKOFF_BLUE	  	'K'

#define COMM_PENALTY_YELLOW	  	'p'
#define COMM_PENALTY_BLUE	  	'P'

#define COMM_DIRECT_YELLOW	  	'f'
#define COMM_DIRECT_BLUE      	'F'

#define COMM_INDIRECT_YELLOW  	'i'
#define COMM_INDIRECT_BLUE	  	'I'


// acceptable referee commands
#define COMM_CMD_STRING      	"iIfFpPkKnrRyYdDgGcztTaoO2h1 HsS"
