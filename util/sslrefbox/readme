========================================================================
  RoboCup F180 League Referee Box
========================================================================

  Project Leader: Brett Browning   (brettb@cs.cmu.edu)
  Affiliation:  Computer Science Department
                Carnegie Mellon University
                5000 Forbes Avenue
                Pittsburgh, PA 15213 USA
                phone: (412) 268-2601

The referee box is aimed pushing the small size league towards full
autonomy with little physical human contact with the robots. The
referee box, which has been long talked about in the small size
league, is a PC program designed to provide a GUI interface to the
assistant referee and serial commands to the two competing teams. The
referee box will keep a record of the timeouts, time remaining, score
and penalties.

More information can be found at:
  http://sourceforge.net/projects/referee-box

RoboCup 2003 rules can be found at
  http://www.itee.uq.edu.au/~wyeth/f180rules.htm

========================================================================
  Build Instructions
------------------------------------------------------------------------
To make the programs simply go to the main directory (ie ./referee-box
after untarring it) and type make. This has been developed on a Debian
linux system using GCC3.2, with GTKmm 2.0. Note that you need GCC 3.2
in order for this to work. Finally, the refbox uses Jim Bruce's serial
code library.

========================================================================
  Running the System
------------------------------------------------------------------------

To fully understand the running of the referee box you must know the RoboCup
2003 rules. These can be found at the above web site.

Basic operation of the referee box is as follows:

The assistant referee runs the referee box (a laptop with split serial
cables) and the referee just does his/her usual thing. The referee box
*only* comes in to play during restarts and stoppages. When a game is
running the assistant referee does nothing. The only exception to this
is when halftime and full time occur. I'll describe this scenario
later.

The referee box has some built-in checks to force the referee to
follow the correct procedures for running the game.  Normal procedures
for common restart conditions are listed below.

Kick-Off for the Blue Team

1. Press "Stop"
2. Press "Kickoff Blue"
3. Press "Ready"
4. Blue team can kick the ball, thus causing normal play to begin.


Direct Free Kick for the Blue Team

1. Press "Stop"
2. Press "Direct Blue"
3. Blue team can kick the ball, thus causing normal play to begin.

Penalty kicks and indirect free kicks follow the same method for
kickoff and direct free kicks, respectively.

In general...

Halt: Available in any state.  All robots must stop moving.
Stop: Available in any state.  Robots must stay 30cm from ball.
Start: Available from stopped state.  Normal play begins.

Kickoff, Penalty, Direct, Indirect: Available after pressing Stop (ie in stopped state)
For Direct and Indirect, play begins after a kick.  For Kickoff and
Penalty, the Ready command is required.

Ready: Available after pressing Kickoff and Penalty.  Plays begins
after a kick.

Cancel Commands: Anytime, but preferably after a Stop.

All timeing of the game is measured by the referee box. Transition to
halftime etc is handled automatically. 

To shortcut the halftime duration, click stop (from the halted state),
then you can proceed as per normal

The enable check button disables/enables the buttons from having any
effect on the referee box operation. The idea is that during a timeout
you can test the operation without affecting the game results.

========================================================================
  Starting the Referee Box
------------------------------------------------------------------------

To start the referee box run ./referee

You can start with a number of options. To see all the options start with
   ./referee -h

Options:

The current options are:
 -C <file>       Use config file <file>
 -f <file>       log file (no extension)
 -r              restart from existing game state

ConfigFiles:

By default, the conf file used is 
   referee.conf

This controls the length of each portion of the game, number of
timeouts etc. The current version matches the RoboCup 2003 rules.

LogFiles:

The log file passed on the parameter line will be appended with the current date
time and .log ie:

 -f gamelog  ->  gamelog.Fri May 23 17:36:09 2003.log

If no logfile is specified, gamelog is used by default. The log file
is a text format that includes the timestamp of each command sent, a
short string, and the game stage and state.

Sav files:

In order to restart in the middle of a game, the -r command can be
used. This will reload the .sav file that is generated during
execution. This file restores the referee game state you to continue
where you left off. The game begins in a halted mode.

=======================================================================
  Code Overview
------------------------------------------------------------------------

The code is pretty simple. main contains the GUI relevant
software. Gamecontrol has the code to keep track of events, interfaces
to serial, and logs all events to a file. The commands and baud rate
are listed in commands.h. Finally referee.conf is the configuration file
for game times, and number of timeouts.

========================================================================
  FILES
------------------------------------------------------------------------
referee.conf      Configuration file
Makefile          the main makefile
Makefile.gtkmm	  makefile to build gtk1.2 version of the referee
refereemm.cc/h	  GUI Code also includes the main function
gamecontrol.cc/h  the game controller class
gameinfo.h        the gameinfo class
commands.h        contains all the protocols
main.cc           For Gtk1.2: The main file also runs all the GUI interfacing
                  build with 'make -f Makefile.nogtkmm'
serial.cc/h       The serial library written by Jim Bruce

========================================================================

See you at RoboCup! - BB
