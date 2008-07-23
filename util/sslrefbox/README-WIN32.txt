Referee Box 2007 for Windows
============================

Install some packages:
- Dev-C++
- GTK+ and GTKmm from http://www.pcpm.ucl.ac.be/~gustin/win32_ports/.

Compile the referee box:
- Edit the Makefile and replace the string "`pkg-config gtkmm-2.4 --cflags`"
  with the output of the command "pkg-config gtkmm-2.4 --cflags". The 
  pkg-config tool is a part of the GTK+ development environment. 
- Replace also the string "`pkg-config gtkmm-2.4 --libs`" with the output of 
  the comand "pkg-config gtkmm-2.4 --libs".
- Compile with Makefile.win32: "make -f Makefile.win32"

Use it:
- Edit referee.conv and replace the SERIALDEVICE entry "/dev/ttyS0" with 
  "COM1:" for the first serial device.  
- Start the program referee.exe. If it doesn't start, call it from a shell 
  to see the error message. 
 
