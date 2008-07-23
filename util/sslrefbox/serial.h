/*========================================================================
    Serial.h : Class wrapper for Serial I/O on Linux
  ------------------------------------------------------------------------
    Copyright (C) 1999-2002  James R. Bruce
    School of Computer Science, Carnegie Mellon University
  ------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ========================================================================*/


#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <unistd.h>

#ifdef WIN32
#include <windows.h>
#endif

class Serial{
#ifdef WIN32
  HANDLE hCom;
#else
  int fd;
#endif
public:
  Serial() 
  {
#ifdef WIN32
     hCom = INVALID_HANDLE_VALUE;
#else
     fd=0;
#endif
  }

  ~Serial() 
  {
#ifdef WIN32
    if (hCom!=INVALID_HANDLE_VALUE) Close(); 
#else  
    if (fd) close(fd);
#endif
  }


  bool Open(char *device,int baud);
  void Close(void);

  int Read(void *buf,int size)
#ifdef WIN32
    ;
#else
    {return(::read(fd,buf,size));}
#endif
  int Write(void *buf,int size)
#ifdef WIN32
    ;
#else
    {return(::write(fd,buf,size));}
#endif    
  int WriteByte(char b);
  int ReadByte(void);
  	
};

#endif
