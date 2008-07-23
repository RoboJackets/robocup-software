#include <stdio.h>
#include <unistd.h>

#include "Serial.hpp"

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Usage: old_test <device>\n");
        return 1;
    }
    
    Serial serial(argv[1], 38400);
    serial.dtr(true);
    serial.write(0x7e);
    serial.write(75);
    serial.write(75);
    serial.dtr(false);
    usleep(10 * 1000);

    while (1)
    {
        serial.write('U');
    }
        
    return 0;
}
