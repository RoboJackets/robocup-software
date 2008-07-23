#include <stdio.h>
#include <unistd.h>

#include "Radio.hpp"

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Usage: new_test <device>\n");
        return 1;
    }
    
    Radio radio(argv[1]);
    radio.command_mode(true);
    radio.set_channels(45, Radio::Off);
    radio.command_mode(false);

    while (1)
    {
        printf("%02x\n", radio.serial()->read());
    }
        
    return 0;
}
