#pragma once

#include "mbed.h" 

#define PACKETS_PER_SEC 60
#define BULK_TRANSMITTER_EN 0

// Create a file system if needed for writing startup information to the boot log
#if RJ_BOOT_LOG
LocalFileSystem local("local");     // Create the local filesystem object
#endif

void radioThreadHandler(void const* args);
