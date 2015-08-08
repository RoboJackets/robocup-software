#include "console.hpp"

#include "rtos.h"
#include "logger.hpp"


/**
 * Initializes the console
 */
void Task_SerialConsole(void const* args)
{
  // Store the thread's ID
  osThreadId threadID = Thread::gettid();


  // Initalize the console buffer
  Console::Init();


  // Let everyone know we're ok
  LOG(INIT, "Serial console ready! Thread ID: %u", threadID);


  // Print out the header to show the user we're ready for input
  Console::PrintHeader();


  while (true) {

    // Check console communications, currently does nothing
    Console::ConComCheck();


    // Execute any active iterative command
    executeIterativeCommand();


    // Check if a system stop is requested
    if (Console::IsSystemStopRequested() == true)
      break;


    // Yield to other threads when not needing to execute anything
    Thread::yield();
  }


  // Terminate the thread if the while loop is ever broken out of
  osThreadTerminate(threadID);
}
