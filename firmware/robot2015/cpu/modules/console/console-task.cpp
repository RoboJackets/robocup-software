#include "console.hpp"

/**
 * initializes the console
 */
void Task_SerialConsole(void const *args)
{
  Console::Init();

  while (true) {
    //check console communications, currently does nothing
    //then execute any active iterative command
    Console::ConComCheck();
    
    //execute any active iterative command
    executeIterativeCommand();

    //check if a system stop is requested
    if (Console::IsSystemStopRequested() == true)
      break;

    Thread::wait(100);
  }
}