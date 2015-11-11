#include "commands.hpp"
#include "TaskSignals.hpp"

#include <rtos.h>
#include <mbed_rpc.h>
#include <Console.hpp>
#include <logger.hpp>


/**
 * Initializes the console
 */
void Task_SerialConsole(void const* args)
{
  // Store the thread's ID
  osThreadId threadID = Thread::gettid();

  // Store our priority so we know what to reset it to after running a command
  osPriority threadPriority;

  if (threadID != nullptr)
    threadPriority  = osThreadGetPriority(threadID);
  else
    threadPriority = osPriorityIdle;

  // Setup some of the RPC objects so we can create new ones in the console
  // RPC::add_rpc_class<RpcDigitalIn>();
  RPC::add_rpc_class<RpcDigitalOut>();
  RPC::add_rpc_class<RpcDigitalInOut>();
  // RPC::add_rpc_class<RpcSPI>();

  // Initalize the console buffer and save the char buffer's starting address
  Console::Init();

  // Set the console username to whoever the git author is
  Console::changeUser(git_head_author);

  // Let everyone know we're ok
  LOG(INIT, "Serial console ready!\r\n    Thread ID:\t%u\r\n    Priority:\t%d", threadID, threadPriority);

  // Lower our priority so we will yield to other, more important, startup tasks
  if (osThreadSetPriority(threadID, osPriorityLow) == osOK) {
    // Yield to other threads during startup so that the below lines will print to the console last
    Thread::yield();

    // Reset our priorty here and make sure we've successfully raised it back up
    if (osThreadSetPriority(threadID, threadPriority) != osOK) {
      LOG(SEVERE, "Console configuration error");
      osThreadTerminate(threadID);
    }
  }

  // Display RoboJackets if we're up and running at this point during startup
  Console::ShowLogo();

  // Print out the header to show the user we're ready for input
  Console::PrintHeader();

  while (true) {

    // Check console communications, currently does nothing
    Console::ConComCheck();

    // Execute any active iterative command
    executeIterativeCommand();

    // If there is a new command to handle, parse and process it
    if (Console::CommandReady() == true) {

      // Increase the thread's priority first so we can make sure the scheduler will select it to run
      if (osThreadSetPriority(threadID, osPriorityAboveNormal) == osOK) {

        // Disable UART interrupts & execute the command
        NVIC_DisableIRQ(UART0_IRQn);
        executeLine(Console::rxBufferPtr());

        // Now, reset the priority of the thread to its idle state
        if (osThreadSetPriority(threadID, threadPriority) == osOK) {
          Console::CommandHandled(true);
        }

        // Enable UART interrupts again
        NVIC_EnableIRQ(UART0_IRQn);
      }

    }

    // Check if a system stop is requested
    if (Console::IsSystemStopRequested() == true)
      break;

    // Yield to other threads when not needing to execute anything
    Thread::yield();
  }

  // Terminate the thread if the while loop is ever broken out of
  osThreadTerminate(threadID);
}
