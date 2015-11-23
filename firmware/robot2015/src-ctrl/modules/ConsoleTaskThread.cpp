#include "commands.hpp"
#include "TaskSignals.hpp"

#include <rtos.h>
#include <mbed_rpc.h>

#include <Console.hpp>
#include <logger.hpp>
#include <assert.hpp>

/**
 * Initializes the console
 */
void Task_SerialConsole(void const* args) {
    // Store the thread's ID
    osThreadId threadID = Thread::gettid();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
    ASSERT(threadID != nullptr);
#pragma GCC diagnostic pop

    // Store our priority so we know what to reset it to after running a command
    osPriority threadPriority = osThreadGetPriority(threadID);

    // Setup some of the RPC objects so we can create new ones in the console
    // RPC::add_rpc_class<RpcDigitalIn>();
    // RPC::add_rpc_class<RpcDigitalOut>();
    // RPC::add_rpc_class<RpcDigitalInOut>();
    // RPC::add_rpc_class<RpcSPI>();

    // Initalize the console buffer and save the char buffer's starting address
    Console::Init();

    // Set the console username to whoever the git author is
    Console::changeUser(git_head_author);

    // Let everyone know we're ok
    LOG(INIT,
        "Serial console ready!\r\n    Thread ID:\t%u\r\n    Priority:\t%d",
        threadID, threadPriority);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
    // Lower our priority so we will yield to other, more important, startup
    // tasks
    ASSERT(osThreadSetPriority(threadID, osPriorityLow) == osOK);
#pragma GCC diagnostic pop

    // Yield to other threads during startup so that the below lines will
    // print to the console last
    Thread::yield();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
    // Reset our priorty
    ASSERT(osThreadSetPriority(threadID, threadPriority) == osOK);
#pragma GCC diagnostic pop

    // Display RoboJackets if we're up and running at this point during startup
    Console::ShowLogo();

    // Print out the header to show the user we're ready for input
    Console::PrintHeader();

    while (true) {
        // Execute any active iterative command
        executeIterativeCommand();

        // If there is a new command to handle, parse and process it
        if (Console::CommandReady() == true) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
            // Increase the thread's priority first so we can make sure the
            // scheduler will select it to run
            ASSERT(osThreadSetPriority(threadID, osPriorityAboveNormal) ==
                   osOK);
#pragma GCC diagnostic pop

            // Disable UART interrupts & execute the command
            NVIC_DisableIRQ(UART0_IRQn);
            executeLine(Console::rxBufferPtr());

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
            // Now, reset the priority of the thread to its idle state
            ASSERT(osThreadSetPriority(threadID, threadPriority) == osOK);
#pragma GCC diagnostic pop
            Console::CommandHandled(true);

            // Enable UART interrupts again
            NVIC_EnableIRQ(UART0_IRQn);
        }

        // Check if a system stop is requested
        if (Console::IsSystemStopRequested() == true) break;

        // Yield to other threads when not needing to execute anything
        Thread::yield();
    }

    // Terminate the thread if the while loop is ever broken out of
    osThreadTerminate(threadID);
}
