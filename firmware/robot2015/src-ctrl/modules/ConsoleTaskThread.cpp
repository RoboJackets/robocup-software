#include <rtos.h>
#include <mbed_rpc.h>

#include <Console.hpp>
#include <logger.hpp>
#include <assert.hpp>

#include "task-signals.hpp"
#include "task-globals.hpp"
#include "commands.hpp"

/**
 * Initializes the console
 */
void Task_SerialConsole(void const* args) {
    const osThreadId* mainID = (const osThreadId*)args;

    // Store the thread's ID
    osThreadId threadID = Thread::gettid();
    ASSERT(threadID != nullptr);

    // Store our priority so we know what to reset it to after running a command
    osPriority threadPriority = osThreadGetPriority(threadID);

    // Initalize the console buffer and save the char buffer's starting address
    Console::Init();

    // Set the console username to whoever the git author is
    Console::changeUser(git_head_author);

    // Let everyone know we're ok
    LOG(INIT,
        "Serial console ready!\r\n"
        "    Thread ID: %u, Priority: %d",
        threadID, threadPriority);

    // Signal back to main and wait until we're signaled to continue
    osSignalSet((osThreadId)mainID, MAIN_TASK_CONTINUE);
    Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

    // Display RoboJackets if we're up and running at this point during startup
    Console::ShowLogo();

    // Print out the header to show the user we're ready for input
    Console::PrintHeader();

    // Set the title of the terminal window
    Console::SetTitle(std::string("RoboJackets"));

    while (true) {
        // Execute any active iterative command
        execute_iterative_command();

        // If there is a new command to handle, parse and process it
        if (Console::CommandReady() == true) {
            // Increase the thread's priority first so we can make sure the
            // scheduler will select it to run
            osStatus tState =
                osThreadSetPriority(threadID, osPriorityAboveNormal);
            ASSERT(tState == osOK);

            // Execute the command
            NVIC_DisableIRQ(UART0_IRQn);
            execute_line(Console::rxBufferPtr());
            NVIC_EnableIRQ(UART0_IRQn);

            // Now, reset the priority of the thread to its idle state
            tState = osThreadSetPriority(threadID, threadPriority);
            ASSERT(tState == osOK);

            Console::CommandHandled(true);
        }

        // Check if a system stop is requested
        if (Console::IsSystemStopRequested() == true) break;

        // Yield to other threads when not needing to execute anything
        Thread::yield();
    }

    // Terminate the thread if the while loop is ever broken out of
    osThreadTerminate(threadID);
}
