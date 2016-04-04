#include <rtos.h>
#include <mbed_rpc.h>

#include <Console.hpp>
#include <logger.hpp>
#include <assert.hpp>

#include "task-signals.hpp"
#include "commands.hpp"

/**
 * Initializes the console
 */
void Task_SerialConsole(void const* args) {
    const osThreadId mainID = (const osThreadId)args;

    // Store the thread's ID
    const osThreadId threadID = Thread::gettid();
    ASSERT(threadID != nullptr);

    // Store our priority so we know what to reset it to after running a command
    const osPriority threadPriority = osThreadGetPriority(threadID);

    // Initalize the console buffer and save the char buffer's starting address
    std::shared_ptr<Console> console = Console::Instance();

    // Set the console username to whoever the git author is
    console->changeUser(git_head_author);

    // Let everyone know we're ok
    LOG(INIT,
        "Serial console ready!\r\n"
        "    Thread ID: %u, Priority: %d",
        threadID, threadPriority);

    // Signal back to main and wait until we're signaled to continue
    osSignalSet(mainID, MAIN_TASK_CONTINUE);
    Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

    // Display RoboJackets if we're up and running at this point during startup
    console->ShowLogo();

    // Print out the header to show the user we're ready for input
    console->PrintHeader();

    // Set the title of the terminal window
    console->SetTitle("RoboJackets");

    while (true) {
        // Execute any active iterative command
        execute_iterative_command();

        // If there is a new command to handle, parse and process it
        if (console->CommandReady() == true) {
            // Increase the thread's priority first so we can make sure the
            // scheduler will select it to run
            osStatus tState =
                osThreadSetPriority(threadID, osPriorityAboveNormal);
            ASSERT(tState == osOK);

            // Execute the command
            size_t rxLen = console->rxBuffer().size() + 1;
            char rx[rxLen];
            memcpy(rx, console->rxBuffer().c_str(), rxLen - 1);
            rx[rxLen - 1] = '\0';

            // Detach the console from reading stdin while the comamnd is
            // running to allow the command to read input.  We re-attach the
            // Console's handler as soon as the command is done executing.
            console->detachInputHandler();
            execute_line(rx);
            // flush any extra characters that were input while executing cmd
            while (console->pc.readable()) console->pc.getc();
            console->attachInputHandler();

            // Now, reset the priority of the thread to its idle state
            tState = osThreadSetPriority(threadID, threadPriority);
            ASSERT(tState == osOK);

            console->CommandHandled();
        }

        // Check if a system stop is requested
        if (console->IsSystemStopRequested() == true) break;

        // Yield to other threads when not needing to execute anything
        Thread::yield();
    }
}
