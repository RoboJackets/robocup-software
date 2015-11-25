#include "commands.hpp"

#include <iomanip>
#include <fstream>

#include <rtos.h>
#include <mbed_rpc.h>

#include <CommModule.hpp>
#include <numparser.hpp>
#include <logger.hpp>

#include "ds2411.hpp"

using namespace std;

extern struct OS_XCB os_rdy;

namespace {
/**
 * error message when a typed command isn't found
 */
const string COMMAND_NOT_FOUND_MSG =
    "Command '%s' not found. Type 'help' for a list of commands.";

/**
 * error message when too many args are provided
 */
const string TOO_MANY_ARGS_MSG = "*** too many arguments ***";

/**
 * indicates if the command held in "iterativeCommand"
 */
volatile bool itCmdState = false;

/**
 * current iterative command args
 */
vector<string> iterativeCommandArgs;

/**
 * the current iterative command handler
 */
command_fp_t iterativeCommandHandler;

}  // anonymous namespace

// Create an object to help find files
LocalFileSystem local("local");

/**
 * Commands list. Add command handlers to commands.hpp.
 *
 * Alphabetical order please (here addition and in handler function
 * declaration).
 */
static const std::vector<command_t> commands = {
    /* COMMAND TEMPALATE
    {
        {"<alias>", "<alias2>", "<alias...>"},
        is the command iterative,
        command handler function,
        "description",
        "usage"},
    */
    {{"alias", "a"},
     false,
     cmd_alias,
     "Lists aliases for commands.",
     "alias | a"},
    {{"clear", "cls"}, false, cmd_clear, "Clears the screen.", "clear | cls"},
    {{"echo"},
     false,
     cmd_echo,
     "Echos text for debugging the serial link.",
     "echo <text>"},
    {{"exit", "quit"},
     false,
     cmd_exitSys,
     "Breaks the main loop.",
     "exit | quit"},
    {{"help", "h", "?"},
     false,
     cmd_help,
     "Prints this message.",
     "help | h | ? [[--list | -l] | [--all | -a] | <command names>]"},
    {{"ping"}, true, cmd_ping, "Check console responsiveness.", "ping"},
    {{"ls", "l"},
     false,
     cmd_ls,
     "List contents of current directory",  //\r\n",  Bugs:\t\tsometimes
     // displays train animations.",
     "ls | l [folder/device]"},
    {{"info", "version", "i"},
     false,
     cmd_info,
     "Display information about the current version of the firmware.",
     "info | version | i"},
    {{"reboot", "reset", "restart"},
     false,
     cmd_resetMbed,
     "Resets the mbed (like pushing the reset button).",
     "reboot | reset | restart"},
    {{"rmdev", "unconnect"},
     false,
     cmd_disconnectInterface,
     "Disconnects the mbed interface chip from the microcontroller.",
     "rmdev | unconnect [-P]"},
    {{"isconn", "checkconn"},
     false,
     cmd_checkInterfaceConn,
     "Checks the connection with a debugging unit.",
     "isconn | checkconn"},
    {{"baud", "baudrate"},
     false,
     cmd_baudrate,
     "Set the serial link's baudrate.",
     "baud | baudrate [[--list | -l] | [<target_rate>]]"},
    {{"su", "user"},
     false,
     cmd_switchUser,
     "Set active user.",
     "su | user <new_username>"},
    {{"host", "hostname"},
     false,
     cmd_switchHostname,
     "Set the system hostname.",
     "host | hostname <new_hostname>"},
    {{"loglvl", "loglevel"},
     false,
     cmd_logLevel,
     "Change the active logging output level.",
     "loglvl | loglevel {+,-}..."},
    {{"motor"},
     false,
     cmd_motors,
     "Show information about the motors.",
     "motor <motor_id>"},
    {{"motorscroll"},
     true,
     cmd_motors_scroll,
     "Continuously update the console with new motor values.",
     "motorscroll"},
    {{"radio"},
     false,
     cmd_radio,
     "Show information about the radio & perform basic radio tasks.",
     "radio [port | [test-tx | test-rx] <port-num>] [[open, close, show, "
     "reset] "
     "<port_num>]"},
    {{"rpc"},
     false,
     cmd_rpc,
     "Execute RPC commands on the mbed.",
     "rpc <rpc-command>"},
    {{"ps"},
     false,
     cmd_ps,
     "List information about all active threads.",
     "ps"}};

/**
* Lists aliases for commands, if args are present, it will only list aliases
* for those commands.
*/
ostream& cmd_alias(ostream& s, const vector<string>& args) {
    // If no args given, list all aliases
    if (args.empty() == true) {
        for (uint8_t i = 0; i < commands.size(); i++) {
            s << "\t" << commands[i].aliases[0] << ":\t";

            // print aliases
            uint8_t a = 0;
            while (a < commands[i].aliases.size() &&
                   commands[i].aliases[a] != "\0") {
                s << commands[i].aliases[a];

                // print commas
                if (a < commands[i].aliases.size() - 1 &&
                    commands[i].aliases[a + 1] != "\0") {
                    s << ", ";
                }
                a++;
            }
            s << endl;
        }
    } else {
        bool aliasFound = false;

        for (uint8_t argInd = 0; argInd < args.size(); argInd++) {
            for (uint8_t cmdInd = 0; cmdInd < commands.size(); cmdInd++) {
                // check match against args
                if (find(commands[cmdInd].aliases.begin(),
                         commands[cmdInd].aliases.end(),
                         args[argInd]) != commands[cmdInd].aliases.end()) {
                    aliasFound = true;

                    s << commands[cmdInd].aliases[0] << ":" << endl;

                    // print aliases
                    uint8_t a = 0;
                    while (a < commands[cmdInd].aliases.size() &&
                           commands[cmdInd].aliases[a] != "\0") {
                        s << "\t" << commands[cmdInd].aliases[a];

                        // print commas
                        if (a < commands[cmdInd].aliases.size() - 1 &&
                            commands[cmdInd].aliases[a + 1] != "\0") {
                            s << ",";
                        }
                        a++;
                    }
                }
            }

            if (aliasFound == false)
                s << "Error listing aliases: command '" << args[argInd]
                  << "' not found";

            s << endl;
        }
    }
    return s;
}

/**
* Clears the console.
*/
ostream& cmd_clear(ostream& s, const vector<string>& args) {
    if (args.empty() == false) {
        s << showInvalidArgs(s, args);
    } else {
        Console::Flush();
        s << ENABLE_SCROLL_SEQ << CLEAR_SCREEN_SEQ;
    }
    return s;
}

/**
 * Echos text.
 */
ostream& cmd_echo(ostream& s, const vector<string>& args) {
    for (uint8_t argInd = 0; argInd < args.size(); argInd++)
        s << args[argInd] << " ";
    s << endl;
    return s;
}

/**
 * Requests a system stop. (breaks main loop, or whatever implementation this
 * links to).
 */
ostream& cmd_exitSys(ostream& s, const vector<string>& args) {
    if (args.empty() == false) {
        s << showInvalidArgs(s, args);
    } else {
        s << "Shutting down serial console. Goodbye!" << endl;
        Console::RequestSystemStop();
    }
    return s;
}

/**
 * Prints command help.
 */
ostream& cmd_help(ostream& s, const vector<string>& args) {
    // Prints all commands, with details
    if (args.empty() == true) {
        // Default to a short listing of all the commands
        for (uint8_t i = 0; i < commands.size(); i++)
            s << "\t" << commands[i].aliases[0] << ":\t"
              << commands[i].description << endl;
    }
    // Check if there's only 1 argument passed. It may be an option flag to the
    // command
    // Prints all commands - either as a list block or all detailed
    else {
        if (strcmp(args[0].c_str(), "--list") == 0 ||
            strcmp(args[0].c_str(), "-l") == 0) {
            for (uint8_t i = 0; i < commands.size(); i++) {
                if (i % 5 == 4) {
                    s << commands[i].aliases[0] << endl;
                } else if (i == commands.size() - 1) {
                    s << commands[i].aliases[0];
                } else {
                    s << commands[i].aliases[0] << ",\t\t";
                }
            }
        } else if (strcmp(args[0].c_str(), "--all") == 0 ||
                   strcmp(args[0].c_str(), "-a") == 0) {
            for (uint8_t i = 0; i < commands.size(); i++) {
                // print info about ALL commands
                s << commands[i].aliases.front()
                  << (commands[i].isIterative ? " [ITERATIVE]" : "") << endl
                  << "    Description:\t" << commands[i].description << endl
                  << "    Usage:\t\t" << commands[i].usage << endl;
            }
        } else {
            // Show detailed info of the command given since it was not an
            // option flag
            s << cmd_help_detail(s, args);
        }
        s << endl;
    }
    return s;
}

ostream& cmd_help_detail(ostream& s, const vector<string>& args) {
    // iterate through args
    for (uint8_t argInd = 0; argInd < args.size(); argInd++) {
        // iterate through commands
        bool commandFound = false;

        for (uint8_t i = 0; i < commands.size(); i++) {
            // check match against args
            if (find(commands[i].aliases.begin(), commands[i].aliases.end(),
                     args[argInd]) != commands[i].aliases.end()) {
                commandFound = true;

                // print info about a command
                s << commands[i].aliases.front()
                  << (commands[i].isIterative ? " [ITERATIVE]" : "") << endl
                  << "    Description:\t" << commands[i].description << endl
                  << "    Usage:\t\t" << commands[i].usage << endl;
            }
        }

        // if the command wasn't found, notify
        if (!commandFound)
            s << "Command \"" << args.at(argInd) << "\" not found." << endl;
    }
    return s;
}

/**
 * Console responsiveness test.
 */
ostream& cmd_ping(ostream& s, const vector<string>& args) {
    if (args.empty() == false) {
        s << showInvalidArgs(s, args);
    } else {
        Console::Flush();
        s << "reply: " << time(nullptr) << endl;
        Console::Flush();
        Thread::wait(600);
    }
    return s;
}

/**
 * Resets the mbed (should be the equivalent of pressing the reset button).
 */
ostream& cmd_resetMbed(ostream& s, const vector<string>& args) {
    if (args.empty() == false) {
        s << showInvalidArgs(s, args);
    } else {
        s << "The system is going down for reboot NOW!\033[0J" << endl;
        Console::Flush();

        // give some time for the feedback to get back to the console
        Thread::wait(800);

        mbed_interface_reset();
    }
    return s;
}

/**
 * Lists files.
 */
ostream& cmd_ls(ostream& s, const vector<string>& args) {
    DIR* d;
    struct dirent* p;

    std::vector<std::string> filenames;

    if (args.empty() == true) {
        d = opendir("/local");
    } else {
        d = opendir(args.front().c_str());
    }

    if (d != nullptr) {
        while ((p = readdir(d)) != nullptr) {
            filenames.push_back(string(p->d_name));
        }

        closedir(d);

        // don't use printf until we close the directory
        for (auto& i : filenames) {
            s << " - " << i << endl;
            Console::Flush();
        }

    } else {
        if (args.empty() == false) {
            s << "Could not find " << args.front() << endl;
        }

        LOG(FATAL, "CODE ERROR! FIX ME!");
    }
    return s;
}

/**
 * Prints system info.
 */
ostream& cmd_info(ostream& s, const vector<string>& args) {
    if (args.empty() == false) {
        s << showInvalidArgs(s, args);

    } else {
        char buf[33];
        DS2411_t id;
        unsigned int Interface[5] = {0};
        time_t sys_time = time(nullptr);
        typedef void (*CallMe)(unsigned int[], unsigned int[]);

        strftime(buf, 25, "%c", localtime(&sys_time));
        s << "\tSys Time:\t" << buf << endl;

        // kernel information
        s << "\tKernel Ver:\t" << osKernelSystemId << endl;
        s << "\tAPI Ver:\t" << osCMSIS << endl;

        s << "\tCommit Hash:\t" << git_version_hash << endl
          << "\tCommit Date:\t" << git_head_date << endl
          << "\tCommit Author:\t" << git_head_author << endl;

        s << "\tBuild Date:\t" << __DATE__ << " " << __TIME__ << endl;

        s << "\tBase ID:\t";
        if (ds2411_read_id(RJ_BASE_ID, &id, true) == ID_HANDSHAKE_FAIL)
            s << "N/A";
        else
            for (int i = 0; i < 6; i++)
                s << "0x" << std::uppercase << setfill('0') << setw(2)
                  << std::hex << id.serial[i] << " ";
        s << endl;

        // info about the mbed's interface chip on the bottom of the mbed
        if (mbed_interface_uid(buf) == -1) memcpy(buf, "N/A\0", 4);
        s << "\tmbed UID:\t" << buf << endl;

        // Prints out a serial number, taken from the mbed forms
        // https://developer.mbed.org/forum/helloworld/topic/2048/
        Interface[0] = 58;
        CallMe CallMe_entry = (CallMe)0x1FFF1FF1;
        CallMe_entry(Interface, Interface);

        if (!Interface[0])
            s << "\tMCU UID:\t" << Interface[1] << Interface[2] << Interface[3]
              << Interface[4];
        else
            s << "\tMCU UID:\t\tN/A";
        s << endl;

        // Should be 0x26013F37
        Interface[0] = 54;
        CallMe_entry(Interface, Interface);

        if (!Interface[0])
            s << "\tMCU ID:\t\t" << Interface[1];
        else
            s << "\tMCU ID:\t\tN/A";
        s << endl;

        // show info about the core processor. ARM cortex-m3 in our case
        s << "\tCPUID:\t\t0x" << std::uppercase << setfill('0') << setw(2)
          << std::hex << SCB->CPUID << endl;

        // ** NOTE: THE mbed_interface_mac() function does not work! It hangs
        // the mbed... **

        /*
            *****
            THIS CODE IS SUPPOSED TO GIVE USB STATUS INFORMATION. IT HANGS AT
           EACH WHILE LOOP.
            [DON'T UNCOMMENT IT UNLESS YOU INTEND TO CHANGE/IMPROVE/FIX IT
           SOMEHOW]
            *****

            LPC_USB->USBDevIntClr = (0x01 << 3);    // clear the DEV_STAT
           interrupt bit before beginning
            LPC_USB->USBDevIntClr = (0x03 << 4);    // make sure CCEmpty &
           CDFull are cleared before starting
            // Sending a COMMAND transfer type for getting the [USB] device
           status. We expect 1 byte.
            LPC_USB->USBCmdCode = (0x05 << 8) | (0xFE << 16);
            while (!(LPC_USB->USBDevIntSt & 0x10)); // wait for the command
           to be completed
            LPC_USB->USBDevIntClr = 0x10;   // clear the CCEmpty interrupt bit

            // Now we request a read transfer type for getting the same thing
            LPC_USB->USBCmdCode = (0x02 << 8) | (0xFE << 16);
            while (!(LPC_USB->USBDevIntSt & 0x20)); // Wait for CDFULL. data
           ready after this in USBCmdData
            uint8_t regVal = LPC_USB->USBCmdData;   // get the byte
            LPC_USB->USBDevIntClr = 0x20;   // clear the CDFULL interrupt bit

            s << "\tUSB Byte:\t0x%02\r\n" << regVal;
        */
    }
    return s;
}

/**
 * [cmd_disconnectMbed description]
 * @param args [description]
 */
ostream& cmd_disconnectInterface(ostream& s, const vector<string>& args) {
    if (args.empty() > 1) {
        s << showInvalidArgs(s, args);
    } else {
        if (args.empty() == true) {
            mbed_interface_disconnect();
            s << "Disconnected mbed interface." << endl;

        } else if (strcmp(args.at(0).c_str(), "-P") == 0) {
            s << "Powering down mbed interface." << endl;
            mbed_interface_powerdown();
        }
    }
    return s;
}

ostream& cmd_checkInterfaceConn(ostream& s, const vector<string>& args) {
    if (args.empty() == false) {
        s << showInvalidArgs(s, args);
    } else {
        s << "mbed interface connected: "
          << (mbed_interface_connected() ? "YES" : "NO") << endl;
    }
    return s;
}

ostream& cmd_baudrate(ostream& s, const vector<string>& args) {
    std::vector<int> valid_rates = {110,   300,    600,    1200,   2400,
                                    4800,  9600,   14400,  19200,  38400,
                                    57600, 115200, 230400, 460800, 921600};

    if (args.size() > 1) {
        s << showInvalidArgs(s, args);
    } else if (args.empty() == true) {
        s << "Baudrate: " << Console::Baudrate() << endl;
    } else if (args.size() == 1) {
        std::string str_baud = args.front();

        if (strcmp(str_baud.c_str(), "--list") == 0 ||
            strcmp(str_baud.c_str(), "-l") == 0) {
            s << "Valid baudrates:" << endl;
            for (unsigned int i = 0; i < valid_rates.size(); i++) {
                s << valid_rates[i] << "\t";
                if (i % 4 == 0 && i != 0) s << endl;
            }

        } else if (isInt(str_baud)) {
            int new_rate = atoi(str_baud.c_str());

            if (std::find(valid_rates.begin(), valid_rates.end(), new_rate) !=
                valid_rates.end()) {
                Console::Baudrate(new_rate);
                s << "New baudrate: " << new_rate << endl;
            } else {
                s << new_rate << " is not a valid baudrate. Use \"--list\" to "
                                 "show valid baudrates." << endl;
            }
        } else {
            s << "Invalid argument \"" << str_baud << "\"." << endl;
        }
    }
    return s;
}

ostream& cmd_switchUser(ostream& s, const vector<string>& args) {
    if (args.empty() == true || args.size() > 1) {
        s << showInvalidArgs(s, args);
    } else {
        Console::changeUser(args.front());
    }
    return s;
}

ostream& cmd_switchHostname(ostream& s, const vector<string>& args) {
    if (args.empty() == true || args.size() > 1) {
        s << showInvalidArgs(s, args);
    } else {
        Console::changeHostname(args.front());
    }
    return s;
}

ostream& cmd_logLevel(ostream& s, const vector<string>& args) {
    if (args.size() > 1) {
        s << showInvalidArgs(s, args);
    } else if (args.empty() == true) {
        s << "Log level: %s\r\n" << LOG_LEVEL_STRING[rjLogLevel];
    } else {
        if (strcmp(args.front().c_str(), "on") == 0 ||
            strcmp(args.front().c_str(), "enable") == 0) {
            isLogging = true;
            s << "Logging enabled." << endl;
        } else if (strcmp(args.front().c_str(), "off") == 0 ||
                   strcmp(args.front().c_str(), "disable") == 0) {
            isLogging = false;
            s << "Logging disabled." << endl;
        } else {
            // this will return a signed int, so the level
            // could increase or decrease...or stay the same.
            int newLvl = (int)rjLogLevel;  // rjLogLevel is unsigned, so we'll
            // need to change that first
            newLvl += logLvlChange(args.front());

            if (newLvl >= LOG_LEVEL_END) {
                s << "Unable to set log level above maximum value." << endl;
                newLvl = rjLogLevel;
            } else if (newLvl <= LOG_LEVEL_START) {
                s << "Unable to set log level below minimum value." << endl;
                newLvl = rjLogLevel;
            }

            if (newLvl != rjLogLevel) {
                rjLogLevel = newLvl;
                s << "New log level: " << LOG_LEVEL_STRING[rjLogLevel] << endl;
            } else {
                s << "Log level unchanged. Level: "
                  << LOG_LEVEL_STRING[rjLogLevel] << endl;
            }
        }
    }
    return s;
}

ostream& cmd_rpc(ostream& s, const vector<string>& args) {
    if (args.empty() == true) {
        s << showInvalidArgs(s, args);

    } else {
        // remake the original string so it can be passed to RPC
        std::string in_buf(args.at(0));
        for (unsigned int i = 1; i < args.size(); i++) {
            in_buf += " " + args.at(i);
        }

        char out_buf[200] = {0};

        RPC::call(in_buf.c_str(), out_buf);

        s << out_buf << endl;
    }
    return s;
}

ostream& cmd_ps(ostream& s, const vector<string>& args) {
    if (args.empty() != true) {
        s << s << showInvalidArgs(s, args);
    } else {
        unsigned int num_threads = 0;
        P_TCB p_b = (P_TCB)&os_rdy;
        s << "ID\tPRIOR\tB_PRIOR\tSTATE\tDELTA TIME" << endl;
        // iterate over the linked list of tasks
        while (p_b != NULL) {
            s << p_b->task_id << "\t" << p_b->task_id << "\t" << p_b->prio
              << "\t" << p_b->state << "\t" << p_b->delta_time << "\t" << endl;

            num_threads++;
            p_b = p_b->p_lnk;
        }
        s << "==============" << endl
          << "Total Threads:\t" << num_threads << endl;
    }
    return s;
}

/**
 * [comm_cmdProcess description]
 * @param args [description]
 */
ostream& cmd_radio(ostream& s, const vector<string>& args) {
    if (args.empty() == true) {
        // Default to showing the list of ports
        CommModule::PrintInfo(true);

    } else if (args.size() == 1) {
        if (strcmp(args.front().c_str(), "port") == 0) {
            CommModule::PrintInfo(true);

        } else {
            if (CommModule::isReady() == true) {
                rtp::packet pck;
                std::string msg = "LINK TEST PAYLOAD";

                pck.header_link = RTP_HEADER(rtp::port::LINK, 1, false, false);
                pck.payload_size = msg.length();
                memcpy((char*)pck.payload, msg.c_str(), pck.payload_size);
                pck.address = BASE_STATION_ADDR;

                if (strcmp(args.front().c_str(), "test-tx") == 0) {
                    s << "Placing " << pck.payload_size
                      << " byte packet in TX buffer." << endl;
                    CommModule::send(pck);

                } else if (strcmp(args.front().c_str(), "test-rx") == 0) {
                    s << "Placing " << pck.payload_size
                      << " byte packet in RX buffer." << endl;
                    CommModule::receive(pck);

                } else if (strcmp(args.front().c_str(), "loopback") == 0) {
                    pck.ack = true;
                    pck.subclass = 2;
                    pck.address = LOOPBACK_ADDR;
                    s << "Placing " << pck.payload_size
                      << " byte packet in TX buffer with ACK set." << endl;
                    CommModule::send(pck);

                } else {
                    s << showInvalidArgs(s, args.front());
                }
            } else {
                s << "The radio interface is not ready!" << endl;
            }
        }
    } else if (args.size() == 2) {
        // Default to showing all port info if no specific port number is given
        // for the 'show' option
        if (strcmp(args.front().c_str(), "ports") == 0) {
            if (strcmp(args.at(1).c_str(), "show") == 0) {
                CommModule::PrintInfo(true);
            }
        }
    } else if (args.size() == 3) {
        if (strcmp(args.front().c_str(), "ports") == 0) {
            if (isInt(args.at(2).c_str())) {
                unsigned int portNbr = atoi(args.at(2).c_str());

                if (strcmp(args.at(1).c_str(), "open") == 0) {
                    CommModule::openSocket(portNbr);

                } else if (strcmp(args.at(1).c_str(), "close") == 0) {
                    CommModule::Close(portNbr);
                    s << "Port " << portNbr << " closed." << endl;

                } else if (strcmp(args.at(1).c_str(), "show") == 0) {
                    // Change to show only the requested port's info
                    CommModule::PrintInfo(true);

                } else if (strcmp(args.at(1).c_str(), "reset") == 0) {
                    CommModule::ResetCount(portNbr);
                    s << "Reset packet counts for port " << portNbr << endl;

                } else {
                    s << showInvalidArgs(s, args);
                }
            } else {
                s << showInvalidArgs(s, args.at(2));
            }
        } else {
            s << showInvalidArgs(s, args.at(2));
        }
    } else {
        s << showInvalidArgs(s, args);
    }
    return s;
}

/**
 * Command executor.
 *
 */
std::ostream& executeLine(ostream& s, char* rawCommand) {
    char* endCmd;
    char* cmds = strtok_r(rawCommand, ";", &endCmd);

    while (cmds != nullptr) {
        uint8_t argc = 0;
        string cmdName = "\0";
        vector<string> args;
        args.reserve(MAX_COMMAND_ARGS);

        char* endArg;
        char* pch = strtok_r(cmds, " ", &endArg);

        while (pch != nullptr) {
            // Check args length
            if (argc > MAX_COMMAND_ARGS) {
                s << TOO_MANY_ARGS_MSG << endl;
                break;
            }

            // Set command name
            if (argc == 0)
                cmdName = string(pch);
            else
                args.push_back(pch);

            argc++;
            pch = strtok_r(nullptr, " ", &endArg);
        }

        if (cmdName.empty() == false) {
            bool commandFound = false;

            for (uint8_t cmdInd = 0; cmdInd < commands.size(); cmdInd++) {
                // check for name match
                if (find(commands[cmdInd].aliases.begin(),
                         commands[cmdInd].aliases.end(),
                         cmdName) != commands[cmdInd].aliases.end()) {
                    commandFound = true;

                    // If the command is desiged to be run every
                    // iteration of the loop, set the handler and
                    // args and flag the loop to execute on each
                    // iteration.
                    if (commands[cmdInd].isIterative) {
                        itCmdState = false;

                        // Sets the current arg count, args, and
                        // command function in fields to be used
                        // in the iterative call.
                        iterativeCommandArgs = args;
                        iterativeCommandHandler = commands[cmdInd].handler;

                        itCmdState = true;
                    }
                    // If the command is not iterative, execute it
                    // once immeidately.
                    else {
                        s << commands[cmdInd].handler(s, args);
                    }

                    break;
                }
            }
            // if the command wasnt found, print an error
            if (!commandFound) {
                std::size_t pos = COMMAND_NOT_FOUND_MSG.find("%s");

                if (pos == std::string::npos) {  // no format specifier found in
                    // our defined message
                    s << COMMAND_NOT_FOUND_MSG << endl;
                } else {
                    std::string not_found_cmd = COMMAND_NOT_FOUND_MSG;
                    not_found_cmd.replace(pos, 2, cmdName);
                    s << not_found_cmd << endl;
                }
            }
        }

        cmds = strtok_r(nullptr, ";", &endCmd);
        Console::Flush();  // make sure we force everything out of stdout
    }
    return s;
}

ostream& showInvalidArgs(ostream& s, const vector<string>& args) {
    // create a local copy
    vector<string> a(args);

    s << "Invalid arguments";
    if (a.empty() == false) {
        s << " ";
        for (vector<string>::iterator it = a.begin(); it != a.end(); ++it)
            s << "'" << *it << "'";
    } else {
        s << ". No arguments given.";
    }
    s << endl;
    return s;
}
ostream& showInvalidArgs(ostream& s, const string& c) {
    return s << "Invalid argument '" << c << "'" << endl;
}

/**
 * Executes iterative commands, and is nonblocking regardless
 * of if an iterative command is not running or not.
 */
ostream& executeIterativeCommand(ostream& s) {
    if (itCmdState == true) {
        if (Console::IterCmdBreakReq() == true) {
            itCmdState = false;
            // reset the flag for receiving a break
            // character in the Console class
            Console::IterCmdBreakReq(false);
        } else {
            s << iterativeCommandHandler(s, iterativeCommandArgs);
        }
    }
    return s;
}
