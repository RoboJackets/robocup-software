#include "commands.hpp"

#include <map>
#include <ctime>

#include <rtos.h>
#include <mbed_rpc.h>
#include <CommModule.hpp>
#include <CC1201.hpp>
#include <numparser.hpp>
#include <logger.hpp>

#include "ds2411.hpp"
#include "neostrip.hpp"
#include "fpga.hpp"

using std::string;
using std::vector;

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
bool iterative_command_state = false;

/**
 * current iterative command args
 */
vector<string> iterative_command_args;

/**
 * the current iterative command handler
 */
int (*iterative_command_handler)(cmd_args_t& args);

}  // end of anonymous namespace

// Create an object to help find files
LocalFileSystem local("local");

/**
 * Commands list. Add command handlers to commands.hpp.
 *
 * Alphabetical order please (here addition and in handler function
 * declaration).
 */
static const vector<command_t> commands = {
    /* command definition example
    {
        {"<alias>", "<alias2>", "<alias...>"},
        is the command iterative,
        command handler function,
        "description",
        "usage"},
    */

    {{"alias", "a"}, false, cmd_alias, "List aliases for commands.", "alias"},

    {{"baud", "baudrate"},
     false,
     cmd_baudrate,
     "set the active baudrate.",
     "baud [[--list|-l] | <rate>]"},

    {{"clear", "cls"}, false, cmd_console_clear, "Clears the screen.", "clear"},

    {{"echo"},
     false,
     cmd_console_echo,
     "echo text back to the console.",
     "echo [<text>...]"},

    {{"exit", "quit"},
     false,
     cmd_console_exit,
     "terminate the console thread.",
     "exit"},

    {{"help", "h", "?"},
     false,
     cmd_help,
     "print this message.",
     "help [{[--list|-l], [--all|-a]}] [<command name>...]"},

    {{"host", "hostname"},
     false,
     cmd_console_hostname,
     "set system hostname.",
     "host <new-name>"},

    {{"info", "version", "i"}, false, cmd_info, "Display system info.", "info"},

    {{"isconn", "checkconn"},
     false,
     cmd_interface_check_conn,
     "determine the mbed interface's connectivity state.",
     "isconn"},

    {{"led"},
     false,
     cmd_led,
     "control the RGB LED.",
     "led {bright <level>, state {on,off}, color <color>}"},

    {{"loglvl", "loglevel"},
     false,
     cmd_log_level,
     "set the console's log level.",
     "loglvl {+,-}..."},

    {{"ls", "l"}, false, cmd_ls, "List contents of current directory", "ls"},

    {{"motors"},
     false,
     cmd_motors,
     "show/set motor parameters.",
     "motors {on, off, show, set <motor-id> <duty-cycle>}"},

    {{"motorscroll"},
     true,
     cmd_motors_scroll,
     "show motor info (until receiving Ctrl-C).",
     "motorscroll"},

    {{"ps"}, false, cmd_ps, "list the active threads.", "ps"},

    {{"radio"},
     false,
     cmd_radio,
     "test radio connectivity.",
     "radio [show, {set {up,down,reset} <port>, {test-tx,test-rx} [<port>], "
     "loopback [<count>], "
     "debug, "
     "ping, "
     "pong, "
     "strobe <num>, "
     "stress-test <count> <delay> <pck-size>}]"},

     {{"ping"},
     true,
     cmd_ping,
     "periodically send ping packets out over radio broadcast"},

     {{"pong"},
     true,
     cmd_pong,
     "reply to ping"},

    {{"reboot", "reset", "restart"},
     false,
     cmd_interface_reset,
     "perform a software reset.",
     "reboot"},

    {{"rmdev"},
     false,
     cmd_interface_disconnect,
     "disconnect the mbed interface chip.",
     "rmdev [-P]"},

    {{"rpc"},
     false,
     cmd_rpc,
     "execute RPC commands.",
     "rpc <rpc-cmd> [<rpc-arg>...]"},

    {{"su", "user"},
     false,
     cmd_console_user,
     "set the active user.",
     "su <user>"}};

/**
* Lists aliases for commands, if args are present, it will only list aliases
* for those commands.
*/
int cmd_alias(cmd_args_t& args) {
    // If no args given, list all aliases
    if (args.empty()) {
        for (uint8_t i = 0; i < commands.size(); i++) {
            printf("\t%s:\t", commands[i].aliases[0].c_str());

            // print aliases
            uint8_t a = 0;

            while (a < commands[i].aliases.size() &&
                   commands[i].aliases[a] != "\0") {
                printf("%s", commands[i].aliases[a].c_str());

                // print commas
                if (a < commands[i].aliases.size() - 1 &&
                    commands[i].aliases[a + 1] != "\0") {
                    printf(", ");
                }

                a++;
            }

            printf("\r\n");
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

                    printf("%s:\r\n", commands[cmdInd].aliases[0].c_str());

                    // print aliases
                    uint8_t a = 0;

                    while (a < commands[cmdInd].aliases.size() &&
                           commands[cmdInd].aliases[a] != "\0") {
                        printf("\t%s", commands[cmdInd].aliases[a].c_str());

                        // print commas
                        if (a < commands[cmdInd].aliases.size() - 1 &&
                            commands[cmdInd].aliases[a + 1] != "\0") {
                            printf(",");
                        }

                        a++;
                    }
                }
            }

            if (!aliasFound) {
                printf("Error listing aliases: command '%s' not found",
                       args[argInd].c_str());
            }

            printf("\r\n");
        }
    }

    return 0;
}

/**
* Clears the console.
*/
int cmd_console_clear(cmd_args_t& args) {
    if (!args.empty()) {
        show_invalid_args(args);
        return 1;
    } else {
        Console::Instance()->Flush();
        printf(ENABLE_SCROLL_SEQ.c_str());
        printf(CLEAR_SCREEN_SEQ.c_str());
    }

    return 0;
}

/**
 * Echos text.
 */
int cmd_console_echo(cmd_args_t& args) {
    for (uint8_t argInd = 0; argInd < args.size(); argInd++)
        printf("%s ", args[argInd].c_str());

    printf("\r\n");

    return 0;
}

/**
 * Requests a system stop. (breaks main loop, or whatever implementation this
 * links to).
 */
int cmd_console_exit(cmd_args_t& args) {
    if (!args.empty()) {
        show_invalid_args(args);
        return 1;
    } else {
        printf("Shutting down serial console. Goodbye!\r\n");
        Console::Instance()->RequestSystemStop();
    }

    return 0;
}

/**
 * Prints command help.
 */
int cmd_help(cmd_args_t& args) {
    // Prints all commands, with details
    if (args.empty()) {
        // Default to a short listing of all the commands
        for (size_t i = 0; i < commands.size(); i++)
            printf("\t%s:\t%s\r\n", commands[i].aliases[0].c_str(),
                   commands[i].description.c_str());

    }
    // Check if there's only 1 argument passed. It may be an option flag to the
    // command
    // Prints all commands - either as a list block or all detailed
    else {
        if (args[0] == "--list" || args[0] == "-l") {
            for (uint8_t i = 0; i < commands.size(); i++) {
                if (i % 5 == 4) {
                    printf("%s\r\n", commands[i].aliases[0].c_str());
                } else if (i == commands.size() - 1) {
                    printf("%s", commands[i].aliases[0].c_str());
                } else {
                    printf("%s,\t\t", commands[i].aliases[0].c_str());
                }
            }
        } else if (args[0] == "--all" || args[0] == "-a") {
            for (uint8_t i = 0; i < commands.size(); i++) {
                // print info about ALL commands
                printf(
                    "%s%s:\r\n"
                    "    Description:\t%s\r\n"
                    "    Usage:\t\t%s\r\n",
                    commands[i].aliases[0].c_str(),
                    (commands[i].is_iterative ? " [ITERATIVE]" : ""),
                    commands[i].description.c_str(), commands[i].usage.c_str());
            }
        } else {
            // Show detailed info of the command given since it was not an
            // option flag
            cmd_help_detail(args);
        }
    }

    return 0;
}

int cmd_help_detail(cmd_args_t& args) {
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
                printf(
                    "%s%s:\r\n"
                    "    Description:\t%s\r\n"
                    "    Usage:\t\t%s\r\n",
                    commands[i].aliases[0].c_str(),
                    (commands[i].is_iterative ? " [ITERATIVE]" : ""),
                    commands[i].description.c_str(), commands[i].usage.c_str());
            }
        }
        // if the command wasn't found, notify
        if (!commandFound) {
            printf("Command \"%s\" not found.\r\n", args[argInd].c_str());
        }
    }

    return 0;
}

/**
 * Resets the mbed (should be the equivalent of pressing the reset button).
 */
int cmd_interface_reset(cmd_args_t& args) {
    if (!args.empty()) {
        show_invalid_args(args);
        return 1;
    } else {
        printf("The system is going down for reboot NOW!\033[0J\r\n");
        Console::Instance()->Flush();

        // give some time for the feedback to get back to the console
        Thread::wait(800);

        mbed_interface_reset();
    }

    return 0;
}

/**
 * Lists files.
 */
int cmd_ls(cmd_args_t& args) {
    DIR* d;
    struct dirent* p;

    std::vector<std::string> filenames;

    if (args.empty()) {
        d = opendir("/local");
    } else {
        d = opendir(args[0].c_str());
    }

    if (d != nullptr) {
        while ((p = readdir(d)) != nullptr) {
            filenames.push_back(string(p->d_name));
        }

        closedir(d);

        // don't use printf until we close the directory
        for (auto& i : filenames) {
            printf(" - %s\r\n", i.c_str());
            Console::Instance()->Flush();
        }

    } else {
        if (!args.empty()) {
            printf("Could not find %s\r\n", args[0].c_str());
        }

        return 1;
    }

    return 0;
}

/**
 * Prints system info.
 */
int cmd_info(cmd_args_t& args) {
    if (!args.empty()) {
        show_invalid_args(args);
        return 1;
    } else {
        char buf[33];
        DS2411_t id;
        unsigned int Interface[5] = {0};
        time_t sys_time = time(nullptr);
        typedef void (*CallMe)(unsigned int[], unsigned int[]);

        strftime(buf, 25, "%c", localtime(&sys_time));
        printf("\tSys Time:\t%s\r\n", buf);
        printf("\tRuntime:\t%.1fs\r\n",
               (clock() - start_s) / static_cast<double>(CLOCKS_PER_SEC));

        // kernel information
        printf("\tKernel Ver:\t%s\r\n", osKernelSystemId);
        printf("\tAPI Ver:\t%u\r\n", osCMSIS);
        printf("\tCommit Hash:\t%s%s\r\n", git_version_hash,
               git_version_dirty ? " (dirty)" : "");

        // show the fpga build hash
        printf("\tFPGA Hash:\t");
        if (FPGA::Instance()->isReady()) {
            std::vector<uint8_t> fpga_version;
            bool dirty_check = FPGA::Instance()->git_hash(fpga_version);

            for (auto const& i : fpga_version) printf("%0x", i);
            if (dirty_check) printf(" (dirty)");
        } else {
            printf("N/A");
        }
        printf("\r\n");

        printf(
            "\tCommit Date:\t%s\r\n"
            "\tCommit Author:\t%s\r\n",
            git_head_date, git_head_author);

        printf("\tBuild Date:\t%s %s\r\n", __DATE__, __TIME__);

        printf("\tBase ID:\t");

        if (ds2411_read_id(RJ_BASE_ID, &id) == ID_HANDSHAKE_FAIL)
            printf("[id chip not connected]\r\n");
        else
            for (int i = 0; i < 6; i++) printf("%02X\r\n", id.serial[i]);

        // info about the mbed's interface chip on the bottom of the mbed
        if (mbed_interface_uid(buf) == -1) memcpy(buf, "N/A\0", 4);

        printf("\tmbed UID:\t0x%s\r\n", buf);

        // Prints out a serial number, taken from the mbed forms
        // https://developer.mbed.org/forum/helloworld/topic/2048/
        Interface[0] = 58;
        CallMe CallMe_entry = (CallMe)0x1FFF1FF1;
        CallMe_entry(Interface, Interface);

        if (!Interface[0])
            printf("\tMCU UID:\t%u %u %u %u\r\n", Interface[1], Interface[2],
                   Interface[3], Interface[4]);
        else
            printf("\tMCU UID:\t\tN/A\r\n");

        // Should be 0x26013F37
        Interface[0] = 54;
        CallMe_entry(Interface, Interface);

        if (!Interface[0])
            printf("\tMCU ID:\t\t%u\r\n", Interface[1]);
        else
            printf("\tMCU ID:\t\tN/A\r\n");

        // show info about the core processor. ARM cortex-m3 in our case
        printf("\tCPUID:\t\t0x%08lX\r\n", SCB->CPUID);

        // ** NOTE: The `mbed_interface_mac()` function does not work! It hangs
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

            printf("\tUSB Byte:\t0x%02\r\n", regVal);
        */
    }

    return 0;
}

int cmd_interface_disconnect(cmd_args_t& args) {
    if (args.size() > 1) {
        show_invalid_args(args);
        return 1;
    }

    else if (args.empty()) {
        mbed_interface_disconnect();
        printf("Disconnected mbed interface.\r\n");

    }

    else if (args[0] == "-P") {
        printf("Powering down mbed interface.\r\n");
        mbed_interface_powerdown();
    }

    return 0;
}

int cmd_interface_check_conn(cmd_args_t& args) {
    if (!args.empty()) {
        show_invalid_args(args);
        return 1;
    }

    else {
        printf("mbed interface connected: %s\r\n",
               mbed_interface_connected() ? "YES" : "NO");
    }

    return 0;
}

int cmd_baudrate(cmd_args_t& args) {
    std::vector<int> valid_rates = {110,   300,    600,    1200,   2400,
                                    4800,  9600,   14400,  19200,  38400,
                                    57600, 115200, 230400, 460800, 921600};

    if (args.empty() || args.size() > 1) {
        printf("Baudrate: %u\r\n", Console::Instance()->Baudrate());
    }

    else if (args.size() == 1) {
        std::string str_baud = args[0];

        if (str_baud == "--list" || str_baud == "-l") {
            printf("Valid baudrates:\r\n");

            for (unsigned int i = 0; i < valid_rates.size(); i++)
                printf("%u\r\n", valid_rates[i]);

        } else if (isPosInt(str_baud)) {
            int new_rate = atoi(str_baud.c_str());

            if (std::find(valid_rates.begin(), valid_rates.end(), new_rate) !=
                valid_rates.end()) {
                Console::Instance()->Baudrate(new_rate);
                printf("New baudrate: %u\r\n", new_rate);
            } else {
                printf(
                    "%u is not a valid baudrate. Use \"--list\" to show valid "
                    "baudrates.\r\n",
                    new_rate);
            }
        } else {
            show_invalid_args(args);
            return 1;
        }
    }

    return 0;
}

int cmd_console_user(cmd_args_t& args) {
    if (args.empty() || args.size() > 1) {
        show_invalid_args(args);
        return 1;
    }

    else {
        Console::Instance()->changeUser(args[0]);
    }

    return 0;
}

int cmd_console_hostname(cmd_args_t& args) {
    if (args.empty() || args.size() > 1) {
        show_invalid_args(args);
        return 1;
    }

    else {
        Console::Instance()->changeHostname(args[0]);
    }

    return 0;
}

int cmd_log_level(cmd_args_t& args) {
    if (args.size() > 1) {
        show_invalid_args(args);
        return 1;
    }

    else if (args.empty()) {
        printf("Log level: %s\r\n", LOG_LEVEL_STRING[rjLogLevel]);
    }

    else {
        // bool storeVals = true;

        if (args[0] == "on" || args[0] == "enable") {
            isLogging = true;
            printf("Logging enabled.\r\n");
        } else if (args[0] == "off" || args[0] == "disable") {
            isLogging = false;
            printf("Logging disabled.\r\n");
        } else {
            // this will return a signed int, so the level
            // could increase or decrease...or stay the same.
            int newLvl = (int)rjLogLevel;  // rjLogLevel is unsigned, so we'll
            // need to change that first
            newLvl += logLvlChange(args[0]);

            if (newLvl >= LOG_LEVEL_END) {
                printf("Unable to set log level above maximum value.\r\n");
                newLvl = rjLogLevel;
            } else if (newLvl <= LOG_LEVEL_START) {
                printf("Unable to set log level below minimum value.\r\n");
                newLvl = rjLogLevel;
            }

            if (newLvl != rjLogLevel) {
                rjLogLevel = newLvl;
                printf("New log level: %s\r\n", LOG_LEVEL_STRING[rjLogLevel]);
            } else {
                // storeVals = false;
                printf("Log level unchanged. Level: %s\r\n",
                       LOG_LEVEL_STRING[rjLogLevel]);
            }
        }
    }

    return 0;
}

int cmd_rpc(cmd_args_t& args) {
    if (args.empty()) {
        show_invalid_args(args);
        return 1;
    } else {
        // remake the original string so it can be passed to RPC
        std::string in_buf("/RPC");
        for (auto const& i : args) in_buf += " " + i;

        std::vector<char> out_buf(100);

        RPC::call(in_buf.c_str(), out_buf.data());

        printf("%s\r\n", out_buf.data());
    }

    return 0;
}

int cmd_led(cmd_args_t& args) {
    if (args.empty()) {
        show_invalid_args(args);
        return 1;
    } else {
        NeoStrip led(RJ_NEOPIXEL);
        led.setFromDefaultBrightness();
        led.setFromDefaultColor();

        if (args[0] == "bright") {
            if (args.size() > 1) {
                float bri = atof(args[1].c_str());
                printf("Setting LED brightness to %.2f.\r\n", bri);
                if (bri > 0 && bri <= 1.0) {
                    NeoStrip::defaultBrightness(bri);
                    led.setFromDefaultBrightness();
                    led.setFromDefaultColor();
                    led.write();
                } else {
                    printf(
                        "Invalid brightness level of %.2f. Use 'state' command "
                        "to toggle LED's state.\r\n",
                        bri);
                }
            } else {
                printf("Current brightness:\t%.2f\r\n", led.brightness());
            }
        } else if (args[0] == "color") {
            if (args.size() > 1) {
                std::map<std::string, NeoColor> colors;
                // order for struct is green, red, blue
                colors["red"] = {0x00, 0xFF, 0x00};
                colors["green"] = {0xFF, 0x00, 0x00};
                colors["blue"] = {0x00, 0x00, 0xFF};
                colors["yellow"] = {0xFF, 0xFF, 0x00};
                colors["purple"] = {0x00, 0xFF, 0xFF};
                colors["white"] = {0xFF, 0xFF, 0xFF};
                auto it = colors.find(args[1]);
                if (it != colors.end()) {
                    printf("Changing color to %s.\r\n", it->first.c_str());
                    led.setPixel(1, it->second.red, it->second.green,
                                 it->second.blue);
                } else {
                    show_invalid_args(args);
                    return 1;
                }
            } else {
                return 1;
            }
            // push out the changes to the led
            led.write();
        } else if (args[0] == "state") {
            if (args.size() > 1) {
                if (args[1] == "on") {
                    printf("Turning LED on.\r\n");
                } else if (args[1] == "off") {
                    printf("Turning LED off.\r\n");
                    led.brightness(0.0);
                } else {
                    show_invalid_args(args[1]);
                    return 1;
                }
                led.setFromDefaultColor();
            } else {
                return 1;
            }
            // push out the changes to the led
            led.write();
        } else {
            show_invalid_args(args);
            return 1;
        }
    }
    return 0;
}

int cmd_ps(cmd_args_t& args) {
    if (args.empty() != true) {
        show_invalid_args(args);
        return 1;
    } else {
        unsigned int num_threads = 0;
        P_TCB p_b = (P_TCB)&os_rdy;
        std::printf("ID\tPRIOR\tB_PRIOR\tSTATE\tDELTA TIME\r\n");
        // iterate over the linked list of tasks
        while (p_b != NULL) {
            std::printf("%u\t%u\t%u\t%u\t%u\r\n", p_b->task_id, p_b->prio,
                        p_b->state, p_b->delta_time);

            num_threads++;
            p_b = p_b->p_lnk;
        }
        std::printf("==============\r\nTotal Threads:\t%u\r\n", num_threads);
    }

    return 0;
}

int cmd_radio(cmd_args_t& args) {
    shared_ptr<CommModule> commModule = CommModule::Instance();

    if (args.empty()) {
        // Default to showing the list of ports
        commModule->printInfo();
        return 0;
    }

    if (!commModule->isReady()) {
        printf("The radio interface is not ready! Unseen bugs may occur!\r\n");
    }

    if (args.size() == 1 || args.size() == 2) {
        rtp::packet pck("LINK TEST PAYLOAD");
        unsigned int portNbr = rtp::port::DISCOVER;

        if (args.size() > 1) portNbr = atoi(args[1].c_str());

        pck.port(portNbr);
        pck.subclass(1);
        pck.address(BASE_STATION_ADDR);

        if (args[0] == "show") {
            commModule->printInfo();

        } else if (args[0] == "test-tx") {
            printf("Placing %u byte packet in TX buffer.\r\n",
                   pck.payload.size());
            commModule->send(pck);

        } else if (args[0] == "test-rx") {
            printf("Placing %u byte packet in RX buffer.\r\n",
                   pck.payload.size());
            commModule->receive(pck);

        } else if (args[0] == "loopback") {
            pck.port(rtp::port::LINK);

            unsigned int i = 1;
            if (args.size() > 1) {
                i = atoi(args[1].c_str());
                portNbr = rtp::port::LINK;
            }

            pck.port(portNbr);
            pck.subclass(2);
            pck.ack(true);
            pck.address(LOOPBACK_ADDR);

            printf(
                "Placing %u, %u byte packet(s) in TX buffer with ACK set.\r\n",
                i, pck.payload.size());

            for (size_t j = 0; j < i; ++j) {
                rtp::packet pck2;
                pck2 = pck;
                commModule->send(pck2);
                Thread::wait(50);
            }

        } else if (args[0] == "strobe") {
            global_radio->strobe(0x30 + atoi(args[1].c_str()));
        } else if (args[0] == "debug") {
            bool wasEnabled = global_radio->isDebugEnabled();
            global_radio->setDebugEnabled(!wasEnabled);
            printf("Radio debugging now %s\r\n",
                   wasEnabled ? "DISABLED" : "ENABLED");
            if (!wasEnabled)
                printf("All strobes will appear in the INF2 logs\r\n");
        } else {
            show_invalid_args(args[0]);
            return 1;
        }
    } else if (args.size() == 3) {
        if (args[0] == "set") {
            if (isPosInt(args[2].c_str())) {
                unsigned int portNbr = atoi(args[2].c_str());

                if (args[1] == "up") {
                    // commModule->openSocket(portNbr);

                } else if (args[1] == "down") {
                    commModule->close(portNbr);
                    printf("Port %u closed.\r\n", portNbr);

                } else if (args[1] == "reset") {
                    commModule->resetCount(portNbr);
                    printf("Reset packet counts for port %u.\r\n", portNbr);

                } else {
                    show_invalid_args(args);
                    return 1;
                }
            } else {
                show_invalid_args(args[2]);
                return 1;
            }
        } else {
            show_invalid_args(args[2]);
            return 1;
        }
    } else if (args.size() >= 4) {
        if (args[0] == "stress-test") {
            unsigned int packet_cnt = atoi(args[1].c_str());
            unsigned int ms_delay = atoi(args[2].c_str());
            unsigned int pck_size = atoi(args[3].c_str());
            rtp::packet pck(std::string(pck_size - 2, '~') + ".");

            pck.port(rtp::port::LINK);
            pck.subclass(3);
            pck.address(LOOPBACK_ADDR);

            if (args.size() > 4) pck.ack(true);
            printf(
                "Beginning radio stress test with %u %sACK, %u byte "
                "packets. %ums delay between packets.\r\n",
                packet_cnt, (args.size() > 4 ? "" : "NON-"), pck.payload.size(),
                ms_delay);

            int start_tick = clock();
            for (size_t i = 0; i < packet_cnt; ++i) {
                Thread::wait(ms_delay);
                commModule->send(pck);
            }
            printf("Stress test finished in %.1fms.\r\n",
                   (clock() - start_tick) /
                       static_cast<double>(CLOCKS_PER_SEC) * 1000);
        }
    } else {
        show_invalid_args(args);
        return 1;
    }

    return 0;
}

#include <time.h>
#include <sstream>

int cmd_pong(cmd_args_t& args) {
    CommModule::Instance()->setTxHandler((CommLink*)global_radio,
        &CommLink::sendPacket,
        rtp::port::PING);
    static Queue<rtp::packet, 2> pings;
    CommModule::Instance()->setRxHandler( [](rtp::packet* pkt) {
        pings.put(new rtp::packet(*pkt));
    }, rtp::port::PING);

    uint32_t timeout_ms = 200;
    osEvent maybePing = pings.get(timeout_ms);
    if (maybePing.status == 16) {
        rtp::packet* ping = (rtp::packet*)maybePing.value.p;

        int pingNbr = ping->payload.d[0];

        // reply with ack
        rtp::packet ack;
        ack.payload.d.push_back(pingNbr);
        ack.port(rtp::port::PING);
        CommModule::Instance()->send(ack);

        printf("Got ping %d\r\n", pingNbr);

        delete ping;
    }

    return 0;
}

int cmd_ping(cmd_args_t& args) {
    CommModule::Instance()->setTxHandler((CommLink*)global_radio,
        &CommLink::sendPacket,
        rtp::port::PING);
    static Queue<rtp::packet, 2> acks;
    CommModule::Instance()->setRxHandler( [](rtp::packet* pkt) {
        acks.put(new rtp::packet(*pkt));
    }, rtp::port::PING);

    static int ping_count = 0;
    static int last_ping = 0;
    static const int Interval = 2;

    if ((clock() - last_ping) / CLOCKS_PER_SEC > Interval) {
        last_ping = clock();

        rtp::packet pck;
        pck.payload.d.push_back(ping_count);
        pck.port(rtp::port::PING);
        CommModule::Instance()->send(pck);

        printf("Sent ping %d\r\n", ping_count);

        ping_count++;
    }

    uint32_t timeout_ms = 10;
    osEvent maybeAck = acks.get(timeout_ms);
    if (maybeAck.status == 16) {
        rtp::packet* ack = (rtp::packet*)maybeAck.value.p;
        printf("  got ack %d\r\n", ack->payload.d[0]);
        delete ack;
    }

    return 0;
}

int cmd_imu(cmd_args_t& args) { return 0; }

/**
 * Command executor.
 *
 * Much of this taken from `console.c` from the old robot firmware (2011).
 */
void execute_line(char* rawCommand) {
    char* endCmd;
    char* cmds = strtok_r(rawCommand, ";", &endCmd);

    while (cmds != nullptr) {
        uint8_t argc = 0;
        string cmdName = "\0";
        vector<string> args;

        char* endArg;
        char* pch = strtok_r(cmds, " ", &endArg);

        while (pch != nullptr) {
            // Check args length
            if (argc > MAX_COMMAND_ARGS) {
                printf("%s\r\n", TOO_MANY_ARGS_MSG.c_str());
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

        if (!cmdName.empty()) {
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
                    if (commands[cmdInd].is_iterative) {
                        iterative_command_state = false;

                        // Sets the current arg count, args, and
                        // command function in fields to be used
                        // in the iterative call.
                        iterative_command_args = args;
                        iterative_command_handler = commands[cmdInd].handler;
                        iterative_command_state = true;
                    }
                    // If the command is not iterative, execute it
                    // once immeidately.
                    else {
                        int exitState = commands[cmdInd].handler(args);
                        if (exitState != 0) {
                            printf("\r\n");
                            cmd_args_t cmdN = {cmdName};
                            cmd_help_detail(cmdN);
                        }
                    }

                    break;
                }
            }
            // if the command wasnt found, print an error
            if (!commandFound) {
                std::size_t pos = COMMAND_NOT_FOUND_MSG.find("%s");

                if (pos == std::string::npos) {  // no format specifier found in
                    // our defined message
                    printf("%s\r\n", COMMAND_NOT_FOUND_MSG.c_str());
                } else {
                    std::string not_found_cmd = COMMAND_NOT_FOUND_MSG;
                    not_found_cmd.replace(pos, 2, cmdName);
                    printf("%s\r\n", not_found_cmd.c_str());
                }
            }
        }

        cmds = strtok_r(nullptr, ";", &endCmd);
        Console::Instance()
            ->Flush();  // make sure we force everything out of stdout
    }
}

/**
 * Executes iterative commands, and is nonblocking regardless
 * of if an iterative command is not running or not.
 */
void execute_iterative_command() {
    if (iterative_command_state) {
        if (Console::Instance()->IterCmdBreakReq()) {
            iterative_command_state = false;

            // reset the flag for receiving a break character in the Console
            // class
            Console::Instance()->IterCmdBreakReq(false);
            // make sure the cursor is enabled
            printf("\033[?25h");
        } else {
            iterative_command_handler(iterative_command_args);
        }
    }
}

void show_invalid_args(cmd_args_t& args) {
    printf("Invalid arguments");

    if (!args.empty()) {
        printf(" ");

        for (unsigned int i = 0; i < args.size() - 1; i++)
            printf("'%s', ", args[i].c_str());

        printf("'%s'.", args.back().c_str());
    } else {
        printf(". No arguments given.");
    }

    printf("\r\n");
}

void show_invalid_args(const string& s) {
    printf("Invalid argument '%s'.\r\n", s.c_str());
}
