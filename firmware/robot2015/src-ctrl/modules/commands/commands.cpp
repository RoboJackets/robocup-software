#include "commands.hpp"

#include <mbed_rpc.h>
#include <CommModule.hpp>
#include <numparser.hpp>
#include <logger.hpp>

#include "ds2411.hpp"


namespace
{
/**
 * error message when a typed command isn't found
 */
const string COMMAND_NOT_FOUND_MSG = "Command '%s' not found. Type 'help' for a list of commands.";


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
void (*iterativeCommandHandler)(const vector<string>& args);

}	// anonymous namespace


// Create an object to help find files
LocalFileSystem local("local");


/**
 * Commands list. Add command handlers to commands.hpp.
 *
 * Alphabetical order please (here addition and in handler function declaration).
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

	{
		{"alias", "a"},
		false,
		cmd_alias,
		"Lists aliases for commands.",
		"alias | a"
	},
	{
		{"clear", "cls"},
		false,
		cmd_clear,
		"Clears the screen.",
		"clear | cls"
	},
	{
		{"echo"},
		false,
		cmd_echo,
		"Echos text for debugging the serial link.",
		"echo <text>"
	},
	{
		{"exit", "quit"},
		false,
		cmd_exitSys,
		"Breaks the main loop.",
		"exit | quit"
	},
	{
		{"help", "h", "?"},
		false,
		cmd_help,
		"Prints this message.",
		"help | h | ? [[--list | -l] | [--all | -a] | <command names>]"
	},
	{
		{"ping"},
		true,
		cmd_ping,
		"Check console responsiveness.",
		"ping"
	},
	{
		{"ls", "l"},
		false,
		cmd_ls,
		"List contents of current directory",//\r\n",  Bugs:\t\tsometimes displays train animations.",
		"ls | l [folder/device]"
	},
	{
		{"info", "version", "i"},
		false,
		cmd_info,
		"Display information about the current version of the firmware.",
		"info | version | i"
	},
	{
		{"reboot", "reset", "restart"},
		false,
		cmd_resetMbed,
		"Resets the mbed (like pushing the reset button).",
		"reboot | reset | restart"
	},
	{
		{"rmdev", "unconnect"},
		false,
		cmd_disconnectInterface,
		"Disconnects the mbed interface chip from the microcontroller.",
		"rmdev | unconnect [-P]"
	},
	{
		{"isconn", "checkconn"},
		false,
		cmd_checkInterfaceConn,
		"Checks the connection with a debugging unit.",
		"isconn | checkconn"
	},
	{
		{"baud", "baudrate"},
		false,
		cmd_baudrate,
		"Set the serial link's baudrate.",
		"baud | baudrate [[--list | -l] | [<target_rate>]]"
	},
	{
		{"su", "user"},
		false,
		cmd_switchUser,
		"Set active user.",
		"su | user <new_username>"
	},
	{
		{"host", "hostname"},
		false,
		cmd_switchHostname,
		"Set the system hostname.",
		"host | hostname <new_hostname>"
	},
	{
		{"loglvl", "loglevel"},
		false,
		cmd_logLevel,
		"Change the active logging output level.",
		"loglvl | loglevel {+,-}..."
	},
	{
		{"motor"},
		false,
		motors_cmdProcess,
		"Show information about the motors.",
		"motor <motor_id>"
	},
	{
		{"motorscroll"},
		true,
		motors_cmdScroll,
		"Continuously update the console with new motor values.",
		"motorscroll"
	},
	{
		{"radio"},
		false,
		comm_cmdProcess,
		"Show information about the radio & perform basic radio tasks.",
		"radio [ports | test-tx | test-rx] [[open, close, show, reset] <port_num>]"
	},
	{
		{"rpc"},
		false,
		cmd_rpc,
		"Execute RPC commands on the mbed.",
		"rpc <rpc-command>"
	}
};


/**
* Lists aliases for commands, if args are present, it will only list aliases
* for those commands.
*/

void cmd_alias(const vector<string>& args)
{
	// If no args given, list all aliases
	if (args.empty() == true) {
		for (uint8_t i = 0; i < commands.size(); i++) {
			printf("\t%s:\t", commands[i].aliases[0].c_str());

			//print aliases
			uint8_t a = 0;

			while (a < commands[i].aliases.size()
			        && commands[i].aliases[a] != "\0") {
				printf("%s", commands[i].aliases[a].c_str());

				//print commas
				if (a < commands[i].aliases.size() - 1
				        && commands[i].aliases[a + 1] != "\0") {
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
				//check match against args
				if (find(commands[cmdInd].aliases.begin(),
				         commands[cmdInd].aliases.end(),
				         args[argInd]) != commands[cmdInd].aliases.end()) {
					aliasFound = true;

					printf("%s:\r\n",
					       commands[cmdInd].aliases[0].c_str());

					//print aliases
					uint8_t a = 0;

					while (a < commands[cmdInd].aliases.size()
					        && commands[cmdInd].aliases[a] != "\0") {

						printf("\t%s",
						       commands[cmdInd].aliases[a].c_str());

						//print commas
						if (a < commands[cmdInd].aliases.size() - 1
						        && commands[cmdInd].aliases[a + 1] != "\0") {

							printf(",");
						}

						a++;
					}
				}
			}

			if (aliasFound == false) {
				printf("Error listing aliases: command '%s' not found",
				       args[argInd].c_str());
			}

			printf("\r\n");
		}
	}
}


/**
* Clears the console.
*/
void cmd_clear(const vector<string>& args)
{
	if (args.empty() == false) {
		showInvalidArgs(args);
	} else {
		Console::Flush();
		printf(ENABLE_SCROLL_SEQ.c_str());
		printf(CLEAR_SCREEN_SEQ.c_str());
	}
}


/**
 * Echos text.
 */
void cmd_echo(const vector<string>& args)
{
	for (uint8_t argInd = 0; argInd < args.size(); argInd++)
		printf("%s ", args[argInd].c_str());

	printf("\r\n");
}


/**
 * Requests a system stop. (breaks main loop, or whatever implementation this
 * links to).
 */
void cmd_exitSys(const vector<string>& args)
{
	if (args.empty() == false) {
		showInvalidArgs(args);
	} else {
		printf("Shutting down serial console. Goodbye!\r\n");
		Console::RequestSystemStop();
	}
}


/**
 * Prints command help.
 */
void cmd_help(const vector<string>& args)
{
	// printf("\r\nCtrl + C stops iterative commands\r\n\r\n");

	// Prints all commands, with details
	if (args.empty() == true) {
		// Default to a short listing of all the commands
		for (uint8_t i = 0; i < commands.size(); i++)
			printf("\t%s:\t%s\r\n", commands[i].aliases[0].c_str(), commands[i].description.c_str());

	}
	// Check if there's only 1 argument passed. It may be an option flag to the command
	// Prints all commands - either as a list block or all detailed
	else {
		if (strcmp(args[0].c_str(), "--list") == 0 || strcmp(args[0].c_str(), "-l") == 0) {
			for (uint8_t i = 0; i < commands.size(); i++) {
				if (i % 5 == 4) {
					printf("%s\r\n", commands[i].aliases[0].c_str());
				} else if (i == commands.size() - 1) {
					printf("%s", commands[i].aliases[0].c_str());
				} else {
					printf("%s,\t\t", commands[i].aliases[0].c_str());
				}
			}
		} else if (strcmp(args[0].c_str(), "--all") == 0 || strcmp(args[0].c_str(), "-a") == 0) {
			for (uint8_t i = 0; i < commands.size(); i++) {
				//print info about ALL commands
				printf("%s%s:\r\n"
				       "    Description:\t%s\r\n"
				       "    Usage:\t\t%s\r\n",
				       commands[i].aliases.front().c_str(),
				       (commands[i].isIterative ? " [ITERATIVE]" : ""),
				       commands[i].description.c_str(),
				       commands[i].usage.c_str()
				      );
			}
		} else {
			// Show detailed info of the command given since it was not an option flag
			cmd_help_detail(args);
		}

		printf("\r\n");
	}
}


void cmd_help_detail(const vector<string>& args)
{
	// iterate through args
	for (uint8_t argInd = 0; argInd < args.size(); argInd++) {
		// iterate through commands
		bool commandFound = false;

		for (uint8_t i = 0; i < commands.size(); i++) {
			// check match against args
			if (find(commands[i].aliases.begin(),
			         commands[i].aliases.end(),
			         args[argInd]) != commands[i].aliases.end()) {

				commandFound = true;

				//print info about a command
				printf("%s%s:\r\n"
				       "    Description:\t%s\r\n"
				       "    Usage:\t\t%s\r\n",
				       commands[i].aliases.front().c_str(),
				       (commands[i].isIterative ? " [ITERATIVE]" : ""),
				       commands[i].description.c_str(),
				       commands[i].usage.c_str()
				      );
			}
		}

		//if the command wasn't found, notify
		if (!commandFound) {
			printf("Command \"%s\" not found.\r\n", args.at(argInd).c_str());
		}
	}
}


/**
 * Console responsiveness test.
 */
void cmd_ping(const vector<string>& args)
{
	if (args.empty() == false) {
		showInvalidArgs(args);
	} else {
		time_t sys_time = time(nullptr);
		Console::Flush();
		printf("reply: %lu\r\n", sys_time);
		Console::Flush();

		Thread::wait(600);
	}
}


/**
 * Resets the mbed (should be the equivalent of pressing the reset button).
 */
void cmd_resetMbed(const vector<string>& args)
{
	if (args.empty() == false) {
		showInvalidArgs(args);
	} else {
		printf("The system is going down for reboot NOW!\033[0J\r\n");
		Console::Flush();

		// give some time for the feedback to get back to the console
		Thread::wait(800);

		mbed_interface_reset();
	}
}


/**
 * Lists files.
 */
void cmd_ls(const vector<string>& args)
{
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
		for ( auto& i : filenames ) {
			printf(" - %s\r\n", i.c_str());
			Console::Flush();
		}

	} else {
		if ( args.empty() == false ) {
			printf("Could not find %s\r\n", args.front().c_str());
		}

		LOG(FATAL, "CODE ERROR! FIX ME!");
	}
}


/**
 * Prints system info.
 */
void cmd_info(const vector<string>& args)
{
	if (args.empty() == false) {
		showInvalidArgs(args);

	} else {
		char buf[33];
		DS2411_t id;
		unsigned int Interface[5] = { 0 };
		time_t sys_time = time(nullptr);
		typedef void (*CallMe)(unsigned int[], unsigned int[]);

		strftime(buf, 25, "%c", localtime(&sys_time));
		printf("\tSys Time:\t%s\r\n", buf);

		// kernel information
		printf("\tKernel Ver:\t%s\r\n", osKernelSystemId);
		printf("\tAPI Ver:\t%u\r\n", osCMSIS);

		printf("\tCommit Hash:\t%s\r\n\tCommit Date:\t%s\r\n\tCommit Author:\t%s\r\n",
		       git_version_hash,
		       git_head_date,
		       git_head_author
		      );

		printf("\tBuild Date:\t%s %s\r\n", __DATE__, __TIME__);

		printf("\tBase ID:\t");

		if (ds2411_read_id(RJ_BASE_ID, &id, true) == ID_HANDSHAKE_FAIL)
			printf("N/A\r\n");
		else
			for (int i = 0; i < 6; i++)
				printf("%02X\r\n", id.serial[i]);

		// info about the mbed's interface chip on the bottom of the mbed
		if (mbed_interface_uid(buf) == -1)
			memcpy(buf, "N/A\0", 4);

		printf("\tmbed UID:\t0x%s\r\n", buf);

		// Prints out a serial number, taken from the mbed forms
		// https://developer.mbed.org/forum/helloworld/topic/2048/
		Interface[0] = 58;
		CallMe CallMe_entry = (CallMe)0x1FFF1FF1;
		CallMe_entry(Interface, Interface);

		if (!Interface[0])
			printf("\tMCU UID:\t%u %u %u %u\r\n",
			       Interface[1],
			       Interface[2],
			       Interface[3],
			       Interface[4]
			      );
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


		// ** NOTE: THE mbed_interface_mac() function does not work! It hangs the mbed... **


		/*
			*****
			THIS CODE IS SUPPOSED TO GIVE USB STATUS INFORMATION. IT HANGS AT EACH WHILE LOOP.
			[DON'T UNCOMMENT IT UNLESS YOU INTEND TO CHANGE/IMPROVE/FIX IT SOMEHOW]
			*****

			LPC_USB->USBDevIntClr = (0x01 << 3);	// clear the DEV_STAT interrupt bit before beginning
			LPC_USB->USBDevIntClr = (0x03 << 4);	// make sure CCEmpty & CDFull are cleared before starting
			// Sending a COMMAND transfer type for getting the [USB] device status. We expect 1 byte.
			LPC_USB->USBCmdCode = (0x05 << 8) | (0xFE << 16);
			while (!(LPC_USB->USBDevIntSt & 0x10));	// wait for the command to be completed
			LPC_USB->USBDevIntClr = 0x10;	// clear the CCEmpty interrupt bit

			// Now we request a read transfer type for getting the same thing
			LPC_USB->USBCmdCode = (0x02 << 8) | (0xFE << 16);
			while (!(LPC_USB->USBDevIntSt & 0x20));	// Wait for CDFULL. data ready after this in USBCmdData
			uint8_t regVal = LPC_USB->USBCmdData;	// get the byte
			LPC_USB->USBDevIntClr = 0x20;	// clear the CDFULL interrupt bit

			printf("\tUSB Byte:\t0x%02\r\n", regVal);
		*/
	}
}


/**
 * [cmd_disconnectMbed description]
 * @param args [description]
 */
void cmd_disconnectInterface(const vector<string>& args)
{
	if (args.empty() > 1) {
		showInvalidArgs(args);
	} else {
		if (args.empty() == true) {

			mbed_interface_disconnect();
			printf("Disconnected mbed interface.\r\n");

		} else if (strcmp(args.at(0).c_str(), "-P") == 0) {

			printf("Powering down mbed interface.\r\n");
			mbed_interface_powerdown();
		}
	}
}


void cmd_checkInterfaceConn(const vector<string>& args)
{
	if (args.empty() == false) {
		showInvalidArgs(args);
	} else {
		printf("mbed interface connected: %s\r\n", mbed_interface_connected() ? "YES" : "NO");
	}
}


void cmd_baudrate(const vector<string>& args)
{
	std::vector<int> valid_rates = {110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

	if (args.size() > 1) {
		showInvalidArgs(args);
	} else if (args.empty() == true) {
		printf("Baudrate: %u\r\n", Console::Baudrate());
	} else if (args.size() == 1) {
		std::string str_baud = args.front();

		if (strcmp(str_baud.c_str(), "--list") == 0 || strcmp(str_baud.c_str(), "-l") == 0) {

			printf("Valid baudrates:\r\n");

			for (unsigned int i = 0; i < valid_rates.size(); i++)
				printf("%u\r\n", valid_rates[i]);

		} else if (isInt(str_baud)) {
			int new_rate = atoi(str_baud.c_str());

			if (std::find(valid_rates.begin(), valid_rates.end(), new_rate) != valid_rates.end()) {
				Console::Baudrate(new_rate);
				printf("New baudrate: %u\r\n", new_rate);
			} else {
				printf("%u is not a valid baudrate. Use \"--list\" to show valid baudrates.\r\n", new_rate);
			}
		} else {
			printf("Invalid argument \"%s\".\r\n", str_baud.c_str());
		}
	}
}


void cmd_switchUser(const vector<string>& args)
{
	if (args.empty() == true || args.size() > 1) {
		showInvalidArgs(args);
	} else {
		Console::changeUser(args.front());
	}
}


void cmd_switchHostname(const vector<string>& args)
{
	if (args.empty() == true || args.size() > 1) {
		showInvalidArgs(args);
	} else {
		Console::changeHostname(args.front());
	}
}


void cmd_logLevel(const vector<string>& args)
{
	if (args.size() > 1) {
		showInvalidArgs(args);
	} else if (args.empty() == true) {
		printf("Log level: %s\r\n", LOG_LEVEL_STRING[rjLogLevel]);
	} else {
		// bool storeVals = true;

		if (strcmp(args.front().c_str(), "on") == 0 || strcmp(args.front().c_str(), "enable") == 0) {
			isLogging = true;
			printf("Logging enabled.\r\n");
		} else if (strcmp(args.front().c_str(), "off") == 0 || strcmp(args.front().c_str(), "disable") == 0) {
			isLogging = false;
			printf("Logging disabled.\r\n");
		} else {
			// this will return a signed int, so the level
			// could increase or decrease...or stay the same.
			int newLvl = (int)rjLogLevel;	// rjLogLevel is unsigned, so we'll need to change that first
			newLvl += logLvlChange(args.front());

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
				printf("Log level unchanged. Level: %s\r\n", LOG_LEVEL_STRING[rjLogLevel]);
			}
		}

		// if ( storeVals == true ) {
		// 	// Store the new log level in FLASH memory
		// 	IAP iap;
		// 	char mem[MEM_SIZE] = { 0 };   //  memory, it should be aligned to word boundary

		// 	mem[0] = isLogging ? 0xFF : 0x00;
		// 	mem[1] = ( rjLogLevel & 0xFF );

		// 	iap.prepare( TARGET_SECTOR, TARGET_SECTOR );
		// 	iap.write( mem, sector_start_adress[TARGET_SECTOR], MEM_SIZE );
		// }
	}
}

void cmd_rpc(const vector<string>& args)
{
	if (args.empty() == true) {
		showInvalidArgs(args);

	} else {
		// remake the original string so it can be passed to RPC
		std::string in_buf(args.at(0));
		for (unsigned int i = 1; i < args.size(); i++) {
			in_buf += " " + args.at(i);
		}

		char out_buf[200] = { 0 };

		RPC::call(in_buf.c_str(), out_buf);

		std::printf("%s\r\n", out_buf);
	}
}

/**
 * [comm_cmdProcess description]
 * @param args [description]
 */
void comm_cmdProcess(const vector<string>& args)
{
	if (args.empty() == true) {
		// Default to showing the list of ports
		CommModule::PrintInfo(true);

	} else if (args.size() == 1) {

		if (strcmp(args.front().c_str(), "ports") == 0 ) {
			CommModule::PrintInfo(true);

		} else {
			if (CommModule::isReady() == true) {
				rtp::packet pck;
				std::string msg = "LINK TEST PAYLOAD";

				pck.header_link = RTP_HEADER(rtp::port::LINK, 1, true, false);
				pck.payload_size = msg.length();
				strcpy((char*)pck.payload, msg.c_str());
				pck.address = BASE_STATION_ADDR;

				if (strcmp(args.front().c_str(), "test-tx") == 0 ) {
					printf("Placing %u byte packet in TX buffer.\r\n", pck.payload_size);
					CommModule::send(pck);

				} else if (strcmp(args.front().c_str(), "test-rx") == 0 ) {
					printf("Placing %u byte packet in RX buffer.\r\n", pck.payload_size);
					CommModule::receive(pck);

				} else {
					showInvalidArgs(args.front());

				}
			} else {
				printf("The radio interface is not ready.\r\n");
			}
		}
	} else if (args.size() == 2) {
		// Default to showing all port info if no specific port number is given for the 'show' option
		if (strcmp(args.front().c_str(), "ports") == 0) {
			if (strcmp(args.at(1).c_str(), "show") == 0) {
				CommModule::PrintInfo(true);
			}
		}
	} else if (args.size() == 3) {

		if (strcmp(args.front().c_str(), "ports") == 0) {

			if ( isInt(args.at(2).c_str()) ) {
				unsigned int portNbr = atoi(args.at(2).c_str());

				if (strcmp(args.at(1).c_str(), "open") == 0) {
					CommModule::openSocket(portNbr);

				} else if (strcmp(args.at(1).c_str(), "close") == 0) {
					CommModule::Close(portNbr);
					printf("Port %u closed.\r\n", portNbr);

				} else if (strcmp(args.at(1).c_str(), "show") == 0) {
					// Change to show only the requested port's info
					CommModule::PrintInfo(true);

				} else if (strcmp(args.at(1).c_str(), "reset") == 0) {
					CommModule::ResetCount(portNbr);
					printf("Reset packet counts for port %u.\r\n", portNbr);

				} else {
					showInvalidArgs(args);
				}
			} else {
				showInvalidArgs(args.at(2));
			}
		} else {
			showInvalidArgs(args.at(2));
		}
	} else {
		showInvalidArgs(args);
	}
}

/**
 * Command executor.
 *
 * Much of this taken from `console.c` from the old robot firmware (2011).
 */
void executeLine(char* rawCommand)
{
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

		if (cmdName.empty() == false) {
			bool commandFound = false;

			for (uint8_t cmdInd = 0; cmdInd < commands.size(); cmdInd++) {
				//check for name match
				if (find(commands[cmdInd].aliases.begin(),
				         commands[cmdInd].aliases.end(),
				         cmdName) != commands[cmdInd].aliases.end()) {
					commandFound = true;

					//If the command is desiged to be run every
					//iteration of the loop, set the handler and
					//args and flag the loop to execute on each
					//iteration.
					if (commands[cmdInd].isIterative) {
						itCmdState = false;

						//Sets the current arg count, args, and
						//command function in fields to be used
						//in the iterative call.
						iterativeCommandArgs = args;
						iterativeCommandHandler =
						    commands[cmdInd].handler;

						itCmdState = true;
					}
					//If the command is not iterative, execute it
					//once immeidately.
					else {
						commands[cmdInd].handler(args);
					}

					break;
				}
			}
			//if the command wasnt found, print an error
			if (!commandFound) {
				std::size_t pos = COMMAND_NOT_FOUND_MSG.find("%s");

				if (pos == std::string::npos) {	// no format specifier found in our defined message
					printf("%s\r\n", COMMAND_NOT_FOUND_MSG.c_str());
				} else {
					std::string not_found_cmd = COMMAND_NOT_FOUND_MSG;
					not_found_cmd.replace(pos, 2, cmdName);
					printf("%s\r\n", not_found_cmd.c_str());
				}
			}
		}

		cmds = strtok_r(nullptr, ";", &endCmd);
		Console::Flush();	// make sure we force everything out of stdout
	}
}


/**
 * Executes iterative commands, and is nonblocking regardless
 * of if an iterative command is not running or not.
 */
void executeIterativeCommand()
{
	if (itCmdState == true) {
		if (Console::IterCmdBreakReq() == true) {
			itCmdState = false;

			// reset the flag for receiving a break character in the Console class
			Console::IterCmdBreakReq(false);
		} else {
			iterativeCommandHandler(iterativeCommandArgs);
		}
	}
}


void showInvalidArgs(const vector<string>& args)
{
	printf("Invalid arguments");

	if (args.empty() == false) {
		printf(" ");

		for (unsigned int i = 0; i < args.size() - 1; i++)
			printf("'%s', ", args.at(i).c_str());

		printf("'%s'.", args.back().c_str());
	} else {
		printf(". No arguments given.");
	}

	printf("\r\n");
}


void showInvalidArgs(const string & s)
{
	printf("Invalid argument '%s'.\r\n", s.c_str());
}

