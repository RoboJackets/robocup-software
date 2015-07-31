#include "commands.hpp"

/**
 * error message when a typed command isn't found
 */
const string COMMAND_NOT_FOUND_MSG = "Command not found. Type 'help' for a list of commands.";


/**
 * error message when too many args are provided
 */
const string TOO_MANY_ARGS_MSG = "*** too many arguments ***";


/**
 * indicates if the command held in "iterativeCommand"
 */
volatile bool executingIterativeCommand = false;


/**
 * current iterative command args
 */
vector<string> iterativeCommandArgs;


/**
 * the current iterative command handler
 */
void (*iterativeCommandHandler)(const vector<string> &args);


// Create an object to help find files
LocalFileSystem local("local");


/**
 * Commands list. Add command handlers to commands.hpp.
 *
 * Alphabetical order please (here addition and in handler function declaration).
 */
static const vector<command_t> commands = {
	/* COMMAND TEMPALATE
	{
		{"<alias>", "<alias2>", "<alias...>"},
		is the command iterative,
		command handler function,
		"description",
		"usage"}, */
	{
		{"alias"},
		false,
		cmd_alias,
		"Lists aliases for commands.",
		"alias"
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
		"help | h | ? (<--list> | <command names>)"
	},
	{
		{"ping"},
		true,
		cmd_ping,
		"Check console responsiveness.",
		"ping"
	},
	{
		{"ls"},
		false,
		cmd_ls,
		"List contents of current directory",//\r\n",  Bugs:\t\tsometimes displays train animations.",
		"ls [folder/device]"
	},
	{
		{"info", "version", "s"},
		false,
		cmd_info,
		"Display information about the current version of the firmware.",
		"info | version | s"
	},
	{
		{"reset", "reboot", "restart"},
		false,
		cmd_resetMbed,
		"Resets the mbed (like pushing the reset button).",
		"reset | reboot | restart"
	},
	{
		{"rmdev", "unconnect"},
		false,
		cmd_disconnectInterface,
		"Disconnects the mbed interface chip from the microcontroller.",
		"rmdev | unconnect"
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
		cmd_setBaudrate,
		"Set the serial link's baudrate.",
		"baud [-r <rate>]"
	},
	{
		{"su", "user"},
		false,
		cmd_switchUser,
		"Set active user.",
		"su | user"
	},
	{
		{"host", "hostname"},
		false,
		cmd_switchHostname,
		"Set the system hostname.",
		"host | hostname"
	},
	{
		{"loglvl", "loglevel"},
		false,
		cmd_logLevel,
		"Change the active logging output level.",
		"loglvl | loglevel"
	}
};


/**
* Lists aliases for commands, if args are present, it will only list aliases
* for those commands.
*/

void cmd_alias(const vector<string> &args)
{
//if no args given, list all aliases
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

			if (aliasFound) {
				printf("\r\n");
			} else {
				printf("Error listing aliases: command \"%s\" not found\r\n",
				       args[argInd].c_str());
			}
		}
	}

	Console::Flush();
}


/**
* Clears the console.
*/
void cmd_clear(const vector<string> &args)
{
	if (args.empty() == false) {
		printf("Invalid arguments!\r\n");
		Console::Flush();
		return;
	}

	Console::Flush();
	printf(ENABLE_SCROLL_SEQ.c_str());
	printf(CLEAR_SCREEN_SEQ.c_str());
	Console::Flush();
}


/**
 * Echos text.
 */
void cmd_echo(const vector<string> &args)
{
	if (args.empty() == true) {
		printf("Invalid arguments!\r\n");
		Console::Flush();
		return;
	}


	for (uint8_t argInd = 0; argInd < args.size(); argInd++)
		printf("%s ", args[argInd].c_str());

	printf("\r\n");
	Console::Flush();
}


/**
 * Requests a system stop. (breaks main loop, or whatever implementation this
 * links to).
 */
void cmd_exitSys(const vector<string> &args)
{
	if (args.empty() == false) {
		printf("Unknown argument \"%s\".\r\n", args.at(0).c_str());
		Console::Flush();
		return;
	}

	Console::RequestSystemStop();
}


/**
 * Prints command help.
 */
void cmd_help(const vector<string> &args)
{
	// printf("\r\nCtrl + C stops iterative commands\r\n\r\n");
	// Console::Flush();

	// Prints all commands, with details
	if (args.size() == 0) {
		for (uint8_t i = 0; i < commands.size(); i++) {
			printf("\t%s:\t", commands[i].aliases[0].c_str());
			// Console::Flush();
			printf("%s\r\n", commands[i].description.c_str());

			// Console::Flush();
		}
		Console::Flush();
	}
	//prints all commands
	else if (args.size() == 1 && (strcmp(args[0].c_str(), "--list") == 0 || strcmp(args[0].c_str(), "-l") == 0)) {
		for (uint8_t i = 0; i < commands.size(); i++) {
			if (i % 4 == 3) {
				printf("%s\r\n", commands[i].aliases[0].c_str());
			} else if (i == commands.size() - 1) {
				printf("%s", commands[i].aliases[0].c_str());
			} else {
				printf("%s,\t", commands[i].aliases[0].c_str());
			}
		}

		printf("\r\n");
		Console::Flush();
	}
	//prints arguments with details
	else {
		//iterate through args
		for (uint8_t argInd = 0; argInd < args.size(); argInd++) {
			//iterate through commands
			bool commandFound = false;

			for (uint8_t i = 0; i < commands.size(); i++) {
				//check match against args
				if (find(commands[i].aliases.begin(),
				         commands[i].aliases.end(),
				         args[argInd]) != commands[i].aliases.end()) {
					commandFound = true;

					//print info
					printf("%s:\r\n",
					       commands[i].aliases[0].c_str());
					Console::Flush();
					printf("  Description:\t%s\r\n",
					       commands[i].description.c_str());
					Console::Flush();
					printf("  Usage:\t%s\r\n",
					       commands[i].usage.c_str());
					Console::Flush();
					printf("  Iterative:\t%s\r\n\r\n",
					       commands[i].isIterative ? "YES" : "NO");
					Console::Flush();
				}
			}

			//if the command wasn't found, notify
			if (!commandFound) {
				printf("Command \"%s\" not found.\r\n", args[argInd].c_str());
				Console::Flush();
			}
		}
	}
}


/**
 * Console responsiveness test.
 */
void cmd_ping(const vector<string> &args)
{
	if (args.empty() == false) {
		printf("Unknown argument \"%s\".\r\n", args.at(0).c_str());
		Console::Flush();
		return;
	}

	time_t sys_time = time(NULL);
	printf("reply: %d\r\n", sys_time);
	Console::Flush();
	Thread::wait(1000);
}


/**
 * Resets the mbed (should be the equivalent of pressing the reset button).
 */
void cmd_resetMbed(const vector<string> &args)
{
	if (args.empty() == false) {
		printf("Unknown argument \"%s\".\r\n", args.at(0).c_str());
		Console::Flush();
		return;
	}

	Console::Flush();
	printf("rebooting...\r\n");
	mbed_interface_reset();
	Console::Flush();
}


/**
 * Lists files.
 */
void cmd_ls(const vector<string> &args)
{
	DIR *d;
	struct dirent *p;

	if (args.empty() == false) {
		printf("Unknown argument \"%s\".\r\n", args.at(0).c_str());
		Console::Flush();
		return;
	}

	if (args.empty() == true) {
		d = opendir("/local");
	} else {
		d = opendir(args[0].c_str());
	}

	if (d != NULL) {
		while ((p = readdir(d)) != NULL) {
			printf(" - %s\r\n", p->d_name);

		}

		closedir(d);
	} else {
		printf("Could not open directory!\r\n");
	}

	Console::Flush();
}


/**
 * Prints system info.
 */
void cmd_info(const vector<string> &args)
{
	if (args.empty() == false) {
		printf("Invalid arguments!\r\n");
		Console::Flush();
		return;
	}

	DS2411_t id;

	printf("Commit Hash:\t%s\r\nCommit Date:\t%s\r\nCommit Author:\t%s\r\n",
	       git_version_hash,
	       git_head_date,
	       git_head_author
	      );

	printf("Build Date:\t%s %s\r\n", __DATE__, __TIME__);

	printf("Base ID:\t");
	if (ds2411_read_id(RJ_BASE_ID, &id, true) == ID_HANDSHAKE_FAIL)
		printf("N/A\r\n");
	else
		for (int i = 0; i < 6; i++)
			printf("%02X\r\n", id.serial[i]);

	// Prints out a serial number, taken from the mbed forms
	// https://developer.mbed.org/forum/helloworld/topic/2048/
	unsigned int Interface[5] = {58, 0, 0, 0, 0};
	typedef void (*CallMe)(unsigned int[], unsigned int[]);
	CallMe CallMe_entry = (CallMe)0x1FFF1FF1;
	CallMe_entry(Interface, Interface);

	if (!Interface[0])
		printf("MCU UID:\t%u %u %u %u\r\n",
		       Interface[1],
		       Interface[2],
		       Interface[3],
		       Interface[4]
		      );
	else
		printf("MCU UID:\t\tN/A\r\n");

	// Should be 0x26013F37
	Interface[0] = 54;
	CallMe_entry(Interface, Interface);

	if (!Interface[0])
		printf("MCU ID:\t\t%u\r\n", Interface[1]);
	else
		printf("MCU ID:\t\tN/A\r\n");

	char buf[33];
	mbed_interface_uid(buf);
	printf("mbed UID:\t%s\r\n", buf);

	// mbed_mac_address(buf);
	printf("Eth MAC:\t");
	for (int i = 0; i < 5; i++)
		printf("%02X-", buf[i]);
	printf("%02X\r\n", buf[5]);

	Console::Flush();
}


/**
 * [cmd_disconnectMbed description]
 * @param args [description]
 */
void cmd_disconnectInterface(const vector<string> &args)
{
	if (args.empty() == false) {
		printf("Unknown argument \"%s\".\r\n", args.at(0).c_str());
		Console::Flush();
		return;
	}

	Console::Flush();
	mbed_interface_disconnect();
}


void cmd_checkInterfaceConn(const vector<string> &args)
{
	if (args.empty() == false) {
		printf("Unknown argument \"%s\".\r\n", args.at(0).c_str());
		Console::Flush();
		return;
	}

	printf("mbed interface connected: %s\r\n", mbed_interface_connected() ? "YES" : "NO");
	Console::Flush();
}


void cmd_setBaudrate(const vector<string> &args)
{
	if (args.size() > 1) {
		printf("Invalid arguments \"%s\".\r\n", args.at(1).c_str());
		Console::Flush();
		return;
	}

	if (args.empty() == true) {
		printf("Not implemented yet!\r\n");
		Console::Flush();
		return;
	}

	std::string str_baud = args.at(0);
	int new_rate = atoi(str_baud.c_str());

	int valid_rates[] = {110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
	std::vector<int> rates (valid_rates, valid_rates + sizeof(valid_rates) / sizeof(int) );

	if (std::find(rates.begin(), rates.end(), new_rate) != rates.end()) {
		Serial pc(USBTX, USBRX);
		pc.baud(new_rate);
		printf("New baudrate: %u\r\n", new_rate);
	} else {
		printf("%u is not a valid baudrate.\r\n", new_rate);
	}

	Console::Flush();
}

void cmd_switchUser(const vector<string> &args)
{
	if (args.empty() == true || args.size() > 1) {
		printf("Invalid arguments!\r\n");
		Console::Flush();
		return;
	}

	Console::CONSOLE_USER = args.at(0);
}

void cmd_switchHostname(const vector<string> &args)
{
	if (args.empty() == true || args.size() > 1) {
		printf("Invalid arguments!\r\n");
		Console::Flush();
		return;
	}

	Console::CONSOLE_HOSTNAME = args.at(0);
}

void cmd_logLevel(const vector<string> &args)
{
	if (args.size() > 1) {
		printf("Invalid arguments \"%s\".\r\n", args.at(1).c_str());
		Console::Flush();
		return;
	}

	if (args.empty()) {
		printf("Log level: %s\r\n", LOG_LEVEL_STRING[rjLogLevel]);
		Console::Flush();
		return;
	}

	if (args.at(0) == "+") {
		rjLogLevel++;
	} else if (args.at(0) == "-") {
		rjLogLevel--;
	}

	printf("New log level: %s\r\n", LOG_LEVEL_STRING[rjLogLevel]);
}

/**
 * Command executor.
 *
 * Much of this taken from `console.c` from the old robot firmware (2011).
 */
void executeCommand(char *rawCommand)
{
	uint8_t argc = 0;
	string cmdName = "\0";
	vector<string> args;
	args.reserve(MAX_COMMAND_ARGS);

	char *pch = strtok(rawCommand, " ");

	while (pch != NULL) {

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
		pch = strtok(NULL, " ");
	}

	if (cmdName.size() > 0) {
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
					executingIterativeCommand = false;

					//Sets the current arg count, args, and
					//command function in fields to be used
					//in the iterative call.
					iterativeCommandArgs = args;
					iterativeCommandHandler =
					    commands[cmdInd].handler;

					executingIterativeCommand = true;
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
			printf("%s\r\n", COMMAND_NOT_FOUND_MSG.c_str());
			Console::Flush();
		}
	}
}


/**
 * Returns if an iterative command is active.
 */
bool isExecutingIterativeCommand(void)
{
	return executingIterativeCommand;
}


/**
 * Executes iterative commands, and is nonblocking regardless
 * of if an iterative command is not running or not.
 */
void executeIterativeCommand(void)
{
	if (executingIterativeCommand)
		iterativeCommandHandler(iterativeCommandArgs);
}


/**
 * Halts iterative command execution. It should be called by
 * the console when a break sequence is sent.
 */
void cancelIterativeCommand(void)
{
	executingIterativeCommand = false;
}
