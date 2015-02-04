#include "robot.hpp"
#include "commands.hpp"
#include "console.hpp"
#include "reset.hpp"

#include <algorithm>

using namespace std;

/**
 * error message when a typed command isn't found
 */
const string COMMAND_NOT_FOUND_MSG = "Command not found.";

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

/**
 * commands list. Add command handlers to commands.hpp.
 *
 * Alphabetical order please (here addition and in handler function declaration)
 */
const vector<command_t> commands =  
{
	// COMMAND TEMPALATE
	// {
	// 	{"<alias>", "<alias2>", "<alias...>"},
	// 	is the command iterative,
	// 	command handler function,
	// 	"description",
	// 	"usage"},
	{
		{"alias"},
		false,
		cmd_alias,
		"lists aliases for commands",
		"alias"},
	{
		{"clear", "cls"},
		false,
		cmd_clear,
		"clears the screen"
		"clear | cls"},
	{
		{"echo"},
		false,
		cmd_echo,
		"echos test"
		"echo <text>"},
	{
		{"exit", "quit"},
		false, 
		cmd_exitSys, 
		"breaks the main loop", 
		"exit | quit"},
	{
		{"help", "h", "?"},
		false, 
		cmd_help, 
		"prints this message", 
		"help | h | ? (<--list> | <command names>)"},
	{
		{"ping"},
		true,
		cmd_ping,
		"check console responsiveness. Ping pong.",
		"ping"},
	{
		{"reset", "reboot"}, 
		false,
		cmd_resetMbed, 
		"resets the mbed (like pushing the reset button)", 
		"reset | reboot"}
};

/**
 * lists aliases for commands, if args are present, it will only list aliases
 * for those commands
 */
void cmd_alias(const vector<string> &args)
{
	//if no args given, list all aliases
	if (args.size() == 0)
	{
		for (uint8_t i = 0; i < commands.size(); i++)
		{
			printf("%s:\r\n", commands[i].aliases[0].c_str());

			//print aliases
			uint8_t a = 0;
			while (a < commands[i].aliases.size()
			       && commands[i].aliases[a] != "\0")
			{
				printf("\t%s", commands[i].aliases[a].c_str());

				//print commas
				if (a < commands[i].aliases.size() - 1
				    && commands[i].aliases[a + 1] != "\0")
				{
					printf(",");
				}

				a++;
			}

			printf("\r\n");	
		}
	}
	else
	{
		bool aliasFound = false;
		for (uint8_t argInd = 0; argInd < args.size(); argInd++)
		{
			for (uint8_t cmdInd = 0; cmdInd < commands.size(); cmdInd++)
			{
				//check match against args
				if (find(commands[cmdInd].aliases.begin(),
					 commands[cmdInd].aliases.end(),
					 args[argInd]) != commands[cmdInd].aliases.end())
				{
					aliasFound = true;

					printf("%s:\r\n", 
						commands[cmdInd].aliases[0].c_str());

					//print aliases
					uint8_t a = 0;
					while (a < commands[cmdInd].aliases.size()
					       && commands[cmdInd].aliases[a] != "\0")
					{
						printf("\t%s", 
						       commands[cmdInd].aliases[a].c_str());

						//print commas
						if (a < commands[cmdInd].aliases.size() - 1
						    && commands[cmdInd].aliases[a + 1] != "\0")
						{
							printf(",");
						}

						a++;
					}	
				}		
			}
			
			if (aliasFound)
			{
				printf("\r\n");
			}
			else
			{
				printf("error listing aliases: command \"%s\" not found\r\n",
				       args[argInd].c_str());
			}
		}		
	}

	flush();
}

/**
 * clears the console
 */ 
void cmd_clear(const vector<string> &args)
{
	printf(CLEAR_SCREEN_SEQ.c_str());
	flush();
}

/**
 * echos text
 */
void cmd_echo(const vector<string> &args)
{
	for (uint8_t argInd = 0; argInd < args.size(); argInd++)
	{
		printf("%s ", args[argInd].c_str());
	}	
	
	printf("\r\n");
	flush();
}

/**
 * requests a system stop. (breaks main loop, or whatever implementation this
 * links to)
 */
void cmd_exitSys(const vector<string> &args)
{
	reqSysStop();
}

/**
 * prints command help
 */
void cmd_help(const vector<string> &args)
{
	printf("\nCtrl + C stops iterative commands\r\n\r\n");
	flush();

	//prints all commands, with details
	if (args.size() == 0)
	{
		for (uint8_t i = 0; i < commands.size(); i++)
		{
			printf("%s:\r\n", 
			       commands[i].aliases[0].c_str());
			flush();
			printf("\tDescription: %s\r\n", 
			       commands[i].description.c_str());
			flush();
			printf("\tUsage: %s\r\n",
			       commands[i].usage.c_str());
			flush();
			printf("\tIterative: %s\r\n\r\n", 
			       commands[i].isIterative ? "true" : "false");
			flush();
		}

		printf("Screen Overflow? Try \"help <command>\"\r\n\r\n");
		flush();
	}
	//prints all commands
	else if (args.size() == 1 && strcmp(args[0].c_str(), "--list") == 0)
	{
		for (uint8_t i = 0; i < commands.size(); i++)
		{
			if (i % 4 == 3)
			{
				printf("%s\r\n", commands[i].aliases[0].c_str());	
			}
			else if (i == commands.size() - 1)
			{
				printf("%s", commands[i].aliases[0].c_str());
			}
			else
			{
				printf("%s,\t", commands[i].aliases[0].c_str());
			}
		}	

		printf("\r\n");
		flush();
	}
	//prints arguments with details
	else
	{
		//iterate through args
		for (uint8_t argInd = 0; argInd < args.size(); argInd++)
		{
			//iterate through commands
			bool commandFound = false;
			for (uint8_t i = 0; i < commands.size(); i++)
			{
				//check match against args
				if (find(commands[i].aliases.begin(),
					 commands[i].aliases.end(),
					 args[argInd]) != commands[i].aliases.end())
				{
					commandFound = true;

					//print info
					printf("%s:\r\n", 
					       commands[i].aliases[0].c_str());
					flush();
					printf("\tDescription: %s\r\n", 
					       commands[i].description.c_str());
					flush();
					printf("\tUsage: %s\r\n",
					       commands[i].usage.c_str());
					flush();
					printf("\tIterative: %s\r\n\r\n", 
					       commands[i].isIterative ? "true" : "false");
					flush();	
				}
			}

			//if the command wasn't found, notify
			if (!commandFound)
			{
				printf("Command \"%s\" not found.\r\n", args[argInd].c_str());
				flush();
			}
		}
	}	
}

/**
 * console responsiveness test
 */
void cmd_ping(const vector<string> &args)
{
	printf("pong.\r\n");
	flush();
}

/**
 * Resets the mbed (should be the equivalent of pressing the reset button)
 */
void cmd_resetMbed(const vector<string> &args)
{
	mbed_reset();
}

/**
 * command executor
 *
 * much of this taken from console.c under old robot firmware
 */
void executeCommand(char* rawCommand)
{
	uint8_t argc = 0;
	string cmdName = "\0";
	vector<string> args;
	args.reserve(MAX_COMMAND_ARGS);
	
	char* pch = strtok(rawCommand, " ");
	while (pch != NULL)
	{		
		//check args length
		if (argc > MAX_COMMAND_ARGS)
		{
			printf("%s\r\n", TOO_MANY_ARGS_MSG.c_str());
			break;
		}

		//set command name
		if (argc == 0)
		{
			cmdName = string(pch);
		}
		//set args
		else
		{
			args.push_back(pch);
		}
		argc++;

		pch = strtok(NULL, " ");
	}

	if (cmdName.size() > 0)
	{		
		bool commandFound = false;
		for (uint8_t cmdInd = 0; cmdInd < commands.size(); cmdInd++)
		{
			//check for name match
			if (find(commands[cmdInd].aliases.begin(),
				 commands[cmdInd].aliases.end(),
				 cmdName) != commands[cmdInd].aliases.end())
			{
				commandFound = true;

				//If the command is desiged to be run every
				//iteration of the loop, set the handler and
				//args and flag the loop to execute on each
				//iteration.
				if (commands[cmdInd].isIterative)
				{
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
				else
				{
					commands[cmdInd].handler(args);
				}

				break;
			}		
		}	

		//if the command wasnt found, print an error
		if (!commandFound)
		{
			printf("%s\r\n", COMMAND_NOT_FOUND_MSG.c_str());
			flush();
		}
	}	
}

/**
 * returns if an iterative command is active
 */
bool isExecutingIterativeCommand(void)
{
	return executingIterativeCommand;
}

/**
 * executes iterative commands, and is nonblocking regardless  
 * of if an iterative command is not running or not.
 */
void executeIterativeCommand(void)
{
	if (executingIterativeCommand)
	{
		iterativeCommandHandler(iterativeCommandArgs);
	}	
}

/**
 * halts iterative command execution. It should be called by
 * the console when a break sequence is sent.
 */
void cancelIterativeCommand(void)
{
	executingIterativeCommand = false;
}

