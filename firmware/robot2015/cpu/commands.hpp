#if not defined(COMMANDS_HEADER_FILE)
#define COMMANDS_HEADER_FILE

#include <string>
#include <array>

/**
 * max number of command aliases
 */
const uint8_t MAX_ALIASES = 4;

/**
 *
 */
const uint8_t MAX_COMMAND_ARGS = 16;

/**
 * command structure
 */
typedef struct
{
	/**
	 * aliases
 	 */
	const std::array<std::string, MAX_ALIASES> aliases;

	/**
	 * iterative flag. Should the command be executed iteratively (in the
	 * main loop) until the break signal is sent?
	 */
	const bool isIterative;

	/**
	 * command handler function pointer
 	 */
	void (*handler)(uint8_t argc, std::array<std::string, MAX_COMMAND_ARGS> argv);

	/**
	 * command description. Used by help
	 */
	const std::string description;

	/**
	 * usage description. Used by help
	 */
	const std::string usage;
} command_t;

/*
 * command functions
 */
void executeCommand(char* rawCommand);
bool isExecutingIterativeCommand(void);
void cancelIterativeCommand(void);
void executeIterativeCommand(void);

/*
 * some command function that have circular definitions. Alphabetical order
 * please.
 */
void cmd_alias(uint8_t argc, std::array<std::string, MAX_COMMAND_ARGS>);
void cmd_clear(uint8_t argc, std::array<std::string, MAX_COMMAND_ARGS>);
void cmd_echo(uint8_t argc, std::array<std::string, MAX_COMMAND_ARGS>);
void cmd_exitSys(uint8_t argc, std::array<std::string, MAX_COMMAND_ARGS>);
void cmd_help(uint8_t argc, std::array<std::string, MAX_COMMAND_ARGS>);
void cmd_ping(uint8_t argc, std::array<std::string, MAX_COMMAND_ARGS>);
void cmd_resetMbed(uint8_t argc, std::array<std::string, MAX_COMMAND_ARGS>);

#endif

