#pragma once

#include <string>
#include <array>
#include <vector>

/**
 * max number of command aliases
 */
const uint8_t MAX_ALIASES = 4;

/**
 * max command args safety check. Args are now vector based upon creation, so
 * this can be changed in size safely.
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
	void (*handler)(const std::vector<std::string> &args);

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
 * Command definitions. Some command functions have circular definitions.
 *
 * Alphabetical order please.
 */
void cmd_alias(const std::vector<std::string> &args);
void cmd_clear(const std::vector<std::string> &args);
void cmd_echo(const std::vector<std::string> &args);
void cmd_exitSys(const std::vector<std::string> &args);
void cmd_help(const std::vector<std::string> &args);
void cmd_ping(const std::vector<std::string> &args);
void cmd_ls(const std::vector<std::string> &args);
void cmd_info(const std::vector<std::string> &args);
void cmd_resetMbed(const std::vector<std::string> &args);

