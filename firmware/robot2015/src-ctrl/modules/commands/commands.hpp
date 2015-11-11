#pragma once

#include <string>
#include <array>
#include <vector>
#include <algorithm>

#include "robot-devices.hpp"
#include "motors.hpp"


void Task_CommCtrl(void const*);
void comm_cmdProcess(const std::vector<std::string>&);


// forward declaration for the console's task function
void Task_SerialConsole(void const* args);

/**
 * Max number of command aliases.
 */
static const uint8_t MAX_ALIASES = 5;


/**
 * max command args safety check. Args are now vector based upon creation, so
 * this can be changed in size safely.
 */
static const uint8_t MAX_COMMAND_ARGS = 16;

/**
 * command structure
 */
struct command_t {
	// command_t() = default;

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
	void (*handler)(const std::vector<std::string>& args);

	/**
	 * command description. Used by help
	 */
	const std::string description;

	/**
	 * usage description. Used by help
	 */
	const std::string usage;
};

/*
 * Command functions.
 */
void executeLine(char*);
void executeIterativeCommand();
void showInvalidArgs(const std::vector<std::string>&);
void showInvalidArgs(const std::string&);

/*
 * Command definitions. Some command functions have circular definitions.
 *
 * Alphabetical order please.
 */
void cmd_alias(const std::vector<std::string>&);
void cmd_clear(const std::vector<std::string>&);
void cmd_echo(const std::vector<std::string>&);
void cmd_exitSys(const std::vector<std::string>&);
void cmd_help(const std::vector<std::string>&);
void cmd_help_detail(const std::vector<std::string>&);
void cmd_ping(const std::vector<std::string>&);
void cmd_ls(const std::vector<std::string>&);
void cmd_info(const std::vector<std::string>&);
void cmd_resetMbed(const std::vector<std::string>&);
void cmd_disconnectInterface(const std::vector<std::string>&);
void cmd_checkInterfaceConn(const std::vector<std::string>&);
void cmd_baudrate(const std::vector<std::string>&);
void cmd_switchHostname(const std::vector<std::string>&);
void cmd_switchUser(const std::vector<std::string>&);
void cmd_logLevel(const std::vector<std::string>&);
void cmd_rpc(const std::vector<std::string>&);

