#pragma once

#include <string>
#include <array>
#include <vector>
#include <algorithm>

#include "robot-devices.hpp"
#include "motors.hpp"

// forward declaration of tasks
void Task_SerialConsole(void const* args);
void Task_CommCtrl(void const*);

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
 * typedef for a command function's argument(s)
 */
typedef const std::vector<std::string> cmd_args_t;

/**
 * command structure
 */
struct command_t {
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
void execute_line(char*);
void execute_iterative_command();
void show_invalid_args(const std::vector<std::string>&);
void show_invalid_args(const std::string&);

/*
 * Command definitions. Some command functions have circular definitions.
 *
 * Alphabetical order please.
 */
void cmd_alias(cmd_args_t&);
void cmd_baudrate(const std::vector<std::string>&);
void cmd_console_clear(const std::vector<std::string>&);
void cmd_console_echo(const std::vector<std::string>&);
void cmd_console_exit(const std::vector<std::string>&);
void cmd_console_hostname(const std::vector<std::string>&);
void cmd_console_user(const std::vector<std::string>&);
void cmd_help(const std::vector<std::string>&);
void cmd_help_detail(const std::vector<std::string>&);
void cmd_info(const std::vector<std::string>&);
void cmd_interface_check_conn(const std::vector<std::string>&);
void cmd_interface_disconnect(const std::vector<std::string>&);
void cmd_interface_reset(const std::vector<std::string>&);
void cmd_led(const std::vector<std::string>& args);
void cmd_log_level(const std::vector<std::string>&);
void cmd_ls(const std::vector<std::string>&);
void cmd_ping(const std::vector<std::string>&);
void cmd_ps(const std::vector<std::string>& args);
void cmd_radio(const std::vector<std::string>&);
void cmd_rpc(const std::vector<std::string>&);
