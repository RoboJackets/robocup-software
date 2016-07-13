#pragma once

#include <string>
#include <array>
#include <vector>
#include <algorithm>
#include <memory>

#include "robot-devices.hpp"
#include "motors.hpp"
#include "SharedSPI.hpp"

// forward declaration of tasks
void Task_SerialConsole(void const* args);
void InitializeCommModule(std::shared_ptr<SharedSPI> sharedSPI);

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
    const bool is_iterative;

    /**
     * command handler function pointer
     */
    int (*handler)(cmd_args_t& args);

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
void show_invalid_args(cmd_args_t&);
void show_invalid_args(const std::string&);

/*
 * Command definitions. Some command functions have circular definitions.
 *
 * Alphabetical order please.
 */
int cmd_alias(cmd_args_t&);
int cmd_baudrate(cmd_args_t&);
int cmd_console_clear(cmd_args_t&);
int cmd_console_echo(cmd_args_t&);
int cmd_console_exit(cmd_args_t&);
int cmd_console_hostname(cmd_args_t&);
int cmd_console_user(cmd_args_t&);
int cmd_help(cmd_args_t&);
int cmd_help_detail(cmd_args_t&);
int cmd_serial_ping(cmd_args_t&);
int cmd_info(cmd_args_t&);
int cmd_interface_check_conn(cmd_args_t&);
int cmd_interface_disconnect(cmd_args_t&);
int cmd_interface_reset(cmd_args_t&);
int cmd_led(cmd_args_t&);
int cmd_log_level(cmd_args_t&);
int cmd_ls(cmd_args_t&);
int cmd_ping(cmd_args_t&);
int cmd_ps(cmd_args_t&);
int cmd_heapfill(cmd_args_t& args);
int cmd_radio(cmd_args_t&);
int cmd_ping(cmd_args_t&);
int cmd_pong(cmd_args_t&);
int cmd_rpc(cmd_args_t&);
int cmd_imu(cmd_args_t&);
