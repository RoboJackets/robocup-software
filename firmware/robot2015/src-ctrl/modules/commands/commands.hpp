#pragma once

#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <algorithm>
#include <memory>
#include <functional>

#include "robot-devices.hpp"
#include "motors.hpp"

/// Forward declaration for the console & communication tasks.
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
 * Typedef for the kind of function pointer used for the command_t struct
 */
typedef std::function<std::ostream&(
    std::ostream&, const std::vector<std::string>&)> command_fp_t;

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
    command_fp_t handler;

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
std::ostream& executeLine(std::ostream&, char*);
std::ostream& executeIterativeCommand(std::ostream&);
std::ostream& showInvalidArgs(std::ostream&, const std::vector<std::string>&);
std::ostream& showInvalidArgs(std::ostream&, const std::string&);

/*
 * Command definitions. Some command functions have circular definitions.
 *
 * Alphabetical order please.
 */
std::ostream& cmd_alias(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_clear(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_echo(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_exitSys(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_help(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_help_detail(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_ping(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_ls(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_info(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_resetMbed(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_disconnectInterface(std::ostream&,
                                      const std::vector<std::string>&);
std::ostream& cmd_checkInterfaceConn(std::ostream&,
                                     const std::vector<std::string>&);
std::ostream& cmd_baudrate(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_switchHostname(std::ostream&,
                                 const std::vector<std::string>&);
std::ostream& cmd_switchUser(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_logLevel(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_rpc(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_radio(std::ostream& s, const std::vector<std::string>&);
std::ostream& cmd_ps(std::ostream&, const std::vector<std::string>&);
