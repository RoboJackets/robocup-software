//FIXME - Killing a CGI that has not flushed seems to get the socket stuck.  Why?

#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <utmp.h>
#include <paths.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <boost/format.hpp>

extern "C"
{
#include <statgrab.h>
}

#include <stdexcept>

#include "comm.h"
#include "process.h"
#include "errno_error.h"
#include "log.h"
#include "watchdog.h"
#include "config.h"
#include "stats.h"

using namespace std;
using namespace boost;

pthread_t Connection::thread;
int Connection::server_socket;

const char *connection_closed::what() const throw()
{
    return "Connection closed";
}

/*
Commands:

add <name>
rename <name> <new_name>
remove <name>
cwd <name> <dir>
command <name> <command>
disable <name>
enable <name>
delay <name> <delay>
max_restarts <name> <num>
auto_display <name> <bool>
display <name> [<text>]

start <name> [<host for which to find DISPLAY>]
signal <name> <signal>
attach <name> <pid>

status
machine_status
reload
shutdown
restart
*/
Connection::CommandInfo Connection::command_table[] =
{
    {"status",      0,  false,  &Connection::status},
    {"machine_status", 1,  false, &Connection::machine_status},
    {"reload",      1,  false,  &Connection::reload},
    {"shutdown",    1,  false,  &Connection::shutdown},
    {"restart",     1,  false,  &Connection::restart},
    
    // Configuration
    {"add",         2,  false,  &Connection::add},
    {"rename",      3,  true,   &Connection::rename},
    {"remove",      2,  true,   &Connection::remove},
    {"cwd",         3,  true,   &Connection::cwd},
    {"command",     0,  true,   &Connection::command},
    {"disable",     2,  true,   &Connection::disable},
    {"enable",      2,  true,   &Connection::enable},
    {"delay",       3,  true,   &Connection::delay},
    {"max_restarts", 3, true,   &Connection::max_restarts},
    {"auto_display", 3, true,   &Connection::auto_display},
    {"display",     0,  true,   &Connection::display},
    
    // Operation
    {"start",       0,  true,   &Connection::start},
    {"signal",      3,  true,   &Connection::signal},
    {"attach",      3,  true,   &Connection::attach},
    
    // End-of-table marker - do not change
    {0,             0,  false,  0}
};

Connection::Connection(int socket, const struct sockaddr_in &addr)
{
    this->conn_socket = socket;
    this->remote_addr = addr;
}

void Connection::run()
{
    // True while the process list mutex is held.
    // This lets the exception handler know when to unlock the mutex.
    bool locked = false;
    
//    in_addr_t a = ntohl(remote_addr.sin_addr.s_addr);
//    log_printf("Received connection from %d.%d.%d.%d:%d\n", a >> 24, (a >> 16) & 255, (a >> 8) & 255, a & 255, ntohs(remote_addr.sin_port));
    
    try
    {
        // Identify ourselves
        write("Watchdog v2.0\n");
        
        while (1)
        {
            // Read a command
            CommandLine command;
            if (!read_command(command))
            {
                write("Command cancelled\n");
                continue;
            }
            
            int words = command.size();
            
            if (words == 0)
            {
                // Blank line
                continue;
            }
            
#if 0
            printf("%d words:\n", words);
            for (Command::const_iterator i = command.begin(); i != command.end(); ++i)
            {
                printf("    \"%s\"\n", (*i).c_str());
            }
#endif
            
            // Find a handler for this command
            for (int i = 0; command_table[i].command; i++)
            {
                // Check the command name
                if (command[0] == command_table[i].command)
                {
                    // Check the number of words
                    if (command_table[i].words > 0 && words != command_table[i].words)
                    {
                        write("Wrong number of parameters\n");
                        goto next_command;
                    }
                    
                    Process::lock();
                    locked = true;
                    
                    // Check the process name if required by the table
                    Process *process = 0;
                    if (command_table[i].check_process)
                    {
                        // If a process name is given, try to find it.
                        //
                        // This test handles the case where the table
                        // requires zero words but check_process is true.
                        if (words > 1)
                        {
                            process = Process::find(command[1]);
                        }
                        
                        if (!process)
                        {
                            Process::unlock();
                            locked = false;
                            
                            write("No such process\n");
                            goto next_command;
                        }
                    }
                    
                    // Call the handler
                    if (command_table[i].handler)
                    {
                        if ((this->*command_table[i].handler)(command, process))
                        {
                            write("OK\n");
                        }
                    } else {
                        write("Unimplemented command\n");
                    }
                    
                    Process::unlock();
                    locked = false;

                    goto next_command;
                }
            }
            write("Unrecognized command\n");

next_command: ;
        }
    } catch (connection_closed)
    {
    } catch (exception &ex)
    {
        log_printf("Connection::run() exception: %s\n", ex.what());
    }

    // Unlock the process list in case an exception was thrown while it was locked.
    if (locked)
    {
        Process::unlock();
    }

//    log_printf("Closed connection from %d.%d.%d.%d:%d\n", a >> 24, (a >> 16) & 255, (a >> 8) & 255, a & 255, ntohs(remote_addr.sin_port));
    close(conn_socket);
}

bool Connection::add(CommandLine &command, Process *process)
{
    // Make sure there is not already a process with this name
    if (Process::find(command[1]))
    {
        write("A process with that name already exists\n");
        return false;
    }

    // Create the new process object
    process = new Process();
    process->name = command[1];
    process->add();
    
    log_printf("Added process \"%s\"\n", process->name.c_str());
    
    return true;
}

bool Connection::rename(CommandLine &command, Process *process)
{
    log_printf("Renamed process \"%s\" to \"%s\"\n", process->name.c_str(), command[2].c_str());
    process->name = command[2];
    
    return true;
}

bool Connection::remove(CommandLine &command, Process *process)
{
    if (process->running)
    {
        log_printf("Warning: Removing a running process.  Process will not be terminated.\n");
    }
    
    delete process;
    
    return true;
}

bool Connection::cwd(CommandLine &command, Process *process)
{
    process->cwd = command[2];
    log_printf("Set working directory of process \"%s\" to \"%s\"\n", process->name.c_str(), process->cwd.c_str());
    
    return true;
}

bool Connection::command(CommandLine &command, Process *process)
{
    // Remove the command and process name, leaving only the process command line
    CommandLine::iterator start = command.begin();
    CommandLine::iterator end = start + 2;
    command.erase(start, end);
    
    // Change the command line for the process
    process->command = command;

    //FIXME - Log just the executable or everything?
    log_printf("Changed command for \"%s\"\n", process->name.c_str());
    log_printf("    argc == %d:\n", command.size());
    for (CommandLine::const_iterator i = command.begin(); i != command.end(); ++i)
    {
        log_printf("        \"%s\"\n", (*i).c_str());
    }
    
    return true;
}

bool Connection::disable(CommandLine &command, Process *process)
{
    process->enabled = false;
    log_printf("Disabled process \"%s\"\n", process->name.c_str());
    
    return true;
}

bool Connection::enable(CommandLine &command, Process *process)
{
    // Reset the restart counter so the process will stay enabled until max_restarts is hit again.
    process->restarts = 0;
    process->enabled = true;
    log_printf("Enabled process \"%s\"\n", process->name.c_str());
    
    return true;
}

bool Connection::delay(CommandLine &command, Process *process)
{
    char *end = 0;
    int delay = strtol(command[2].c_str(), &end, 10);
    if (!end || *end)
    {
        // strtol didn't stop at the end of the string - bad number
        write("Delay must be given as an integer in milliseconds\n");
        return false;
    }
    
    process->delay = delay;
    log_printf("Set delay of process \"%s\" to %dms\n", process->name.c_str(), delay);
    
    return true;
}

bool Connection::max_restarts(CommandLine &command, Process *process)
{
    char *end = 0;
    int max_restarts = strtol(command[2].c_str(), &end, 10);
    if (!end || *end)
    {
        // strtol didn't stop at the end of the string - bad number
        write("Second parameter must be an integer\n");
        return false;
    }
    
    process->max_restarts = max_restarts;
    log_printf("Set maximum restarts of process \"%s\" to %d\n", process->name.c_str(), max_restarts);
    
    return true;
}

bool Connection::auto_display(CommandLine &command, Process *process)
{
    char *end = 0;
    int flag = strtol(command[2].c_str(), &end, 10);
    if (!end || *end || (flag != 0 && flag != 1))
    {
        // strtol didn't stop at the end of the string - bad number
        write("Second parameter must be 0 or 1\n");
        return false;
    }
    
    process->auto_display = flag;
    log_printf("Set auto_display of process \"%s\" to %d\n", process->name.c_str(), flag);
    
    return true;
}

bool Connection::display(CommandLine &command, Process *process)
{
    if (command.size() == 3)
    {
        process->display = command[2];
    } else if (command.size() == 2)
    {
        process->display.clear();
    } else {
        write("Optional second parameter must be DISPLAY text\n");
        return false;
    }
    
    log_printf("Set DISPLAY of process \"%s\" to \"%s\"\n", process->name.c_str(), process->display.c_str());
    
    return true;
}

bool Connection::start(CommandLine &command, Process *process)
{
    if (command.size() != 2 && command.size() != 3)
    {
        writef("Specify process name and optional display address\n");
        return false;
    }

    if (process->running)
    {
        write("Process is already running\n");
        return false;
    }
    
    if (process->auto_display)
    {
        // If a hostname was given after the process name, use that instead of
        // the address the request came from.
        //
        // This is needed to support CGI scripts which run on the web server but
        // act on behalf of the web server's client.
        uint8_t addr[4];
        if (command.size() == 3)
        {
            struct hostent *ent = gethostbyname(command[2].c_str());
            if (ent)
            {
                memcpy(addr, ent->h_addr, 4);
            } else {
                process->display = Process::default_display;
                goto no_host;
            }
        } else {
            memcpy(addr, &remote_addr.sin_addr.s_addr, 4);
        }
        
        set_display(process, addr);
no_host:;
    }
    
    log_printf("Starting process \"%s\"\n", process->name.c_str());
    process->start();
    
    return true;
}

void Connection::set_display(Process *process, const uint8_t *addr)
{
    if (!addr)
    {
        // Can't find a DISPLAY without an address
        process->display = Process::default_display;
        
        return;
    }
    
    // Get status of all running processes
    list<Process_Status> stats;
    get_process_stats(stats);
    
    // Map from parent to list of children of that parent
    map<int, list<const Process_Status *> > process_map;
    for (list<Process_Status>::const_iterator i = stats.begin(); i != stats.end(); ++i)
    {
        const Process_Status &ps = *i;
        
        process_map[ps.parent_pid].push_back(&ps);
    }
    
    // 1) Find a login from the address the command came from.
    // 2) Find the shell process.
    // 3) Set the given process's DISPLAY to DISPLAY from the shell's environment.
    
    //WARNING - This is threadsafe only because the process list mutex is held here,
    //  so this function will never be called reentrantly.
    utmpname(_PATH_UTMP);
    struct utmp *ut;
    while ((ut = getutent()))
    {
        if (ut->ut_type == USER_PROCESS)
        {
            struct hostent *ent = gethostbyname(ut->ut_host);
            if (ent)
            {
#if 0
                const uint8_t *a = (uint8_t *)ent->h_addr;
                const uint8_t *b = addr;
                log_printf("DISPLAY: found %d.%d.%d.%d  req %d.%d.%d.%d\n",
                    a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]);
#endif

                if (!memcmp(ent->h_addr, addr, 4))
                {
//                    log_printf("DISPLAY: login on %s pid %d\n", ut->ut_line, ut->ut_pid);
                    
                    if (set_display_search(process, ut->ut_pid, process_map))
                    {
                        // Found a DISPLAY
                        break;
                    }
                }
            }
        }
    }
    
    if (!ut)
    {
        // No login process found, so use the default.
        process->display = Process::default_display;
    }
}

bool Connection::set_display_search(Process *process, pid_t pid, map<int, list<const Process_Status *> > &process_map)
{
    // If process (pid) has DISPLAY in its environment, use that.
    string filename = str(format("/proc/%d/environ") % pid);
    FILE *fp = fopen(filename.c_str(), "rb");
    if (fp)
    {
        while (!feof(fp))
        {
            string name, value;
            int ch;
            
            // Read a "name=value" pair (zero terminated) from the environment file
            while ((ch = fgetc(fp)) != '=' && ch != 0 && ch != EOF)
            {
                name.push_back(ch);
            }
            
            if (ch != '=')
            {
                continue;
            }
                
            while ((ch = fgetc(fp)) != '=' && ch != 0 && ch != EOF)
            {
                value.push_back(ch);
            }
            
            if (name == "DISPLAY")
            {
//                log_printf("DISPLAY \"%s\" from pid %d\n", value.c_str(), pid);
                process->display = value;
                fclose(fp);
                
                return true;
            }
        }
        
        fclose(fp);
    }

    // Check children
    const list<const Process_Status *> &children = process_map[pid];
    
    for (list<const Process_Status *>::const_iterator i = children.begin(); i != children.end(); ++i)
    {
        if (set_display_search(process, (*i)->pid, process_map))
        {
            return true;
        }
    }
    
    // Didn't find DISPLAY in this process or any of its descendents
    return false;
}

bool Connection::signal(CommandLine &command, Process *process)
{
    if (!process->running || process->pid < 1)
    {
        write("Process is not running\n");
        return false;
    }
    
    char *end = 0;
    int sig = strtol(command[2].c_str(), &end, 10);
    if (!end || *end)
    {
        // strtol didn't stop at the end of the string - bad number
        write("Second parameter must be a signal number\n");
        return false;
    }
    
    log_printf("Sending signal %d to process \"%s\"\n", sig, process->name.c_str());
    
    try
    {
        process->signal(sig);
    } catch (exception &ex)
    {
        writef("Failed: %s\n", ex.what());
        log_printf("    Failed: %s\n", ex.what());
    }
    
    return true;
}

bool Connection::attach(CommandLine &command, Process *process)
{
    if (process->running)
    {
        writef("Process is already running as pid %d\n", process->pid);
        return false;
    }
    
    char *end = 0;
    pid_t pid = strtol(command[2].c_str(), &end, 10);
    if (!end || *end)
    {
        // strtol didn't stop at the end of the string - bad number
        write("Second parameter must be a pid\n");
        return false;
    }
    
    log_printf("Attaching pid %d to process \"%s\"\n", pid, process->name.c_str());
    
    try
    {
        process->attach(pid);
    } catch (exception &ex)
    {
        writef("Attach failed: %s\n", ex.what());
        return false;
    }
    
    return true;
}

bool Connection::status(CommandLine &command, Process *process)
{
    // Update CPU and memory usage for all processes
    Process::update_stats();

    write("Status:\n");
    int words = command.size();
    if (words == 1)
    {
        // Show all processes
        for (list<Process *>::const_iterator i = Process::process_list.begin(); i != Process::process_list.end(); ++i)
        {
            write_status(*i);
        }
    } else {
        // Show only specified processes
        for (int i = 1; i < words; i++)
        {
            Process *process = Process::find(command[i]);
            
            if (process)
            {
                write_status(process);
            }
            
            // Processes that are specified but not found are silently ignored.
        }
    }
    
    return true;
}

void Connection::write_status(Process *process)
{
    write("name ");
    write_escaped(process->name);
    
    write("\ncwd ");
    write_escaped(process->cwd);
    
    write("\ncommand");
    for (CommandLine::const_iterator arg = process->command.begin(); arg != process->command.end(); ++arg)
    {
        write(" ");
        write_escaped(*arg);
    }
    
    writef("\nenabled %d\n", process->enabled ? 1:0);
    writef("auto_display %d\n", process->auto_display ? 1:0);
    
    write("display ");
    write_escaped(process->display);
    
    writef("\ndelay %d\n", process->delay);
    writef("restarts %d\n", process->restarts);
    writef("max_restarts %d\n", process->max_restarts);
    writef("running %d\n", process->running ? 1:0);
    writef("cpu %.1f\n", process->cpu);
    writef("memory %.1f\n", process->memory);
    writef("pid %d\n", process->pid);
    writef("end\n");
}

bool Connection::machine_status(CommandLine &command, Process *process)
{
    writef("Machine Status:\n");
    
    // Core temperatures
    for (int core = 0; core < 2; core++)
    {
        char filename[64];
        snprintf(filename, sizeof(filename), "/sys/devices/platform/coretemp.%d/temp1_input", core);
        
        int fd = open(filename, O_RDONLY);
        if (fd < 0)
        {
            // Skip this one if it can't be opened
            continue;
        }
        
        char buf[32];
        int len = read(fd, buf, sizeof(buf));
        
        close(fd);
        
        if (len < 0)
        {
            // Skip this one if it can't be read
            continue;
        }
        
        buf[len] = 0;
        
        // Send the temperature
        writef("temp Core_%d %.1f\n", core, (float)atoi(buf) / 1000);
    }
#if 0
    writef("temp Core_%d %.1f\n", 2, 123.0);
    writef("temp Core_%d %.1f\n", 3, 13.0);
    writef("temp Core_%d %.1f\n", 4, 63.0);
#endif
    
    // Network usage
    update_network_stats();
    for (Interface_Map::const_iterator i = interface_status.begin(); i != interface_status.end(); ++i)
    {
        const string &name = (*i).first;
        const Interface_Status &stat = (*i).second;
        
        writef("net %s %lu %lu %f %f\n", name.c_str(), stat.rx_bytes, stat.tx_bytes, stat.rx_rate, stat.tx_rate);
    }
    
    // Total CPU usage
    sg_cpu_percents *cpu = sg_get_cpu_percents();
    //FIXME - statgrab reports this scaled to 100%, regardless of number of cores
    writef("cpu_usage %.1f\n", 2 * (100 - cpu->idle));
    
    // Total memory usage
    sg_mem_stats *mem = sg_get_mem_stats();
    writef("mem_usage %.1f\n", (double)(mem->used - mem->cache) * 100 / (double)mem->total);
    
    return true;
}

bool Connection::shutdown(CommandLine &command, Process *process)
{
    write("OK\n");
    
    log_printf("--- Watchdog pid %d exiting due to shutdown command ---\n", getpid());
    exit(0);
    
    return false;
}

//_syscall2(int, tkill, int, tid, int, sig)

bool Connection::restart(CommandLine &command, Process *process)
{
    write("OK\n");
    
    // Kill the monitoring thread so attached processes will get detached.
    if (pthread_cancel(Process::monitor_thread))
    {
        log_printf("cancel: %m\n");
    }
    
    if (pthread_join(Process::monitor_thread, 0))
    {
        log_printf("join: %m\n");
    }
    
    if (fork())
    {
        // Original watchdog (parent)
        log_printf("--- Watchdog restarting: old pid %d exiting ---\n", getpid());
        exit(0);
    } else {
        // New watchdog (child)
        
        log_printf("--- Watchdog restarting: new pid %d ---\n", getpid());
        
        // Copy argv since there is no guarantee that there is a NULL as execv requires
        char **argv = new char *[original_argc + 1];
        for (int i = 0; i < original_argc; i++)
        {
            argv[i] = original_argv[i];
        }
        argv[original_argc] = 0;
        
        execv(argv[0], argv);
        
        log_printf("Connection::restart() execv failed: %m\n");
    }
    
    return false;
}

bool Connection::reload(CommandLine &command, Process *process)
{
    config_load(config_file);
    
    return true;
}

void Connection::write(const char *str)
{
    int len = strlen(str);
    int ret = ::write(conn_socket, str, len);
    
    if (ret < 0)
    {
        throw errno_error();
    } else if (ret != len)
    {
        throw runtime_error("Partial write");
    }
}

void Connection::write_escaped(const string &str)
{
    char esc = '\\';
    
    for (string::const_iterator i = str.begin(); i != str.end(); ++i)
    {
        char ch = *i;
        
        if (ch == ' ' || ch < 32 || ch > 126)
        {
            // This character needs to be escaped
            ::write(conn_socket, &esc, 1);
        }
        
        ::write(conn_socket, &ch, 1);
    }
}

void Connection::writef(const char *fmt, ...)
{
    va_list args;
    
    va_start(args, fmt);
    vdprintf(conn_socket, fmt, args);
    va_end(args);
}

char Connection::read_char()
{
    char ch = 0;
    
    int len = recv(conn_socket, &ch, sizeof(ch), 0);
    if (len < 0)
    {
        throw errno_error();
    } else if (len == 0)
    {
        throw connection_closed();
    }
    
    return ch;
}

bool Connection::read_command(CommandLine &command)
{
    string *str = 0;
    int num = 0;
    
    while (1)
    {
        // Read a character
        char ch = read_char();
        
        num++;
        if (num > 1000)
        {
            // Unreasonably long line
            return false;
        }
        
        if (ch == '\r' || ch == '\n')
        {
            // End of line
            break;
        } else if (ch < 32 || ch > 126)
        {
            // Cancel
            return false;
        } else if (ch == ' ')
        {
            // End of string
            str = 0;
            
            // Don't use this character
            continue;
        } else if (!str)
        {
            // Beginning of a new string
            command.push_back("");
            str = &command.back();
        }
        
        if (ch == '\\')
        {
            // Escape character: Always use the following character so
            // spaces and other special characters can be used in a word.
            ch = read_char();
        }
        
        // Add this character to the current string
        str->push_back(ch);
    }
    
    return true;
}

bool Connection::init()
{
    struct sockaddr_in sin;
    
    server_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_socket < 0)
    {
        log_printf("Connection::server() socket failed: %m\n");
        return false;
    }
    
    // Make the socket get closed on exec
    fcntl(server_socket, F_SETFD, FD_CLOEXEC);
    
    int value = 1;
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &value, sizeof(value)))
    {
        close(server_socket);
        server_socket = -1;
        
        log_printf("Connection::server() setsockopt failed: %m\n");
        return false;
    }
    
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = INADDR_ANY;
    sin.sin_port = htons(5005);
    if (bind(server_socket, (struct sockaddr *)&sin, sizeof(sin)))
    {
        close(server_socket);
        server_socket = -1;
        
        log_printf("Connection::server() bind failed: %m\n");
        return false;
    }
    
    return true;
}

void *Connection::thread_main(void *param)
{
    // Block SIGPIPE to prevent this process from dying if a remote host closes connections unexpectedly.
    struct sigaction act;
    act.sa_handler = SIG_IGN;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    sigaction(SIGPIPE, &act, 0);
    
    while (!init())
    {
        // Wait for the port to become available
        usleep(500 * 1000);
    }
    
    listen(server_socket, 3);
    while (1)
    {
        struct sockaddr_in sin;
        
        // Get a new connection
        socklen_t len = sizeof(sin);
        int conn_socket = accept(server_socket, (struct sockaddr *)&sin, &len);
        if (conn_socket < 0)
        {
            log_printf("Connection::server() accept failed: %m\n");
            
            // Don't kill the machine if this is a persistent error
            usleep(500 * 1000);
            continue;
        }
        
        // Sanity check
        if (len != sizeof(sin))
        {
            log_printf("Connection::server() accept returned weird address size %d (should be %d)\n", len, sizeof(sin));
            log_printf("    Closing connection\n");
            close(conn_socket);
            
            continue;
        }
        
        // Make the socket get closed on exec
        fcntl(conn_socket, F_SETFD, FD_CLOEXEC);
        
        // Handle the connection
        Connection comm(conn_socket, sin);
        // This could be run as a separate thread if necessary
        comm.run();
    }
    
    return 0;
}

void Connection::start_server()
{
    pthread_create(&thread, NULL, thread_main, NULL);
}
