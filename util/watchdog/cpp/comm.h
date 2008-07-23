// Communications, for remote control and status reporting

#ifndef _COMM_H_
#define _COMM_H_

#include <map>

#include <pthread.h>
#include <netinet/in.h>

#include "process.h"

class Process_Status;

// Exception that is thrown by read_char when the connection is closed
// by the peer.
class connection_closed: public std::exception
{
public:
    const char *what() const throw();
};

// Handler for a TCP connection
class Connection
{
public:
    Connection(int socket, const struct sockaddr_in &addr);

    // Processes commands from the connection
    void run();

    // Listens for new connections and runs a Connection on them
    static void start_server();

protected:
    // Server main loop
    static void *thread_main(void *param);
    
    // Creates and initializes the server socket
    static bool init();
    
    // Server thread
    static pthread_t thread;
    
    // Server socket
    static int server_socket;

    // Socket for this connection
    int conn_socket;
    
    // Writes a string to conn_socket.
    // Throws runtime_error or errno_error on failure.
    void write(const char *str);
    
    void write_escaped(const std::string &str);
    void writef(const char *fmt, ...);
    
    // Reads one character from conn_socket.
    // Throws runtime_error or errno_error on failure.
    char read_char();
    
    // Reads a command as an array of escaped, space-delimited words on a line.
    // Returns true if the command was read correctly or false if CTRL-C was
    // received inside the command.
    bool read_command(CommandLine &command);

    // Writes status information for a process to conn_socket
    void write_status(Process *process);
    
    // Sets DISPLAY for a process based on a remote address.
    // addr is the address of the machine from which the request originated.
    // addr points to the s_addr part of a struct sockaddr_in, or h_addr in struct hostent.
    void set_display(Process *process, const uint8_t *addr);
    
    static bool set_display_search(Process *process, pid_t pid, std::map<int, std::list<const Process_Status *> > &process_map);
    
    // Address of the remote host
    struct sockaddr_in remote_addr;

    ////////
    // Command handlers

    typedef struct
    {
        // First word of the command
        const char *command;
        
        // Number of words in the command.
        //
        // Zero is a special case: It allows any number of words, but if
        // check_process is true, there must be at least two (command and process).
        int words;
        
        // True if the second parameter must be the name of an existing process
        bool check_process;
    
        // Function to be called when the command is received.
        // process points to the process named by the second parameter or 0 if none.
        //
        // This will be called with the process list mutex held.
        bool (Connection::*handler)(CommandLine &command, Process *process);
    } CommandInfo;
    
    static CommandInfo command_table[];

    bool add(CommandLine &command, Process *process);
    bool rename(CommandLine &command, Process *process);
    bool remove(CommandLine &command, Process *process);
    bool cwd(CommandLine &command, Process *process);
    bool command(CommandLine &command, Process *process);
    bool disable(CommandLine &command, Process *process);
    bool enable(CommandLine &command, Process *process);
    bool delay(CommandLine &command, Process *process);
    bool max_restarts(CommandLine &command, Process *process);
    bool auto_display(CommandLine &command, Process *process);
    bool display(CommandLine &command, Process *process);
    bool start(CommandLine &command, Process *process);
    bool signal(CommandLine &command, Process *process);
    bool attach(CommandLine &command, Process *process);
    bool status(CommandLine &command, Process *process);
    bool machine_status(CommandLine &command, Process *process);
    bool reload(CommandLine &command, Process *process);
    bool shutdown(CommandLine &command, Process *process);
    bool restart(CommandLine &command, Process *process);
};

#endif // _COMM_H_
