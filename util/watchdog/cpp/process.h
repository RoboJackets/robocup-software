#ifndef _PROCESS_H_
#define _PROCESS_H_

#include <vector>
#include <list>
#include <string>

// Array of strings used for command lines.
// This is also used for command received on TCP control connections.
typedef std::vector<std::string> CommandLine;

// Configuration and state for a process.
//
// The caller must hold process_list_mutex before calling any members of this class (except lock() or monitor_loop()).
//
class Process
{
public:
    Process();
    ~Process();
    
    // Adds this process to the list
    void add();
    
    // Removes this process from the list
    void remove();
    
    // Replaces the configuration of this process with that of another
    void replace_with(const Process *other);
    
    // Starts the process.
    void start();
    
    // Sends a signal to the process.
    void signal(int sig);
    
    // Attaches this process to the given pid
    void attach(pid_t pid);
    
    static void find_existing();
    
    // Finds the Process with the given pid or name.
    static Process *find(pid_t pid);
    static Process *find(const std::string &name);
    
    // The monitoring loop
    static void monitor_loop();
    
    // Locks the list mutex
    static void lock()
    {
        pthread_mutex_lock(&process_list_mutex);
    }
    
    // Unlocks the list mutex
    static void unlock()
    {
        pthread_mutex_unlock(&process_list_mutex);
    }
    
    // Reads a value from /proc/meminfo.
    // Returns -1 if the value was not found.
    static int get_meminfo(const char *name);
        
    // Updates CPU and memory usage for all processes.
    // Child processes are included in the amounts.
    //
    // The process list must be locked when this function is called.
    static void update_stats();
    
    ////////
    // Configuration
    
    // A short name for the process, used to identify it in status reports.
    std::string name;
    
    // The working directory in which the process should be run.
    std::string cwd;
    
    // If true, the DISPLAY environment variable is determined (and the value of display, below, is changed).
    bool auto_display;
    
    // The DISPLAY environment variable for this process.
    //
    // By default, all processes inherit the watchdog's DISPLAY.
    // The value given in the config file (which may be blank) replaces it.
    // If this process has no <display> tag in the config file, this is left at the default.
    //
    // The value found by Connection::set_display, if any, also replaces it.
    // If Connection::set_display can't find a suitable value, it resets this to the watchdog's DISPLAY.
    //
    // If this is empty, the process does not get a DISPLAY variable.
    // There is no provision for setting the DISPLAY variable to an empty string.
    std::string display;
    
    // The command line for the process.
    // Note that this is not passed to a shell - the first word is the full path to the executable
    // and the remaining words are arguments.
    CommandLine command;
    
    // If true, the process will be restarted when it dies.
    bool enabled;
    
    // Time to wait before restarting this process, in milliseconds.
    unsigned int delay;

    // The maximum number of times the process can be restarted before it is automatically disabled.
    // No limit if <= 0.
    int max_restarts;
    
    ////////
    // Runtime state
    
    // The process ID most recently assigned to the process.
    // This is also the process group ID of the process and all its descendents.
    pid_t pid;
    
    // True if the process is running.
    bool running;
    
    // The number of times the process has been restarted.
    int restarts;
    
    // Percentage of CPU time used.
    // Updated by read_status().
    double cpu;
    
    // Percentage of memory used.
    // Updated by read_status().
    double memory;
    
    // utime + stime at last check
    unsigned long last_cpu_time;
    
    // Time (in ticks) at last check or process start
    struct timeval last_check_time;
    
    // True if the process was attached, rather than started by this watchdog
    bool attached;
    
    ////////
    // Static data
    
    // This mutex protects process_list.
    static pthread_mutex_t process_list_mutex;
    
    // The list of all processes.
    // Protected by process_list_mutex.
    static std::list<Process *> process_list;

    // Total memory in kB.
    // Set by get_mem_info().
    static unsigned long mem_total_kb;
    
    // Size of a page in kB
    // Set by get_mem_info().
    static unsigned long page_size_kb;
    
    // Number of kernel clock ticks per second used by process status information
    // Set by start_thread.
    static unsigned long hz;

    // Monitoring thread's TID
    static pid_t monitor_tid;
    
    static pthread_t monitor_thread;
    
    // Default DISPLAY value when no other value can be found
    static std::string default_display;

private:
    // Handler for SIGUSR1, used to attach to processes.
    //
    // WARNING: This function cannot access process_list, because it can't get the lock because it
    // might deadlock with monitor_loop().
    static void sigusr1(int sig);
    
    // List of processes that need to be attached, and its mutex.
    //
    // WARNING: This mutex must never be locked by the monitoring thread, as it is locked
    // by sigusr1(), which would deadlock.  sigusr1() is called in response to the SIGUSR1 sent
    // by the comms thread to attach to a process.
    static std::list<pid_t> attach_list;
    static pthread_mutex_t attach_mutex;

    Process(Process &other);
    Process &operator=(Process &other);
};

#endif // _PROCESS_H_
