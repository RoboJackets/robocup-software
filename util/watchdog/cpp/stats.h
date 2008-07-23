#ifndef _STATS_H_
#define _STATS_H_

#include <map>

// Information about one process obtained from /proc/*/stat
class Process_Status
{
public:
    Process_Status()
    {
        group_id = 0;
        cpu_time = 0;
        memory = 0;
    }
    
    // PID of this process
    pid_t pid;
    
    // Parent process's PID
    pid_t parent_pid;
    
    // Process group id
    pid_t group_id;
    
    // User time + system time
    unsigned long cpu_time;
    
    // Resident set size in pages
    unsigned long memory;
};

// Information about one network interface
class Interface_Status
{
public:
    Interface_Status();

    // Time this structure was last updated
    struct timeval last_time;
    
    // Received bytes
    unsigned long rx_bytes;
    
    // Transmitted bytes
    unsigned long tx_bytes;
    
    // Receive data rate in bytes/s
    float rx_rate;
    
    // Transmit data rate in bytes/s
    float tx_rate;
    
    // Used internally by update_network_stats.
    // Indicates that the interface was updated.
    // This will always be true after update_network_stats finishes.
    bool updated;
};

typedef std::map<std::string, Interface_Status> Interface_Map;
extern Interface_Map interface_status;

void get_process_stats(std::list<Process_Status> &stats);

void update_network_stats(void);

#endif // _STATS_H_
