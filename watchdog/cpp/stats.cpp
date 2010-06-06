#include <string.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <linux/sockios.h>
#include <linux/if.h>
#include <boost/format.hpp>

#include <list>
#include <map>

#include "process.h"
#include "stats.h"
#include "log.h"

using namespace std;
using namespace boost;

Interface_Map interface_status;

////////

Interface_Status::Interface_Status()
{
    rx_bytes = tx_bytes = 0;
    rx_rate = tx_rate = 0;
    updated = false;
    last_time.tv_sec = 0;
    last_time.tv_usec = 0;
}

////////

// Reads a line from the file and appends it to str.
// Returns the number of characters read.
//
// The EOL character is not added to str.
size_t read_line(FILE *fp, string &str)
{
    size_t n = 0;
    while (!feof(fp))
    {
        int ch = fgetc(fp);
        if (ch == EOF || ch == '\r' || ch == '\n')
        {
            break;
        }
        
        str.push_back(ch);
    }
    
    return n;
}

#define WHITESPACE      " \t\r\n"

// Splits a string by whitespace.
// Consecutive whitespaces are ignored, so empty fields are undetectable.
//
// tokens is not cleared.
void split_string(const string &str, vector<string> &tokens, string::size_type start = 0)
{
    while (1)
    {
        // Find the beginning and end of the next token
        start = str.find_first_not_of(WHITESPACE, start);
        if (start == string::npos)
        {
            // No more non-whitespace characters, so no more tokens
            break;
        }
        string::size_type end = str.find_first_of(WHITESPACE, start);
        
        // Copy it to the vector
        tokens.push_back(str.substr(start, end - start));
        
        // Move to the end of this token
        start = end;
    }
}

void get_process_stats(list<Process_Status> &stats)
{
    // Read all process entries from /proc
    DIR *dir = opendir("/proc");
    if (!dir)
    {
        return;
    }
    
    while (struct dirent *ent = readdir(dir))
    {
        const char *name = ent->d_name;
        for (; isdigit(*name); name++)
            ;
        
        if (*name || !(ent->d_type & DT_DIR))
        {
            // Skip any name that is not entirely digits or not a directory
            continue;
        }
        
        pid_t pid = atoi(ent->d_name);
    
        // Read the data from /proc/<pid>/stat
        string filename = str(format("/proc/%d/stat") % pid);
        FILE *fp = fopen(filename.c_str(), "rt");
        if (!fp)
        {
            log_printf("update_process_stats: Can't read stat for pid %d\n", pid);
            continue;
        }
        
        string line;
        read_line(fp, line);
        
        fclose(fp);
        
        vector<string> tokens;
        split_string(line, tokens, line.find(')'));
        
        if (tokens.size() < 21)
        {
            log_printf("update_process_stats: Invalid process stat line in %s\n", filename.c_str());
            continue;
        }
        
        Process_Status stat;
        
        stat.pid = pid;
        stat.parent_pid = atoi(tokens[2].c_str());
        stat.group_id = atoi(tokens[3].c_str());
        
        // Ticks spent in user code
        stat.cpu_time = strtoul(tokens[12].c_str(), 0, 10);
        
        // Ticks spent in system code
        stat.cpu_time += strtoul(tokens[13].c_str(), 0, 10);
        
        stat.memory = strtoul(tokens[22].c_str(), 0, 10);
        
        stats.push_back(stat);
    }
    
    closedir(dir);
}

void update_network_stats()
{
    FILE *fp = fopen("/proc/net/dev", "rt");
    if (!fp)
    {
        interface_status.clear();
        return;
    }
    
    // Clear all updated flags
    for (Interface_Map::iterator i = interface_status.begin(); i != interface_status.end(); ++i)
    {
        (*i).second.updated = false;
    }
    
    // This socket is used to get interface flags with ioctl().
    int s = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    
    while (!feof(fp))
    {
        string line;
        read_line(fp, line);
        
        string::size_type colon = line.find(':');
        if (colon == string::npos)
        {
            // Header line
            continue;
        }
        
        string::size_type start = line.find_first_not_of(WHITESPACE);
        string name = line.substr(start, colon - start);
        
        if (name.compare(0, 3, "eth"))
        {
            // Not an ethernet interface
            continue;
        }
        
        // Skip if the interface is not up
        struct ifreq ifreq;
        ifreq.ifr_flags = IFF_UP;   // Fail safe/silent
        strncpy(ifreq.ifr_name, name.c_str(), IFNAMSIZ);
        if (s != -1 && ioctl(s, SIOCGIFFLAGS, &ifreq) == 0 && !(ifreq.ifr_flags & IFF_UP))
        {
            continue;
        }
        
        vector<string> tokens;
        split_string(line, tokens, colon + 1);
        
        // Get RX/TX bytes transferred
        unsigned long rx_bytes = strtoul(tokens[0].c_str(), 0, 10);
        unsigned long tx_bytes = strtoul(tokens[8].c_str(), 0, 10);
        
        // Get the current time
        struct timeval cur_time;
        gettimeofday(&cur_time, 0);
        double cur_secs = cur_time.tv_sec + (double)cur_time.tv_usec / 1000000;
        
        // Get or create an entry for this interface
        Interface_Status &stat = interface_status[name];
        
        // Calculate transfer rates if the interface has been seen before
        double last_secs = stat.last_time.tv_sec + (double)stat.last_time.tv_usec / 1000000;
        double dtime = cur_secs - last_secs;
        if (dtime && (stat.last_time.tv_sec || stat.last_time.tv_usec))
        {
            stat.rx_rate = (rx_bytes - stat.rx_bytes) / dtime;
            stat.tx_rate = (tx_bytes - stat.tx_bytes) / dtime;
        } else {
            stat.rx_rate = 0;
            stat.tx_rate = 0;
        }
        
        stat.rx_bytes = rx_bytes;
        stat.tx_bytes = tx_bytes;
        
        // Update last_time
        stat.last_time = cur_time;
        
        stat.updated = true;
    }
    
    close(s);
    fclose(fp);

    // Remove all interfaces that were not updated, as this means they were not
    // in /proc/net/dev and thus don't exist anymore.
    for (Interface_Map::iterator i = interface_status.begin(); i != interface_status.end();)
    {
        const Interface_Status &stat = (*i).second;
        
        // Get the next iterator here, before the item is erased.
        Interface_Map::iterator next = i;
        ++next;
        
        if (!stat.updated)
        {
            interface_status.erase(i);
        }
        
        i = next;
    }
}
