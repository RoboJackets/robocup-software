#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <sys/ptrace.h>
#include <sys/stat.h>

#include <boost/format.hpp>

extern "C"
{
#include <statgrab.h>
}

#include "process.h"
#include "log.h"
#include "errno_error.h"
#include "stats.h"

// Syscalls for thread ID stuff (used for attaching)
#include <linux/unistd.h>
#ifndef _syscall0
    // Use glibc's interface
    #include <sys/syscall.h>
    
    pid_t gettid(void)
    {
        return syscall(SYS_gettid);
    }
    
    int tgkill(int tgid, int tid, int sig)
    {
        return syscall(SYS_tgkill, tgid, tid, sig);
    }
#else
    // Use the kernel syscall macros.
    _syscall0(pid_t,gettid)
    _syscall3(int, tgkill, int, tgid, int, tid, int, sig)
#endif

using namespace std;
using namespace boost;

string Process::default_display;
pid_t Process::monitor_tid;
pthread_t Process::monitor_thread;
unsigned long Process::hz;
unsigned long Process::mem_total_kb;
unsigned long Process::page_size_kb;

pthread_mutex_t Process::attach_mutex = PTHREAD_MUTEX_INITIALIZER;
list<pid_t> Process::attach_list;

pthread_mutex_t Process::process_list_mutex = PTHREAD_MUTEX_INITIALIZER;
list<Process *> Process::process_list;

Process::Process()
{
    enabled = false;
    delay = 500;
    max_restarts = 0;
    auto_display = false;
    
    pid = 0;
    running = false;
    restarts = 0;
    cpu = 0;
    memory = 0;
    last_cpu_time = 0;
    memset(&last_check_time, 0, sizeof(last_check_time));
    attached = false;
}

Process::~Process()
{
    remove();
}

Process *Process::find(pid_t pid)
{
    for (list<Process *>::iterator i = process_list.begin(); i != process_list.end(); ++i)
    {
        Process *process = *i;
        if (process->pid == pid)
        {
            return process;
        }
    }
    
    return 0;
}

Process *Process::find(const std::string &name)
{
    for (list<Process *>::iterator i = process_list.begin(); i != process_list.end(); ++i)
    {
        Process *process = *i;
        if (process->name == name)
        {
            return process;
        }
    }
    
    return 0;
}

void Process::add()
{
    process_list.push_back(this);
}

void Process::remove()
{
    process_list.remove(this);
}

void Process::replace_with(const Process *other)
{
    name = other->name;
    cwd = other->cwd;
    command = other->command;
    enabled = other->enabled;
    delay = other->delay;
    max_restarts = other->max_restarts;
}

void Process::start()
{
    last_cpu_time = 0;
    gettimeofday(&last_check_time, 0);

    // Generate pointers to the command line arguments
    int argc = command.size();
    char **argv = new char*[argc + 1];
    if (!argv)
    {
        log_printf("Process::start() is out of memory for command line arguments\n");
        exit(0);
    }
        
    for (int i = 0; i < argc; i++)
    {
        // The cast is OK: we're not going to change it and it won't last long anyway.
        argv[i] = (char *)command[i].c_str();
    }
    argv[argc] = 0;
 
    // Create a child process
    pid_t fork_pid = fork();
    if (fork_pid < 0)
    {
        // Error
        log_printf("Process::start() fork failed: %m\n");
    } else if (fork_pid == 0)
    {
        // Child
        //
        // Don't return from this function after this point,
        // since this process is not supposed to act as a watchdog.
        
        // Open a new log file
        log_open(name.c_str());
        
        // Kill other threads
        //
        // This is required for old versions of glibc.
        // In newer versions either it is a no-op or not present.
#ifdef pthread_kill_other_threads_np
        pthread_kill_other_threads_np();
#endif
        
        // Change to the proper directory
        if (!cwd.empty())
        {
            int ret = chdir(cwd.c_str());
            if (ret)
            {
                log_printf("Process::start() chdir failed: %m\n");
                log_printf("    Executing anyway...\n");
            }
        }
        
        // Start a new process group.  This causes signals sent to this process
        // to be sent to its children, and makes it easier to tally resource usage for children.
        setpgrp();

        // Set the DISPLAY environment variable
        if (!display.empty())
        {
            setenv("DISPLAY", display.c_str(), 1);
        } else {
            unsetenv("DISPLAY");
        }
       
        // Make a symlink to the log file.
        // 
        // This used to be:
        //  const char *recent_log_name = str(format stuff...).c_str();
        // which is wrong because the std::string was temporary and was deleted after that
        // line, so the pointer returned by c_str() went bad.  valgrind caught this.
        string recent_log_name = str(format("/tmp/watchdog/latest/%s-latest.log") % name);
        unlink(recent_log_name.c_str());
        if (symlink(log_filename.c_str(), recent_log_name.c_str()))
        {
            log_printf("Can't make symlink %s: %s\n", recent_log_name.c_str(), strerror(errno));
        }

        // Redirect stdout and stderr to the log file and close the original handle to it
        dup2(fileno(log_real), 1);
        dup2(fileno(log_real), 2);
        log_close();
        
        // Run the executable
        execvp(argv[0], argv);
        
        // This will go to the log file, but without a timestamp since the file cookie was closed.
        log_printf("Process::start() exec failed: %m\n");
        exit(0);
    }
    
    // Parent
    pid = fork_pid;
    running = true;
    log_printf("Started \"%s\" (pid %d)\n", name.c_str(), pid);
    delete[] argv;
}

void Process::signal(int sig)
{
    // Send the signal to the process group, which has the same ID as the process used to start it with fork().
    // This will send the signal to the process and all its children.
    int ret = kill(-pid, sig);
    if (ret < 0 && errno == ESRCH)
    {
        // If that didn't work, we must have attached to a process that was not a process group leader,
        // so the best we can do (without searching the entire process tree) is to send the signal
        // to just the one process.
        ret = kill(pid, sig);
    }
    
    if (ret)
    {
        throw errno_error();
    }
}

// You can't just reparent a process.  The only way to become a process's parent
// on command is with ptrace().
//
// This is gross: ptrace operations only work when called by the *thread*, not *process*,
// which originally attached to the process being traced.  This means the comms thread
// can't attach to a process by itself, as the attached process can't be resumed by
// the monitoring thread.
//
// To get around this, attach() (called by the comms thread) sends a signal to
// the monitoring thread.  The signal handler calls ptrace(PTRACE_ATTACH, ...),
// and the monitoring loop resumes the process when wait() finds it.

void Process::attach(pid_t pid)
{
    running = false;
    attached = true;
    this->pid = pid;
    
    pthread_mutex_lock(&attach_mutex);
    attach_list.push_back(pid);
    pthread_mutex_unlock(&attach_mutex);
    
    // Send SIGUSR1 to the monitoring thread so it will ptrace() the process.
    //
    // getpid() actually returns the thread group ID of the current thread.
    tgkill(getpid(), monitor_tid, SIGUSR1);
}

void Process::sigusr1(int sig)
{
    pthread_mutex_lock(&attach_mutex);
    
    for (list<pid_t>::const_iterator i = attach_list.begin(); i != attach_list.end(); ++i)
    {
        pid_t pid = *i;
        if (ptrace(PTRACE_ATTACH, pid, 0, 0))
        {
            log_printf("PTRACE_ATTACH for pid %d: %m\n", pid);
        }
    }
    
    attach_list.clear();
    
    pthread_mutex_unlock(&attach_mutex);
}

void Process::monitor_loop()
{
    monitor_tid = gettid();
    monitor_thread = pthread_self();
    
    struct sigaction act;
    act.sa_handler = sigusr1;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    sigaction(SIGUSR1, &act, 0);
    siginterrupt(SIGUSR1, 1);

    hz = sysconf(_SC_CLK_TCK);
    
    // Page size
    //FIXME - Probably should find a way to get this at runtime
    page_size_kb = 4;
    
    // Get total memory from /proc/meminfo
    sg_mem_stats *mem = sg_get_mem_stats();
    mem_total_kb = mem->total / 1024;

    // Find processes that are already running, and attach to them
    find_existing();

    while (1)
    {
        int status = 0;
        pid_t pid = wait(&status);
        
        if (pid < 0)
        {
            //FIXME - Wait for a condition to indicate that processes are available?
            if (errno != ECHILD)
            {
                log_printf("process thread: wait failed: %m\n");
            }
            
            usleep(1000 * 1000);
            
            continue;
        }
        
        // Find the process that died/stopped
        lock();
        
        Process *proc = find(pid);
        if (!proc)
        {
            log_printf("Unknown pid %d died or attached, ignoring\n", pid);
            unlock();
            
            continue;
        }
        
        // Check for an attached process that stopped due to PTRACE_ATTACH or a signal.
        if (WIFSTOPPED(status))
        {
            int signal = WSTOPSIG(status);
            
            if (proc->attached && !proc->running)
            {
                log_printf("Resuming attached process \"%s\" (pid %d)\n", proc->name.c_str(), pid);
            
                // We're done attaching
                proc->running = true;
            } else {
                log_printf("Attached process \"%s\" (pid %d) received signal %d\n", proc->name.c_str(), pid, signal);
            }
            
            // Don't forward SIGSTOP because it doesn't work anyway, and will show up twice here.
            // I consider this mysterious but irrelevant.
            if (signal == SIGSTOP)
            {
                signal = 0;
            }
            
            // Continue the process.
            //
            // Note that SIGSTOP doesn't work anymore - we can't send SIGSTOP with PTRACE_CONT,
            // and raising it will just bring us back here.  Not continuing the process will
            // prevent SIGCONT from being received.  I don't know how to fix this.
            //
            // BTW, it doesn't work under strace either.
            if (ptrace(PTRACE_CONT, pid, 0, signal))
            {
                log_printf("    PTRACE_CONT failed: %d %m\n", errno);
            }
            
            unlock();
            continue;
        }
        
        // Indicate that the process is no longer running
        proc->running = false;
        
        log_printf("Process \"%s\" (pid %d) exited ", proc->name.c_str(), pid);
        if (WIFEXITED(status))
        {
            log_printf("with status %d\n", WEXITSTATUS(status));
        } else if (WIFSIGNALED(status))
        {
            int sig = WTERMSIG(status);
            log_printf("due to signal %d (%s)\n", sig, strsignal(sig));
        }
        
        // Check for maximum restarts
        if (proc->enabled && proc->max_restarts > 0 && proc->restarts >= proc->max_restarts)
        {
            log_printf("Process \"%s\" has restarted %d times, disabling\n", proc->name.c_str(), proc->restarts);
            proc->enabled = false;
        }
        proc->restarts++;
        
        // Automatically restart if enabled
        if (proc->enabled)
        {
            // Wait before restarting
            unsigned int delay = proc->delay;
            if (delay)
            {
                unlock();
                
                // Wait for the restart delay
                usleep(delay * 1000);
                
                // Find the process again in case it was removed
                lock();
                proc = find(pid);
            }
            
            // Check these in case the process was changed during the restart delay
            // so that it doesn't need to be started anymore.
            if (proc && !proc->running && proc->enabled)
            {
                // Restart the process
                proc->start();
            }
        }
        
        unlock();
    }
}

void Process::find_existing()
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
        
        // Read the data from /proc/<pid>/cmdline
        char filename[64];
        snprintf(filename, sizeof(filename), "/proc/%d/cmdline", pid);
        
        int fd = open(filename, O_RDONLY);
        if (fd < 0)
        {
            continue;
        }
        
        // Find a process with this command line
        for (list<Process *>::const_iterator p = process_list.begin(); p != process_list.end(); ++p)
        {
            Process *process = *p;
            
            lseek(fd, 0, SEEK_SET);
            
            // Compare each argument
            for (unsigned int arg = 0; arg < process->command.size(); ++arg)
            {
                char ch;
                
                for (unsigned int i = 0; i < process->command[arg].size(); ++i)
                {
                    if (read(fd, &ch, 1) != 1)
                    {
                        goto next_process;
                    }
                    
                    if (ch != process->command[arg][i])
                    {
                        goto next_process;
                    }
                }
                
                if (read(fd, &ch, 1) != 1 || ch != 0)
                {
                    goto next_process;
                }
            }
            
            // Complete match
            log_printf("Found existing proc %s: pid %d:\n", process->name.c_str(), pid);
            if (!process->running)
            {
                process->attach(pid);
            }

next_process: ;
        }
        
        close(fd);
    }

    closedir(dir);
}

void Process::update_stats()
{
    list<Process_Status> stat_list;
    get_process_stats(stat_list);
    
    // Find information for each process tracked by the watchdog
    for (list<Process *>::const_iterator i = Process::process_list.begin(); i != Process::process_list.end(); ++i)
    {
        Process *process = *i;
        
        // Get the real time since the last check
        struct timeval cur_time;
        gettimeofday(&cur_time, 0);
        
        if (process->pid == 0)
        {
            // Process has never run: no stats
            
            process->last_cpu_time = 0;
            process->last_check_time = cur_time;
            process->cpu = 0;
            process->memory = 0;
            
            continue;
        }
        
        unsigned long cpu_time = 0;
        unsigned long memory = 0;
        
//        printf("Status of %s (pid %d):\n", process->name.c_str(), process->pid);
        
        // Add up the usage of all processes in this process' group,
        // which will include the original process and all descendents.
        for (list<Process_Status>::const_iterator j = stat_list.begin(); j != stat_list.end(); ++j)
        {
            const Process_Status &info = *j;
            
            if (info.group_id == process->pid)
            {
//                printf("    Add %lu ticks and %lu kb\n", info.cpu_time, info.memory);
                cpu_time += info.cpu_time;
                memory += info.memory;
            }
        }
        
        // Calculate CPU usage as a fraction of one CPU
        double used_time;
        //FIXME - This handles the case where a child has died so the total CPU time has gone down.
        //  This makes the CPU usage wrong until the next check, as long as no more children die.
        //  The correct way to fix this is to keep track of all child processes individually, but I'm
        //  not sure it's worth the trouble.
        if (cpu_time > process->last_cpu_time)
        {
            used_time = (double)(cpu_time - process->last_cpu_time) / Process::hz;
        } else {
            used_time = 0;
        }
        
        double cur_secs = cur_time.tv_sec + (double)cur_time.tv_usec / 1000000;
        double last_secs = process->last_check_time.tv_sec + (double)process->last_check_time.tv_usec / 1000000;
        double run_time = cur_secs - last_secs;
        
//        printf("    Used %.2fs in %.2fs\n", used_time, run_time);
        
        process->last_cpu_time = cpu_time;
        process->last_check_time = cur_time;
        
        if (run_time > 0)
        {
            process->cpu = (double)used_time * 100 / run_time;
        } else {
            process->cpu = 0;
        }
        
        // Calculate memory usage
        if (Process::mem_total_kb > 0)
        {
            process->memory = (double)memory * Process::page_size_kb * 100 / Process::mem_total_kb;
        } else {
            process->memory = 0;
        }
    }
}
