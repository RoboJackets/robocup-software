from watchdog import *

# If x is a list, returns x.
# Otherwise, returns a list containing x.
#
# In all cases, a list is returned.
#
# This is used to allow iterating through tuples in machine_status
# regardless of how many tuples there are.
def make_list(x):
    if type(x) == type([]):
        return x
    else:
        return [x]

def write_host_list(host_status):
    print '<div class="host_list">Show host:'
    print '<a href="status?host=all">All</a></td>'
    for x in hosts:
        if type(host_status[x]) != type(()):
            c = 'fail'
        else:
            c = None
            
            (status, machine_status) = host_status[x]
            
            if 'temp' in machine_status:
                # Warn if any temperature is above 70C
                for (loc, temp) in make_list(machine_status['temp']):
                    temp = float(temp)
                    if temp > 70:
                        c = 'warn'
            
            if 'cpu_usage' in machine_status and float(machine_status['cpu_usage']) >= 50:
                c = 'warn'
        
            if 'mem_usage' in machine_status and float(machine_status['mem_usage']) >= 50:
                c = 'warn'
            
        print make_link(x, 'status', {'host':x}, c)
    print '</div>'

def write_process_status(host, name, props, ret = 'status'):
    print '<td>' + make_link(name, 'edit', {'host':host, 'proc':name, 'ret':ret}) + '</td>'
    
    # Running/stopped
    if props['running'] == '1':
        str = '<td class="running_check"><input type="checkbox" checked onclick="location.href=\''
        str += make_href('stop', {'host':host, 'proc':name, 'ret':ret})
        str += '\'"></td><td class="running_text">Running'
    else:
        str = '<td class="stopped_check"><input type="checkbox" onclick="location.href=\''
        str += make_href('command', {'host':host, 'cmd':'start', 'proc':name, 'arg':remote_addr, 'ret':ret})
        str += '\'"></td><td class="stopped_text">Stopped'
    print str + '</td>'
    
    # Enabled
    str = '<td class="enable"><input type="checkbox"'
    if props['enabled'] == '1':
        str += ' checked'
        command = 'disable'
    else:
        command = 'enable'
    str += ' onclick="location.href=\''
    str += make_href('command', {'host':host, 'cmd':command, 'proc':name, 'ret':ret})
    print str + '\'"/></td>'
    
    # Number of restarts
    print '<td>' + props["restarts"] + '</td>'
    
    # CPU usage
    print '<td'
    cpu = props['cpu']
    fcpu = float(cpu)
    if fcpu >= 50:
        print 'class="high"'
    elif fcpu >= 25:
        print 'class="medium"'
    print '>' + cpu + '%</td>'
    
    # Memory usage
    print '<td'
    memory = props['memory']
    fmemory = float(memory)
    if fmemory >= 50:
        print 'class="high"'
    elif fmemory >= 25:
        print 'class="medium"'
    print '>' + memory + '%</td>'
    
    # PID
    print '<td>'
    if props['pid'] != '0':
        print props['pid']
    print '</td>'
    
    # Commands
    print '<td class="control">' + make_link('Start', 'command', {'host':host, 'cmd':'start', 'proc':name, 'arg':remote_addr, 'ret':ret}) + '</td>'
    print '<td class="control">' + make_link('Stop', 'stop', {'host':host, 'proc':name, 'ret':ret}) + '</td>'
    print '<td class="control">' + make_link('Terminate', 'command', {'host':host, 'cmd':'signal', 'proc':name, 'arg':'15', 'ret':ret}) + '</td>'
    print '<td class="control">' + make_link('Kill', 'command', {'host':host, 'cmd':'signal', 'proc':name, 'arg':'9', 'ret':ret}) + '</td>'
    print '<td class="control"><a href="http://' + host + ':5080/log/latest/' + name + '-latest.log">Log</a></td>'

