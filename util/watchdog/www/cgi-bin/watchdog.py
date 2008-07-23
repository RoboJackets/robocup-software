from config import *

import os
import socket
import sys
import re
import cgi
import cgitb
import Cookie

# Turn on pretty error reporting if running from the web server
if os.environ.has_key('REQUEST_METHOD'):
    cgitb.enable()

# Get the address of the host that caused this request, so it can be
# used for autodisplay.
if 'REMOTE_ADDR' in os.environ:
    remote_addr = os.environ['REMOTE_ADDR']
else:
    remote_addr = ''

########################

# Replaces None with an empty string
def none_to_blank(str):
    if str == None:
        return ''
    else:
        return str

# Prints a standard HTTP response header
def header():
    print 'Content-type: text/html\n'

def escape(str):
    ret = ''
    for ch in str:
        n = ord(ch)
        if n <= 32 or n > 126 or ch == '\\':
            ret += '\\'
        ret += ch
    return ret

re_unescape = re.compile(r'\\(.)', re.DOTALL)
def unescape(str):
    return re_unescape.sub(r'\1', str)

def make_href(url, params = None):
    str = url
    if params != None:
        str += '?'
        first = 1
        for name in params:
            if first == 0:
                str += '&'
            else:
                first = 0
            # FIXME - Escape name and value
            str += name + '=' + params[name]
    return str;

def make_link(text, url, params = None, css_class = None):
    str = '<a'
    if css_class != None:
        str += ' class="' + css_class + '"'
    str += ' href="' + make_href(url, params) + '">' + text + '</a>'
    return str

# Looks for name in GET/POST parameters, then cookies.
# Returns the first value found or default if not found.
def get_param(name, default = ''):
    if name in form:
        return form.getvalue(name)
    elif name in cookies:
        return cookies[name].value
    else:
        return default

def write_refresh_header():
    global refresh
    
    if refresh.isdigit():
        print '<meta http-equiv="refresh" content="' + refresh + '"/>'
    else:
        refresh = 'off'
    
    print '<meta http-equiv="Set-Cookie" content="refresh=' + refresh + '"/>'

def write_refresh_combo(url):
    global refresh
    
    print '<div class="refresh">Refresh: '
    print '<select name="refresh">'
    for x in (('off', 'Off'), ('1', '1s'), ('2', '2s'), ('5', '5s'), ('10', '10s')):
        str = '    <option value="' + x[0] + '"'
        if x[0] == refresh:
            str += ' selected'
        print str + ' onclick="location.href=\'' + url + '?refresh=' + x[0] + '\'">' + x[1] + '</option>'
    print '</select></div>'

# A connection to a watchdog process
class watchdog:
    # Socket connection to the watchdog
    socket = None

    # Connection to the watchdog as a file object
    conn = None

    # Name of the host we are connected to
    host = None

    # Port on host that we are connected to
    port = None

    def __init__(self, host, port = 5005):
        if host == None:
            raise ValueError, 'No host specified'

        # Connect to the watchdog
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(.5);
        self.socket.connect((host, port))
        self.conn = self.socket.makefile()

        # Read the version line
        ver = self.readline()
        if ver != 'Watchdog v2.0':
            raise ValueError, 'Got bad version string from watchdog: ' + ver
        
        self.host = host
        self.port = port

    def close(self):
        if self.socket != None:
            self.socket.close()
            self.socket = None
        
        if self.conn != None:
            self.conn.close()
            self.conn = None
        
        self.host = None
        self.port = None

    # Writes a line
    def write(self, str):
        self.conn.write(str)
        self.conn.flush()

    # Reads one line and removes the trailing \n.
    # Returns an empty string if the connection was closed.
    def readline(self):
        return self.conn.readline().rstrip('\n')

    # Gets the status of all processes and returns a dictionary of dictionaries.
    # All properties are returned unescaped.
    def process_status(self, process = None):
        line = 'status'
        if type(process) == type(''):
            # Single process
            line += ' ' + escape(process)
        elif type(process) == type(()):
            # List of processes
            for p in process:
                line += ' ' + escape(p)
        elif process != None:
            raise TypeError, 'Unknown type for process ' + process

        self.write(line + '\n')
        return self.parse_status()

    # Internal: Parses the result of a status command
    def parse_status(self):
        # First line is always 'Status:'
        line = self.readline()
        if line != 'Status:':
            raise RuntimeError, 'Got bad reply from status: ' + line
        
        status = {}
        props = {}
        while 1:
            # Read and clean a line
            line = self.readline()
            
            # Check for end of data or close connection.
            #
            # Must check for empty string here, as a closed connection
            # would otherwise make this loop run forever.
            if line == 'OK' or line == '':
                # Done
                break
            
            # Check for the end of this process's data
            if line == 'end':
                status[props['name']] = props
                props = {}
            else:
                # Store the data as a key/value pair
                part = line.split(' ', 1)
                if len(part) < 2:
                    props[part[0]] = ''
                else:
                    props[part[0]] = unescape(part[1])
        
        return status

    # Sends the command <str> and raises an exception if the response is not OK.
    # <str> should not have a trailing \n.
    def command(self, str):
        self.write(str + '\n')
        response = self.readline()

        if response != 'OK':
            raise RuntimeError(response, str)

    # Returns a dictionary of machine status parameters.
    # Raises an exception on failure.
    def machine_status(self):
        self.write('machine_status\n')
        response = self.readline()
        if response != 'Machine Status:':
            raise RuntimeError(response)
        
        status = {}
        while 1:
            line = self.readline()
            if line == 'OK':
                break;
            
            words = line.split(None)
            key = words[0]
            params = words[1:]
            
            # A single word is stored by itself.
            # Multiple words are stored as a tuple.
            if len(params) == 1:
                params = params[0]
            else:
                params = tuple(params)
            
            if key in status:
                # A recurring key causes a list of params to be made
                if type(status[key]) == type([]):
                    # This key has been seen multiple times before, so add
                    # these parameters to the list
                    status[key].append(params);
                else:
                    # This key has been seen once before, so replace it with a
                    # list.
                    status[key] = [status[key], params]
            else:
                status[key] = params
        
        return status

# Get status from all hosts
#
# Returns a dictionary of host name -> (process status, machine status)
def get_all_status():
    host_status = {}
    for x in hosts:
        wd = None
        try:
            wd = watchdog(x)
            status = wd.process_status()
            machine_status = wd.machine_status()
            host_status[x] = (status, machine_status)
        except Exception, ex:
            host_status[x] = ex
        
        if wd != None:
            wd.close()
    
    return host_status

########################

# Get standard form fields
form = cgi.FieldStorage()

# Get cookies
cookies = Cookie.SimpleCookie()
if 'HTTP_COOKIE' in os.environ:
    cookies.load(os.environ['HTTP_COOKIE'])

refresh = get_param('refresh', '')
