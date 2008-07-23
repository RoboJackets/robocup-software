#include <ctype.h>
#include <string.h>
#include <pwd.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include "process.h"
#include "log.h"

using namespace std;

// Replaces $VAR with the contents of the environment variable VAR,
// and replaces $$ with $.
void var_escape(const char *in, string &out)
{
    out.clear();
    for (; *in; ++in)
    {
        if (*in == '$')
        {
            ++in;
            
            if (*in == '$')
            {
                // Literal '$'
                out.push_back('$');
            } else {
                // Variable substitution
                
                // Find the length of the variable name
                int len;
                for (len = 0; isalnum(in[len]) || in[len] == '_'; ++len)
                    ;
                
                // Get the variable
                string name(in, len);
                const char *value = getenv(name.c_str());
                if (value)
                {
                    out.append(value);
                } else {
                    log_printf("config:  No environment variable named \"%s\"\n", name.c_str());
                }
                
                // Subtract one because the for loop will add one
                in += len - 1;
            }
        } else {
            out.push_back(*in);
        }
    }
}

void parse_command(Process *process, xmlNodePtr root)
{
    for (xmlNodePtr node = root->children; node; node = node->next)
    {
        if (node->type == XML_ELEMENT_NODE)
        {
            const char *name = (const char *)node->name;
            if (!name)
            {
                continue;
            }
            
            const char *content;
            if (node->children)
            {
                content = (const char *)node->children->content;
            } else {
                content = 0;
            }
            
            if (!strcasecmp(name, "arg") && content)
            {
                string arg;
                
                var_escape(content, arg);
                process->command.push_back(arg);
            }
        }
    }
}

void parse_process(xmlNodePtr root)
{
    Process *process = new Process();
    
    // Use DISPLAY from the environment.
    // This can be overridden by a display tag in each process.
    process->display = Process::default_display;
    
    for (xmlNodePtr node = root->children; node; node = node->next)
    {
        if (node->type == XML_ELEMENT_NODE)
        {
            const char *name = (const char *)node->name;
            if (!name)
            {
                continue;
            }
            
            const char *content;
            if (node->children)
            {
                content = (const char *)node->children->content;
            } else {
                content = 0;
            }
            
            if (!strcasecmp(name, "name") && content)
            {
                // Process name
                process->name = content;
            } else if (!strcasecmp(name, "dir") && content)
            {
                // Working directory
                var_escape(content, process->cwd);
            } else if (!strcasecmp(name, "command"))
            {
                // Command line arguments (including the path to the executable)
                parse_command(process, node);
            } else if (!strcasecmp(name, "enabled") && content)
            {
                // Enabled
                process->enabled = atoi(content);
            } else if (!strcasecmp(name, "delay") && content)
            {
                process->delay = atoi(content);
            } else if (!strcasecmp(name, "max_restarts") && content)
            {
                process->max_restarts = atoi(content);
            } else if (!strcasecmp(name, "auto_display") && content)
            {
                if (!strcasecmp(content, "true") || !strcmp(content, "1"))
                {
                    process->auto_display = true;
                } else {
                    process->auto_display = false;
                }
            } else if (!strcasecmp(name, "display"))
            {
                process->display = content;
            }
        }
    }
    
    Process *old = Process::find(process->name);
    if (old)
    {
        // The process already exists, so replace its information and delete the new one
        old->replace_with(process);
        
        delete process;
    } else {
        //FIXME - Validation
        process->add();
    }
}

static void parse_user(xmlNodePtr root)
{
    xmlNodePtr node = root->children;
    if (!node)
    {
        return;
    }
    
    const char *content = (const char *)node->content;
    if (!content)
    {
        return;
    }
    
    //FIXME - Use getpwnam_r instead
    struct passwd *pw = getpwnam(content);
    if (!pw)
    {
        log_printf("config: Unknown user \"%s\"\n", content);
        return;
    }
    
    if (getuid() != pw->pw_uid)
    {
        log_printf("Switching to user \"%s\" (uid %d)\n", content, pw->pw_uid);
        if (setuid(pw->pw_uid))
        {
            log_printf("    %m\n");
        }
    }
}

void config_load(const char *filename)
{
    xmlDocPtr doc = xmlReadFile(filename, 0, 0);
    if (!doc)
    {
        log_printf("Can't read config file %s\n", filename);
        return;
    }
    
    xmlNodePtr root = xmlDocGetRootElement(doc);
    if (!root)
    {
        log_printf("%s: No root node\n", filename);
        return;
    }
    
    for (xmlNodePtr node = root->children; node; node = node->next)
    {
        const char *name = (const char *)node->name;
        if (!name)
        {
            continue;
        }
        
        if (node->type == XML_ELEMENT_NODE)
        {
            if (!strcasecmp(name, "process"))
            {
                parse_process(node);
            } else if (!strcasecmp(name, "user"))
            {
                parse_user(node);
            } else if (!strcasecmp(name, "default_display"))
            {
                Process::default_display = (const char *)node->children->content;
            }
        }
    }
    
    xmlFreeDoc(doc);
}
