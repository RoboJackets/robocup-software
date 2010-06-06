#ifndef _CONFIG_H_
#define _CONFIG_H_

// Loads a configuration file.
// The caller must hold the process list mutex when calling this.
void config_load(const char *filename);

#endif // _CONFIG_H_
