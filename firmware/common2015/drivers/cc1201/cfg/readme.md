
# CC1201 Configs

This directory contains config files for the cc1201 radio.  The easiest way to configure the cc1201 is to use TI's SmartRF Studio.  It allows you to use a gui to set register values and easily see their effects and test them on an actual radio with the cc debugger.


## Config format

Each folder in this directory contains two files:
* A SmartRF Studio xml config file.  In SmartRF Studio, use the "File" menu to load and save these config files.
* A C code register export file.  This is an array of register values that we use import in our radio code to set the radio settings at startup.  Click the "Register Export" button in RF Studio to pull up the exporter.  Click "Select", then make sure that all register values are exported (the default is to only export the ones changed from the default config).  Then select the "trxEB RF Settings Value Line" template, so it's in the format that our code expects.
