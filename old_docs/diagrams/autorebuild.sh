#!/bin/bash

# Run this script to rebuild the image files every time a .dot file changes

filewatcher *.dot make
