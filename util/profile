#!/bin/bash

BASE=$(basename $1)

# Directory containing this script
UTIL_DIR=$(readlink -f $(dirname $0))

echo "Running $*"

# We have this mess because there is no way to get the PID
# of the most recent *foreground* process in bash,
# and backgrounding the process is even more trouble than this
# if you want SIGINT to work correctly.
#
# We use a temp file to communicate from the child shell to this one.
# We can't have any code execute after the program to be profiled because
# we will use exec, and we can't make any assumptions about the program's output
# so we can't use stdout.
PIDFILE=$(mktemp)
# Undocumented glibc feature: change the name of gmon.out, but it will have the PID on the end.
export GMON_OUT_PREFIX=$BASE.out
# Run the program, saving its PID beforehand
bash -c "echo \$\$ > $PIDFILE; exec $*"
# Get the PID
OLDPID=$(cat $PIDFILE)
rm -f $PIDFILE

# Output filenames
MONFILE=$GMON_OUT_PREFIX.$OLDPID
DOTFILE=$BASE.$OLDPID.dot
TXTFILE=$BASE.$OLDPID.txt

# Generate a call graph
gprof $1 $MONFILE > $TXTFILE
$UTIL_DIR/gprof2dot -s < $TXTFILE > $DOTFILE

# We shouldn't need the original file now that the text file has been generated
rm -f $MONFILE

echo
echo "Profiling output in $TXTFILE"
echo "Graph in $DOTFILE"
$UTIL_DIR/xdot $DOTFILE