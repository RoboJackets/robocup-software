#!/bin/bash -e

HELP_STR="Usage: ./debug-cpp.sh [-n] [-h]
\tn:\t\t\t name of node to debug (default soccer)
\th:\t\t\t print this message!
"

while getopts ":n:h" arg; do case "${arg}" in
	h) printf "$HELP_STR" && exit 1 ;;
	n) node=${OPTARG} ;;
	?) printf "Unrecognized/Invalid Option: -${OPTARG}"
	   printf "\n$HELP_STR"
	   exit 1 ;;
esac done

[ -z "$node" ] && node="soccer"

echo -e "for now we will assume the node "$node" is valid...\n"

BASE=$(readlink -f $(dirname $0)/..)

file="$BASE/launch/soccer.launch.py"

FOUND_NODE_FLAG="0"
FOUND_METHOD_FLAG="0"
METHOD_NAME="generate_launch_description"
while read -r line; do
	if [ ${FOUND_NODE_FLAG} -ne "2" ] && [[ "$line" =~ ^"$node" ]]; then 
		FOUND_NODE_FLAG="1"
	elif [ ${FOUND_NODE_FLAG} -eq "1" ]; then
		if [[ "$line" =~ ")"$ ]]; then
			FOUND_NODE_FLAG="2"
		fi
	else
		if ! [[ "$line" =~ "$node" ]]; then
			if [[ "$line" =~ "$METHOD_NAME" ]]; then
				FOUND_METHOD_FLAG="1"
				echo -e "$line\n"
			elif [ ${FOUND_METHOD_FLAG} -eq 1 ]; then
				echo -e "\t$line\n"
			else
				echo -e "$line\n"
			fi
		fi
	fi
done <$file > "$BASE"/launch/debug-soccer.launch.py 

[ ${FOUND_NODE_FLAG} -eq 2 ] || (printf "invalid node" && exit 1)

sed 's/soccer.launch.py/debug-soccer.launch.py/g' "$BASE"/launch/sim.launch.py > "$BASE"/launch/debug-sim.launch.py

echo -e "
	done!\n
	steps to debug now:\n
	1. ros2 launch rj_robocup debug-sim.launch.py\n
	in a new terminal tab:\n
	(recommended debugger is lldb-10 which is already installed by util/ubuntu-setup)  
	2. ros2 run --prefix 'lldb-10 run' rj_robocup executable_name\n
	"
