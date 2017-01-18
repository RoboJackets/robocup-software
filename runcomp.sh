#!/bin/bash

read -p "Defend Plus X (0 or 1): " defendPlusX
read -p "Vision Channel (1,2 or Full): " visionChannel
read -p "Specify Playbook file: " playbook

make

until ./run/soccer -pbk $playbook -defend $defendPlusX -vision $visionChannel; do
	echo "Soccer crashed with exit code $?. Restarting" >&2
	sleep 1
done

echo "Soccer exited normally."


