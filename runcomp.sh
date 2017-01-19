#!/bin/bash

read -p "Defend Plus X (0 or 1): " defendPlusX
read -p "Vision Channel (1,2 or Full): " visionChannel
read -p "Specify Playbook file (default comp.pbk): " playbook

make

until ./run/soccer -pbk ${playbook:-"comp.pbk"} -defend ${defendPlusX:-"0"} -vision ${visionChannel:-"full"}; do
	echo "Soccer crashed with exit code $?. Restarting" >&2
	sleep 1
done

echo "Soccer exited normally."


