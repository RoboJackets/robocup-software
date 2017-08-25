#!/bin/bash

read -p "Defend Plus X (plus or minus): " defendPlusX
read -p "Vision Channel (1,2 or Full): " visionChannel
read -p "Specify Playbook file (default comp.pbk): " playbook

make all-release

until ./run/soccer -pbk "${playbook:-"comp.pbk"}" -defend "${defendPlusX:-"minus"}" -vision "${visionChannel:-"full"}"; do
	echo "Soccer crashed with exit code $?. Restarting" >&2
	sleep 1
done

echo "Soccer exited normally."
