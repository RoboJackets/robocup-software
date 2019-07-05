#!/bin/bash

if ! command -v git >/dev/null; then
    echo "Git is not installed!!!" >&2
    exit 1
fi

if [ "$(git rev-parse --abbrev-ref HEAD)" = "HEAD" ]; then
    echo "HEAD IS DETATCHED AT COMMIT: $(git rev-parse HEAD)"
elif ! git rev-parse --abbrev-ref HEAD | grep -q comp; then
    echo "YOUR BRANCH DOES NOT CONTAIN COMP. PLEASE RECONSIDER OR COMMENT ME OUT." >&2
    exit 1
else
    echo "Branch is: $(git rev-parse --abbrev-ref HEAD)"
fi

read -p "Defend Plus X (plus or minus): " defendPlusX
read -p "Vision Channel (1,2 or Full): " visionChannel
read -p "Specify Playbook file (default comp.pbk): " playbook
read -p "Specify Config File (default realComp.xml): " config

make all-release

until ./run/soccer -c "soccer/config/${config:-"realComp.xml"}" -pbk "${playbook:-"comp.pbk"}" -defend "${defendPlusX:-"minus"}" -vision "${visionChannel:-"full"}"; do
	echo "Soccer crashed with exit code $?. Restarting" >&2
	sleep 1
done

echo "Soccer exited normally."
