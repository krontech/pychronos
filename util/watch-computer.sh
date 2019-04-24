#!/bin/bash
set -uo pipefail
IFS=$'\n'

trap "exit;" SIGINT SIGTERM #The following loop works fine in Konsole but won't ever stop on Gnome Terminal.

while true; do 
	find -regex '\./[^_\.].*' ! -name git_description ! -path 'util/stats_reported' | entr -d bash -c "
		if [[ \$(cat git_description) != \$(git describe --tags --always) ]]; then #Only update when description changes, results in some thrashing otherwise.
			git describe --tags --always > git_description
		fi
		rsync -ir ./ \"${CAM_ADDRESS:-root@192.168.12.1}:/root/pychronos/\" \
			--delete --links \
			--rsh=\"/usr/bin/sshpass -p $CHRONOS_PASSWORD ssh -l root\" --times --inplace \
			--exclude \"__pycache__\" --exclude \"/.git\" \
	"
done