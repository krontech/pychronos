#!/bin/bash
set -euo pipefail
IFS=$'\n'

trap "exit;" SIGINT SIGTERM

cd "$(dirname "$0")/.." #Always sync project root directory.

#The following loop works fine in Konsole but won't ever stop on Gnome Terminal.
while true; do 
	find -regex '\./[^_\.].*' ! -name git_description ! -path 'util/stats_reported' | entr -d bash -c "
		if [[ \$(cat git_description) != \$(git describe --tags --always) ]]; then #Only update when description changes, results in some thrashing otherwise.
			git describe --tags --always > git_description
		fi
		
		rsync --recursive --delete --links --inplace --times --itemize-changes \
			./ \"${CAM_ADDRESS:-root@192.168.12.1}:~/pychronos/\" \
			--exclude \"__pycache__\" \
			--exclude \"/.git\" \
			--exclude \".directory\" \
	" || true
done