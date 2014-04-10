#!/bin/bash

HISTFILE=~/.bash_history
set -o history
#i=$(history | awk '{CMD[$2]++;count++;}END { for (a in CMD)print CMD[a] " " a;}' | grep -v "./" | column -c3 -s " " -t | sort -nr |  head -n10)
i=$(history | awk '{print $2}' | sort | uniq -c | sort -nr | sed -n 1,10p)
echo "$i"

exit 0
