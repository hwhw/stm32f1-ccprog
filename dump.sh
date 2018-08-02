#!/bin/sh
stty -F $1 raw -echo -echoe -echok
cat $1 > $2 &
sleep 2
echo -e "reset\ninit\ndump" > $1
