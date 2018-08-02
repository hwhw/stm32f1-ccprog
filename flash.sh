#!/bin/sh
stty -F $1 raw -echo -echoe -echok
cat $1 &
sleep 2
cat $2 > $1
