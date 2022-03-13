#!/bin/bash

gnome-terminal -e "killall -9 /usr/lib/gnome-terminal/gnome-terminal-server"
sleep 1
gnome-terminal -e "sshpass -p hmc2020 ssh -t hmcl@192.168.0.121 './pc2_kill.sh'"

