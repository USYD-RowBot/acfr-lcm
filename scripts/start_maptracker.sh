#!/bin/bash

echo "Starting maptracker in tmux session in background"
tmux new -d -s maptracker-server 'cd ~/themaptracker; env/bin/python start_webserver.py'
echo "Starting maptracker-lcm-service in tmux session"
tmux new -d -s maptracker-lcm-service 'cd ~/maptracker-integration; python platform_scripts/acfr_lcm_apipost.py -c 8081'

echo "To see output of process type:"
echo "tmux attach"

{
    sleep 5;
    tmux detach
} &

tmux attach
