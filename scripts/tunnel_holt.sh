#!/bin/bash
bot-lcm-tunnel -r '.*AUVSTAT.*' -s '.*TASK_PLANNER_COMMAND.*|HOLT.SPEKTRUM_CONTROL' -u 172.16.154.123 -p 6142
