#!/bin/bash

sudo /sbin/route del -net 0.0.0.0 netmask 0.0.0.0 gw 172.16.154.254
sudo /sbin/route add -net 0.0.0.0 netmask 0.0.0.0 gw 172.16.154.207
