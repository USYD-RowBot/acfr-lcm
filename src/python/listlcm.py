#!/usr/bin/python

import lcm

channels = []

def handler(channel, data):
	if channel not in channels:
		channels.append(channel)
	s = sorted(channels)
	for ch in s:
		print ch

lc = lcm.LCM()
lc.subscribe(".*", handler)

while 1:
	lc.handle()
