#!/usr/bin/env python
from __future__ import print_function

import lcm
import sys
import curses
import curses.ascii
import time

try:
    from cStringIO.StringIO import BytesIO
except:
    from io import BytesIO


class ProcmanLog(object):
    def __init__(self, lcm_handle):
        self.lcm_handle = lcm_handle
        
        lc.subscribe("PMD_PRINTF", self.handle_print)
        lc.subscribe("PMD_DISCOVER", self.handle_discover)
        lc.subscribe("PMD_ORDERS2", self.handle_orders)
        lc.subscribe("PMD_INFO2", self.handle_info)

        self.messages = {}
        self.command_ids = {}

    def handle_print(self, channel_name, raw_data):
        msg = bot_procman.printf_t.decode(raw_data)

        if msg.sheriff_id not in self.messages:
            self.messages[msg.sheriff_id] = list()

        # TODO: try and decode these a little - so that they don't take up multiple lines
        # so \n appears as \n on screen.
        # NOTE: it appears that field() with curses.ascii.unctrl() achieves this
        self.messages[msg.sheriff_id].append((msg.utime, msg.text))

    def handle_discover(self, channel_name, raw_data):
        msg = bot_procman.discovery_t.decode(raw_data)
        # don't think this is super important...

    def handle_orders(self, channel_name, raw_data):
        msg = bot_procman.orders2_t.decode(raw_data)
        # the sheriff sends the orders

    def handle_info(self, channel_name, raw_data):
        msg = bot_procman.info2_t.decode(raw_data)
        # and the deputy says what it is
        # this might be the one to extract program information from

        # go through all the commands and add the sid to the command name
        # this makes it easier to track across multiple instantiations
        for command in msg.cmds:
            sid = command.sheriff_id
            name = command.cmd.command_name

            if not name in self.command_ids:
                self.command_ids[name] = set()
            
            self.command_ids[name].add(sid)


class Interface(object):
    def __init__(self, data):
        self.selected_application = None  # the name of the channel, not index
        self.data = data
        self.quitting = False

        self.application_pad = None
        self.log_pad = None

        self.height = None
        self.width = None

        self.known_applications = 0

    def resize(self):
        # we expect a min width of 80, can probably handle less
        self.height, self.width = self.screen.getmaxyx()

    def __call__(self, screen):
        self.screen = screen
        self.application_pad = curses.newpad(100, 40)
        self.log_pad = curses.newpad(100, 100)

        self.resize()

        self.application_pad.border()

        self.screen.nodelay(1)
        self.quitting = False
        while not self.quitting:
            self.render()
            try:
                lc.handle_timeout(100)
            except IOError:
                break

            key = self.screen.getch()

            sorted_channels = sorted(self.data.command_ids.keys())
            if key == -1:
                continue

            elif key == curses.KEY_UP:
                if self.selected_application is None:
                    self.selected_application = sorted_channels[-1]
                else:
                    index = sorted_channels.index(self.selected_application) - 1
                    self.selected_application = sorted_channels[index]

            elif key == curses.KEY_DOWN:
                if self.selected_application is None:
                    self.selected_application = sorted_channels[0]
                else:
                    index = sorted_channels.index(self.selected_application) + 1
                    if index >= len(sorted_channels):
                        index = 0
                    self.selected_application = sorted_channels[index]
            elif key == curses.KEY_RESIZE:
                self.resize()

            elif key == ord('q') or key == ord('Q'):
                self.quitting = True

    def render(self):
        if self.known_applications != len(self.data.command_ids):
            # need to update the application list
            for i, name in enumerate(sorted(self.data.command_ids.iterkeys())):
                if name == self.selected_application:
                    self.application_pad.addstr(i+1, 1, field(name, 18), curses.A_BOLD)
                else:
                    self.application_pad.addstr(i+1, 1, field(name, 18))
            self.application_pad.border()
            self.application_pad.refresh(0, 0, 0, 0, min(100, self.height - 1), 19)
            #self.known_applications = len(self.data.command_ids)

        last_messages = []
        if self.selected_application in self.data.command_ids:
            for sid in self.data.command_ids[self.selected_application]:
                if sid in self.data.messages:
                    last_messages.extend(self.data.messages[sid][-100:])

        last_messages.sort()

        self.log_pad.clear()
        for i, msg in enumerate(last_messages[-98:]):
            self.log_pad.addstr(i+1, 1, field(msg[1], 98))

        self.log_pad.border(0, 0, 0, 0, curses.ACS_TTEE, 0, curses.ACS_BTEE, 0)
        self.log_pad.refresh(0, 0, 0, 20, self.height - 1, self.width - 1)
        

def field(string, width):
    printable = ""
    for c in bytearray(string, encoding='ascii', errors='replace'):
        try:
            printable += curses.ascii.unctrl(c)
        except UnicodeEncodeError:
            pass
    return printable.ljust(width)[:width]

if __name__ == '__main__':
    sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
    import bot_procman

    if len(sys.argv) == 2:
        lcm_url = "file://"+sys.argv[1]+"?speed=0"
    else:
        lcm_url = ""

    lc = lcm.LCM(lcm_url)

    log = ProcmanLog(lc)
    interface = Interface(log)


    while not interface.quitting:
        curses.wrapper(interface)
