#!/usr/bin/env python
import lcm
import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
from bot_procman import printf_t
from bot_procman import info2_t

class ProcessLogs(object):
    def __init__(self):
        self.messages = {}
        self.processes = {}

    def printf_handler(self, channel, data):
        msg = printf_t.decode(data)

        if msg.sheriff_id not in self.messages:
            self.messages[msg.sheriff_id] = []
        self.messages[msg.sheriff_id].append(msg)

    def info_handler(self, channel_name, raw_data):
        msg = info2_t.decode(raw_data)

        for cmd in msg.cmds:
            if cmd.sheriff_id not in self.processes:
                self.processes[cmd.sheriff_id] = cmd.cmd.command_name

    def output(self):
        # print it all to files - but beware of process restarts
        # check if process names are unique
        # currently use a single file per command name <command>.txt
        # but could use a single file per process with <command>-<sid>.txt
        ordered_messages = {}

        for sid, process_messages in self.messages.iteritems():
            name = self.processes[sid]
            if name in ordered_messages:
                print "{} restarted, second sid found".format(name)
                for i, messages in enumerate(ordered_messages[name]):
                    # use the time of the first message as reference
                    if process_messages[0].utime < messages[0].utime:
                        ordered_messages[name].insert(process_messages, i)
            else:
                ordered_messages[name] = [process_messages]

        # output now what is hopefully the sorted list of messages
        for name, message_lists in ordered_messages.iteritems():
            print 'Writing to {}.'.format(name)
            filename = '{}.txt'.format(name)
            with open(filename, 'w') as f:
                for message_list in message_lists:
                    for msg in message_list:
                        try:
                            f.write('{} {}\n'.format(msg.utime, msg.text))
                        except:
                            print 'bad data in {} logs'.format(name)


if __name__ == "__main__":
    if (len(sys.argv) < 2):
        exit
    lcm_url = 'file://'+sys.argv[1]+'?speed=0'

    lc = lcm.LCM(lcm_url)

    pl = ProcessLogs()

    lc.subscribe('PMD_PRINTF', pl.printf_handler)
    lc.subscribe('PMD_INFO2', pl.info_handler)

    while True:
        try:
            lc.handle()
        except IOError:
            break

    pl.output()
