#!/usr/bin/env python

## @package goby-topside
#
# @author Paul Ozog - paulozog@umich.edu
#
# @details Run to abort/jump subsea ivers over UDP
#
# @brief abort/jump over UDP
#

import sys
import os
import getopt
import time

import lcm

SCRIPT_PATH             = os.path.dirname(__file__)
SCRIPT_NAME             = os.path.basename(sys.argv[0])

IVER28                  = 'iver28'
IVER31                  = 'iver31'
AUV_TO_NODE             = {IVER28: 2,
                           IVER31: 3}

import perls

auv_abort_t = perls.lcmtypes.perllcm.auv_abort_t
auv_jump_t = perls.lcmtypes.perllcm.auv_jump_t

class UsageException (BaseException):
    pass

def getNodeId(node):
    try:
        return AUV_TO_NODE[node]
    except Exception, err:
        raise UsageException('Must provide a valid node')

##
# Parse command line args
# @param : array of args
#
# @return tuple of command line args
# (node_id, node, abort, jump)
def parse_args (args):
    
    try:
        opts, args = getopt.getopt (sys.argv[1:], "hn:aj:", ["help", "node=", "abort", "jump="])
    except getopt.GetoptError, err:
        raise UsageException (err)

    node  = None
    abort = 0
    jump  = 0

    for o, a in opts:
        if o in ("-h", "--help"):
            raise UsageException('')
        elif o in ("-n", "--node"):
            node = a
        elif o in ("-a", "--abort"):
            abort = 1
        elif o in ("-j", "--jump"):
            jump = int (a)

    node_id = getNodeId(node)

    if not abort and not jump:
        raise UsageException ('Must provide either abort or jump flag')

    if abort and jump:
        raise UsageException ('Cannot supply both abort and jump flag')
 
    return (node_id, node, abort, jump)

##
# Show usage of goby-topside.py
# @param None
def print_usage ():
    print ' '.join(['usage:', 
                    os.path.basename(sys.argv[0]),
                    '<-n, --node node_name>',
                    '[-a, --abort]',
                    '[-j, --jump wypnt_num]'])

def main (args):

    try:
        (node_id, node, abort, jump) = parse_args (args)
    except UsageException, err:
        print err
        print_usage ()
        return 1

    url = os.getenv('LCM_DEFAULT_URL')
    print "The host's LCM_DEFAULT_URL is %s ... is this correct?" % url

    lc    = lcm.LCM ()
    param = perls.BotParam.BotParam()
    
    if abort:
        print 'User has commanded', node, 'to abort'
        msg = auv_abort_t ()
        msg.utime = int (time.time ())
        msg.dest  = node_id
        msg.abort = auv_abort_t.ABORT_HARD
        lc.publish ("%s_OS_REMOTEHELM_ABORT" % node.upper(), msg.encode ())
    elif jump:
        print 'User has commanded ', node, ' to jump to waypoint ', jump
        msg = auv_jump_t ()
        msg.utime = int (time.time ())
        msg.dest  = node_id
        msg.next_wypnt = jump
        lc.publish ("%s_OS_REMOTEHELM_JUMP" % node.upper(), msg.encode ())

    return 0

if __name__ == '__main__':
    sys.exit (main (sys.argv))
