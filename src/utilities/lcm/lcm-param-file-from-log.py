import sys
import lcm
from perls import lcmtypes

if __name__ == "__main__":
    if len (sys.argv) < 2:
        print ("Usage %s <lcm-log> [dest-file]" % (sys.argv[0]))
        sys.exit (1)

    try:
        log = lcm.EventLog (sys.argv[1])
    except:
        print ("Error opening lcm-log file %s" % (sys.argv[1]))
        sys.exit (1)

    if len (sys.argv) == 3:
        fname = sys.argv[2]
    else:
        fname = "master.cfg"

    try:
        f = open (fname, 'w')
    except:
        print ("Error opening dest file %s" % (fname))
        sys.exit (1)

    updates = [lcmtypes.bot_param.update_t.decode (e.data) for e in log if "PARAM_UPDATE" in e.channel]
    f.write (updates[0].params)

    f.close ()
    sys.exit (0)


