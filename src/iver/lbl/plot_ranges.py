import sys

import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":
    if len (sys.argv) < 4:
        print "Usage: %s <raw_file> <filtered_file> <fix_file>" % (sys.argv[0])
        sys.exit (1)

    try:
        raw = np.loadtxt (sys.argv[1])
        filt = np.loadtxt (sys.argv[2])
    except:
        print "could not load raw and filt files!!!"
        sys.exit (1)

    #for ii in range (1,2):
    plt.figure (1)
    for ii in range (1,raw.shape[1]):
        t_raw = (raw[:,0]-raw[0,0])*1e-6
        r_raw = raw[:,ii]*1500

        jj = np.nonzero (filt[:,ii]!=-1)[0]
        t_filt = (filt[jj,0]-raw[0,0])*1e-6
        r_filt = filt[jj,ii]*1500
        lab_str = " lbl %d" % ii
        plt.plot (t_filt, r_filt, 'o', label=("filt"+lab_str))
        plt.plot (t_raw, r_raw, '.', label=("raw"+lab_str))

    plt.legend ()
    plt.title ("raw ranges and median filtered ranges")

    try:
        fix = np.loadtxt (sys.argv[3])
    except:
        print "could not load fix file!!!"
        sys.exit (1)

    plt.figure (2)
    sol = fix[:,1:3]
    plt.plot (sol[:,0], sol[:,1], 'bo-', label="lbl sol")

    seed = fix[:,3:]
    plt.plot (seed[:,0], seed[:,1], 'r.-', label="seed pos")
    plt.axis ("equal")

    plt.legend ()
    plt.title ("seed position and lbl solution")

    plt.figure (3)
    err = seed - sol;
    err_norm = np.sqrt (np.dot (err,err.T).diagonal ())
    plt.plot (fix[:,0], err_norm, '.-')

    plt.show ()

    sys.exit (0);




