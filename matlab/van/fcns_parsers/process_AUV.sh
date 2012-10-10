#!/usr/bin/env bash
#
# SeaBED log file parsing script.
#
# 2006.04.22  Created and written by Ryan Eustice, Johns Hopkins University

outdir='.';
while getopts C: OPTION; do
    case "$OPTION" in
        'C') 
            outdir=$OPTARG
            ;;
        
        '?')
            print -u2 "Usage: ${PROG_NAME}  [-C outdir] file1.auv ... filesN.auv"
            exit 2
            ;;
    esac
done
shift $(($OPTIND-1))
outdir="${outdir%*/}" # remove trailing '/' if present

# modem parse list
mlist=("CATOA");

export LANG='C';
for fullname in "$@"; do
    # strip path and extract logger extension (e.g., "RAW", "CTL", "TOPSIDE")
    fname=$(basename $fullname)
    ftype=$(basename $fname .auv)
    ftype=${ftype##*.}

    case "$ftype" in
        "TOPSIDE" | "RAW" | "CTL")
            echo "processing... $fullname";
            # automagically generated grep parse list composed from the unique entries
            # in the first column of log file (assumes white space is the delimiter).
            plist=($(cut -f1 -d" " $fullname | sort -u))
            ;;

        "MODEMDAT")
            echo "processing... $fullname"
            plist=("LBL")
            ;;

        *)
            # default ignore
            echo "skipping..... $fullname"
            continue
            ;;
    esac

    bname=${fname%*.$ftype.auv} #basename
    for p in ${plist[@]}; do
        # remove any trailing ':' in log file extension
        ext=${p%*:}
        
        oname="$bname.$ext.puv" # outname
        fgrep "$p" < "$fullname" > "$outdir/$oname"
        if [ -s "$outdir/$oname" ]; then
            echo -e "\tgenerated $outdir/$oname"
        else
            /bin/rm -f "$outdir/$oname"
        fi
        
        if [ "$ext" = "MICROMODEM" ]; then
            for m in ${mlist[@]}; do
                mname="$bname.$m.puv"
                fgrep "$m" < "$outdir/$oname" > "$outdir/$mname"
                if [ -s "$outdir/$mname" ]; then
                    echo -e "\tgenerated $outdir/$mname"
                else
                    /bin/rm -f "$outdir/$mname"
                fi
            done
        fi
    done
    echo ""

done
