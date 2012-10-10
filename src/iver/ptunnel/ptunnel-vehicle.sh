#!/usr/bin/env bash

SCRIPTNAME=$(basename $0)

trap cleanup INT
trap cleanup TERM

function cleanup {
    echo "${SCRIPTNAME}: Caught INT or SIGTERM signal"
    kill -9 $deputyPid $tunnelPid
    exit 0
}

function printUsage {
    echo "Usage: ./${SCRIPTNAME}"
    echo
    echo "Launches bot-procman-deputy and initiates persistent bot-lcm-tunnel server."
    echo
    echo "To use:"
    echo "1. cd to PERLS/bin on vehicle, and run this script with no arguments"
    echo "2. After running ${SCRIPTNAME}, run persistant-tunnel-topside on client"
    echo
    echo "Options"
    echo "  -h      Show this."
    echo "  -D      Run as daemon."
    echo "          NOTE: All output is redirected to LOGFILE=/var/log/persistent-tunnel-vehicle.log"
    echo "          To ensure that you have write permission to this path, do the following:"
    echo "          sudo touch LOGFILE && chown auv LOGFILE"
    echo
    echo "NOTE: To have ${SCRIPTNAME} launch as a daemon upon vehicle boot,"
    echo "simply drop the following lines into your /etc/rc.local file:"
    echo
    echo "PERLS=\"/home/auv/perls/bin\""
    echo "PATH=\"\${PERLS}:\${PATH}\""
    echo "export PATH"
    echo "cd \${PERLS} && ${SCRIPTNAME} -D"
    echo
}


while getopts 'hD' option; do
    case "$option" in
	h)  printUsage
            exit 0
	    ;;
        D)  # daemonize
            LOGFILE="/tmp/persistent-tunnel-vehicle.log"
            >${LOGFILE} && chmod 664 ${LOGFILE}
            $0 -- 1>> ${LOGFILE} 2>> $LOGFILE &
            exit 0
            ;;
	\?) printUsage
            exit 1
	    ;;
    esac
done

#Keep trying to start the vehicle's deputy and tunnel, if it's not running
if pkill -0 -f bot-procman-deputy > /dev/null; then
    echo -e "Warning: you have other deputies running [$(pgrep -d, -f bot-procman-deputy)]\n"
fi

if pkill -0 -f bot-lcm-tunnel > /dev/null; then
    echo -e "Warning: you have other tunnels running [$(pgrep -d, -f bot-lcm-tunnel)]\n"
fi

echo "Launching deputy"
echo "PATH=$PATH"
echo "LCM_DEFAULT_URL=$LCM_DEFAULT_URL"
echo "BOT_PARAM_SERVER_NAME=$BOT_PARAM_SERVER_NAME"
bot-procman-deputy &
deputyPid=$!
sleep 1

echo -e "\nLaunching tunnel"
bot-lcm-tunnel &
tunnelPid=$!
sleep 1

DELAY=1
while [ 1 ]; do
    if ! kill -0 ${deputyPid} &> /dev/null; then
        echo -e "\n${SCRIPTNAME}: pid=${deputyPid} died; relaunching bot-procman-deputy" >&2
	bot-procman-deputy &
	deputyPid=$!
    fi

    if ! kill -0 ${tunnelPid} &> /dev/null; then
        echo -e "\n${SCRIPTNAME}: pid=${tunnelPid} died; relaunching bot-lcm-tunnel" >&2
	bot-lcm-tunnel --tcp-max-age-ms=1000 &
	tunnelPid=$!
    fi

    sleep ${DELAY}
done
