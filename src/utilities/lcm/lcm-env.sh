#!/usr/bin/env bash


if [ "$1" = "iver28" ] ; then
    export LCM_DEFAULT_URL="udpm://239.255.76.67:2828?ttl=1"
    echo "LCM_DEFAULT_URL set to iver28"
elif [ "$1" = "iver31" ] ; then
    export LCM_DEFAULT_URL="udpm://239.255.76.67:3131?ttl=1"
    echo "LCM_DEFAULT_URL set to iver31"

elif [ "$1" = "kupa" ] ; then
    export LCM_DEFAULT_URL="udpm://239.255.32.32:4040?ttl=1"
    echo "LCM_DEFAULT_URL set to kupa"
elif [ "$1" = "kupa0" ] ; then
    export LCM_DEFAULT_URL="udpm://239.255.32.32:4041?ttl=1"
    echo "LCM_DEFAULT_URL set to kupa0"
elif [ "$1" = "kupa1" ] ; then
    export LCM_DEFAULT_URL="udpm://239.255.32.32:4042?ttl=1"
    echo "LCM_DEFAULT_URL set to kupa1"
elif [ "$1" = "kupa2" ] ; then
    export LCM_DEFAULT_URL="udpm://239.255.32.32:4043?ttl=1"
    echo "LCM_DEFAULT_URL set to kupa2"
elif [ "$1" = "kupa3" ] ; then
    export LCM_DEFAULT_URL="udpm://239.255.32.32:4044?ttl=1"
    echo "LCM_DEFAULT_URL set to kupa3"
else #default
    export LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=1"
    echo "LCM_DEFAULT_URL set to default"
fi
