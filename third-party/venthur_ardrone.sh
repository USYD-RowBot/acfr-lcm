#!/bin/bash
THIRD_PARTY=$(readlink -f $(dirname $0))

if [ $1 = 'install' ]; then
    grep 'perls ardrone python path' $HOME/.bashrc > /dev/null
    if [ ! $? -eq 0 ]; then
        #append to file
        echo -e "# perls ardrone python path\nexport PYTHONPATH=$THIRD_PARTY/venthur_drone:\$PYTHONPATH" >> $HOME/.bashrc
    fi
elif [ $1 = 'uninstall' ]; then
    #perl -pe 's@# perls ardrone python path\nexport PYTHONPATH=.*@@' $HOME/.bashrc
    perl -i -pe 's@# perls ardrone python path\n@@' $HOME/.bashrc
    perl -i -pe "s@export PYTHONPATH=$THIRD_PARTY/venthur_drone.*\n@@" $HOME/.bashrc
fi

