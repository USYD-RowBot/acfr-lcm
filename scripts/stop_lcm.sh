#!/bin/bash
<<<<<<< HEAD
pkill bot-procman-dep
=======

pkill bot-procman-de

result=$?

if [ $result -eq 1 ]; then
    echo "No deputy running."
elif [ $result -eq 0 ]; then
    echo "Killed deputy."
else
    echo "Unknown output of pkill ($result)"
fi
>>>>>>> 42476afddd2fd5d78707f598a8883364747d4174
