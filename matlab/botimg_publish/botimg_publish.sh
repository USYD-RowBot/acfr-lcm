#!/usr/bin/env bash

unset LD_LIBRARY_PATH
../../build/bin/perls-vis-botimg-publish -t $1 -f $2 -c $3
