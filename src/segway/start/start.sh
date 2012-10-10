#!/usr/bin/env sh

xboxdrv --daemon --dpad-as-button --detach
bot-procman-deputy&
bot-procman-sheriff ../../config/procman/procman-segway.cfg&
