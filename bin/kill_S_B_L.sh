#!/bin/bash

s_b_l_results=`ps -h | grep StartButtonListener.py | wc -l`

if [ $s_b_l_results -eq 2 ]
then
  bash -c "kill -9 `ps -h | grep StartButtonListener.py -m 1 | awk '{print $1;}'`"
fi


event_results=`ps -h |grep EventRecorder.py | wc -l`

if [ $event_results -eq 2 ]
then
  bash -c "kill -9 `ps -h | grep EventRecorder.py -m 1 | awk '{print $1;}'`"
fi

~/dev/rpi-sound-acc-recorder/bin/ledsOff.sh
