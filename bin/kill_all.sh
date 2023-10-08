#!/bin/bash

# kill StartButtonListener if running
s_b_l_results=`ps -h | grep StartButtonListener.py | wc -l`

if [ $s_b_l_results -eq 2 ]
then
  bash -c "kill -n 15 `ps -h | grep StartButtonListener.py -m 1 | awk '{print $1;}'`"
fi

# kill Blue_Control_Listener if running
b_c_l_results=`ps -h | grep Blue_Control_Listener.py | wc -l`

if [ $b_c_l_results -eq 2 ]
then
  bash -c "kill -n 15 `ps -h | grep Blue_Control_Listener.py -m 1 | awk '{print $1;}'`"
fi


# kill EventRecorder if running
event_results=`ps -h |grep EventRecorder.py | wc -l`

if [ $event_results -eq 2 ]
then
  bash -c "kill -n 15 `ps -h | grep EventRecorder.py -m 1 | awk '{print $1;}'`"
fi

# kill AccelerometerClipRecorder if running
accel_results=`ps -h |grep AccelerometerClipRecorder.py | wc -l`

if [ $accel_results -eq 2 ]
then
  bash -c "kill -n 15 `ps -h | grep AccelerometerClipRecorder.py -m 1 | awk '{print $1;}'`"
fi

# kill MicrophoneClipRecorder if running
mic_results=`ps -h |grep MicrophoneClipRecorder.py | wc -l`

if [ $mic_results -eq 2 ]
then
  bash -c "kill -n 15 `ps -h | grep MicrophoneClipRecorder.py -m 1 | awk '{print $1;}'`"
fi

#~/dev/rpi-sound-acc-recorder/bin/ledsOff.sh
