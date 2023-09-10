#!/bin/bash

bash -c "killall -9 /home/rich/.local/share/virtualenvs/data_grabber-v78n5DF2/bin/python /home/rich/dev/data_grabber/src/StartButtonListener.py"

/home/rich/dev/data_grabber/bin/ledsOff.sh

/home/rich/dev/data_grabber/bin/launch_S_B_L.sh &
