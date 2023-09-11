#!/bin/bash

bash -c "cd ~/dev/rpi-sound-acc-recorder && pipenv run python src/RebootButtonListener.py" &

bash -c "cd ~/dev/rpi-sound-acc-recorder && pipenv run python src/StartButtonListener.py" &

