
#!/bin/bash

bash -c "cd /home/rich/dev/data_grabber && pipenv run python src/RebootButtonListener.py" &

bash -c "cd /home/rich/dev/data_grabber && pipenv run python src/StartButtonListener.py" &

