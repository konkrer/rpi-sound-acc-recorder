# rpi-souund-acc-recorder

## Python sound and accelerometer data recording. Threshold controlled recording.

### Used with RaspberryPi based Seeed reTerminal.

#### Installation Notes:

##### Using pipenv:

- Use "pipenv --python=/usr/bin/python --site-packages" to create venv with system site-packages.
- "sudo apt install libatlas-base-dev" for numpy use on RPI.
- "pipenv run pip install --no-binary numpy numpy" to compile from source to allow use of latest version.
