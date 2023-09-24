# rpi-souund-acc-recorder

## Python sound and accelerometer data recording. Threshold controlled recording.

### Used with RaspberryPi.

#### Installation Notes:

##### Using pipenv:

- `sudo apt update`
- `sudo apt upgrade`
- `sudo apt install git`
- `sudo apt install python3-distutils`
- `sudo apt install libatlas-base-dev` for numpy use on RPI.
- `sudo apt install python3-dbus` for bluedot
- `git config --global core.autocrlf input`
- `curl -O https://bootstrap.pypa.io/get-pip.py`
- `python get-pip.py`
- `pip install pipenv --user`
- `mkdir dev && cd dev`
- `git clone https://github.com/konkrer/rpi-sound-acc-recorder.git`
- `cd rpi-sound-acc-recorder`
- `cp "Pipfiles/RPI_Accel_Only/Pipfile" ./`
- Use `pipenv --python=/usr/bin/python --site-packages` to create venv with system site-packages.
- `pipenv install`
- (If needed) `pipenv run pip install --no-binary numpy numpy` to compile from source to allow use of latest version.
- enable i2c. https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial
- `sudo raspi-config` Under System, click "Boot / Auto-Login" and then set the console to automatically log in your user. (allow .profile to autostart scripts after boot)
