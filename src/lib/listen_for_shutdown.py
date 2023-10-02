#! /usr/bin/python

"""Script to listen for shutdown button press.

  Move to /usr/local/bin and make executable.
  """

import RPi.GPIO as GPIO
import subprocess

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.wait_for_edge(21, GPIO.FALLING)

subprocess.call(['shutdown', '-h', 'now'], shell=False)
