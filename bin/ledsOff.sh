#!/usr/bin/env bash

if [ -f "/sys/class/leds/usr_led2/brightness" ]; then
  bash -c "sudo chmod 777 /sys/class/leds/usr_led0/brightness && echo 0 > /sys/class/leds/usr_led0/brightness"

  bash -c "sudo chmod 777 /sys/class/leds/usr_led1/brightness && echo 0 > /sys/class/leds/usr_led1/brightness"

  bash -c "sudo chmod 777 /sys/class/leds/usr_led2/brightness && echo 0 > /sys/class/leds/usr_led2/brightness"

  bash -c "sudo chmod 777 /sys/class/leds/usr_buzzer/brightness && echo 0 > /sys/class/leds/usr_buzzer/brightness"
fi
