#!/usr/bin/env bash

bash -c "sudo chmod 777 /sys/class/leds/usr_led0/brightness && echo 0 > /sys/class/leds/usr_led0/brightness"

bash -c "sudo chmod 777 /sys/class/leds/usr_led1/brightness && echo 0 > /sys/class/leds/usr_led1/brightness"

bash -c "sudo chmod 777 /sys/class/leds/usr_led2/brightness && echo 0 > /sys/class/leds/usr_led2/brightness"
