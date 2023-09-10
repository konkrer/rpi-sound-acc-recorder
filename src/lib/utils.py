"""Utility functions for hardware control on the reTerminal Raspberry pi
    device.
    """
import os
import shutil
import subprocess
import sys
import time

columns, _ = shutil.get_terminal_size()

SYS_PLATFORM = sys.platform
IS_RETERMINAL = False

# if reTerminal file paths exist set IS_RETERMINAL variable
# and make some permission changes for hardware control
if SYS_PLATFORM == 'linux':
    if os.path.exists('/sys/class/leds/usr_buzzer/brightness'):
        IS_RETERMINAL = True
        subprocess.run(
            ["sudo", "chmod", "777", "/sys/class/leds/usr_buzzer/brightness"])
        subprocess.run(
            ["sudo", "chmod", "777", "/sys/class/leds/usr_led0/brightness"])
        subprocess.run(
            ["sudo", "chmod", "777", "/sys/class/leds/usr_led1/brightness"])
        subprocess.run(
            ["sudo", "chmod", "777", "/sys/class/leds/usr_led2/brightness"])


def beep_buzzer(twice: bool = False, delay: int = 100,
                twice_pause_ms: int = 100) -> None:
    """Function to buzz or double buzz the Buzzer when recording on a
    reTerminal RaspberryPi.

        Params:
            twice: controls double buzz.
            delay: how long the buzzer stays on in ms.
    """
    ms_convertor = .001

    if IS_RETERMINAL:
        def buzz():
            nonlocal ms_convertor, delay
            with open('/sys/class/leds/usr_buzzer/brightness',
                      'w') as f:
                f.write(str(int(1)))
            time.sleep(ms_convertor * delay)
            with open('/sys/class/leds/usr_buzzer/brightness',
                      'w') as f:
                f.write(str(int(0)))
        buzz()
        if twice:
            time.sleep(twice_pause_ms * ms_convertor)
            buzz()


def buzz_buzzer(on: bool = False) -> None:
    if IS_RETERMINAL:
        with open('/sys/class/leds/usr_buzzer/brightness',
                  'w') as f:
            f.write(str(int(on)))


def led_change(brightness: bool or int = 0, led_num: int = 1, ):
    """Function to turn on/off red LED when recording on a
    reTerminal RaspberryPi.
    """
    assert led_num in [0, 1, 2]
    if IS_RETERMINAL:
        if os.path.exists('/sys/class/leds/usr_led1/brightness'):
            with open(f'/sys/class/leds/usr_led{led_num}/brightness',
                      'w') as f:
                f.write(str(int(brightness)))
