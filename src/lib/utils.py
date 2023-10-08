"""Utility functions for hardware control on the reTerminal Raspberry pi
    device.
    """
import os
import shutil
import subprocess
import sys
import time
from lib.gpio import GPIO

columns, _ = shutil.get_terminal_size()

SYS_PLATFORM = sys.platform
DEVICE = None
RPI_BUZZER = None

# I/O settings
buzzer_pin = 6
GPIO.setwarnings(False)
GPIO.setup(buzzer_pin, GPIO.OUT)


# if reTerminal file paths exist set IS_RETERMINAL variable
# and make some permission changes for hardware control
if SYS_PLATFORM == 'linux':
    if os.path.exists('/sys/class/leds/usr_buzzer/brightness'):
        DEVICE = 'reTerminal'
        subprocess.run(
            ["sudo", "chmod", "777", "/sys/class/leds/usr_buzzer/brightness"])
        subprocess.run(
            ["sudo", "chmod", "777", "/sys/class/leds/usr_led0/brightness"])
        subprocess.run(
            ["sudo", "chmod", "777", "/sys/class/leds/usr_led1/brightness"])
        subprocess.run(
            ["sudo", "chmod", "777", "/sys/class/leds/usr_led2/brightness"])
    elif os.path.exists('/sys/class/leds/'):
        DEVICE = 'RPI'
        # RPI_BUZZER = Buzzer(6)


def beep_buzzer(twice: bool = False, delay: int = 100,
                twice_delay: int = 100, thrice: bool = False) -> None:
    """Function to buzz or double buzz the Buzzer when recording on a
    reTerminal RaspberryPi.

        Params:
            twice: controls double buzz.
            delay: how long the buzzer stays on in ms.
    """
    ms_convertor = .001

    if DEVICE == 'reTerminal':
        def buzz():
            nonlocal ms_convertor, delay
            with open('/sys/class/leds/usr_buzzer/brightness',
                      'w') as f:
                f.write(str(int(1)))
            time.sleep(ms_convertor * delay)
            with open('/sys/class/leds/usr_buzzer/brightness',
                      'w') as f:
                f.write(str(int(0)))

    elif DEVICE == 'RPI':
        def buzz():
            nonlocal ms_convertor, delay
            # RPI_BUZZER.on()
            GPIO.output(buzzer_pin, True)
            time.sleep(ms_convertor * delay)
            # RPI_BUZZER.off()
            GPIO.output(buzzer_pin, False)
    buzz()
    if twice or thrice:
        time.sleep(twice_delay * ms_convertor)
        buzz()
    if thrice:
        time.sleep(twice_delay * ms_convertor)
        buzz()


def buzz_buzzer(enable: bool = False) -> None:
    if DEVICE == 'reTerminal':
        with open('/sys/class/leds/usr_buzzer/brightness',
                  'w') as f:
            f.write(str(int(enable)))

    if DEVICE == 'RPI':
        if enable:
            # RPI_BUZZER.on()
            GPIO.output(buzzer_pin, True)
        else:
            # RPI_BUZZER.off()
            GPIO.output(buzzer_pin, False)


def led_change(brightness: bool or int = 0, led_num: int = 1, ):
    """Function to turn on/off red LED when recording on a
    reTerminal RaspberryPi.
    """
    assert led_num in [0, 1, 2]
    if DEVICE == 'reTerminal':
        if os.path.exists('/sys/class/leds/usr_led1/brightness'):
            with open(f'/sys/class/leds/usr_led{led_num}/brightness',
                      'w') as f:
                f.write(str(int(brightness)))
