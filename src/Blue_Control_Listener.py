"""Listen for button press to enable capturing of sound and
    vibration events.

    Create a hot key that launches an instance of EventRecorder.

    Returns:
        None
    Creates:
        Subprocess running instance of EventRecorder.
    """
import subprocess
import sys
from bluedot import BlueDot
from lib.utils import led_change, beep_buzzer, buzz_buzzer
import signal
import pathlib

from lib.gpio import GPIO

home_dir = pathlib.Path.home()
bd = BlueDot()


def kill_recorder():
    buzz_buzzer(enable=False)
    beep_buzzer()
    subprocess.run(
        [f'{home_dir}/dev/rpi-sound-acc-recorder/bin/kill_S_B_L.sh'])


def launch_recorder():
    beep_buzzer(twice=True)
    # subprocess call allows EventRecorder to run properly on Raspberry Pi
    # from this button listening startup script. Avoids input overflows.
    subprocess.run(
        [f'{home_dir}/dev/rpi-sound-acc-recorder/bin/launch_AccelRecorder.sh'])


def listen_for_press():
    led_change(True, 0)
    print('\nListening for BlueDot button press.\n' +
          'Press blue dot to Start.')
    bd.wait_for_press()


def main():
    recorder_on = False
    while True:
        if not recorder_on:
            listen_for_press()
            recorder_on = True
            launch_recorder()
        else:
            listen_for_press()
            recorder_on = False
            kill_recorder()


def exit_clean():
    print('\nExiting Blue_Control_Listener\n')
    kill_recorder()
    led_change(False, 0)
    led_change(False, 1)
    led_change(False, 2)
    GPIO.cleanup()


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *args: sys.exit(0))

    try:
        main()

    except (KeyboardInterrupt, SystemExit):
        exit_clean()
