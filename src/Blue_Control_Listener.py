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
# from signal import pause
import pathlib

home_dir = pathlib.Path.home()
bd = BlueDot()


def kill_event_recorder():
    buzz_buzzer(on=False)
    beep_buzzer()
    subprocess.run(
        [f'{home_dir}/dev/rpi-sound-acc-recorder/bin/kill_S_B_L.sh'])


def launch_event_recorder():
    beep_buzzer(twice=True)
    # subprocess call allows EventRecorder to run properly on Raspberry Pi
    # from this button listening startup script. Avoids input overflows.
    subprocess.run(
        [f'{home_dir}/dev/rpi-sound-acc-recorder/bin/launch_AccelRecorder.sh'])


def exit_control_listener():
    kill_event_recorder()
    sys.exit(0)


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
            launch_event_recorder()
        else:
            listen_for_press()
            recorder_on = False
            kill_event_recorder()


if __name__ == '__main__':
    try:
        main()

    except (KeyboardInterrupt):
        print('\nExiting Start Button Listener\n')
        led_change(False, 0)
        led_change(False, 1)
        led_change(False, 2)
        sys.exit(0)
