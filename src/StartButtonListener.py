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
from pynput.keyboard import Listener
from lib.utils import led_change, beep_buzzer, buzz_buzzer
import pathlib
import signal

from lib.gpio import GPIO

RECORDER = 'AccelerometerClipRecorder'
HOT_KEY_LAUNCH = 'f4'
HOT_KEY_REBOOT = 'f1'
HOT_KEY_KILL = 'f2'
PRESSED_HOT_KEY = None

HOME_DIR = str(pathlib.Path.home())
PATH_LAUNCHER = f'/dev/rpi-sound-acc-recorder/bin/launch_{RECORDER}.sh'
FULL_PATH_LAUNCHER = f'{HOME_DIR}{PATH_LAUNCHER}'


def key_press_logic(key):
    global PRESSED_HOT_KEY
    if hasattr(key, 'name') and key.name in [
            HOT_KEY_LAUNCH, HOT_KEY_REBOOT, HOT_KEY_KILL
    ]:
        PRESSED_HOT_KEY = key.name
        return False


def kill_recorder():
    buzz_buzzer(enable=False)
    beep_buzzer()
    subprocess.run(
        ['pkill', '-9', '-f', f'^.*{RECORDER}.py$'])


def main():
    """subprocess call allows EventRecorder to run properly on Raspberry Pi
    from this button listening startup script. Avoids input overflows.
    """
    recorder_running = False
    while True:
        with Listener(on_press=key_press_logic) as listener:
            led_change(True, 0)
            print(
                '\nListening for button press.\n' +
                f'Press {HOT_KEY_LAUNCH.upper()} to Start.\n' +
                f'Press {HOT_KEY_REBOOT.upper()} to Reboot.\n' +
                f'Press {HOT_KEY_KILL.upper()} to Kill.')
            listener.join()

        if PRESSED_HOT_KEY == HOT_KEY_LAUNCH and not recorder_running:
            recorder_running = True
            beep_buzzer(twice=True)
            subprocess.run(
                [FULL_PATH_LAUNCHER])

        elif PRESSED_HOT_KEY == HOT_KEY_REBOOT and recorder_running:
            recorder_running = False
            buzz_buzzer(enable=False)
            beep_buzzer()
            kill_recorder()

        elif PRESSED_HOT_KEY == HOT_KEY_KILL:
            buzz_buzzer(enable=False)
            beep_buzzer()
            exit_clean()


def exit_clean():
    print('\nExiting Start Button Listener\n')
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
