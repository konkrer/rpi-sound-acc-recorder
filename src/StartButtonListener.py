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
from lib.utils import led_change, beep_buzzer
import pathlib

home_dir = str(pathlib.Path.home())
HOT_KEY_LAUNCH = 'f4'


def key_press_logic(key):
    if hasattr(key, 'name') and key.name == HOT_KEY_LAUNCH:
        return False
    # Listen for ctrl+c in key listener thread.
    # else:
    #     for_canonical(hotkey.press)(key)


# Listen for ctrl+c in key listener thread.
# An attempt to be able respond to SIGINT signal when S.B.Listener
# is launched from shell script. Doesn't work.
# def for_canonical(f):
#     return lambda k: f(listener.canonical(k))
# def keyboard_interrupt():
#     raise KeyboardInterrupt
# hotkey = HotKey(
#     HotKey.parse('<ctrl>+c'),
#     keyboard_interrupt)


if __name__ == '__main__':
    try:
        with Listener(on_press=key_press_logic) as listener:
            led_change(True, 0)
            print(
                '\nListening for Start button press.\n' +
                f'Press {HOT_KEY_LAUNCH.upper()} to Start.')
            listener.join()
        beep_buzzer(twice=True)
        # subprocess call allows EventRecorder to run properly on Raspberry Pi
        # from this button listening startup script. Avoids input overflows.
        subprocess.run([f'{home_dir}/dev/rpi-sound-acc-recorder/bin/launch_EventRecorder.sh'])

    except (KeyboardInterrupt):
        print('\nExiting Start Button Listener\n')
        led_change(False, 0)
        led_change(False, 1)
        led_change(False, 2)
        sys.exit(0)
