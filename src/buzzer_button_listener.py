"""
    Listen for button press to enable buzzer.
"""
import sys
import time
from pynput.keyboard import Listener
from lib.utils import led_change, buzz_buzzer, beep_buzzer


HOT_KEY_LAUNCH = 'f4'
HOT_KEY_EXIT = 'f2'


def key_press_logic(key):
    if hasattr(key, 'name') and key.name == HOT_KEY_LAUNCH:
        buzz_buzzer(on=True)
        time.sleep(3)
        buzz_buzzer(on=False)
    if hasattr(key, 'name') and key.name == HOT_KEY_EXIT:
        return False


if __name__ == '__main__':
    try:
        with Listener(on_press=key_press_logic) as listener:
            led_change(True, 0)
            print(
                '\nListening for Buzzer button press.\n' +
                f'Press {HOT_KEY_LAUNCH.upper()} to Start.')
            listener.join()
        beep_buzzer(twice=True)
        led_change(False, 0)
    except (KeyboardInterrupt):
        print('\nExiting Buzzer Button Listener\n')
        led_change(False, 0)
        led_change(False, 1)
        led_change(False, 2)
        sys.exit(0)
