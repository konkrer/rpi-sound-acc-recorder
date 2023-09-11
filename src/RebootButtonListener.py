import shutil
import subprocess
import sys
import pathlib
from pynput.keyboard import Listener
from lib.utils import led_change, beep_buzzer, buzz_buzzer

columns, _ = shutil.get_terminal_size()
home_dir = str(pathlib.Path.home())

HOT_KEY_REBOOT = 'f1'
HOT_KEY_KILL = 'f2'
PRESSED_HOT_KEY = None


def start_key_listener(key):
    global PRESSED_HOT_KEY
    if hasattr(key, 'name') and key.name == HOT_KEY_REBOOT:
        PRESSED_HOT_KEY = key.name
        return False
    if hasattr(key, 'name') and key.name == HOT_KEY_KILL:
        PRESSED_HOT_KEY = key.name
        return False


try:
    with Listener(on_press=start_key_listener) as listener:
        led_change(True, 0)
        print('\nListening for Reboot or Kill button press.\n' +
              f'Press {HOT_KEY_REBOOT.upper()} to Reboot.\n' +
              f'Press {HOT_KEY_KILL.upper()} to Kill.')
        listener.join()
    if PRESSED_HOT_KEY == HOT_KEY_REBOOT:
        buzz_buzzer(on=False)
        beep_buzzer()
        subprocess.run([f'{home_dir}/dev/rpi-sound-acc-recorder/bin/reboot_S_B_L.sh'])
    elif PRESSED_HOT_KEY == HOT_KEY_KILL:
        buzz_buzzer(on=False)
        beep_buzzer()
        subprocess.run([f'{home_dir}/dev/rpi-sound-acc-recorder/bin/kill_S_B_L.sh'])


except (KeyboardInterrupt):

    print('Exiting Reboot Button Listener')
finally:
    led_change(False, 0)
    buzz_buzzer(on=False)
    sys.exit(0)


