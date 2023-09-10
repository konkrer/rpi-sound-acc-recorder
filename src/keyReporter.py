
# import pdb
import sys
from pynput.keyboard import Listener


def start_key_listener(key):
    if hasattr(key, 'char'):
        print(key.char)
    else:
        print(key.name)


try:
    with Listener(on_press=start_key_listener) as listener:
        print('\nListening')
        listener.join()
except (KeyboardInterrupt, Exception):
    print('Exiting Listener')
    sys.exit(0)
