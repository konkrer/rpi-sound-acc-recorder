"""Detect sound or vibrational events and records clips
    of sound and accelerometer data.

    Asynchronous management of sound and vibration
    monitoring and recording.
    """

import shutil
# import time
from MicrophoneClipRecorder import MicrophoneClipRecorder
from lib.utils import led_change
import asyncio
import sys

try:
    from AccelerometerClipRecorder import AccelerometerClipRecorder
except ModuleNotFoundError:
    pass

# Accel Settings
ACCEL_WAKE_UP_G_THRESHOLD = None  # None equivalent to 0.5g
ACCEL_BTS_G_THRESHOLD = None
ACCEL_G_RANGE = None  # None 1equivalent to '2G'

# Microphone Settings
SOUND_THRESHOLD = 20  # .04
SOUND_BTS_THRESHOLD = .02
SOUND_THRESHOLD_SAMPLERATE = 22050
CLIP_DURATION = 4  # SECS
CONTINUOUS_REC_SAMPLERATE = 44100
MAX_CONTINUOUS_CLIP_DURATION = 2  # MINS

columns, _ = shutil.get_terminal_size()


class SoundVibrationEventRecorder:
    def __init__(self, threshold=SOUND_THRESHOLD, **kwargs) -> None:
        self.sound = MicrophoneClipRecorder(
            threshold=threshold, bts_threshold=SOUND_BTS_THRESHOLD, **kwargs)
        self.vibration = AccelerometerClipRecorder(
            startup_mode='WAKE_UP_TRIGGER',
            g_range=ACCEL_G_RANGE,
            wake_up_g_threshold=ACCEL_WAKE_UP_G_THRESHOLD,
            bts_threshold=ACCEL_BTS_G_THRESHOLD,
            **kwargs
        ) if 'AccelerometerClipRecorder' in globals() else None

    async def record_events(self) -> None:
        led_change(True, 2)
        while True:
            # create sound and/or vibration threshold tasks
            sound_alert = asyncio.create_task(
                self.sound.threshold_alert(
                    samplerate=SOUND_THRESHOLD_SAMPLERATE),
                name='sound_thresh')
            tasks = [sound_alert]
            if self.vibration:
                vibration_alert = asyncio.create_task(
                    self.vibration.threshold_alert(),
                    name='accel_thresh')
                tasks.append(vibration_alert)

            #  look for first to resolve
            done, pending = await asyncio.wait(
                tasks, return_when=asyncio.FIRST_COMPLETED)

            # get time stamp and cancel pending tasks
            task_completed = done.pop()
            trigger_info = task_completed.result()
            if len(pending):
                task_pending = pending.pop()
                task_pending.cancel()

            # record clips
            tasks = [self.sound.record_clip(trigger_info)]
            if self.vibration:
                tasks.append(asyncio.to_thread(
                    self.vibration.record_clip, trigger_info))
            await asyncio.gather(*tasks)
            # print('Listening!!'.center(columns, 'x'))

    async def record_events_continuous(self):
        # time.sleep(10)
        led_change(True, 2)

        # messaging queues
        mic_events = asyncio.Queue()
        accel_events = asyncio.Queue()

        # create sound and/or vibration threshold tasks
        sound_alert = asyncio.create_task(
            self.sound.messaging_threshold_recorder(
                msg_out_queue=mic_events, msg_in_queue=accel_events,
                samplerate=CONTINUOUS_REC_SAMPLERATE),
            name='sound_rec')

        tasks = [sound_alert]

        if self.vibration:
            vibration_alert = asyncio.create_task(
                asyncio.to_thread(
                    self.vibration.messaging_threshold_recorder,
                    msg_out_queue=accel_events, msg_in_queue=mic_events,
                    loop=asyncio.get_event_loop()),
                name='accel_rec')
            tasks.append(vibration_alert)

        await asyncio.wait(
            tasks, return_when=asyncio.FIRST_COMPLETED)


def main():
    try:
        event_recorder = SoundVibrationEventRecorder(
            clip_duration=CLIP_DURATION,
            max_continuous_clip_duration=MAX_CONTINUOUS_CLIP_DURATION)
        print('\n')
        print(" Sensors Active ".center(columns, '!'))
        # asyncio.run(event_recorder.record_events())
        asyncio.run(event_recorder.record_events_continuous())
    except (KeyboardInterrupt, SystemExit):
        print('\n ~> Aborted ~x_x~\n')
        sys.exit(0)
    finally:
        led_change(False, 1)
        led_change(False, 2)
        if event_recorder.vibration:
            event_recorder.vibration.cleanup()


if __name__ == '__main__':
    main()
