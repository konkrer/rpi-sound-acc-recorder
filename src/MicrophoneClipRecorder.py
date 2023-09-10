"""Record clips from known microphones manually or automatically triggered.

    Main functionality:
      -Record from threshold.
      -Threshold Alert
      -Record Clip
      -Asynchronous messaging threshold recorder. Arbitrary length clips.
      -Save wav file with metadata in file name
    """

import os
import sys
import shutil
import numpy as np
import sounddevice as sd
import soundfile as sf
from datetime import datetime as dt
from lib.utils import led_change, buzz_buzzer
from lib.common import (
    TME_STMP_FORMAT, MIC_TRIGGER_NAME, ACCEL_TRIGGER_NAME,
    MIC_RECORD_EVENT_MSG, MIC_RECORD_STOP_EVENT_MSG, ACCEL_RECORD_EVENT_MSG,
    ACCEL_RECORD_STOP_EVENT_MSG)
import asyncio

SYS_PLATFORM = sys.platform

# sampling settings
CLIP_DURATION = 4  # seconds
MAX_CONTINUOUS_CLIP_DURATION = 1  # minutes
SAMPLE_RATE = None  # None uses default device rate.
CHANNELS = 1
SAVE_FOLDER = './microphone_data/'


# Debounce count per minute of recording to wait for accelerometer data to be
# saved. This may have to be increased to compensate for higher accelerometer
# ODR and larger numpy array saves.
# used for vars for messaging_threshold_recorder()
DEBOUNCE_COUNT = 700
# The back to sleep threshold has to be met for this many data frames checked
# sequentially to trigger bts condition (data frames are "indata" frames from
# inputstream_generator()).
BTS_THRESHOLD_COUNT = 100


SND_DEVICE_SETTINGS = {
    # mems mic needs voltage adjust, initial data ignored,
    # significant input boost.
    'mems': {
        'threshold': 0.3,
        'bts_threshold': 0.15,
        'boost_coeff': 10,
        'voltage_offset': 0.52,
        'loop_skip': 500,
    },
    'usb': {
        'threshold': .1,
        'bts_threshold': 0.05,
        'boost_coeff': 1.5,
        'voltage_offset': 0,
        'loop_skip': 0,
    },
}

# choose mems or usb mic for input default by index
KNOWN_SOUND_DEVICES = {
    0: {'name': 'snd_rpi_i2s_card', 'type': 'mems'},  # raspberrypi
    1: {'name': 'USB Audio Device', 'type': 'usb'},   # raspberrypi
    2: {'name': 'Microphone (USB Audio Device), MME', 'type': 'usb'}  # pc
}
RASBERRYPI_DEVICE_CHOICE = 0
PC_DEVICE_CHOICE = 2
SOUND_DEVICE_CHOICE = \
    RASBERRYPI_DEVICE_CHOICE if SYS_PLATFORM == 'linux' else PC_DEVICE_CHOICE


columns, _ = shutil.get_terminal_size()


class MicrophoneClipRecorder:
    def __init__(self, samplerate=SAMPLE_RATE,
                 save_folder=SAVE_FOLDER,
                 channels=CHANNELS,
                 device=SOUND_DEVICE_CHOICE,
                 clip_duration=CLIP_DURATION,
                 max_continuous_clip_duration=MAX_CONTINUOUS_CLIP_DURATION,
                 threshold=None,
                 bts_threshold=None,
                 ) -> None:
        self.channels = channels
        self.clip_duration = clip_duration
        self.max_continuous_clip = max_continuous_clip_duration
        self.samplerate = samplerate
        self.save_folder = save_folder
        self.device = KNOWN_SOUND_DEVICES[device]['name']
        # back to sleep threshold count
        self.bts_thresh_count = 0

        self.threshold = threshold or SND_DEVICE_SETTINGS[
            KNOWN_SOUND_DEVICES[device]['type']]['threshold']
        self.bts_threshold = bts_threshold or SND_DEVICE_SETTINGS[
            KNOWN_SOUND_DEVICES[device]['type']]['bts_threshold']
        self.boost_coeff = SND_DEVICE_SETTINGS[
            KNOWN_SOUND_DEVICES[device]['type']]['boost_coeff']
        self.voltage_offset = SND_DEVICE_SETTINGS[
            KNOWN_SOUND_DEVICES[device]['type']]['voltage_offset']
        self.loop_skip = SND_DEVICE_SETTINGS[
            KNOWN_SOUND_DEVICES[device]['type']]['loop_skip']

        if samplerate is None:
            device_info = sd.query_devices(
                self.device, 'input')
            self.samplerate = int(device_info['default_samplerate'])
        if not os.path.isdir(SAVE_FOLDER):
            os.mkdir(SAVE_FOLDER)

    def threshold_trigger(self, array: np.array) -> bool:
        if np.max(np.abs(array)) > self.threshold:
            return True
        else:
            return False

    def bts_threshold_trigger(self, array: np.array) -> bool:
        if np.max(np.abs(array)) < self.bts_threshold:
            self.bts_thresh_count += 1
            if self.bts_thresh_count == BTS_THRESHOLD_COUNT:
                self.bts_thresh_count = 0
                return True
        else:
            self.bts_thresh_count = 0
        return False

    async def record_clips_from_threshold(self, callback=None, **kwargs):
        try:
            print(' Listening! '.center(columns, '!'))
            await self._record_clips_from_threshold(
                callback, **kwargs
            )
        except KeyboardInterrupt:
            print(' < Threshold Recording Stopped > '.center(columns, '-'))
            sys.exit(0)
        finally:
            led_change()

    async def _record_clips_from_threshold(self, callback=None):
        buffer_frames = self.samplerate * self.clip_duration
        buffer = np.zeros((buffer_frames, self.channels), dtype=np.float32)
        write_idx = None
        ts = None

        loop = asyncio.get_event_loop()
        stream_generator = self.inputstream_generator()

        loop_idx = 0
        async for data, status in stream_generator:
            if status:
                print(status, file=sys.stderr)
            loop_idx += 1
            if loop_idx < self.loop_skip:
                continue

            # begin recording with loud events
            if write_idx is None and self.threshold_trigger(data):
                write_idx = 0
                led_change(True)
                ts = dt.now()
                if callback:
                    loop.call_soon_threadsafe(
                        callback, ts)
                print(' < Microphone Rec. Started > '.center(columns, '*'))

            # if recording in progress
            if type(write_idx) == int:
                remainder = len(buffer) - write_idx
                data = data[:remainder]
                buffer[write_idx:write_idx + len(data)] = data
                write_idx += len(data)

                if write_idx == len(buffer):
                    print(' > Microphone Rec. Stopped < '.center(
                        columns, '-'))
                    loop.call_soon_threadsafe(
                        self.save_data_to_file, buffer.copy(), [ts, ''])
                    led_change()
                    buffer[:] = 0
                    write_idx = None

    async def record_clip(self, trigger_info: list[dt, str] = None, **kwargs):
        samplerate = kwargs.get('samplerate') or self.samplerate
        buffer_frames = samplerate * self.clip_duration
        buffer = np.zeros((buffer_frames, self.channels), dtype=np.float32)
        write_idx = 0
        # max_amp = 0

        led_change(True)
        # print(' < Microphone Rec. Started > '.center(columns, '*'))
        metadata = trigger_info or [dt.now(), '']

        loop = asyncio.get_event_loop()
        stream_generator = self.inputstream_generator(**kwargs)

        loop_idx = 0
        try:
            async for data, status in stream_generator:
                if status:
                    print(status, file=sys.stderr)
                loop_idx += 1
                if loop_idx < self.loop_skip:
                    continue

                # max_amp = max(max_amp, np.max(np.abs(data)))

                remainder = len(buffer) - write_idx
                data = data[:remainder]
                buffer[write_idx:write_idx + len(data)] = data
                write_idx += len(data)

                if write_idx == len(buffer):
                    # print(f"{max_amp=}")
                    # print(' > Mic Rec. Stopped < '.center(columns, '-'))
                    loop.call_soon_threadsafe(
                        self.save_data_to_file, buffer.copy(), metadata)
                    led_change()

                    break
            return True
        except KeyboardInterrupt:
            led_change()
            sys.exit(0)
        except Exception as e:
            led_change()
            raise e

    async def threshold_alert(self, **kwargs):
        stream_generator = self.inputstream_generator(**kwargs)
        loop_idx = 0

        try:
            async for data, status in stream_generator:
                if status:
                    print(status, file=sys.stderr)
                loop_idx += 1
                if loop_idx < self.loop_skip:
                    continue

                # look for loud events
                if self.threshold_trigger(data):
                    # print(' >-Sound Trigger-< '.center(columns, 'S'))
                    ts = dt.now()
                    return [ts, MIC_TRIGGER_NAME]

        except KeyboardInterrupt:
            led_change()
            sys.exit(0)
        except Exception as e:
            led_change()
            raise e

    async def messaging_threshold_recorder(
            self, msg_out_queue: asyncio.Queue,
            msg_in_queue: asyncio.Queue, **kwargs):
        try:
            # print(' Listening! '.center(columns, '!'))
            await self._messaging_threshold_recorder(
                msg_out_queue, msg_in_queue, **kwargs
            )
        except KeyboardInterrupt:
            print(
                ' < Async Threshold Recorder Stopped > '.center(columns, '-'))
            sys.exit(0)
        finally:
            led_change()

    async def _messaging_threshold_recorder(
            self, msg_out_queue: asyncio.Queue, msg_in_queue: asyncio.Queue,
            **kwargs):
        samplerate = kwargs.get('samplerate', self.samplerate)
        max_clip_duration_in_seconds = int(self.max_continuous_clip * 60)
        buffer_frames = samplerate * max_clip_duration_in_seconds
        buffer = np.zeros((buffer_frames, self.channels), dtype=np.float32)
        stop_frame = buffer_frames
        write_idx = None
        ts = None

        loop = asyncio.get_event_loop()
        stream_generator = self.inputstream_generator(**kwargs)

        loop_idx = 0
        sound_triggered_rec = False
        debounce = 0

        async for data, status in stream_generator:
            if status:
                print(status, file=sys.stderr)
            loop_idx += 1

            msg, ts = None, None
            while not msg_in_queue.empty():
                msg, ts = msg_in_queue.get_nowait()
            accel_has_triggered = msg == ACCEL_RECORD_EVENT_MSG
            if not accel_has_triggered:
                if loop_idx < self.loop_skip:
                    continue
                if debounce:
                    debounce -= 1
                    continue
            else:
                debounce = 0

            # begin recording with loud events or accel trigger
            if write_idx is None and (
                    accel_has_triggered or self.threshold_trigger(data)):
                write_idx = 0
                led_change(True)
                buzz_buzzer(True)
                if not accel_has_triggered:
                    sound_triggered_rec = True
                    # print(' >-Sound Trigger-< '.center(columns, 'S'))
                    ts = dt.now()
                    loop.call_soon_threadsafe(
                        msg_out_queue.put_nowait, (MIC_RECORD_EVENT_MSG, ts))
                # print(' < Microphone Rec. Started > '.center(columns, '*'))
                metadata = [ts, ACCEL_TRIGGER_NAME
                            if accel_has_triggered
                            else MIC_TRIGGER_NAME]

            # if recording in progress
            if type(write_idx) == int:
                if (not sound_triggered_rec and
                        msg == ACCEL_RECORD_STOP_EVENT_MSG):
                    stop_frame = int(ts * samplerate)
                    # print('mic setting stop frame from message from accel')

                elif (sound_triggered_rec and
                      write_idx >= self.clip_duration * samplerate
                        and self.bts_threshold_trigger(data)):
                    # stop the recording
                    stop_frame = write_idx
                    # print('mic sending stop message')
                    loop.call_soon_threadsafe(
                        msg_out_queue.put_nowait,
                        (MIC_RECORD_STOP_EVENT_MSG, write_idx/samplerate))

                if write_idx >= stop_frame:
                    buzz_buzzer(False)
                    # print(' > Microphone Rec. Stopped < '.center(
                    # columns, '-'))
                    # print(f' > {sound_triggered_rec=} < '.center(
                    # columns, '-'))
                    buffer_copy = buffer.copy()[:write_idx]
                    loop.call_soon_threadsafe(
                        self.save_data_to_file, buffer_copy, metadata)
                    led_change()
                    buffer[:] = 0
                    if sound_triggered_rec:
                        debounce = int((DEBOUNCE_COUNT/60) *
                                       (write_idx / samplerate))
                    write_idx = None
                    sound_triggered_rec = False
                    stop_frame = buffer_frames

                else:
                    # write to buffer
                    remainder = len(buffer) - write_idx
                    data = data[:remainder]
                    buffer[write_idx:write_idx + len(data)] = data
                    write_idx += len(data)

    async def inputstream_generator(self, **kwargs):
        """Generator that yields blocks of input data as NumPy arrays."""
        loop = asyncio.get_event_loop()
        q_in = asyncio.Queue()

        def callback(indata, frame_count, time_info, status):
            copy = indata.copy()
            # if status == 'input overflow':
            #     copy[:] = 0
            loop.call_soon_threadsafe(q_in.put_nowait, (copy, status))

        kwargs = {'device': self.device, 'samplerate': self.samplerate,
                  'channels': self.channels, **kwargs}
        stream = sd.InputStream(callback=callback, **kwargs)
        with stream:
            while True:
                indata, status = await q_in.get()
                indata = indata * self.boost_coeff + self.voltage_offset
                yield indata, status

    def save_data_to_file(self, buffer, metadata: list[dt, str] = None):
        ts, trigger = metadata
        time_str = ts.strftime(TME_STMP_FORMAT)
        trigger_suffix = f'_trig_{trigger}' if trigger else ''
        fn = f'{self.save_folder}/sound_{time_str}{trigger_suffix}.wav'
        with sf.SoundFile(
            fn,
            mode='x', samplerate=self.samplerate,
            channels=self.channels
        ) as file:
            file.write(buffer)
        # print(f'sound file saved to dir: {self.save_folder}')


if __name__ == '__main__':
    if SYS_PLATFORM == "linux":
        try:
            from AccelerometerClipRecorder import AccelerometerClipRecorder
        except ModuleNotFoundError:
            pass
    #  record from threshold. callback to accel recording if possible.
    try:
        mic_recorder = MicrophoneClipRecorder()

        if 'AccelerometerClipRecorder' in locals():
            accel_recorder = AccelerometerClipRecorder()
            asyncio.run(
                mic_recorder.record_clips_from_threshold(
                    accel_recorder.record_clip))
        else:
            asyncio.run(mic_recorder.record_clips_from_threshold())

    except (KeyboardInterrupt, SystemExit):
        print('Exiting! '.center(20, '~'))
        sys.exit(0)
    finally:
        if 'accel_recorder' in locals():
            accel_recorder.cleanup()
