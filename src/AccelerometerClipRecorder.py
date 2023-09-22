"""Record clips of accelerometer data from kx132 accelerometer
    manually or automatically triggered.

    Main functionality:
      -Threshold Alert
      -Record Clip
      -Asynchronous messaging threshold recorder. Arbitrary length clips.
      -Save numpy file with metadata in file name
    """

import os
import sys
import shutil
import time
import qwiic_kx13x
import numpy as np
import asyncio
from datetime import datetime as dt
from lib.utils import led_change, buzz_buzzer
from lib.common import (
    TME_STMP_FORMAT, MIC_TRIGGER_NAME, ACCEL_TRIGGER_NAME,
    MIC_RECORD_EVENT_MSG, MIC_RECORD_STOP_EVENT_MSG, ACCEL_RECORD_EVENT_MSG,
    ACCEL_RECORD_STOP_EVENT_MSG)
import RPi.GPIO as GPIO

# Accelerometer settings
# output data rate (Hz)
ODR = 400
ODR_SETTINGS = {
    .781: 0,
    1.563: 1,
    3.125: 2,
    6.25: 3,
    12.5: 4,
    25: 5,
    50: 6,
    100: 7,
    200: 8,
    400: 9,
    800: 10,
    1600: 11,
    3200: 12,
    6400: 13,
    12800: 14,
    25600: 15
}

# Recording settings
CLIP_DURATION = 2  # seconds
MAX_CONTINUOUS_CLIP = 1  # mins
# back to sleep threshold
BTS_THRESHOLD = 0.05
SAVE_FOLDER = 'accelerometer_data'

# I/O settings
dataReadyPin = 22
GPIO.setmode(GPIO.BCM)
GPIO.setup(dataReadyPin, GPIO.IN)

columns, _ = shutil.get_terminal_size()


class AccelerometerClipRecorder:
    def __init__(self, startup_mode: str = 'INT_SETTINGS',
                 odr: int = ODR,
                 g_range: str or None = None,
                 save_folder: str = SAVE_FOLDER,
                 wake_up_g_threshold: float or int or None = None,
                 bts_threshold: float or int or None = None,
                 clip_duration: int = CLIP_DURATION,
                 max_continuous_clip_duration: int = MAX_CONTINUOUS_CLIP):
        self.save_folder = save_folder
        # Accelerometer initialization
        self.accel = qwiic_kx13x.QwiicKX132()
        self.accel.set_standby_mode()
        self.accel.output_data_rate = ODR_SETTINGS[odr]
        self.accel.initialize(
            startup_mode, wake_up_g_threshold, g_range=g_range)
        self.wake_up_g_threshold = wake_up_g_threshold
        self.bts_threshold = bts_threshold or BTS_THRESHOLD
        self.clip_duration = clip_duration
        self.max_continuous_clip = max_continuous_clip_duration
        self.odr = odr
        self.read_delay = 1 / self.odr
        self.g_range = g_range

    def adp_off(self) -> None:
        self.accel.initialize('ADP_OFF')
        self.accel.output_data_rate = ODR_SETTINGS[self.odr]

    async def threshold_alert(self):
        self.accel.accel_control(False)
        self.accel.clear_interrupt()
        self.accel.clear_buffer()
        self.accel.initialize(
            settings="WAKE_UP_TRIGGER",
            wake_up_threshold=self.wake_up_g_threshold,
            g_range=self.g_range)

        while True:
            if GPIO.input(dataReadyPin) == 1:
                # print(' >-Vibration Trigger-< '.center(columns, 'V'))
                ts = dt.now()
                return [ts, ACCEL_TRIGGER_NAME]
            # default wake up engine rate @ 50hz. 1/rate
            await asyncio.sleep(.2)

    def record_clip(self, trigger_info: list[dt, str] = None) -> None:
        self.accel.accel_control(False)
        self.accel.clear_interrupt()
        self.accel.clear_buffer()
        self.accel.initialize(settings="INT_SETTINGS", g_range=self.g_range)

        metadata = trigger_info or [dt.now(), '']

        # data buffer
        buffer_frames, num_accel_axes = self.clip_duration * self.odr, 3
        buffer = np.zeros((buffer_frames, num_accel_axes), dtype=np.float32)
        write_idx = 0

        print(' < Accelerometer Rec. Started > '.center(columns, '*'))
        led_change(True)
        buzz_buzzer(True)
        while True:
            if GPIO.input(dataReadyPin) == 1:
                if write_idx == buffer_frames:
                    break
                self.accel.get_accel_data()
                buffer[write_idx][:] = [
                    self.accel.accel.x, self.accel.accel.y, self.accel.accel.z]
                write_idx += 1
                self.accel.clear_interrupt()
            # time.sleep(self.read_delay)  # limits achievable ODR

        buzz_buzzer(False)
        led_change()
        print(' > Accelerometer Rec. Stopped < '.center(columns, '-'))

        self.save_data_to_file(buffer, metadata)
        return True

    def threshold_recorder(self):
        led_change(True, 2)
        # set up accelerometer to trigger mode
        self.accel.clear_buffer()
        self.accel.initialize(
            settings="WAKE_UP_TRIGGER",  # changes ODR
            wake_up_threshold=self.wake_up_g_threshold,
            g_range=self.g_range)

        while True:
            # if accel wake up trigger
            if GPIO.input(dataReadyPin) == 1:
                # return to ODR for recording
                self.accel.output_data_rate = ODR_SETTINGS[self.odr]
                self.record_clip()

                # set up accelerometer to trigger mode
                self.accel.clear_buffer()
                self.accel.initialize(
                    settings="WAKE_UP_TRIGGER",
                    wake_up_threshold=self.wake_up_g_threshold,
                    g_range=self.g_range)

            # default wake up engine rate @ 50hz. 1/rate
            time.sleep(.02)

    def messaging_threshold_recorder(self, msg_out_queue: asyncio.Queue,
                                     msg_in_queue: asyncio.Queue, loop):
        # set up accelerometer to trigger mode
        self.accel.clear_buffer()
        # self.accel.initialize(settings="ADP")
        # self.adp_off()
        self.accel.initialize(
            settings="WAKE_UP_TRIGGER",
            wake_up_threshold=self.wake_up_g_threshold,
            g_range=self.g_range)

        # data buffer
        max_clip_in_seconds = int(self.max_continuous_clip * 60)
        buffer_frames = max_clip_in_seconds * self.odr
        num_accel_axes = 3
        buffer = np.zeros(
            (buffer_frames, num_accel_axes), dtype=np.float32)

        while True:
            msg, ts, stop_frame = None, None, buffer_frames
            while not msg_in_queue.empty():
                msg, ts = msg_in_queue.get_nowait()
            microphone_has_triggered = msg == MIC_RECORD_EVENT_MSG

            # if accel wake up trigger or microphone trigger
            if GPIO.input(dataReadyPin) == 1 or microphone_has_triggered:
                # set up accelerometer to synchronous w/ hardware interrupt
                self.accel.clear_buffer()
                # self.adp_off()
                self.accel.output_data_rate = ODR_SETTINGS[self.odr]
                self.accel.initialize(
                    settings="INT_SETTINGS", g_range=self.g_range)

                accel_triggered_rec = False
                if not microphone_has_triggered:
                    accel_triggered_rec = True
                    # print(' >-Vibration Trigger-< '.center(columns, 'V'))
                    ts = dt.now()
                    loop.call_soon_threadsafe(
                        msg_out_queue.put_nowait, (ACCEL_RECORD_EVENT_MSG, ts))
                # print(' < Accel Rec. Started > '.center(columns, '*'))
                metadata = [ts, MIC_TRIGGER_NAME
                            if microphone_has_triggered
                            else ACCEL_TRIGGER_NAME]

                write_idx = 0
                while True:
                    # if data ready interrupt
                    if GPIO.input(dataReadyPin) == 1:
                        while not msg_in_queue.empty():
                            msg, ts = msg_in_queue.get_nowait()
                            if msg == MIC_RECORD_STOP_EVENT_MSG:
                                break
                        if (not accel_triggered_rec and
                                msg == MIC_RECORD_STOP_EVENT_MSG):
                            stop_frame = int(ts * self.odr)
                        elif (accel_triggered_rec and
                              write_idx >= self.clip_duration * self.odr and
                              write_idx % self.odr == 0 and
                                self.bts_threshold_trigger(
                                    # last second of accel. data
                                    buffer[
                                        write_idx-self.odr:
                                        write_idx:
                                        10])):
                            loop.call_soon_threadsafe(
                                msg_out_queue.put_nowait,
                                (ACCEL_RECORD_STOP_EVENT_MSG,
                                 write_idx / self.odr))
                            break
                        if write_idx >= stop_frame:
                            break

                        self.accel.get_accel_data()
                        buffer[write_idx][:] = [
                            self.accel.accel.x,
                            self.accel.accel.y,
                            self.accel.accel.z]
                        write_idx += 1
                        self.accel.clear_interrupt()
                    # time.sleep(self.read_delay)  # limits achievable ODR

                # print(' > Accel Rec. Stopped < '.center(columns, '-'))

                buffer_copy = buffer.copy()[:write_idx]
                loop.call_soon_threadsafe(
                    self.save_data_to_file, buffer_copy, metadata)

                buffer[:] = 0
                # set up accelerometer to trigger mode
                self.accel.clear_buffer()
                self.accel.initialize(
                    settings="WAKE_UP_TRIGGER",
                    wake_up_threshold=self.wake_up_g_threshold,
                    g_range=self.g_range)

            # default wake up engine rate @ 50hz. 1/rate
            time.sleep(.2)

    def save_data_to_file(self, buffer: np.array, metadata: list[dt, str]
                          ) -> None:
        ts, trigger = metadata
        time_str = ts.strftime(TME_STMP_FORMAT)
        trigger_suffix = f'_trig_{trigger}' if trigger else ''
        if not os.path.exists(self.save_folder):
            os.mkdir(self.save_folder)
        fn = f'{self.save_folder}/accel_{time_str}{trigger_suffix}'
        np.save(file=fn, arr=buffer, fix_imports=False)
        # print(f'accel file saved to dir: {self.save_folder}')

    def bts_threshold_trigger(self, buffer: np.array) -> bool:
        """Back to Sleep threshold trigger.

        Look for condition of low accelerometer change for back to sleep
        function.

        Returns:
            bool: if the back to sleep condition is met
        """
        debounce, debounce_limit = 0, 50
        x_last, y_last, z_last = buffer[0][0], buffer[0][1], buffer[0][2]

        for x, y, z in buffer[1:]:
            if (
                    abs(x - x_last) < self.bts_threshold and
                    abs(y - y_last) < self.bts_threshold and
                    abs(z - z_last) < self.bts_threshold
            ):
                debounce += 1
            else:
                debounce -= 1 if debounce else 0

            if debounce == debounce_limit:
                return True
            x_last, y_last, z_last = x, y, z

        return False

    def cleanup(self) -> None:
        GPIO.cleanup()


if __name__ == '__main__':
    try:
        recorder = AccelerometerClipRecorder(
            wake_up_g_threshold=0.018)
        print(" Sensor Active!! ".center(50, '*'))
        recorder.threshold_recorder()
        print(" Done!! ".center(50, '*'))
    except (KeyboardInterrupt, SystemExit):
        print("\nExiting!")
        sys.exit(0)
    finally:
        GPIO.cleanup()
