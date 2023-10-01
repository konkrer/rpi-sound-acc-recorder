from scipy.fft import rfftfreq
import numpy as np

from src.lib.accel_detect import analyze_fft, get_max_data
from analysis.detect_freq_plot import plot_freq_data


ODR = 400
CLIP_DATA_DURATION = 1

FOLDER_NAME = 'kizashi_long'
BASE_PATH = f'analysis/data/{FOLDER_NAME}'
SAVE_FOLDER = 'plots'


def analyze_file(fname: str) -> None:
    raw_accel_data = np.load(
        f'{BASE_PATH}/accelerometer_data/{fname}')

    # use full clip length
    freq_data = analyze_fft(raw_accel_data, ODR)
    data_map = rfftfreq(len(raw_accel_data), 1 / ODR)
    max_data = get_max_data(np.abs(freq_data))

    # Break up long clip into clips with CLIP_DATA_DURATION length.
    # clip_end_idx = CLIP_DATA_DURATION * ODR
    # data_map = rfftfreq(ODR, 1 / ODR)
    # for idx in range(int(len(raw_accel_data) / ODR)):
    #     cropped_data = raw_accel_data[ODR *
    #                                   idx: (idx+1) * ODR]
    #     freq_data = analyze_fft(cropped_data)
    #     plot_freq_data(freq_data, data_map, f'{fname[:-4]}_-_{idx}')

    # plot data
    plot_freq_data(freq_data, data_map, fname[:-4], max_data)
