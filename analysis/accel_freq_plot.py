from scipy.fft import rfftfreq
import numpy as np
import matplotlib.pyplot as plt
from os import listdir, makedirs
from os.path import isfile, join, exists

from src.lib.accel_detect import detect_freq_spike, analyze_fft, get_max_data

# plt.style.use('seaborn-v0_8-poster')

ODR = 400
CLIP_DATA_DURATION = 1

FOLDER_NAME = 'kizashi_3/h_a_detect'
BASE_PATH = f'analysis/data/{FOLDER_NAME}'
SAVE_FOLDER = 'plots'


def analyze_file(fname: str, spike_detect: bool = False,
                 dice_clip: bool = False) -> None:
    raw_accel_data = np.load(
        f'{BASE_PATH}/accelerometer_data/{fname}')

    if dice_clip is False or spike_detect is True:
        # analyze full clip length
        freq_data = analyze_fft(raw_accel_data, ODR)
        max_data = get_max_data(np.abs(freq_data))
        data_map = rfftfreq(len(raw_accel_data), 1 / ODR)

    spike = ''
    if spike_detect:
        # detect freq spike and alter filename of spike samples
        spike = 'spike_' if detect_freq_spike(
            raw_accel_data, ODR, max_data=max_data) else ''
        spike = f'{spike}h_a_' if spike and len(
            raw_accel_data) / ODR == 4 else spike

    # plot data
    if dice_clip is False:
        plot_freq_data(freq_data, data_map, f'{spike}{fname[:-4]}', max_data)

    # Alternate plotting
    else:
        # Break up clip into segments with CLIP_DATA_DURATION length and plot.
        clip_end_idx = CLIP_DATA_DURATION * ODR
        data_map = rfftfreq(clip_end_idx, 1 / ODR)
        for idx in range(int(len(raw_accel_data) / clip_end_idx)):
            cropped_data = raw_accel_data[clip_end_idx *
                                          idx: (idx+1) * clip_end_idx]
            freq_data = analyze_fft(cropped_data)
            plot_freq_data(freq_data, data_map, f'{spike}{fname[:-4]}_-_{idx}')


def plot_freq_data(freq_data: np.array, data_map: np.array, fname: str,
                   max_data: np.array) -> None:
    plt.figure(figsize=(18, 6))

    for idx, axis in enumerate(freq_data):
        plt.subplot(141 + idx)
        plt.stem(data_map, axis, markerfmt=" ", basefmt=" ")

        # bar plot
        #
        # binned = [np.mean(x[y:y+10]) for y in range(1, 201, 10)]
        # plt.bar(range(5, 205, 10), binned, width=10)
        # # plt.xlim(.8, 2)

    plt.subplot(144)
    plt.stem(data_map, max_data, markerfmt=" ", basefmt=" ")

    # bar plot
    #
    # binned = [np.mean(max_data[y:y+4]) for y in range(1, 201, 4)
    # plt.bar(range(5, 205, 4), binned, width=4)
    # plt.hist(max_data, 20,)
    # plt.xlim(50, 200)

    plt.savefig(fname=f'{BASE_PATH}/{SAVE_FOLDER}/{fname}')
    plt.close()


def main() -> None:
    save_path = f'{BASE_PATH}/{SAVE_FOLDER}'
    if not exists(save_path):
        makedirs(save_path)

    files = [f for f in listdir(
        f'{BASE_PATH}/accelerometer_data/'
    ) if isfile(join(f'{BASE_PATH}/accelerometer_data/', f))]

    npy_filenames = [f for f in files if f.endswith('.npy')]

    for f in npy_filenames:
        analyze_file(f)

    # analyze_file('ac2.npy')
