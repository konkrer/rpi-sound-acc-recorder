from scipy.fft import rfft, rfftfreq
import numpy as np
import matplotlib.pyplot as plt
from os import listdir, makedirs
from os.path import isfile, join, exists

# plt.style.use('seaborn-v0_8-poster')

ODR = 400
CLIP_DATA_DURATION = 2

FOLDER_NAME = 'kizashi'
BASE_PATH = f'analysis/data/{FOLDER_NAME}'
SAVE_FOLDER = 'plots'


def analyze_fft(data: np.array) -> np.array:
    reshaped_data = data.transpose()
    x = rfft(reshaped_data[0])
    x[0:6] = 0
    y = rfft(reshaped_data[1])
    y[0:6] = 0
    z = rfft(reshaped_data[2])
    z[0:6] = 0
    return x, y, z


def get_max_data(fft_data: np.array) -> np.array:
    # pdb.set_trace()
    reshaped_data = fft_data.transpose()
    out = np.empty(len(reshaped_data))
    for idx, data in enumerate(reshaped_data):
        max_ = np.max(data)
        out[idx] = max_
    return out


def analyze_file(fname: str) -> None:
    raw_accel_data = np.load(
        f'{BASE_PATH}/accelerometer_data/{fname}')
    clip_end_idx = CLIP_DATA_DURATION * ODR
    cropped_data = raw_accel_data[:clip_end_idx]

    x, y, z = analyze_fft(cropped_data)
    data_map = rfftfreq(clip_end_idx, 1 / ODR)

    all_data = np.array([x, y, z])
    max_data = get_max_data(np.abs(all_data))

    # plot data

    plt.figure(figsize=(18, 6))

    plt.subplot(141)
    plt.stem(data_map, np.abs(x), markerfmt=" ", basefmt=" ")
    # plt.xlim(.8, 2)

    plt.subplot(142)
    plt.stem(data_map, np.abs(y), markerfmt=" ", basefmt=" ")
    # plt.xlim(1, None)

    plt.subplot(143)
    plt.stem(data_map, np.abs(z), markerfmt=" ", basefmt=" ")
    # plt.xlim(0, 5)

    plt.subplot(144)
    plt.stem(data_map, max_data, markerfmt=" ", basefmt=" ")
    # plt.xlim(0, 100)

    plt.savefig(fname=f'{BASE_PATH}/{SAVE_FOLDER}/{fname[:-4]}')
    # plt.show()
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


if __name__ == '__main__':

    main()
