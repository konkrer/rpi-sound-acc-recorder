from scipy.fft import rfft, rfftfreq
import numpy as np
import matplotlib.pyplot as plt
import pdb
assert pdb

# plt.style.use('seaborn-v0_8-poster')

ODR = 800
CLIP_DATA_DURATION = 4


# car
# folder = 'car_roof_rack'
# fn = 'accel_13_43_09__25_02_23_trig_accel'
# fn = 'accel_13_43_33__25_02_23_trig_accel'
# pickup
# folder = 'pickup1'
# fn = 'accel_16_55_49__01_03_23_trig_accel'
# fn = 'accel_16_53_08__01_03_23_trig_accel'
# folder = 'pickup_on_engine'
# fn = 'accel_19_36_39__17_03_23_trig_accel'
folder = 'accel_test'
fn = 'still_9_7_23'


def remove_offset(data: np.array) -> np.array:
    pass


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


def main() -> None:
    raw_accel_data = np.load(
        f'analysis/data/{folder}/accelerometer_data/{fn}.npy')
    clip_end_idx = CLIP_DATA_DURATION * ODR
    cropped_data = raw_accel_data[:clip_end_idx]

    # pdb.set_trace()

    # accel_data = remove_offset(raw_accel_data)

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

    plt.show()


if __name__ == '__main__':

    main()
