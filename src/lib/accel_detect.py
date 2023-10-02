"""Accelerometer frequency analysis for detection of events
  with identifiable vibration characteristics.

  """

import numpy as np
from scipy.fft import rfft


def analyze_fft(data: np.array, odr: int) -> np.array:
    reshaped_data = data.transpose()
    x = rfft(reshaped_data[0])
    y = rfft(reshaped_data[1])
    z = rfft(reshaped_data[2])

    # remove low freq noise
    hpf = 10  # Hz
    clip_len = int(len(data) / odr)
    hpf_idx = hpf * clip_len
    x[0:hpf_idx], y[0:hpf_idx], z[0:hpf_idx] = 0, 0, 0

    return np.abs(np.array([x, y, z]))


def get_max_data(fft_data: np.array) -> np.array:
    # pdb.set_trace()
    reshaped_data = fft_data.transpose()
    out = np.empty(len(reshaped_data))
    for idx, data in enumerate(reshaped_data):
        max_ = np.max(data)
        out[idx] = max_
    return out


def detect_freq_spike(
        buffer: np.array,
        odr: int,
        max_data: np.array or None = None,
        freq_1: int = 100,
        freq_1_bandwidth: int = 50,
        threshold_1: int or float = 2.125,
        freq_2: int or None = None,  # 175,
        freq_2_bandwidth: int = 50,
        threshold_2: int or float = 2
) -> bool:
    clip_length = len(buffer) / odr  # seconds
    threshold_1 = threshold_1 * clip_length
    threshold_2 = threshold_2 * clip_length

    #  analyze fft
    if max_data is None:
        freq_data = analyze_fft(buffer, odr)
        max_data = get_max_data(np.abs(freq_data))

    #  check freq_1
    half_bandwidth = freq_1_bandwidth / 2
    freq_1_band_start = freq_1 - half_bandwidth
    freq_1_band_end = freq_1 + half_bandwidth

    start_idx = int(freq_1_band_start * clip_length)
    end_idx = int(freq_1_band_end * clip_length)
    band = max_data[start_idx: end_idx]

    for freq_level in band:
        if freq_level > threshold_1:
            if not freq_2:
                return True
            else:
                break

    #  check freq_2
    if freq_2:
        half_bandwidth = freq_2_bandwidth / 2
        freq_2_band_start = freq_2 - half_bandwidth
        freq_2_band_end = freq_2 + half_bandwidth

        start_idx = int(freq_2_band_start * clip_length)
        end_idx = int(freq_2_band_end * clip_length)
        band = max_data[start_idx: end_idx]

        for freq_level in band:
            if freq_level > threshold_2:
                return True

    return False
