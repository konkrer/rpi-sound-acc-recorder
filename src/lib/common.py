"""Common application settings.

    To avoid repetition common variable settings should be set here.
    """

# used in file name for accelerometer and sound clip files
TME_STMP_FORMAT = r"%H_%M_%S__%d_%m_%y"

# used in file name for indicating input that triggered.
ACCEL_TRIGGER_NAME = 'accel'
MIC_TRIGGER_NAME = 'mic'


# EVENT MESSAGES PASSED BETWEEN ACCEL/ MIC RECORDERS
MIC_RECORD_EVENT_MSG = 'mic_rec_event'
MIC_RECORD_STOP_EVENT_MSG = 'mic_stop_rec_event'
ACCEL_RECORD_EVENT_MSG = 'accel_rec_event'
ACCEL_RECORD_STOP_EVENT_MSG = 'accel_stop_rec_event'
