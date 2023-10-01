# -----------------------------------------------------------------------------
# qwiic_kx13x.py
#
# Python library for the SparkFun qwiic KX13X sensor.
#
# ==================================================================================
# Copyright (c) 2019 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# =================================================================================
#
#

"""
qwiic_kx13x
============
Python module for the qwiic kx132/4 accelerometers.
This python package is a port of the existing
[KX13X Arduino](https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library)
This package can be used in conjunction with the overall
[SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)
New to qwiic? Take a look at the entire
[SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).
"""

# -----------------------------------------------------------------------------
# from __future__ import print_function
import qwiic_i2c
from collections import namedtuple
from lib.kx_initializations import Kx13xInitializations

# Wake up from sleep G threshold if one is not explicitly passed in class
# initialization.
WAKE_UP_THRESHOLD = 0.5


# Define the device name and I2C addresses. These are set in the class
# definition as class variables, making them available without having to
# create a class instance. This allows higher level logic to rapidly
# create a index of qwiic devices at runtime.
#
# The name of this device
_DEFAULT_NAME = "Qwiic KX13X"

# Some devices have multiple available addresses - this is a list of these
# addresses.
# NOTE: The first address in this list is considered the default I2C address
# for the device.
_AVAILABLE_I2C_ADDRESS = [0x1F, 0x1E]

# Default Setting Values

# Part ID identifying KX132 and KX134 respectively
_WHO_AM_I = [0x3D, 0x46]

# define the class that encapsulates the device being created. All information
# associated with this device is encapsulated by this class. The device class
# should be the only value exported from this module.


class QwiicKX13XCore(object):
    """
    QwiicKX13XCore
        :param address: The I2C address to use for the device.
                        If not provided, the default address is used.
        :param i2c_driver: An existing i2c driver object. If not provided
                        a driver object is created.
        :return: The KX13X device object.
        :return type: Object
    """
    # Constructor
    device_name = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    TOTAL_ACCEL_DATA_16BIT = 6
    TOTAL_ACCEL_DATA_8BIT = 3
    MAX_BUFFER_LENGTH = 32

    XLSB = 0
    XMSB = 1
    YLSB = 2
    YMSB = 3
    ZLSB = 4
    ZMSB = 5

    DEFAULT_SETTINGS = 0xC0
    INT_SETTINGS = 0xE0
    SOFT_INT_SETTINGS = 0xE1
    BUFFER_SETTINGS = 0xE2
    TILT_SETTINGS = 0xE3

    COTR_DEF_STATE = 0x55
    COTR_POS_STATE = 0xAA

    BUFFER_16BIT_SAMPLES = 0x01
    BUFFER_8BIT_SAMPLES = 0x00
    BUFFER_MODE_FIFO = 0x00
    BUFFER_MODE_STREAM = 0x01
    BUFFER_MODE_TRIGGER = 0x02

    # Register names for the KX13X

    KX13X_MAN_ID = 0x00
    KX13X_PART_ID = 0x01
    KX13X_XADP_L = 0x02
    KX13X_XADP_H = 0x03
    KX13X_YADP_L = 0x04
    KX13X_YADP_H = 0x05
    KX13X_ZADP_L = 0x06
    KX13X_ZADP_H = 0x07
    KX13X_XOUT_L = 0x08
    KX13X_XOUT_H = 0x09
    KX13X_YOUT_L = 0x0A
    KX13X_YOUT_H = 0x0B
    KX13X_ZOUT_L = 0x0C
    KX13X_ZOUT_H = 0x0D
    # 0x0E - 0x11 Reserved
    KX13X_COTR = 0x12
    KX13X_WHO_AM_I = 0x13
    KXI3X_TSCP = 0x14
    KX13X_TSPP = 0x15
    KX13X_INS1 = 0x16
    KX13X_INS2 = 0x17
    KX13X_INS3 = 0x18
    KX13X_STATUS_REG = 0x19
    KX13X_INT_REL = 0x1A
    KX13X_CNTL1 = 0x1B
    KX13X_CNTL2 = 0x1C
    KX13X_CNTL3 = 0x1D
    KX13X_CNTL4 = 0x1E
    KX13X_CNTL5 = 0x1F
    KX13X_CNTL6 = 0x20
    KX13X_ODCNTL = 0x21
    KX13X_INC1 = 0x22
    KX13X_INC2 = 0x23
    KX13X_INC3 = 0x24
    KX13X_INC4 = 0x25
    KX13X_INC5 = 0x26
    KX13X_INC6 = 0x27
    # 0x28 Reserved
    KX13X_TILT_TIMER = 0x29
    KX13X_TDTRC = 0x2A
    KX13X_TDTC = 0x2B
    KX13X_TTH = 0x2C
    KX13X_TTL = 0x2D
    KX13X_FTD = 0x2E
    KX13X_STD = 0x2F
    KX13X_TLT = 0x30
    KX13X_TWS = 0x31
    KX13X_FFTH = 0x32
    KX13X_FFC = 0x33
    KX13X_FFCNTL = 0x34
    # 0x35 - 0x36 Reserved
    KX13X_TILT_ANGLE_LL = 0x37
    KX13X_TILT_ANGLE_HL = 0x38
    KX13X_HYST_SET = 0x39
    KX13X_LP_CNTL1 = 0x3A
    KX13X_LP_CNTL2 = 0x3B
    # 0x3C - 0x48 Reserved
    KX13X_WUFTH = 0x49
    KX13X_BTSWUFTH = 0x4A
    KX13X_BTSTH = 0x4B
    KX13X_BTSC = 0x4C
    KX13X_WUFC = 0x4D
    # 0x4E - 0x5C Reserved
    KX13X_SELF_TEST = 0x5D
    KX13X_BUF_CNTL1 = 0x5E
    KX13X_BUF_CNTL2 = 0x5F
    KX14X_BUF_STATUS_1 = 0x60
    KX13X_BUF_STATUS_2 = 0x61
    KX13X_BUF_CLEAR = 0x62
    KX13X_BUF_READ = 0x63
    KX13X_ADP_CNTL1 = 0x64
    KX13X_ADP_CNTL2 = 0x65
    KX13X_ADP_CNTL3 = 0x66
    KX13X_ADP_CNTL4 = 0x67
    KX13X_ADP_CNTL5 = 0x68
    KX13X_ADP_CNTL6 = 0x69
    KX13X_ADP_CNTL7 = 0x6A
    KX13X_ADP_CNTL8 = 0x6B
    KX13X_ADP_CNTL9 = 0x6C
    KX13X_ADP_CNTL10 = 0x6D
    KX13X_ADP_CNTL11 = 0x6E
    KX13X_ADP_CNTL12 = 0x6F
    KX13X_ADP_CNTL13 = 0x70
    KX13X_ADP_CNTL14 = 0x71
    KX13X_ADP_CNTL15 = 0x72
    KX13X_ADP_CNTL16 = 0x73
    KX13X_ADP_CNTL17 = 0x74
    KX13X_ADP_CNTL18 = 0x75
    KX13X_ADP_CNTL19 = 0x76
    # Reserved 0x77 - 0x7F

    KX13X_SUCCESS = 0x00
    KX13X_GENERAL_ERROR = 0x01
    KX13X_I2C_ERROR = 0x02

    # HARDWARE_INTERRUPTS
    HI_TILT_POSITION = 0x01
    HI_WAKE_UP = 0x02
    HI_TAP_DOUBLE_TAP = 0x04
    HI_BACK_TO_SLEEP = 0x08
    HI_DATA_READY = 0x10
    HI_WATERMARK = 0x20
    HI_BUFFER_FULL = 0x40
    HI_FREEFALL = 0x80

    def __init__(self, address=None, i2c_driver=None,
                 wake_up_threshold: float or int = WAKE_UP_THRESHOLD
                 ):
        self.raw_output_data = namedtuple('raw_output_data', 'x y z')
        self.address = \
            self.available_addresses[0] if address is None else address
        # default read registry and bit depth
        self.data_read_registry = self.KX13X_XOUT_L
        self.bit_depth = 16
        self.wake_up_threshold = wake_up_threshold

        # load the I2C driver if one isn't provided
        if i2c_driver is None:
            self._i2c = qwiic_i2c.getI2CDriver()
            if self._i2c is None:
                print("Unable to load I2C driver for this platform.")
                return
        else:
            self._i2c = i2c_driver

        if not self.is_connected():
            raise IOError(
                "The Qwiic KX13X Accelerometer device isn't connected" +
                " to the system. Please check your connection.")

    def is_connected(self):
        """
            Determine if a KX13X device is connected to the system..
            :return: True if the device is connected, otherwise False.
            :rtype: bool
        """
        return qwiic_i2c.isDeviceConnected(self.address)

    connected = property(is_connected)

    def beginCore(self):
        """
            Initialize the operation of the KX13X module
            :return: Returns true if the initialization was successful,
             otherwise False.
            :rtype: bool
        """
        # are we who we need to be?
        chipID = self._i2c.readByte(self.address, self.KX13X_WHO_AM_I)
        if chipID not in _WHO_AM_I:
            print("Invalid Chip ID: 0x" % chipID)

        return chipID

    @classmethod
    def initialization_choices(cls) -> list[str]:
        return ['DEFAULT_SETTINGS', 'INT_SETTINGS', 'SOFT_INT_SETTINGS',
                'BUFFER_SETTINGS_1', 'WAKE_UP_TRIGGER', 'ADP', 'ADP_OFF']

    def initialize(self, settings: str = 'DEFAULT_SETTINGS',
                   wake_up_threshold: None or float or int = None,
                   g_range: None or int = None,
                   ) -> None:
        """
            Initialize configures the accelerometer's registers into a number
            of different modes: asynchronous, hardware trigger, software
            trigger, buffer, wake up from sleep trigger, and
            advanced data path.
        """
        self.set_standby_mode()
        choices = self.initialization_choices()

        # DEFAULT_SETTINGS (asynchronous)
        if settings == choices[0]:
            Kx13xInitializations.asynchronous(self)
        # INT_SETTINGS (hardware interrupt)
        elif settings == choices[1]:
            Kx13xInitializations.hard_interrupt(self)
        # SOFT_INT_SETTINGS (software interrupt)
        elif settings == choices[2]:
            Kx13xInitializations.soft_interrupt(self)
        # BUFFER_SETTINGS_1
        elif settings == choices[3]:
            Kx13xInitializations.buffer_settings_1(self)
        # WAKE_UP_TRIGGER
        elif settings == choices[4]:
            Kx13xInitializations.wake_up_trigger(self, wake_up_threshold)
        # ADP (advanced data path)
        elif settings == choices[5]:
            Kx13xInitializations.adp_enable(self)
        # ADP OFF
        elif settings == choices[6]:
            Kx13xInitializations.adp_disable(self)
            return
        else:
            choices = self.initialization_choices()
            arg_err(settings, [str],
                    f'settings must be a str in {choices}')

        # if g_range given overwrite setting from default initializations.
        if g_range:
            self.set_range(g_range)

    def write_to_CNTL4(self,
                       C_MODE: 0 or 1 or None = None,
                       TH_MODE: 0 or 1 or None = None,
                       WUFE: 0 or 1 or None = None,
                       BTSE: 0 or 1 or None = None,
                       PR_MODE: 0 or 1 or None = None,
                       OBTS: int or None = None
                       ) -> None:
        """Write settings to CNTL4 register.

        Note: PC1 bit in CNTL1 must be set to 0 before calling this function.
              This function is designed to be called as part of a setup
              procedure.

        Args:
            C_MODE (0 or 1 or None, optional): debounce counter clear mode.
                Defaults to None.
            TH_MODE (0 or 1 or None, optional): wake / back-to-sleep threshold
                mode. Defaults to None.
            WUFE (0 or 1 or None, optional): Wake-Up Function Engine enable.
                Defaults to None.
            BTSE (0 or 1 or None, optional): Back-to-Sleep Engine enable.
                Defaults to None.
            PR_MODE (0 or 1 or None, optional): Pulse Reject mode.
                Defaults to None.
            OBTS (int or None, optional): sets the output data rate
                at which the back-to-sleep (motion detection) performs its
                function during wake state. Setting must be between 0 and 8.
                Defaults to None.
        """
        # if not setting new values use original settings
        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL4)

        if C_MODE is not None:
            if C_MODE not in [0, 1]:
                arg_err(C_MODE, [int], 'C_MODE must be one of "0, 1".')
            C_MODE = C_MODE << 7
            reg_val &= 0b01111111
            reg_val |= C_MODE

        if TH_MODE is not None:
            if TH_MODE not in [0, 1]:
                arg_err(
                    TH_MODE, [int], 'TH_MODE must be one of "0, 1".')
            TH_MODE = TH_MODE << 6
            reg_val &= 0b10111111
            reg_val |= TH_MODE

        if WUFE is not None:
            if WUFE not in [0, 1]:
                arg_err(WUFE, [int], 'WUFE must be one of "0, 1".')
            WUFE = WUFE << 5
            reg_val &= 0b11011111
            reg_val |= WUFE

        if BTSE is not None:
            if BTSE not in [0, 1]:
                arg_err(BTSE, [int], 'BTSE must be one of "0, 1".')
            BTSE = BTSE << 4
            reg_val &= 0b11101111
            reg_val |= BTSE

        if PR_MODE is not None:
            if PR_MODE not in [0, 1]:
                arg_err(
                    PR_MODE, [int], 'PR_MODE must be one of "0, 1".')
            PR_MODE = PR_MODE << 3
            reg_val &= 0b11110111
            reg_val |= PR_MODE

        if OBTS is not None:
            if OBTS not in list(range(8)):
                arg_err(OBTS, [int],
                        'OBTS must be between 0 and 7 inclusive.')
            reg_val &= 0b11111000
            reg_val |= OBTS

        self._i2c.writeByte(self.address, self.KX13X_CNTL4, reg_val)

    def set_wake_from_sleep_g_threshold(
            self, g_threshold: float or int = 0.5) -> None:
        # check input
        if not 0 < g_threshold < 8:
            arg_err(
                g_threshold, [float, int],
                'g_threshold must be number between 0 and 8 exclusive.')

        # convert g to setting
        setting = int(g_threshold * 256)

        # split bits
        # first 8 bits
        wufth_bits = setting & 0xFF
        # last 3 bits
        btswufth_bits = setting >> 8
        # Write to Wakeup Function Threshold(WUFTH)
        self._i2c.writeByte(self.address, self.KX13X_WUFTH, wufth_bits)

        # Write to Back to Sleep Wakeup Function Threshold(BTSWUFTH)
        reg_val = self._i2c.readByte(self.address, self.KX13X_BTSWUFTH)
        reg_val &= 0xF8
        reg_val |= btswufth_bits
        self._i2c.writeByte(self.address, self.KX13X_BTSWUFTH, reg_val)

    def put_to_sleep(self) -> None:
        # Write 0x01 to Control 5 (CNTL5) to put the sensor into sleep mode
        # (MAN_SLEEP=1).
        self._i2c.writeByte(self.address, self.KX13X_CNTL5, 0X01)

    def run_command_test(self):
        """
            This function runs the self test built into the accelerometer.
            :return: Returns true upon successful test, and false otherwise.
            :rtype: boo
        """
        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL2)
        reg_val &= 0xBF
        reg_val |= (1 << 6)
        self._i2c.writeByte(self.address, self.KX13X_CNTL2, reg_val)

        reg_val = self._i2c.readByte(self.address, self.KX13X_COTR)
        if reg_val == self.COTR_POS_STATE:
            return True
        else:
            return False

    def accel_control(self, enable):
        """
            This functions controls the accelerometers power on and off state.
            :param enable: True or false indicating power on or off
            respectively. 
            :return: Returns false when an incorrect argument has been passed.
            :rtype: bool
        """
        if enable not in [True, False, 0, 1]:
            arg_err(enable, [bool, int],
                    'enable must be one of "True, False, 0, 1".')

        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        reg_val &= 0x7F
        reg_val |= (enable << 7)
        self._i2c.writeByte(self.address, self.KX13X_CNTL1, reg_val)

    def get_accel_state(self):
        """
            Retrieves the state of the accelerometer: on or off.
            :return: Returns bit indicating the accelerometers power state.
            :rtype: bool
        """
        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        return (reg_val & 0x80) >> 7

    def set_range(self, kx13x_range: int = 0):
        """
            Sets the range reported by the accelerometer. For the KX132, the
            range is from 2G - 16G and for the KX134 it's 8G - 32G.
            :param kx13x_range: Eight constants (four per version) represent
             values from zero to four indicating the range to be set:
                KX132_RANGE2G,
                KX132_RANGE4G,
                KX132_RANGE8G,
                KX132_RANGE16G
                KX134_RANGE8G,
                KX134_RANGE16G,
                KX134_RANGE32G,
                KX134_RANGE64G.
            :return: Returns false if an incorrect argument is given.
            :rtype: bool
        """

        if kx13x_range not in list(range(4)):
            arg_err(
                kx13x_range, [int],
                "kx13x_range must be integer between 0 to 3 inclusive.")

        accel_state = self.get_accel_state()
        self.accel_control(False)

        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        reg_val &= 0xE7
        reg_val |= (kx13x_range << 3)
        self._i2c.writeByte(self.address, self.KX13X_CNTL1, reg_val)
        self.accel_control(accel_state)

    def set_output_data_rate(self, rate=0x06):
        """
            Sets the ODR - the rate at which the accelerometer outputs data.
            :param rate: A value from zero to fifteen indicating which rate to
            set. 0x60 is default of 50hz.
            :return: Returns false if an an incorrect argument is given.
            :rtype: bool

        """
        if rate not in list(range(16)):
            arg_err(
                rate, [int],
                "rate must be integer in range 0 to 15 inclusive.")

        accel_state = self.get_accel_state()
        self.accel_control(False)

        reg_val = self._i2c.readByte(self.address, self.KX13X_ODCNTL)
        reg_val &= 0xF0
        reg_val |= rate
        self._i2c.writeByte(self.address, self.KX13X_ODCNTL, reg_val)
        self.accel_control(accel_state)

    def get_output_data_rate(self):
        """
            Gets the accelerometers output data rate.
            :return: Accelerometer's data rate in hertz.
            :rtype: float

        """
        reg_val = self._i2c.readByte(self.address, self.KX13X_ODCNTL)
        reg_val &= 0x0F
        return 0.78125 * 2 ** (reg_val)

    output_data_rate = property(get_output_data_rate, set_output_data_rate)

    def set_interrupt_pin(self, enable: bool or int, polarity: int = 1,
                          pulse_width: int = 0, latch_control: bool = True):
        """
            Sets all of whether the data ready bit is reported to the hardware
            interrupt pin, the polarity of the signal (HIGH or LOW), the width
            of the pulse, and how the interrupt is cleared.
            :param enable: Sets hardware interrupt to "on" or "off".
            :param polarity: Sets the active state of the hardware pin - HIGH
            or LOW.
            :param pulse_width: Sets the width of the interrupt pulse.
            :param latch_control: Sets how the interrupt pin is cleared.
            :return: Returns false if an an incorrect argument is given.
            :rtype: bool

        """
        if enable not in [True, False, 0, 1]:
            arg_err(enable, [bool, int],
                    'enable must be one of "True, False, 0, 1".')
        if polarity not in [0, 1]:
            arg_err(polarity, [int],
                    'polarity must be one of "0, 1".')
        if pulse_width not in [0, 1, 2, 3]:
            arg_err(pulse_width, [int],
                    'pulse_width must be one of "0, 1, 2, 3".')
        if not isinstance(latch_control, bool):
            arg_err(latch_control, [bool],
                    'latch_control must be of type bool')
        # latch bit setting:
        # 0 – latched until cleared by reading INT_REL
        # 1 – pulsed. The pulse width is configurable by PW1
        latch_control = 1 if latch_control is False else 0

        accel_state = self.get_accel_state()
        self.accel_control(False)

        combined_arguments = (pulse_width << 6) | (
            enable << 5) | (polarity << 4) | (latch_control << 3)

        reg_val = self._i2c.readByte(self.address, self.KX13X_INC1)
        reg_val &= 0x07
        reg_val |= combined_arguments
        self._i2c.writeByte(self.address, self.KX13X_INC1, reg_val)
        self.accel_control(accel_state)

    def route_hardware_interrupt(self, rdr: int, pin: 1 or 2 = 1):
        """
            Determines which interrupt is reported: freefall, buffer full,
            watermark, data ready, back to sleep, tap/double tap, wakeup or
            tilt. Also which hardware pin its reported on: one or two.
            :param rdr: The interrupt to be reported.
            :param pin: The hardware pin on which the interrupt is reported.
            :return: None
            :rtype: bool

        """
        rdr_choices = [0b00000000, 0b00000001, 0b00000010, 0b00000100,
                       0b00001000, 0b00010000, 0b00100000, 0b01000000,
                       0b10000000]
        if rdr not in rdr_choices:
            arg_err(rdr, [int],
                    f'rdr must be one of {rdr_choices} .')
        if pin not in [1, 2]:
            arg_err(pin, [int], 'pin must be one of "1 or 2".')

        accel_state = self.get_accel_state()
        self.accel_control(False)

        if pin == 1:
            self._i2c.writeByte(self.address, self.KX13X_INC4, rdr)
        else:
            self._i2c.writeByte(self.address, self.KX13X_INC6, rdr)

        self.accel_control(accel_state)

    def clear_interrupt(self):
        """
            Clears the interrupt.
            :return: No return value.

        """
        self._i2c.readByte(self.address, self.KX13X_INT_REL)

    def clear_buffer(self):
        """Writes to BUFF_CLEAR. Latched buffer status information and the
        entire sample buffer are cleared when any data is written to this
        register.

        This causes the sample level bits SMP_LEV[9:0] to be cleared in BUF
        _STATUS_1 and BUF_STATUS_2 registers. In addition, if the sample
        buffer is set to Trigger mode, the BUF_TRIG bit in BUF_STATUS_2 is
        cleared too. Finally, the BFI and WMI bits in INS2 will be cleared
        and physical interrupt latched pin will be changed to its inactive
        state. This register is On-The-Fly (OTF) register and can be written
        to while the KX132-1211 is enabled (PC1 bit in CNTL1 register
        is set to “1”) and the change will be accepted with no interruption
        in the operation.
        """
        self._i2c.writeByte(self.address, self.KX13X_BUF_CLEAR, 0X00)

    def data_trigger(self):
        """
            Reads the register indicating whether data is ready to be read.
            :return: Returns true if data is ready to be read and false
            otherwise.
            :rtype: bool

        """
        reg_val = self._i2c.readByte(self.address, self.KX13X_INS2)
        if reg_val & 0x10:
            return True
        else:
            return False

    def set_buffer_threshold(self, threshold):
        """
            Sets how many samples are stored in the buffer.
            :param threshold: The number of samples to be stored.
            :return: Returns false if an incorrect argument is given.
            :rtype: bool
        """
        if threshold not in list(range(2, 172)):
            arg_err(
                threshold, [int],
                'threshold must be int between 2 and 171 inclusive.')

        resolution = self._i2c.readByte(self.address, self.KX13X_BUF_CNTL2)
        resolution &= 0x40
        resolution = resolution >> 6

        if threshold > 86 and resolution == 1:
            # At 16bit resolution - max samples: 86
            threshold == 86

        self._i2c.writeByte(self.address, self.KX13X_BUF_CNTL1, threshold)

    def set_buffer_operation(self, operation_mode, resolution):
        """
            Sets the mode and resolution of the samples stored in the buffer.
            :param operation_mode: Sets the mode:
                                   BUFFER_MODE_FIFO
                                   BUFFER_MODE_STREAM
                                   BUFFER_MODE_TRIGGER
            :param resolution: Sets the resolution of the samples, 8 or 16 bit.
            :return: Returns false if an incorrect argument is given.
            :rtype: bool
        """
        if resolution not in [0, 1]:
            arg_err(resolution, [int],
                    'resolution must be one of "0, 1".')
        if operation_mode not in [0, 1, 2]:
            arg_err(operation_mode, [
                int], 'operation_mode must be one of "0, 1, 2".')

        combined_arguments = (resolution << 6) | operation_mode

        reg_val = self._i2c.readByte(self.address, self.KX13X_BUF_CNTL2)
        reg_val &= 0xBC
        reg_val |= combined_arguments
        self._i2c.writeByte(self.address, self.KX13X_BUF_CNTL2, reg_val)
        self.set_output_properties()

    def enable_buffer(self, activate_buffer, enable_interrupt):
        """
            Enables the buffer and whether the buffer triggers an interrupt
            when full.
            :param enable: Enables the buffer.
            :param enable: Enables the buffer's interrupt.
            :return: Returns false if an incorrect argument is given.
            :rtype: bool
        """
        if activate_buffer not in [True, False, 0, 1]:
            arg_err(
                activate_buffer, [bool, int],
                'activate_buffer must be one of "True, False, 0, 1".')
        if enable_interrupt not in [True, False, 0, 1]:
            arg_err(
                enable_interrupt, [bool, int],
                'enable_interrupt must be one of "True, False, 0, 1".')

        combined_arguments = (activate_buffer << 7) | (enable_interrupt << 5)

        reg_val = self._i2c.readByte(self.address, self.KX13X_BUF_CNTL2)
        reg_val &= 0x5F
        reg_val |= combined_arguments
        self._i2c.writeByte(self.address, self.KX13X_BUF_CNTL2, reg_val)
        self.set_output_properties()

    def get_buffer_full_interrupt_status(self) -> namedtuple:
        """
            Returns Buffer Full interrupt status.
        """
        status = namedtuple('status', 'buff_active buff_res buff_enabled')
        reg_val = self._i2c.readByte(self.address, self.KX13X_BUF_CNTL2)

        status.buff_active = (reg_val & 0b10000000) >> 7   # BUFE
        status.buff_res = (reg_val & 0b01000000) >> 6      # BRES
        status.buff_enabled = (reg_val & 0b00100000) >> 5  # BFIE
        return status

    def set_output_properties(self) -> None:
        """
            Record the bit depth and output register of the accelerometer data
            depending if Buffer Full interrupt is enabled.
        """
        bfi_status = self.get_buffer_full_interrupt_status()

        if bfi_status.buff_enabled:
            self.data_read_registry = self.KX13X_BUF_READ
            self.bit_depth = 8 if bfi_status.buff_res == 0 else 16
        else:
            self.data_read_registry = self.KX13X_XOUT_L
            self.bit_depth = 16

    def get_raw_accel_data(self):
        """
            Gets raw twos-complement byte data from appropriate data registry
            and converts it to accelerometer counts.
        """
        if self.bit_depth == 16:
            accel_data = self._i2c.readBlock(
                self.address, self.data_read_registry,
                self.TOTAL_ACCEL_DATA_16BIT)
            # combine most significant and least significant bytes
            xData_2s_comp = (accel_data[self.XMSB]
                             << 8) | accel_data[self.XLSB]
            yData_2s_comp = (accel_data[self.YMSB]
                             << 8) | accel_data[self.YLSB]
            zData_2s_comp = (accel_data[self.ZMSB]
                             << 8) | accel_data[self.ZLSB]

            # derive accelerometer counts from twos-compliment binary data
            highest_pos_value = 2**15-1
            xData = xData_2s_comp if xData_2s_comp <= highest_pos_value \
                else xData_2s_comp - 2**16
            yData = yData_2s_comp if yData_2s_comp <= highest_pos_value else \
                yData_2s_comp - 2**16
            zData = zData_2s_comp if zData_2s_comp <= highest_pos_value else \
                zData_2s_comp - 2**16

        elif self.bit_depth == 8:
            accel_data = self._i2c.readBlock(
                self.address, self.data_read_registry,
                self.TOTAL_ACCEL_DATA_8BIT)
            xData_2s_comp = accel_data[0]
            yData_2s_comp = accel_data[1]
            zData_2s_comp = accel_data[2]

            # derive accelerometer counts from twos-compliment binary data
            highest_pos_value = 2**7-1
            xData = xData_2s_comp if xData_2s_comp <= highest_pos_value else \
                xData_2s_comp - 2**8
            yData = yData_2s_comp if yData_2s_comp <= highest_pos_value else \
                yData_2s_comp - 2**8
            zData = zData_2s_comp if zData_2s_comp <= highest_pos_value else \
                zData_2s_comp - 2**8
        else:
            arg_err(
                self.bit_depth, [int],
                "Property bit_depth must be 8 or 16.")

        self.raw_output_data.x = xData
        self.raw_output_data.y = yData
        self.raw_output_data.z = zData

    def set_standby_mode(self):
        self._i2c.writeByte(self.address, self.KX13X_CNTL1, 0X00)


class QwiicKX132(QwiicKX13XCore):

    KX132_WHO_AM_I = 0x3D
    KX132_RANGE2G = 0x00
    KX132_RANGE4G = 0x01
    KX132_RANGE8G = 0x02
    KX132_RANGE16G = 0x03
    MAX_COUNTS_16_BIT = 32768
    CONV_2G_16BIT = 2 / MAX_COUNTS_16_BIT  # original -> .00006103518784142582
    CONV_4G_16BIT = 4 / MAX_COUNTS_16_BIT  # original -> .0001220703756828516
    CONV_8G_16BIT = 8 / MAX_COUNTS_16_BIT  # original -> .0002441407513657033
    CONV_16G_16BIT = 16 / MAX_COUNTS_16_BIT  # original -> .0004882811975463118
    MAX_COUNTS_8_BIT = 128
    CONV_2G_8BIT = 2 / MAX_COUNTS_8_BIT
    CONV_4G_8BIT = 4 / MAX_COUNTS_8_BIT
    CONV_8G_8BIT = 8 / MAX_COUNTS_8_BIT
    CONV_16G_8BIT = 16 / MAX_COUNTS_8_BIT

    accel = namedtuple('accel', 'x y z')

    def __init__(self, address=None, i2c_driver=None):
        super().__init__(address, i2c_driver)
        if not self.begin():
            raise IOError(
                "Wrong accelerometer/ WHO_AM_I. " +
                "Make sure you're using the KX132 and not the KX134.")
        self.set_output_properties()

    def begin(self):
        """
            Checks that communication can be made with the QwiicKX132 by
            checking the WHO_AM_I register.
            :return: Returns true if WHO_AM_I value is the correct one and
            false otherwise.
            :rtype: bool
        """
        chipID = self.beginCore()
        if chipID == self.KX132_WHO_AM_I:
            return True
        else:
            return False

    def set_range(self, G_range: str):
        convertor = {
            '2G': self.KX132_RANGE2G,
            '4G': self.KX132_RANGE4G,
            '8G': self.KX132_RANGE8G,
            '16G': self.KX132_RANGE16G,
        }
        if G_range not in convertor.keys():
            arg_err(
                G_range, [str],
                'argument `G_range` must be "2G", "4G", "8G", or "16G".')

        kx132_range = convertor[G_range]
        super().set_range(kx132_range)

    def get_accel_data(self):
        """
            Retrieves acceleration data and converts it, storing it within a
            named tuple local to the QwiicKX132 class.
        """
        self.get_raw_accel_data()
        self.conv_accel_data()

    def conv_accel_data(self):
        """
            Converts raw acceleration data according to the range setting and
            stores it in a named tuple local to the QwiicKX132.
        """
        accel_range = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        accel_range &= 0x18
        accel_range = accel_range >> 3

        conv_2G = self.CONV_2G_16BIT if self.bit_depth == 16 \
            else self.CONV_2G_8BIT
        conv_4G = self.CONV_4G_16BIT if self.bit_depth == 16 \
            else self.CONV_4G_8BIT
        conv_8G = self.CONV_8G_16BIT if self.bit_depth == 16 \
            else self.CONV_8G_8BIT
        conv_16G = self.CONV_16G_16BIT if self.bit_depth == 16 \
            else self.CONV_16G_8BIT

        if accel_range == self.KX132_RANGE2G:
            self.accel.x = round(self.raw_output_data.x * conv_2G, 6)
            self.accel.y = round(self.raw_output_data.y * conv_2G, 6)
            self.accel.z = round(self.raw_output_data.z * conv_2G, 6)
        elif accel_range == self.KX132_RANGE4G:
            self.accel.x = round(self.raw_output_data.x * conv_4G, 6)
            self.accel.y = round(self.raw_output_data.y * conv_4G, 6)
            self.accel.z = round(self.raw_output_data.z * conv_4G, 6)
        elif accel_range == self.KX132_RANGE8G:
            self.accel.x = round(self.raw_output_data.x * conv_8G, 6)
            self.accel.y = round(self.raw_output_data.y * conv_8G, 6)
            self.accel.z = round(self.raw_output_data.z * conv_8G, 6)
        elif accel_range == self.KX132_RANGE16G:
            self.accel.x = round(self.raw_output_data.x * conv_16G, 6)
            self.accel.y = round(self.raw_output_data.y * conv_16G, 6)
            self.accel.z = round(self.raw_output_data.z * conv_16G, 6)


class QwiicKX134(QwiicKX13XCore):

    KX134_WHO_AM_I = 0x46
    KX134_RANGE8G = 0x00
    KX134_RANGE16G = 0x01
    KX134_RANGE32G = 0x02
    KX134_RANGE64G = 0x03

    conv_8G = .000244140751365703299
    conv_16G = .000488281197546311838
    CONV_32G = .000976523950926236762
    CONV_64G = .001953125095370342112

    kx134_accel = namedtuple('kx134_accel', 'x y z')

    def __init__(self, address=None, i2c_driver=None):
        super().__init__(address, i2c_driver)

    def begin(self):
        """
            Checks that communication can be made with the QwiicKX134 by
            checking the WHO_AM_I register.
            :return: Returns true if WHO_AM_I value is the correct one and
            false otherwise.
            :rtype: bool
        """
        chipID = self.beginCore()
        if chipID == self.KX134_WHO_AM_I:
            return True
        else:
            return False

    def get_accel_data(self):
        """
            Retrieves acceleration data and converts it, storing it within a
            named tuple local to the QwiicKX134 class.
        """
        self.get_raw_accel_data()
        self.conv_accel_data()

    def conv_accel_data(self):
        """
            Converts raw acceleration data according to the range setting and
            stores it in a named tuple local to the QwiicKX132.
        """
        accel_range = self._i2c.readByte(self.address, self.KX13X_CNTL1)
        accel_range &= 0x18
        accel_range = accel_range >> 3

        if accel_range == self.KX134_RANGE8G:
            self.kx134_accel.x = round(
                self.raw_output_data.x * self.CONV_8G, 6)
            self.kx134_accel.y = round(
                self.raw_output_data.y * self.CONV_8G, 6)
            self.kx134_accel.z = round(
                self.raw_output_data.z * self.CONV_8G, 6)
        elif accel_range == self.KX134_RANGE16G:
            self.kx134_accel.x = round(
                self.raw_output_data.x * self.CONV_16G, 6)
            self.kx134_accel.y = round(
                self.raw_output_data.y * self.CONV_16G, 6)
            self.kx134_accel.z = round(
                self.raw_output_data.z * self.CONV_16G, 6)
        elif accel_range == self.KX134_RANGE32G:
            self.kx134_accel.x = round(
                self.raw_output_data.x * self.CONV_32G, 6)
            self.kx134_accel.y = round(
                self.raw_output_data.y * self.CONV_32G, 6)
            self.kx134_accel.z = round(
                self.raw_output_data.z * self.CONV_32G, 6)
        elif accel_range == self.KX134_RANGE64G:
            self.kx134_accel.x = round(
                self.raw_output_data.x * self.CONV_64G, 6)
            self.kx134_accel.y = round(
                self.raw_output_data.y * self.CONV_64G, 6)
            self.kx134_accel.z = round(
                self.raw_output_data.z * self.CONV_64G, 6)


def arg_err(given_input: any,
            valid_types: list[type],
            msg: str) -> None:
    given_input_type = type(given_input)
    if given_input_type in valid_types:
        raise ValueError(msg)
    raise TypeError(msg)
