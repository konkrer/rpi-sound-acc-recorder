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
    def initialize_choices(cls) -> list[str]:
        return ['DEFAULT_SETTINGS', 'INT_SETTINGS', 'SOFT_INT_SETTINGS',
                'BUFFER_SETTINGS_1', 'WAKE_UP_TRIGGER', 'ADP', 'ADP_OFF']

    def initialize(self, settings='DEFAULT_SETTINGS',
                   wake_up_threshold: None or float or int = None
                   ) -> None:
        """
            Initialize configures the accelerometer's registers into a number
            of different modes: asynchronous, hardware trigger, software
            trigger, buffer, wake up from sleep trigger, and
            advanced data path.
        """
        self.set_standby_mode()
        choices = self.initialize_choices()

        # DEFAULT_SETTINGS (asynchronous)
        if settings == choices[0]:
            self._i2c.writeByte(
                self.address, self.KX13X_CNTL1, self.DEFAULT_SETTINGS)
        # INT_SETTINGS (hardware interrupt)
        elif settings == choices[1]:
            self.set_interrupt_pin(True, 1, latch_control=True)
            self.route_hardware_interrupt(self.HI_DATA_READY)
            self._i2c.writeByte(
                self.address, self.KX13X_CNTL1, self.INT_SETTINGS)
        # SOFT_INT_SETTINGS (software interrupt)
        elif settings == choices[2]:
            self._i2c.writeByte(
                self.address, self.KX13X_CNTL1, self.SOFT_INT_SETTINGS)
        # BUFFER_SETTINGS_1
        elif settings == choices[3]:
            self.set_interrupt_pin(True, 1)
            self.route_hardware_interrupt(self.HI_BUFFER_FULL)
            self.set_buffer_operation(
                self.BUFFER_MODE_FIFO, self.BUFFER_16BIT_SAMPLES)
            self._i2c.writeByte(
                self.address, self.KX13X_CNTL1, self.INT_SETTINGS)
        # WAKE_UP_TRIGGER
        elif settings == choices[4]:
            # setup variables
            c_mode = 1  # default: 0, counter resets
            th_mode = 1  # default: 1, relative
            motion_present = 0x01  # default: 0x05, 5 ODR cycles

            # set  Output Data Control Register (ODR) to 200hz
            reg_value = 0b00001000
            self._i2c.writeByte(self.address, self.KX13X_ODCNTL, reg_value)
            # Write 0x73 to Low Power Control Register 1 (LP_CNTL1) to set a
            # 128-sample average for optimizing noise performance.
            self._i2c.writeByte(self.address, self.KX13X_LP_CNTL1, 0x73)
            # Write 0xAE to Control 3 (CNTL3) to set output data rate for the
            # wakeup engine(OWUF) to 50Hz
            # --- reset value: 0b10101000, 0.781Hz
            self._i2c.writeByte(self.address, self.KX13X_CNTL3, 0b10101111)

            # Write 0x30 to Interrupt Control 1 (INC1) to enable physical
            # interrupt pin INT1, set the polarity of the physical interrupt
            # to active high and configure for latched operation.
            # self._i2c.writeByte(self.address, self.KX13X_INC1, 0X30)
            self.set_interrupt_pin(True, 1)
            # Write 0x40 to Interrupt Control 4 (INC4) to set the Buffer Full
            # interrupt to be reported on physical interrupt pin INT1.
            # self._i2c.writeByte(self.address, self.KX13X_INC4, 0X40)
            # --- Altered>!
            self.route_hardware_interrupt(self.HI_WAKE_UP)
            # Write 0x2B (43 dec) to BUF_CNTL1, which sets a watermark level to
            # exactly half of the buffer.
            self._i2c.writeByte(self.address, self.KX13X_BUF_CNTL1, 0X2B)
            # Write 0xE2 to Buffer Control 2 (BUF_CNTL2) to enable the sample
            # buffer(BUFE=1), to set the resolution of the acceleration data
            # samples collected to 16-bit resolution(BRES=1), to enable the
            # buffer full interrupt(BFIE=1), and set the operating mode of the
            # sample buffer to Trigger mode.
            # --- Altered>!
            self._i2c.writeByte(self.address, self.KX13X_BUF_CNTL2, 0X00)
            # Write 0x3F to Interrupt Control 2 (INC2) to enable all positive
            # and negative directions that can cause a wakeup event.
            self._i2c.writeByte(self.address, self.KX13X_INC2, 0X3F)
            # Write 0x60 to Control 4 (CNTL4) to set the counter mode to clear
            # (C_MODE=0), threshold mode to relative(TH_MODE=1), enable the
            # wakeup function(WUFE=1), disable the back to sleep function
            # (BTSE=0), set pulse reject mode set to standard operation and set
            # the output data rate for the back to sleep engine to its default
            # of 0.781Hz
            # self._i2c.writeByte(self.address, self.KX13X_CNTL4, 0X60)
            self.write_to_CNTL4(
                C_MODE=c_mode, TH_MODE=th_mode,
                WUFE=1, BTSE=0, PR_MODE=0, OBTS=0)
            #  Put the sensor into sleep mode
            self.put_to_sleep()
            # Write 0x05 to Wakeup Function Counter (WUFC) to set the time
            # motion must be present for 0.1 second before a Wake-up interrupt
            # is triggered.
            self._i2c.writeByte(
                self.address, self.KX13X_WUFC, motion_present)
            # Write to Wakeup Function Threshold (WUFTH) to set the wakeup
            # threshold.
            self.set_wake_from_sleep_g_threshold(
                wake_up_threshold or self.wake_up_threshold)
            #  Write 0xE0 to Control 1 (CNTL1) to set the accelerometer into
            #  operating mode (PC1=1), full power mode(RES=1), data ready
            # enabled(DRDYE=1), range to ±2g(GSEL=0).
            self._i2c.writeByte(self.address, self.KX13X_CNTL1, 0XE0)
        # ADP (advanced data path)
        elif settings == choices[5]:
            # Write 0x10 to Control 5 (CNTL5) to enable the Advanced Data Path
            self._i2c.writeByte(self.address, self.KX13X_CNTL5, 0X10)
            # Write 0x8B to Output Data Control Register (ODCNTL) to bypass the
            # IIR Filter, set IIR filter corner frequency to ODR/9 (default),
            # disable Fast Start, and set the ODR to 1600Hz
            self._i2c.writeByte(self.address, self.KX13X_ODCNTL, 0X8B)
            # Write 0x73 to Low Power Control Register 1 (LP_CNTL1) to set a
            # 128-sample average for optimizing current and noise performance.
            # ---
            # no averaging
            reg_setting = 0b00000011
            self._i2c.writeByte(self.address, self.KX13X_LP_CNTL1, reg_setting)
            # Write 0x3B to Advanced Data Path Control Register 1 (ADP_CNTL1)
            # to set the number of samples used to calculate RMS output to 16,
            # and to set Advanced Data Path ODR to 1600Hz
            # ---
            # 2 samples/ 1600Hz
            reg_setting = 0b00001011
            self._i2c.writeByte(
                self.address, self.KX13X_ADP_CNTL1, reg_setting)
            # Write 0x03 to Advanced Data Path Control Register 2 (ADP_CNTL2)
            # to make sure Filter 1 and Filter 2 are not bypassed, to select
            # the RMS data out to XADP, YADP, and ZADP registers, and to set
            # Filter-2 as a High-pass filter.
            # ---
            # also route adp to wake up engine/bypass hp filter 2
            reg_setting = 0b01010011
            self._i2c.writeByte(
                self.address, self.KX13X_ADP_CNTL2, reg_setting)
            # Write 0x16 to Advanced Data Path Control Register 3 (ADP_CNTL3)
            # to set Filter-1 Coefficient (1/𝐴) to pass only data below 400Hz.
            # See Table 1, row
            # 𝑆𝑅/4 to set Filter-1 LPF
            # 𝑓𝑐 = 400𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
            # 𝑎𝑑𝑝_𝑓1_1𝑎 = 22 (𝑑𝑒𝑐) = 16 (ℎ𝑒𝑥).
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL3, 0X16)
            # Write 0x00 to Advanced Data Path Control Register 4 (ADP_CNTL4),
            # Advanced Data Path Control Register 5 (ADP_CNTL5),
            # and Advanced Data Path Control Register 6 (ADP_CNTL6)
            # to set Filter-1 Coefficient(B/A) to pass only data below 400Hz.
            # This step is optional as this is also a default setting.
            # See Table 1, row
            # 𝑅/4 to set Filter-1 LPF 𝑓𝑐 = 400𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧 .
            # 𝑎𝑑𝑝_𝑓1_𝑏𝑎 = 0 (𝑑𝑒𝑐) = 00 (ℎ𝑒𝑥).
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL4, 0X00)
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL5, 0X00)
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL6, 0X00)
            # Write 0x1A to Advanced Data Path Control Register 7 (ADP_CNTL7),
            # 0xF6 to Advanced Data Path Control Register 8 (ADP_CNTL8),
            # and 0x15 to Advanced Data Path Control Register 9 (ADP_CNTL9)
            # to set Filter-1 Coefficient(C/A).
            # See Table 1, row
            # 𝑆𝑅/4 to set Filter-1 LPF 𝑓𝑐 = 400𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
            # 𝑎𝑑𝑝_𝑓1_𝑐𝑎 = 1439258 (𝑑𝑒𝑐) = 15𝐹61𝐴(ℎ𝑒𝑥).
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL7, 0X1A)
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL8, 0XF6)
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL9, 0X15)
            # Write 0x01 to Advanced Data Path Control Register 10
            # (ADP_CNTL10) to set ADP shift scale value for Filter-1.
            # See Table 1, row
            # 𝑆𝑅/4 to set Filter-1 LPF 𝑓𝑐 = 400𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
            # 𝑎𝑑𝑝_𝑓1_𝑖𝑠ℎ = 1 (𝑑𝑒𝑐) = 01 (ℎ𝑒𝑥).
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL10, 0X01)
            # Write 0xB5 to Advanced Data Path Control Register 11
            # (ADP_CNTL11). This register consists of 𝑎𝑑𝑝_𝑓𝑖_𝑜𝑠ℎ(1bit) and
            # 𝑎𝑑𝑝_𝑓2_1𝑎(7bit). See Table 1, row
            # 𝑆𝑅/4 to set Filter-1 LPF 𝑓𝑐 = 400𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧 .
            # 𝑎𝑑𝑝_𝑓1_0𝑠ℎ = 1 (𝑑𝑒𝑐) .
            # See Table 3, row
            # 𝑆𝑅/8 to set Filter-2 HPF 𝑓𝑐 = 200𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
            # 𝑎𝑑𝑝_𝑓2_1𝑎 = 53 (𝑑𝑒𝑐).
            # ---
            # 25 hz high pass @ 1600hz ODR = SR/64
            # SR/64 table 3 rms_f2_1a = 116
            # reg_setting = 1 << 7 | 116
            self._i2c.writeByte(
                self.address, self.KX13X_ADP_CNTL11, 0xB5)
            # Write 0x05 to Advanced Data Path Control Register 12
            # (ADP_CNTL12), 0x35 to Advanced Data Path Control Register 13
            # (ADP_CNTL13) to set Filter-2 coefficient(B/A). See Table 3, row
            # 𝑆𝑅/8 to set Filter-2 HPF 𝑓𝑐 = 200𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
            # 𝑎𝑑𝑝_𝑓2_𝑏𝑎 = 13573 (𝑑𝑒𝑐) = 3505 (ℎ𝑒𝑥).
            # ---
            # 25 hz high pass @ 1600hz ODR = SR/64
            # SR/64 table 3 rms_f2_ba = 29699 (dec) = 7403 (hex)
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL12, 0X05)
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL13, 0X35)
            # Write 0x02 to Advanced Data Path Control Register 18
            # (ADP_CNTL18), to set ADP input scale shift value for Filter-2.
            # See Table 3, row
            # 𝑆𝑅/8 to set Filter-2 HPF 𝑓𝑐 = 200𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
            # 𝑎𝑑𝑝_𝑓2_𝑖𝑠ℎ = 2 (𝑑𝑒𝑐) = 02 (ℎ𝑒𝑥).
            # ---
            # 25 hz high pass @ 1600hz ODR = SR/64
            # SR/64 table 3 rms_f2_ish = 5 (dec) = 05 (hex)
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL18, 0X02)
            # Write 0x02 to Advanced Data Path Control Register 19
            # (ADP_CNTL19), to set ADP output scale shift value for Filter-2.
            # See Table 3, row
            # 𝑆𝑅/8 to set Filter-2 HPF 𝑓𝑐 = 200𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
            # 𝑎𝑑𝑝_𝑓2_0𝑠ℎ = 2 (𝑑𝑒𝑐) = 02 (ℎ𝑒𝑥).
            # ---
            # 25 hz high pass @ 1600hz ODR = SR/64
            # SR/64 table 3 rms_f2osh = 5 (dec) = 05 (hex)
            self._i2c.writeByte(self.address, self.KX13X_ADP_CNTL19, 0X02)
            # Write 0x20 to Interrupt Control Register 1 (INC1) to enable the
            # physical interrupt pin in active-low mode, latching until
            # cleared by reading INT_REL.
            # self._i2c.writeByte(self.address, self.KX13X_INC1, 0X20)
            self.set_interrupt_pin(True, 1)
            # Write 0x10 to Interrupt Control Register 4 (INC4) to set the
            # Data ready interrupt to report on physical interrupt pin INT1.
            self._i2c.writeByte(self.address, self.KX13X_INC4, 0X10)
            #  Write 0xE0 to Control 1 (CNTL1) to set the accelerometer into
            #  operating mode (PC1=1), full power mode(RES=1), data ready
            # enabled(DRDYE=1), range to ±2g(GSEL=0).
            self._i2c.writeByte(self.address, self.KX13X_CNTL1, 0XE0)
        # ADP OFF
        elif settings == choices[6]:
            # Write 0x00 to Control 5 (CNTL5) to disable the Advanced Data Path
            self._i2c.writeByte(self.address, self.KX13X_CNTL5, 0X00)
            # reset ADP_CNTL2
            reset_value = 0b00000010
            self._i2c.writeByte(
                self.address, self.KX13X_ADP_CNTL2, reset_value)
        else:
            raise ValueError("Initialization method not recognized.")

    def write_to_CNTL4(self,
                       C_MODE: 0 or 1 or None = None,
                       TH_MODE: 0 or 1 or None = None,
                       WUFE: 0 or 1 or None = None,
                       BTSE: 0 or 1 or None = None,
                       PR_MODE: 0 or 1 or None = None,
                       OBTS: int or float or None = None
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
            OBTS (int or float or None, optional): sets the output data rate
                at which the back-to-sleep (motion detection) performs its
                function during wake state. Setting must be between 0 and 8.
                Defaults to None.
        """
        # if not setting new values use original settings
        reg_val = self._i2c.readByte(self.address, self.KX13X_CNTL4)

        if C_MODE:
            C_MODE = C_MODE << 7
            reg_val &= 0b01111111
            reg_val |= C_MODE

        if TH_MODE:
            TH_MODE = TH_MODE << 6
            reg_val &= 0b10111111
            reg_val |= TH_MODE

        if WUFE:
            WUFE = WUFE << 5
            reg_val &= 0b11011111
            reg_val |= WUFE

        if BTSE:
            BTSE = BTSE << 4
            reg_val &= 0b11101111
            reg_val |= BTSE

        if PR_MODE:
            PR_MODE = PR_MODE << 3
            reg_val &= 0b11110111
            reg_val |= PR_MODE

        if OBTS:
            assert isinstance(OBTS, (int, float)) and 0 <= OBTS < 8, \
                "OBTS must be an number from 0 to eight not inclusive."
            reg_val &= 0b11111000
            reg_val |= OBTS

        self._i2c.writeByte(self.address, self.KX13X_CNTL4, reg_val)

    def set_wake_from_sleep_g_threshold(self, g_threshold: float or int = 0.5
                                        ) -> None:
        # check input
        assert 0 < g_threshold < 8

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
            return False

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

    def set_range(self, kx13x_range):
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

        if kx13x_range < 0 or kx13x_range > 3:
            raise ValueError('range setting must be between 0 to 3 inclusive.')

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
            return False

        accel_state = self.get_accel_state()
        self.accel_control(False)

        reg_val = self._i2c.readByte(self.address, self.KX13X_ODCNTL)
        reg_val &= 0xf0
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

    def set_interrupt_pin(self, enable, polarity=1, pulse_width=0,
                          latch_control=True):
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
            return False
        if polarity not in [0, 1]:
            return False
        if pulse_width not in [0, 1, 2, 3]:
            return False
        if latch_control not in [True, False]:
            return False
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

    def route_hardware_interrupt(self, rdr, pin=1):
        """
            Determines which interrupt is reported: freefall, buffer full,
            watermark, data ready, back to sleep, tap/double tap, wakeup or
            tilt. Also which hardware pin its reported on: one or two.
            :param rdr: The interrupt to be reported.
            :param pin: The hardware pin on which the interrupt is reported.
            :return: Returns true after configuring the register and false if
            an an incorrect argument is given.
            :rtype: bool

        """
        if rdr < 0 or rdr > 128:
            raise ValueError('rdr must be between 0 and 128 inclusive')
        if pin != 1 and pin != 2:
            raise ValueError('pin value must be 0 or 1')

        accel_state = self.get_accel_state()
        self.accel_control(False)

        if pin == 1:
            self._i2c.writeByte(self.address, self.KX13X_INC4, rdr)
        else:
            self._i2c.writeByte(self.address, self.KX13X_INC6, rdr)

        self.accel_control(accel_state)
        return True

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
        if threshold < 2 or threshold > 171:
            return False

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
            return False
        if operation_mode not in [0, 1, 2]:
            return False

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
            return False
        if enable_interrupt not in [True, False, 0, 1]:
            return False

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
            raise ValueError("Property bit_depth must be 8 or 16.")

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

    def set_range(self, G_range):
        convertor = {
            '2G': self.KX132_RANGE2G,
            '4G': self.KX132_RANGE4G,
            '8G': self.KX132_RANGE8G,
            '16G': self.KX132_RANGE16G,
        }
        if G_range not in convertor.keys():
            raise ValueError(
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
