class Kx13xInitializations:
    def __init__(self) -> None:
        pass

    @classmethod
    def asynchronous(cls, kx) -> None:
        kx._i2c.writeByte(
            kx.address, kx.KX13X_CNTL1, kx.DEFAULT_SETTINGS)

    @classmethod
    def hard_interrupt(cls, kx) -> None:
        kx.set_interrupt_pin(True, 1, latch_control=True)
        kx.route_hardware_interrupt(kx.HI_DATA_READY)
        kx._i2c.writeByte(
            kx.address, kx.KX13X_CNTL1, kx.INT_SETTINGS)

    @classmethod
    def soft_interrupt(cls, kx) -> None:
        kx._i2c.writeByte(
            kx.address, kx.KX13X_CNTL1, kx.SOFT_INT_SETTINGS)

    @classmethod
    def buffer_settings_1(cls, kx) -> None:
        kx.set_interrupt_pin(True, 1)
        kx.route_hardware_interrupt(kx.HI_BUFFER_FULL)
        kx.set_buffer_operation(
            kx.BUFFER_MODE_FIFO, kx.BUFFER_16BIT_SAMPLES)
        kx._i2c.writeByte(
            kx.address, kx.KX13X_CNTL1, kx.INT_SETTINGS)

    @classmethod
    def wake_up_trigger(cls, kx, wake_up_threshold
                        ) -> None:
        # setup variables
        c_mode = 0  # default: 0, counter resets
        th_mode = 1  # default: 1, relative
        pr_mode = 1  # default: None or 0 (off)
        motion_present = 2  # default: 0x05, 5 ODR cycles

        # Write 0x06 to Output Data Control (ODCNTL) to set the Output
        # Data Rate (ODR) of the accelerometer to 50 Hz.
        # 200hz -> 0b00001000
        reg_value = 0b01001000  # NOTE:LPRO set at ODR/2, ODR set 200Hz
        kx._i2c.writeByte(kx.address, kx.KX13X_ODCNTL, reg_value)
        # Write 0x73 to Low Power Control Register 1 (LP_CNTL1) to set a
        # 128-sample average for optimizing noise performance.
        kx._i2c.writeByte(kx.address, kx.KX13X_LP_CNTL1, 0x73)
        # Write 0xAE to Control 3 (CNTL3) to set output data rate for the
        # wakeup engine(OWUF) to 50Hz
        # reset value: 0b10101000, 0.781Hz OWUF
        reg_value = 0b10101110  # NOTE: OWUF set 50Hz
        kx._i2c.writeByte(kx.address, kx.KX13X_CNTL3, reg_value)

        # Write 0x30 to Interrupt Control 1 (INC1) to enable physical
        # interrupt pin INT1, set the polarity of the physical interrupt
        # to active high and configure for latched operation.
        # kx._i2c.writeByte(kx.address, kx.KX13X_INC1, 0X30)
        kx.set_interrupt_pin(True, 1)
        # Write 0x40 to Interrupt Control 4 (INC4) to set the Buffer Full
        # interrupt to be reported on physical interrupt pin INT1.
        # NOTE: buffer off.
        # kx._i2c.writeByte(kx.address, kx.KX13X_INC4, 0X40)
        kx.route_hardware_interrupt(kx.HI_WAKE_UP)
        # Write 0x2B (43 dec) to BUF_CNTL1, which sets a watermark level to
        # exactly half of the buffer.
        kx._i2c.writeByte(kx.address, kx.KX13X_BUF_CNTL1, 0X2B)
        # Write 0xE2 to Buffer Control 2 (BUF_CNTL2) to enable the sample
        # buffer(BUFE=1), to set the resolution of the acceleration data
        # samples collected to 16-bit resolution(BRES=1), to enable the
        # buffer full interrupt(BFIE=1), and set the operating mode of the
        # sample buffer to Trigger mode.
        # NOTE: buffer off
        # reset value: 0b00000000, buffer off
        # kx._i2c.writeByte(kx.address, kx.KX13X_BUF_CNTL2, 0xE2)
        #
        # Write 0x3F to Interrupt Control 2 (INC2) to enable all positive
        # and negative directions that can cause a wakeup event.
        kx._i2c.writeByte(kx.address, kx.KX13X_INC2, 0X3F)
        # Write 0x60 to Control 4 (CNTL4) to set the counter mode to clear
        # (C_MODE=0), threshold mode to relative(TH_MODE=1), enable the
        # wakeup function(WUFE=1), disable the back to sleep function
        # (BTSE=0), set pulse reject mode set to standard operation and set
        # the output data rate for the back to sleep engine to its default
        # of 0.781Hz
        # kx._i2c.writeByte(kx.address, kx.KX13X_CNTL4, 0X60)
        kx.write_to_CNTL4(
            C_MODE=c_mode, TH_MODE=th_mode,
            WUFE=1, BTSE=0, PR_MODE=pr_mode, OBTS=0)
        #  Put the sensor into sleep mode
        kx.put_to_sleep()
        # Write 0x05 to Wakeup Function Counter (WUFC) to set the time
        # motion must be present for 0.1 second before a Wake-up interrupt
        # is triggered.
        kx._i2c.writeByte(
            kx.address, kx.KX13X_WUFC, motion_present)
        # Write to Wakeup Function Threshold (WUFTH) to set the wakeup
        # threshold.
        kx.set_wake_from_sleep_g_threshold(
            wake_up_threshold or kx.wake_up_threshold)
        #  Write 0xE0 to Control 1 (CNTL1) to set the accelerometer into
        #  operating mode (PC1=1), full power mode(RES=1), data ready
        # enabled(DRDYE=1), range to ±2g(GSEL=0).
        kx._i2c.writeByte(kx.address, kx.KX13X_CNTL1, 0XE0)

    @classmethod
    def adp_enable(cls, kx) -> None:
        # Write 0x10 to Control 5 (CNTL5) to enable the Advanced Data Path
        kx._i2c.writeByte(kx.address, kx.KX13X_CNTL5, 0X10)
        # Write 0x8B to Output Data Control Register (ODCNTL) to bypass the
        # IIR Filter, set IIR filter corner frequency to ODR/9 (default),
        # disable Fast Start, and set the ODR to 1600Hz
        kx._i2c.writeByte(kx.address, kx.KX13X_ODCNTL, 0X8B)
        # Write 0x73 to Low Power Control Register 1 (LP_CNTL1) to set a
        # 128-sample average for optimizing current and noise performance.
        # ---
        # no averaging
        reg_setting = 0b00000011
        kx._i2c.writeByte(kx.address, kx.KX13X_LP_CNTL1, reg_setting)
        # Write 0x3B to Advanced Data Path Control Register 1 (ADP_CNTL1)
        # to set the number of samples used to calculate RMS output to 16,
        # and to set Advanced Data Path ODR to 1600Hz
        # ---
        # 2 samples/ 1600Hz
        reg_setting = 0b00001011
        kx._i2c.writeByte(
            kx.address, kx.KX13X_ADP_CNTL1, reg_setting)
        # Write 0x03 to Advanced Data Path Control Register 2 (ADP_CNTL2)
        # to make sure Filter 1 and Filter 2 are not bypassed, to select
        # the RMS data out to XADP, YADP, and ZADP registers, and to set
        # Filter-2 as a High-pass filter.
        # ---
        # also route adp to wake up engine/bypass hp filter 2
        reg_setting = 0b01010011
        kx._i2c.writeByte(
            kx.address, kx.KX13X_ADP_CNTL2, reg_setting)
        # Write 0x16 to Advanced Data Path Control Register 3 (ADP_CNTL3)
        # to set Filter-1 Coefficient (1/𝐴) to pass only data below 400Hz.
        # See Table 1, row
        # 𝑆𝑅/4 to set Filter-1 LPF
        # 𝑓𝑐 = 400𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
        # 𝑎𝑑𝑝_𝑓1_1𝑎 = 22 (𝑑𝑒𝑐) = 16 (ℎ𝑒𝑥).
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL3, 0X16)
        # Write 0x00 to Advanced Data Path Control Register 4 (ADP_CNTL4),
        # Advanced Data Path Control Register 5 (ADP_CNTL5),
        # and Advanced Data Path Control Register 6 (ADP_CNTL6)
        # to set Filter-1 Coefficient(B/A) to pass only data below 400Hz.
        # This step is optional as this is also a default setting.
        # See Table 1, row
        # 𝑅/4 to set Filter-1 LPF 𝑓𝑐 = 400𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧 .
        # 𝑎𝑑𝑝_𝑓1_𝑏𝑎 = 0 (𝑑𝑒𝑐) = 00 (ℎ𝑒𝑥).
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL4, 0X00)
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL5, 0X00)
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL6, 0X00)
        # Write 0x1A to Advanced Data Path Control Register 7 (ADP_CNTL7),
        # 0xF6 to Advanced Data Path Control Register 8 (ADP_CNTL8),
        # and 0x15 to Advanced Data Path Control Register 9 (ADP_CNTL9)
        # to set Filter-1 Coefficient(C/A).
        # See Table 1, row
        # 𝑆𝑅/4 to set Filter-1 LPF 𝑓𝑐 = 400𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
        # 𝑎𝑑𝑝_𝑓1_𝑐𝑎 = 1439258 (𝑑𝑒𝑐) = 15𝐹61𝐴(ℎ𝑒𝑥).
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL7, 0X1A)
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL8, 0XF6)
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL9, 0X15)
        # Write 0x01 to Advanced Data Path Control Register 10
        # (ADP_CNTL10) to set ADP shift scale value for Filter-1.
        # See Table 1, row
        # 𝑆𝑅/4 to set Filter-1 LPF 𝑓𝑐 = 400𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
        # 𝑎𝑑𝑝_𝑓1_𝑖𝑠ℎ = 1 (𝑑𝑒𝑐) = 01 (ℎ𝑒𝑥).
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL10, 0X01)
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
        kx._i2c.writeByte(
            kx.address, kx.KX13X_ADP_CNTL11, 0xB5)
        # Write 0x05 to Advanced Data Path Control Register 12
        # (ADP_CNTL12), 0x35 to Advanced Data Path Control Register 13
        # (ADP_CNTL13) to set Filter-2 coefficient(B/A). See Table 3, row
        # 𝑆𝑅/8 to set Filter-2 HPF 𝑓𝑐 = 200𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
        # 𝑎𝑑𝑝_𝑓2_𝑏𝑎 = 13573 (𝑑𝑒𝑐) = 3505 (ℎ𝑒𝑥).
        # ---
        # 25 hz high pass @ 1600hz ODR = SR/64
        # SR/64 table 3 rms_f2_ba = 29699 (dec) = 7403 (hex)
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL12, 0X05)
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL13, 0X35)
        # Write 0x02 to Advanced Data Path Control Register 18
        # (ADP_CNTL18), to set ADP input scale shift value for Filter-2.
        # See Table 3, row
        # 𝑆𝑅/8 to set Filter-2 HPF 𝑓𝑐 = 200𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
        # 𝑎𝑑𝑝_𝑓2_𝑖𝑠ℎ = 2 (𝑑𝑒𝑐) = 02 (ℎ𝑒𝑥).
        # ---
        # 25 hz high pass @ 1600hz ODR = SR/64
        # SR/64 table 3 rms_f2_ish = 5 (dec) = 05 (hex)
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL18, 0X02)
        # Write 0x02 to Advanced Data Path Control Register 19
        # (ADP_CNTL19), to set ADP output scale shift value for Filter-2.
        # See Table 3, row
        # 𝑆𝑅/8 to set Filter-2 HPF 𝑓𝑐 = 200𝐻𝑧 with 𝑆𝑅 = 1600𝐻𝑧.
        # 𝑎𝑑𝑝_𝑓2_0𝑠ℎ = 2 (𝑑𝑒𝑐) = 02 (ℎ𝑒𝑥).
        # ---
        # 25 hz high pass @ 1600hz ODR = SR/64
        # SR/64 table 3 rms_f2osh = 5 (dec) = 05 (hex)
        kx._i2c.writeByte(kx.address, kx.KX13X_ADP_CNTL19, 0X02)
        # Write 0x20 to Interrupt Control Register 1 (INC1) to enable the
        # physical interrupt pin in active-low mode, latching until
        # cleared by reading INT_REL.
        # kx._i2c.writeByte(kx.address, kx.KX13X_INC1, 0X20)
        kx.set_interrupt_pin(True, 1)
        # Write 0x10 to Interrupt Control Register 4 (INC4) to set the
        # Data ready interrupt to report on physical interrupt pin INT1.
        kx._i2c.writeByte(kx.address, kx.KX13X_INC4, 0X10)
        #  Write 0xE0 to Control 1 (CNTL1) to set the accelerometer into
        #  operating mode (PC1=1), full power mode(RES=1), data ready
        # enabled(DRDYE=1), range to ±2g(GSEL=0).
        kx._i2c.writeByte(kx.address, kx.KX13X_CNTL1, 0XE0)

    @classmethod
    def adp_disable(cls, kx) -> None:
        # Write 0x00 to Control 5 (CNTL5) to disable the Advanced Data Path
        kx._i2c.writeByte(kx.address, kx.KX13X_CNTL5, 0X00)
        # reset ADP_CNTL2
        reset_value = 0b00000010
        kx._i2c.writeByte(
            kx.address, kx.KX13X_ADP_CNTL2, reset_value)
