#!/usr/bin/env python
from time import sleep
import subprocess
import re
import platform
import os


class TSP55287:
    """Check the TPS55287.
    :param address: TPS55287 I2C Address
    :type address: int
    """

    REGISTERS = {
        "REF_LSB": {"address": 0x00, "description": "Reference Voltage LSB (7-0 bits)"},
        "REF_MSB": {"address": 0x01, "description": "Reference Voltage MSB (10-8 bits)"},
        "IOUT_LIMIT": {"address": 0x02, "description": "Current Limit Setting"},
        "VOUT_SR": {"address": 0x03, "description": "Slew Rate"},
        "VOUT_FS": {"address": 0x04, "description": "Feedback Selection"},
        "CDC": {"address": 0x05, "description": "Cable Compensation"},
        "MODE": {"address": 0x06, "description": "Mode Control"},
        "STATUS": {"address": 0x07, "description": "Operating Status"},
    }

    OCP_DELAY_VALUES = {
        0b00: "128 μs",
        0b01: "3.072 s",
        0b10: "6.144 s",
        0b11: "12.288 s",
    }

    SR_VALUES = {
        0b00: "1.25 mV/μs",
        0b01: "2.5 mV/μs",
        0b10: "5 mV/μs",
        0b11: "10 mV/μs",
    }

    VOUT_FS_INTFB_VALUES = {
        0b00: "0.2256",
        0b01: "0.1128",
        0b10: "0.0752",
        0b11: "0.0564",
    }

    CDC_VALUES = {
        "SC_MASK": {0b0: "Disabled SC indication", 0b1: "Enable SC indication"},
        "OCP_MASK": {0b0: "Disabled OCP indication", 0b1: "Enable OCP indication"},
        "OVP_MASK": {0b0: "Disabled OVP indication", 0b1: "Enable OVP indication"},
        "CDC_OPTION": {0b0: "Internal CDC compensation", 0b1: "External CDC compensation"},
        "CDC": {
            0b000: "0 V",
            0b001: "0.1 V",
            0b010: "0.2 V",
            0b011: "0.3 V",
            0b100: "0.4 V",
            0b101: "0.5 V",
            0b110: "0.6 V",
            0b111: "0.7 V",
        },
    }

    MODE_VALUES = {
        "OE": {0b0: "Output disabled", 0b1: "Output enabled"},
        "FSWDBL": {0b0: "Keep switching frequency unchanged", 0b1: "Double switching frequency in buck-boost mode"},
        "HICCUP": {0b0: "Disable hiccup during short circuit protection", 0b1: "Enable hiccup during short circuit protection"},
        "DISCHG": {0b0: "Disabled VOUT discharge in shutdown mode", 0b1: "Enable VOUT discharge in shutdown mode"},
        "Force_DISCHG": {0b0: "Disabled VOUT discharge FET", 0b1: "Force enable VOUT discharge FET"},
        "FPWM": {0b0: "PFM operating mode at light load", 0b1: "FPWM operating mode at light load"},
    }

    STATUS_VALUES = {
        "SCP": {0b0: "No short circuit", 0b1: "Short circuit detected"},
        "OCP": {0b0: "No overcurrent", 0b1: "Overcurrent detected"},
        "OVP": {0b0: "No overvoltage", 0b1: "Overvoltage detected"},
        "STATUS": {
            0b00: "Boost mode",
            0b01: "Buck mode",
            0b10: "Buck-Boost mode",
            0b11: "Reserved",
        },
    }


    def __init__(self, bus=None, address=0x74, rshunt=0.02, verbose = False):

        self.addr = address
        self.bus = bus
        self.where = platform.system()
        self.config = None
        self.rshunt = rshunt
        self.verbose = verbose

        self._initialize_bus()

    def _initialize_bus(self):
        """Initialize the I2C bus depending on the platform."""
        try:
            if self.where == "Linux":
                self.bus = self._initialize_linux_bus()
            elif self.where == "Windows":
                from i2c_mp_usb import I2C_MP_USB as SMBus

                self.bus = SMBus()
            else:
                raise EnvironmentError("Platform not supported.")
        except Exception as e:
            print(f"Error initializing the I2C bus: {e}")

    def _initialize_linux_bus(self):
        import smbus

        """Initialize the I2C bus for Linux."""
        if "raspberrypi" or "aarch64" in platform.uname().machine.lower():
            return smbus.SMBus(
                self.bus or 1
            )  # I2C bus number 1 is the default for Raspberry Pi
        return smbus.SMBus(self._detect_i2c_bus())

    def _detect_i2c_bus(self):
        """Detect the I2C bus for devices on Linux."""
        try:
            p = subprocess.Popen(["i2cdetect", "-l"], stdout=subprocess.PIPE)
            for line in p.stdout:
                line = line.decode("utf-8")
                if "i2c-tiny-usb" in line:
                    match = re.search(r"i2c-(\d+)", line)
                    if match:
                        return int(match.group(1))
        except subprocess.SubprocessError as e:
            print(f"Error detecting the I2C bus: {e}")
        return None

    def is_raspberry_pi(self):
        """Check if the current platform is a Raspberry Pi."""
        return os.path.exists("/proc/device-tree/model")

    def __read_byte(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def __write_byte(self, reg, data):
        return self.bus.write_byte_data(self.addr, reg, data)

    def read_register_by_name(self, name):
        """Read a register by its name."""
        if name not in self.REGISTERS:
            raise ValueError(f"Invalid register name: {name}")
        reg = self.REGISTERS[name]["address"]
        return self.__read_byte(reg)

    def write_register_by_name(self, name, data):
        """Write to a register by its name."""
        if name not in self.REGISTERS:
            raise ValueError(f"Invalid register name: {name}")
        reg = self.REGISTERS[name]["address"]
        self.__write_byte(reg, data)

    def get_ref_voltage(self):
        """Read the REF register (combining REF_LSB and REF_MSB)."""
        ref_lsb = self.read_register_by_name("REF_LSB")
        ref_msb = self.read_register_by_name("REF_MSB")
        ref_value = (ref_msb << 8) | ref_lsb
        return ref_value

    def set_ref_voltage(self, value):
        """Write to the REF register (splitting into REF_LSB and REF_MSB)."""
        if not (0 <= value <= 0x7FF):  # REF is 11 bits (0-2047)
            raise ValueError("REF value must be between 0 and 2047 (11 bits).")
        ref_lsb = value & 0xFF
        ref_msb = (value >> 8) & 0x07
        self.write_register_by_name("REF_LSB", ref_lsb)
        self.write_register_by_name("REF_MSB", ref_msb)

    def print_registers(self):
        """Print all registers with their values."""
        for name, info in self.REGISTERS.items():
            try:
                value = self.read_register_by_name(name)
                print(f"Register {name} (Address {info['address']:#02x}): {value:#02x} - {info['description']}")
            except Exception as e:
                print(f"Failed to read register {name}: {e}")

    def set_ref_voltage_in_volts(self, voltage):
        """
        Set the REF value in volts.
        :param voltage: Voltage in volts (from 0.045 to 1.2 V)
        """
        if not (0.045 <= voltage <= 1.2):
            raise ValueError("Voltage must be in the range from 0.045 V to 1.2 V.")

        # Convert voltage to an 11-bit value
        ref_value = round((voltage - 0.045) / 0.0005645)
        if not (0 <= ref_value <= 0x7FF):  # Ensure the value fits in 11 bits
            raise ValueError("Calculated REF value is out of the allowable range.")

        # Split into REF_LSB and REF_MSB
        ref_lsb = ref_value & 0xFF
        ref_msb = (ref_value >> 8) & 0x07

        # Write values to registers
        self.write_register_by_name("REF_LSB", ref_lsb)
        self.write_register_by_name("REF_MSB", ref_msb)

        if self.verbose:
            print(f"REF set to {voltage:.3f} V (REF_LSB={ref_lsb:#02x}, REF_MSB={ref_msb:#02x})")

    def get_ref_voltage_in_volts(self):
        """
        Read the REF value in volts.
        :return: Voltage in volts
        """
        ref_value = self.get_ref_voltage()
        voltage = 0.045 + (ref_value * 0.0005645)
        return voltage
    
    def set_iout_limit(self, enable, setting):
        """
        Write a value to the IOUT_LIMIT register.
        :param enable: Enable/disable current limit (0 or 1).
        :param setting: Set the VISP-VISN voltage (0-127).
        """
        if not (0 <= enable <= 1):
            raise ValueError("Current_Limit_EN must be 0 or 1.")
        if not (0 <= setting <= 0x7F):
            raise ValueError("Current_Limit_Setting must be in the range 0-127.")

        reg_value = (enable << 7) | (setting & 0x7F)  # Assemble the register value
        self.write_register_by_name("IOUT_LIMIT", reg_value)

        if self.verbose:
            print(f"IOUT_LIMIT set: Current_Limit_EN={enable}, Current_Limit_Setting={setting} (VISP-VISN={setting * 0.5:.1f} mV)")
    
    def get_iout_limit(self):
        """
        Read the value of the IOUT_LIMIT register.
        :return: A dictionary with fields Current_Limit_EN and Current_Limit_Setting.
        """
        reg_value = self.read_register_by_name("IOUT_LIMIT")
        # print(f"IOUT_LIMIT: {bin(reg_value)}")
        current_limit_en = (reg_value >> 7) & 0x01  # Most significant bit (7)
        current_limit_setting = reg_value & 0x7F   # Lower 7 bits (6-0)
        return {
            "Current_Limit_EN": current_limit_en,
            "Current_Limit_Setting": current_limit_setting
        }
    
    def set_iout_limit_in_mv(self, enable, voltage_mv):
        """
        Set the IOUT_LIMIT value in millivolts.
        :param enable: Enable/disable current limit (0 or 1).
        :param voltage_mv: VISP-VISN voltage in millivolts (0-63.5 mV).
        """
        if not (0 <= voltage_mv <= 63.5):
            raise ValueError("VISP-VISN voltage must be in the range 0-63.5 mV.")
        
        # Convert voltage to a bit value
        setting = round(voltage_mv / 0.5)  # 1 bit = 0.5 mV
        self.set_iout_limit(enable, setting)

    def get_iout_limit_in_mv(self):
        """
        Read the IOUT_LIMIT value in millivolts.
        :return: A dictionary with fields Current_Limit_EN and VISP-VISN in mV.
        """
        iout_limit = self.get_iout_limit()
        voltage_mv = iout_limit["Current_Limit_Setting"] * 0.5  # 1 bit = 0.5 mV
        return {
            "Current_Limit_EN": iout_limit["Current_Limit_EN"],
            "VISP-VISN (mV)": voltage_mv
        }
    
    def calculate_current(self):
        """
        Calculate the current through the shunt based on the IOUT_LIMIT value.
        :return: Current through the shunt (in amperes).
        """
        iout_limit = self.get_iout_limit_in_mv()
        voltage_mv = iout_limit["VISP-VISN (mV)"]  # VISP-VISN voltage in mV
        current = (voltage_mv / 1000) / self.rshunt  # Convert mV to V and calculate current
        return current
    

    def get_vout_sr(self):
        """
        Read the value of the VOUT_SR register.
        :return: A dictionary with fields OCP_DELAY and SR.
        """
        reg_value = self.read_register_by_name("VOUT_SR")
        ocp_delay = (reg_value >> 4) & 0x03  # Bits 5-4
        sr = reg_value & 0x03               # Bits 1-0
        return {
            "OCP_DELAY": ocp_delay,
            "OCP_DELAY_DESCRIPTION": self.OCP_DELAY_VALUES.get(ocp_delay, "Unknown"),
            "SR": sr,
            "SR_DESCRIPTION": self.SR_VALUES.get(sr, "Unknown"),
        }

    def set_vout_sr(self, ocp_delay, sr):
        """
        Write a value to the VOUT_SR register.
        :param ocp_delay: Overcurrent protection delay (0-3).
        :param sr: Output voltage slew rate (0-3).
        """
        if not (0 <= ocp_delay <= 3):
            raise ValueError("OCP_DELAY must be in the range 0-3.")
        if not (0 <= sr <= 3):
            raise ValueError("SR must be in the range 0-3.")

        # Assemble the register value
        reg_value = (ocp_delay << 4) | (sr & 0x03)
        self.write_register_by_name("VOUT_SR", reg_value)

        if self.verbose:
            print(f"VOUT_SR set: OCP_DELAY={ocp_delay} ({self.OCP_DELAY_VALUES.get(ocp_delay, 'Unknown')}), "
                  f"SR={sr} ({self.SR_VALUES.get(sr, 'Unknown')})")
        
    def get_vout_fs(self):
        """
        Read the value of the VOUT_FS register.
        :return: A dictionary with fields FB and INTFB.
        """
        reg_value = self.read_register_by_name("VOUT_FS")
        fb = (reg_value >> 7) & 0x01  # Bit 7
        intfb = reg_value & 0x03      # Bits 1-0
        return {
            "FB": fb,
            "FB_DESCRIPTION": "external" if fb else "internal",
            "INTFB": intfb,
            "ratio": self.VOUT_FS_INTFB_VALUES.get(intfb, "Unknown"),
        }

    def set_vout_fs(self, fb, intfb):
        """
        Write a value to the VOUT_FS register.
        :param fb: Feedback selection (0 = Internal, 1 = External).
        :param intfb: Internal feedback ratio (0-3).
        """
        if not (0 <= fb <= 1):
            raise ValueError("FB must be 0 or 1.")
        if not (0 <= intfb <= 3):
            raise ValueError("INTFB must be in the range 0-3.")

        # Assemble the register value
        reg_value = (fb << 7) | (intfb & 0x03)
        self.write_register_by_name("VOUT_FS", reg_value)
        if self.verbose:
            print(f"VOUT_FS set: FB={fb} ({'External feedback' if fb else 'Internal feedback'}), "
                  f"INTFB={intfb} ({self.VOUT_FS_INTFB_VALUES.get(intfb, 'Unknown')})")
        
    def get_cdc(self):
        """
        Read the value of the CDC register.
        :return: A dictionary with fields SC_MASK, OCP_MASK, OVP_MASK, CDC_OPTION, and CDC.
        """
        reg_value = self.read_register_by_name("CDC")
        sc_mask = (reg_value >> 7) & 0x01  # Bit 7
        ocp_mask = (reg_value >> 6) & 0x01  # Bit 6
        ovp_mask = (reg_value >> 5) & 0x01  # Bit 5
        cdc_option = (reg_value >> 3) & 0x01  # Bit 3
        cdc = reg_value & 0x07  # Bits 2-0
        return {
            "SC_MASK": sc_mask,
            "SC_MASK_DESCRIPTION": self.CDC_VALUES["SC_MASK"].get(sc_mask, "Unknown"),
            "OCP_MASK": ocp_mask,
            "OCP_MASK_DESCRIPTION": self.CDC_VALUES["OCP_MASK"].get(ocp_mask, "Unknown"),
            "OVP_MASK": ovp_mask,
            "OVP_MASK_DESCRIPTION": self.CDC_VALUES["OVP_MASK"].get(ovp_mask, "Unknown"),
            "CDC_OPTION": cdc_option,
            "CDC_OPTION_DESCRIPTION": self.CDC_VALUES["CDC_OPTION"].get(cdc_option, "Unknown"),
            "CDC": cdc,
            "CDC_DESCRIPTION": self.CDC_VALUES["CDC"].get(cdc, "Unknown"),
        }

    def set_cdc(self, sc_mask, ocp_mask, ovp_mask, cdc_option, cdc):
        """
        Write a value to the CDC register.
        :param sc_mask: Short circuit mask (0 or 1).
        :param ocp_mask: Overcurrent mask (0 or 1).
        :param ovp_mask: Overvoltage mask (0 or 1).
        :param cdc_option: Cable droop compensation option (0 or 1).
        :param cdc: Cable droop compensation value (0-7).
        """
        if not (0 <= sc_mask <= 1):
            raise ValueError("SC_MASK must be 0 or 1.")
        if not (0 <= ocp_mask <= 1):
            raise ValueError("OCP_MASK must be 0 or 1.")
        if not (0 <= ovp_mask <= 1):
            raise ValueError("OVP_MASK must be 0 or 1.")
        if not (0 <= cdc_option <= 1):
            raise ValueError("CDC_OPTION must be 0 or 1.")
        if not (0 <= cdc <= 7):
            raise ValueError("CDC must be in the range 0-7.")

        # Assemble the register value
        reg_value = (sc_mask << 7) | (ocp_mask << 6) | (ovp_mask << 5) | (cdc_option << 3) | (cdc & 0x07)
        self.write_register_by_name("CDC", reg_value)
        if self.verbose:
            print(f"CDC set: SC_MASK={sc_mask} ({self.CDC_VALUES['SC_MASK'].get(sc_mask, 'Unknown')}), "
                f"OCP_MASK={ocp_mask} ({self.CDC_VALUES['OCP_MASK'].get(ocp_mask, 'Unknown')}), "
                f"OVP_MASK={ovp_mask} ({self.CDC_VALUES['OVP_MASK'].get(ovp_mask, 'Unknown')}), "
                f"CDC_OPTION={cdc_option} ({self.CDC_VALUES['CDC_OPTION'].get(cdc_option, 'Unknown')}), "
                f"CDC={cdc} ({self.CDC_VALUES['CDC'].get(cdc, 'Unknown')})")


    def get_mode(self):
        """
        Read the value of the MODE register.
        :return: A dictionary with fields OE, FSWDBL, HICCUP, DISCHG, Force_DISCHG, FPWM.
        """
        reg_value = self.read_register_by_name("MODE")
        oe = (reg_value >> 7) & 0x01  # Bit 7
        fswdbl = (reg_value >> 6) & 0x01  # Bit 6
        hiccup = (reg_value >> 5) & 0x01  # Bit 5
        dischg = (reg_value >> 4) & 0x01  # Bit 4
        force_dischg = (reg_value >> 3) & 0x01  # Bit 3
        fpwm = (reg_value >> 1) & 0x01  # Bit 1
        return {
            "OE": oe,
            "OE_DESCRIPTION": self.MODE_VALUES["OE"].get(oe, "Unknown"),
            "FSWDBL": fswdbl,
            "FSWDBL_DESCRIPTION": self.MODE_VALUES["FSWDBL"].get(fswdbl, "Unknown"),
            "HICCUP": hiccup,
            "HICCUP_DESCRIPTION": self.MODE_VALUES["HICCUP"].get(hiccup, "Unknown"),
            "DISCHG": dischg,
            "DISCHG_DESCRIPTION": self.MODE_VALUES["DISCHG"].get(dischg, "Unknown"),
            "Force_DISCHG": force_dischg,
            "Force_DISCHG_DESCRIPTION": self.MODE_VALUES["Force_DISCHG"].get(force_dischg, "Unknown"),
            "FPWM": fpwm,
            "FPWM_DESCRIPTION": self.MODE_VALUES["FPWM"].get(fpwm, "Unknown"),
        }

    def set_mode(self, oe=0, fswdbl=0, hiccup=1, dischg=0, force_dischg=0, fpwm=0):
        """
        Write a value to the MODE register.
        :param oe: Output enable (0 or 1). Default: 0 (Output disabled).
        :param fswdbl: Switching frequency doubling (0 or 1). Default: 0 (Keep frequency unchanged).
        :param hiccup: Hiccup mode (0 or 1). Default: 1 (Enable hiccup during short circuit protection).
        :param dischg: Output discharge (0 or 1). Default: 0 (Disabled VOUT discharge in shutdown mode).
        :param force_dischg: Force output discharge (0 or 1). Default: 0 (Disabled VOUT discharge FET).
        :param fpwm: Operating mode at light load (0 or 1). Default: 0 (PFM operating mode).
        """
        if not (0 <= oe <= 1):
            raise ValueError("OE must be 0 or 1.")
        if not (0 <= fswdbl <= 1):
            raise ValueError("FSWDBL must be 0 or 1.")
        if not (0 <= hiccup <= 1):
            raise ValueError("HICCUP must be 0 or 1.")
        if not (0 <= dischg <= 1):
            raise ValueError("DISCHG must be 0 or 1.")
        if not (0 <= force_dischg <= 1):
            raise ValueError("Force_DISCHG must be 0 or 1.")
        if not (0 <= fpwm <= 1):
            raise ValueError("FPWM must be 0 or 1.")

        # Assemble the register value
        reg_value = (oe << 7) | (fswdbl << 6) | (hiccup << 5) | (dischg << 4) | (force_dischg << 3) | (fpwm << 1)
        self.write_register_by_name("MODE", reg_value)

        if self.verbose:
            print(f"MODE set: OE={oe} ({self.MODE_VALUES['OE'].get(oe, 'Unknown')}), "
                  f"FSWDBL={fswdbl} ({self.MODE_VALUES['FSWDBL'].get(fswdbl, 'Unknown')}), "
                  f"HICCUP={hiccup} ({self.MODE_VALUES['HICCUP'].get(hiccup, 'Unknown')}), "
                  f"DISCHG={dischg} ({self.MODE_VALUES['DISCHG'].get(dischg, 'Unknown')}), "
                  f"Force_DISCHG={force_dischg} ({self.MODE_VALUES['Force_DISCHG'].get(force_dischg, 'Unknown')}), "
                  f"FPWM={fpwm} ({self.MODE_VALUES['FPWM'].get(fpwm, 'Unknown')})")

    def get_status(self):
        """
        Read the value of the STATUS register.
        :return: A dictionary with fields SCP, OCP, OVP, and STATUS.
        """
        reg_value = self.read_register_by_name("STATUS")
        scp = (reg_value >> 7) & 0x01  # Bit 7
        ocp = (reg_value >> 6) & 0x01  # Bit 6
        ovp = (reg_value >> 5) & 0x01  # Bit 5
        status = reg_value & 0x03      # Bits 1-0
        return {
            "SCP": scp,
            "SCP_DESCRIPTION": self.STATUS_VALUES["SCP"].get(scp, "Unknown"),
            "OCP": ocp,
            "OCP_DESCRIPTION": self.STATUS_VALUES["OCP"].get(ocp, "Unknown"),
            "OVP": ovp,
            "OVP_DESCRIPTION": self.STATUS_VALUES["OVP"].get(ovp, "Unknown"),
            "STATUS": status,
            "STATUS_DESCRIPTION": self.STATUS_VALUES["STATUS"].get(status, "Unknown"),
        }
    
    def calculate_vref_and_intfb_with_min_error(self, vout):
        """
        Calculate VREF and INTFB values for the given VOUT with minimal error.
        :param vout: Desired output voltage (in volts).
        :return: A dictionary with calculated VREF, INTFB, and error values.
        """
        if not (0.8 <= vout <= 22):  # Expected VOUT range
            raise ValueError("VOUT must be in the range 0.8 to 22 V.")

        best_match = None
        min_error = float('inf')

        # Iterate through all possible INTFB values
        for intfb_key, intfb_ratio_str in self.VOUT_FS_INTFB_VALUES.items():
            intfb_ratio = float(intfb_ratio_str)  # Convert string to number
            vref = vout * intfb_ratio  # Calculate VREF
            if 0.045 <= vref <= 1.2:  # Check if VREF is within the valid range
                calculated_vout = vref / intfb_ratio
                error = abs(vout - calculated_vout)  # Calculate error
                if error < min_error:  # If error is smaller than the current minimum
                    min_error = error
                    best_match = {
                        "VREF": round(vref, 3),
                        "INTFB_KEY": intfb_key,
                        "INTFB_RATIO": intfb_ratio,
                        "CALCULATED_VOUT": round(calculated_vout, 3),
                        "ERROR": round(error, 6),
                    }

        if best_match is None:
            raise ValueError("Unable to calculate VREF and INTFB for the given VOUT.")

        return best_match
    
    def set_vout(self, vout):
        """
        Set VREF and INTFB values in the registers.
        :param vref: Reference voltage (in volts, from 0.045 to 1.2).
        :param intfb_key: INTFB key from VOUT_FS_INTFB_VALUES (0b00, 0b01, 0b10, 0b11).
        """

        result = self.calculate_vref_and_intfb_with_min_error(vout)
        vref = result["VREF"]
        intfb_key = result["INTFB_KEY"]

        if not (0.045 <= vref <= 1.2):
            raise ValueError("VREF must be in the range 0.045 to 1.2 V.")
        if intfb_key not in self.VOUT_FS_INTFB_VALUES:
            raise ValueError("Invalid INTFB key. Must be one of 0b00, 0b01, 0b10, 0b11.")

        # Set VREF
        self.set_ref_voltage_in_volts(vref)

        # Set INTFB (internal feedback ratio)
        self.set_vout_fs(fb=0, intfb=intfb_key)  # fb=0 means using internal feedback

        if self.verbose:
            print(f"VREF set to {vref:.3f} V")
            print(f"INTFB set to {bin(intfb_key)} (Ratio: {self.VOUT_FS_INTFB_VALUES[intfb_key]})")

    def set_iout_limit_from_current(self, enable, current):
        """
        Set the IOUT_LIMIT value based on the specified current.
        :param enable: Enable/disable current limit (0 or 1).
        :param current: Specified current (in amperes).
        """
        if not (0 <= current <= 4):  # Maximum VISP-VISN voltage = 63.5 mV
            raise ValueError("Current must be in the range 0 to 4A.")
        
        # Calculate VISP-VISN voltage in millivolts
        voltage_mv = current * self.rshunt * 1000  # Convert to mV

        if not (0 <= voltage_mv <= 63.5):
            raise ValueError("Calculated voltage exceeds the allowable range (0-63.5 mV).")

        # Set the IOUT_LIMIT value
        self.set_iout_limit_in_mv(enable, voltage_mv)

        if self.verbose:
            print(f"IOUT_LIMIT set for current {current:.3f} A (VISP-VISN = {voltage_mv:.1f} mV)")


def main():
    addr = 0x74
    BB = TSP55287()

    # # Example: Read and write REF register
    # ref_voltage = BB.get_ref_voltage_in_volts()
    # print(f"Current REF value: {ref_voltage:.3f} V")

    # iout_limit_mv = BB.get_iout_limit_in_mv()
    # print(f"Current IOUT_LIMIT value: {iout_limit_mv}")

    # Calculate current through the shunt
    current = BB.calculate_current()
    print(f"Current through the shunt: {current:.3f} A")

    # vout_sr = BB.get_vout_sr()
    # print(f"Current VOUT_SR value: {vout_sr}")

    # vout_fs = BB.get_vout_fs()
    # print(f"Current VOUT_FS value: {vout_fs}")

    # # Read the current CDC value
    # cdc = BB.get_cdc()
    # print(f"Current CDC value: {cdc}")

    # mode = BB.get_mode()
    # print(f"Current MODE value: {mode}")

    # status = BB.get_status()
    # print(f"Current STATUS value: {status}")

    vout = 5.3  # Desired output voltage

    # try:
    #     result = BB.calculate_vref_and_intfb_with_min_error(vout)
    #     print(f"For VOUT = {vout} V:")
    #     print(f"  VREF = {result['VREF']} V")
    #     print(f"  INTFB_KEY = {bin(result['INTFB_KEY'])} (Ratio: {result['INTFB_RATIO']})")
    #     print(f"  CALCULATED_VOUT = {result['CALCULATED_VOUT']} V")
    #     print(f"  ERROR = {result['ERROR']} V")
    # except ValueError as e:
    #     print(f"Error: {e}")
    


    # Set VREF and INTFB
    # BB.set_iout_limit_from_current(1, 1.0)  # Set IOUT_LIMIT to 2.5A
    BB.set_vout(vout)

    BB.set_mode(oe=1)
    


if __name__ == "__main__":
    main()
