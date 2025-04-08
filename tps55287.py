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

    def __init__(self, bus=None, address=0x74):
        self.addr = address
        self.bus = bus
        self.where = platform.system()
        self.config = None

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

def main():
    addr = 0x74
    BB = TSP55287()


if __name__ == "__main__":
    main()
