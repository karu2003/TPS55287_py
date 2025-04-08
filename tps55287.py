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

    def __init__(self, bus=None, address=0x28, en_pin=4):
        self.addr = address
        self.bus = bus
        self.en_pin = en_pin
        self.where = platform.system()
        self.config = None

        self._initialize_bus()
        self._initialize_gpio()

    def _initialize_bus(self):
        """Инициализация I2C шины в зависимости от платформы."""
        try:
            if self.where == "Linux":
                self.bus = self._initialize_linux_bus()
            elif self.where == "Windows":
                from i2c_mp_usb import I2C_MP_USB as SMBus

                self.bus = SMBus()
            else:
                raise EnvironmentError("Платформа не поддерживается.")
        except Exception as e:
            print(f"Ошибка инициализации I2C шины: {e}")

    def _initialize_linux_bus(self):
        import smbus

        """Инициализация I2C шины для Linux."""
        if "raspberrypi" in platform.uname().machine.lower():
            return smbus.SMBus(
                self.bus or 1
            )  # I2C bus номер 1 для Raspberry Pi по умолчанию
        return smbus.SMBus(self._detect_i2c_bus())

    def _detect_i2c_bus(self):
        """Определение I2C шины для устройств на Linux."""
        try:
            p = subprocess.Popen(["i2cdetect", "-l"], stdout=subprocess.PIPE)
            for line in p.stdout:
                line = line.decode("utf-8")
                if "i2c-tiny-usb" in line:
                    match = re.search(r"i2c-(\d+)", line)
                    if match:
                        return int(match.group(1))
        except subprocess.SubprocessError as e:
            print(f"Ошибка определения I2C шины: {e}")
        return None

    def _initialize_gpio(self):
        """Инициализация GPIO для Raspberry Pi."""
        if self.is_raspberry_pi():
            try:
                import RPi.GPIO as gpio

                gpio.setwarnings(False)
                gpio.setmode(gpio.BCM)
                gpio.setup(self.en_pin, gpio.OUT)
                self.gpio = gpio
            except ImportError:
                print("Модуль RPi.GPIO не установлен.")

    def is_raspberry_pi(self):
        """Проверка, является ли текущая платформа Raspberry Pi."""
        return os.path.exists("/proc/device-tree/model")

    def __read_byte(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def __write_byte(self, reg, data):
        return self.bus.write_byte_data(self.addr, reg, data)

    def hard_reset(self):
        self.gpio.output(self.en_pin, self.gpio.HIGH)
        sleep(0.2)
        self.gpio.output(self.en_pin, self.gpio.LOW)


def main():
    addr = 0x28
    en_pin = 4
    BB = TSP55287()


if __name__ == "__main__":
    main()
