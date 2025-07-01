import math
import time

import smbus


class __MotorHatInterface:

    # Registers/etc.
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address=0x40, freq=60):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.__write_to_register(self.__MODE1, 0x00)
        self.__set_pwm_freq(freq)

        self.RIGHT_MOTOR_PWM = 0
        self.RIGHT_MOTOR_DIR1 = 1
        self.RIGHT_MOTOR_DIR2 = 2

        self.LEFT_MOTOR_PWM = 5
        self.LEFT_MOTOR_DIR1 = 3
        self.LEFT_MOTOR_DIR2 = 4

    def __write_to_register(self, reg, value):
        """Writes an 8-bit value to the specified register/address"""
        self.bus.write_byte_data(self.address, reg, value)
        print(f"I2C: Write 0x{value:02X} to register 0x{reg:02X}")

    def __read_from_register(self, reg):
        """Read an unsigned byte from the I2C device"""
        result = self.bus.read_byte_data(self.address, reg)
        print(
            f"I2C: Device 0x{self.address:02X} returned 0x{result & 0xFF:02X} from reg 0x{reg:02X}")
        return result

    def __set_pwm_freq(self, freq=60):
        """
            Base frequency for the hat is 25MHz. But motor's PWM signals works at 12-bit resolution.
            So, the frequency is divided by 4096 (2^12) to get the pre-scale value.
            The formula to calculate the pre-scale value is:
            prescale = (25,000,000 / 4096 / freq) - 1
            where freq is the desired frequency in Hz.
        """
        scaled_freq = 25000000.0  # 25MHz
        scaled_freq /= 4096.0      # 12-bit resolution
        scaled_freq /= float(freq)
        scaled_freq -= 1.0
        scaled_freq = math.floor(scaled_freq + 0.5)

        print(f"Setting PWM frequency to {freq} Hz")
        print(f"Final pre-scale: {scaled_freq}")

        current_mode = self.__read_from_register(self.__MODE1)
        updated_mode = (current_mode & 0x7F) | 0x10  # Sleep mode
        self.__write_to_register(self.__MODE1, updated_mode)  # Go to sleep
        self.__write_to_register(self.__PRESCALE, int(scaled_freq))
        self.__write_to_register(self.__MODE1, current_mode)
        time.sleep(0.005)
        self.__write_to_register(self.__MODE1, current_mode | 0x80)

    def __set_pwm(self, channel, on, off):
        self.__write_to_register(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.__write_to_register(
            self.__LED0_ON_H + 4 * channel, 0xff & (on >> 8))
        self.__write_to_register(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.__write_to_register(
            self.__LED0_OFF_H + 4 * channel, 0xff & (off >> 8))

    def set_duty_cycle(self, channel, pulse):
        """
        Sets the PWM duty cycle for a specific channel.
        :param channel: The channel number (0-15).
        :param pulse: The duty cycle percentage (0-100).
        """
        self.__set_pwm(channel, 0, int(pulse * int(4096 / 100)))

    def set_level(self, channel, value):
        """
        Sets the level (on/off) for a specific channel.
        :param channel: The channel number (0-15).
        :param value: 1 for on, 0 for off.
        """
        if value == 1:
            self.__set_pwm(channel, 0, 4095)
        else:
            self.__set_pwm(channel, 0, 0)


class MotorController:
    def __init__(self, address=0x40, freq=60):
        self.hat_controller = __MotorHatInterface(address, freq)

    def set_motor_speed(self, left_speed, right_speed):
        """ 
            takes in speed as a percentage (0-100) and sets left and right motor speeds accordingly.
        """

        # Set left motor speed
        self.hat_controller.set_duty_cycle(
            self.hat_controller.LEFT_MOTOR_PWM, left_speed)
        if left_speed > 0:
            self.hat_controller.set_level(
                self.hat_controller.LEFT_MOTOR_DIR1, 1)
            self.hat_controller.set_level(
                self.hat_controller.LEFT_MOTOR_DIR2, 0)
        elif left_speed < 0:
            self.hat_controller.set_level(
                self.hat_controller.LEFT_MOTOR_DIR1, 0)
            self.hat_controller.set_level(
                self.hat_controller.LEFT_MOTOR_DIR2, 1)
        else:
            self.hat_controller.set_level(
                self.hat_controller.LEFT_MOTOR_DIR1, 0)
            self.hat_controller.set_level(
                self.hat_controller.LEFT_MOTOR_DIR2, 0)

        # Set right motor speed
        self.hat_controller.set_duty_cycle(
            self.hat_controller.RIGHT_MOTOR_PWM, right_speed)
        if right_speed > 0:
            self.hat_controller.set_level(
                self.hat_controller.RIGHT_MOTOR_DIR1, 1)
            self.hat_controller.set_level(
                self.hat_controller.RIGHT_MOTOR_DIR2, 0)
        elif right_speed < 0:
            self.hat_controller.set_level(
                self.hat_controller.RIGHT_MOTOR_DIR1, 0)
            self.hat_controller.set_level(
                self.hat_controller.RIGHT_MOTOR_DIR2, 1)
        else:
            self.hat_controller.set_level(
                self.hat_controller.RIGHT_MOTOR_DIR1, 0)
            self.hat_controller.set_level(
                self.hat_controller.RIGHT_MOTOR_DIR2, 0)

    def stop_motors(self):
        """Stops both motors by setting their speeds to 0."""
        self.set_motor_speed(0, 0)
        print("Motors stopped.")
