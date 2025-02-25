import time
import smbus2
from enum import IntEnum

class RegisterAddress(IntEnum):
    PART_ID     = 0x00
    DIE_REV     = 0x01
    UID         = 0x04
    SYS_CTRL    = 0x10
    SENS_START  = 0x11
    SENS_RUN    = 0x12
    SENS_STOP   = 0x13
    T_VAL       = 0x20

class SystemControl(IntEnum):
    RESET           = 0x80
    ENABLE_LOW_POWER= 0x01
    DISABLE_LOW_POWER=0x00

class Sensor(IntEnum):
    TEMPERATURE                 = 0x01
    HUMIDITY                    = 0x02
    TEMPERATURE_AND_HUMIDITY    = 0x03

class Result(IntEnum):
    STATUS_OK           = 0
    STATUS_I2C_ERROR    = 1
    STATUS_CRC_ERROR    = 2
    STATUS_INVALID      = 3

class SystemTiming:
    BOOTING = 100
    CONVERSION_SINGLE_SHOT = 100  # ms

class ENS21x:
    def __init__(self, bus_num=1, address=0x43):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self.debug_stream = None
        self.solder_correction = 0
        
        self.part_id = 0
        self.die_rev = 0
        self.uid = 0
        self.t_data = 0
        self.h_data = 0
        self.t_status = Result.STATUS_INVALID
        self.h_status = Result.STATUS_INVALID

    def begin(self):
        self.read_identifiers()
        return self.is_connected()

    def is_connected(self):
        return True  # Override in subclass

    def enable_debugging(self, stream):
        self.debug_stream = stream

    def disable_debugging(self):
        self.debug_stream = None

    def update(self, delay_ms=0):
        time.sleep(delay_ms / 1000)
        try:
            data = self.read_register(RegisterAddress.T_VAL, 6)
        except IOError:
            return Result.STATUS_I2C_ERROR
        
        if len(data) != 6:
            return Result.STATUS_I2C_ERROR
        
        # Process temperature data
        t_raw = int.from_bytes(data[0:3], byteorder='little')
        self.t_status = self.check_data(t_raw)
        self.t_data = (t_raw >> 8) & 0xFFFF  # Extract 16-bit temperature data
        
        # Process humidity data
        h_raw = int.from_bytes(data[3:6], byteorder='little')
        self.h_status = self.check_data(h_raw)
        self.h_data = (h_raw >> 8) & 0xFFFF  # Extract 16-bit humidity data
        
        return Result.STATUS_OK

    def single_shot_measure(self, sensor):
        result = self.write_register(RegisterAddress.SENS_START, sensor)
        if result != Result.STATUS_OK:
            return result
        
        result = self.update(SystemTiming.CONVERSION_SINGLE_SHOT)
        if result != Result.STATUS_OK:
            return result
        
        if sensor == Sensor.TEMPERATURE:
            return self.t_status
        elif sensor == Sensor.HUMIDITY:
            return self.h_status
        else:
            return self.t_status if self.t_status != Result.STATUS_OK else self.h_status

    def start_continuous_measure(self, sensor):
        result = self.write_register(RegisterAddress.SENS_RUN, sensor)
        if result == Result.STATUS_OK:
            return self.single_shot_measure(sensor)
        return result

    def stop_continuous_measure(self, sensor):
        return self.write_register(RegisterAddress.SENS_STOP, sensor)

    def set_low_power(self, enable):
        value = SystemControl.ENABLE_LOW_POWER if enable else SystemControl.DISABLE_LOW_POWER
        return self.write_register(RegisterAddress.SYS_CTRL, value)

    def reset(self):
        result = self.write_register(RegisterAddress.SYS_CTRL, SystemControl.RESET)
        if result == Result.STATUS_OK:
            time.sleep(SystemTiming.BOOTING / 1000)
        return result

    def read_register(self, register, length=1):
        try:
            return self.bus.read_i2c_block_data(self.address, register, length)
        except IOError:
            return []

    def write_register(self, register, value):
        try:
            self.bus.write_byte_data(self.address, register, value)
            return Result.STATUS_OK
        except IOError:
            return Result.STATUS_I2C_ERROR

    def read_identifiers(self):
        self.set_low_power(False)
        time.sleep(SystemTiming.BOOTING / 1000)
        
        self.part_id = int.from_bytes(self.read_register(RegisterAddress.PART_ID, 2), byteorder='little')
        self.die_rev = int.from_bytes(self.read_register(RegisterAddress.DIE_REV, 2), byteorder='little')
        self.uid = int.from_bytes(self.read_register(RegisterAddress.UID, 8), byteorder='little')
        
        self.set_low_power(True)

    def check_data(self, data):
        data &= 0xFFFFFF
        valid = (data >> 16) & 0x01
        crc = (data >> 17) & 0x7F
        payload = data & 0x1FFFF
        
        if self.crc7(payload) == crc:
            return Result.STATUS_OK if valid else Result.STATUS_INVALID
        return Result.STATUS_CRC_ERROR

    @staticmethod
    def crc7(val):
        CRC7_POLY = 0x89
        CRC7_IVEC = 0x7F
        DATA7_MASK = 0x1FFFF
        DATA7_MSB = 0x10000
        
        val = (val << 7) | CRC7_IVEC
        pol = CRC7_POLY << 15
        
        bit = DATA7_MSB << 7
        while bit & (DATA7_MASK << 7):
            if bit & val:
                val ^= pol
            bit >>= 1
            pol >>= 1
        
        return val & 0x7F

    @property
    def temperature_kelvin(self):
        return (self.t_data - self.solder_correction) / 64.0

    @property
    def temperature_celsius(self):
        return self.temperature_kelvin - 273.15

    @property
    def temperature_fahrenheit(self):
        return (self.temperature_kelvin * 9/5) - 459.67

    @property
    def humidity_percent(self):
        return self.h_data / 512.0

    @property
    def absolute_humidity(self):
        temp_c = self.temperature_celsius
        rh = self.humidity_percent
        molar_mass = 18.01534
        gas_constant = 8.21447215
        saturation_pressure = 6.1121 * 2.718281828 ** ((17.67 * temp_c) / (temp_c + 243.5))
        return (saturation_pressure * rh * molar_mass) / ((273.15 + temp_c) * gas_constant)

class ENS215(ENS21x):
    def __init__(self, bus_num=1, address=0x47):
        super().__init__(bus_num, address)

    def is_connected(self):
        return self.part_id == 0x0215
    