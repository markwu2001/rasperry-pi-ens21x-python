import smbus
import time
from enum import Enum, IntEnum

class Result(Enum):
    STATUS_I2C_ERROR = 4
    STATUS_CRC_ERROR = 3
    STATUS_INVALID = 2
    STATUS_OK = 1

class RegisterAddress(IntEnum):
    PART_ID = 0x00
    DIE_REV = 0x02
    UID = 0x04
    SYS_CTRL = 0x10
    SENS_RUN = 0x21
    SENS_START = 0x22
    SENS_STOP = 0x23
    T_VAL = 0x30
    H_VAL = 0x33

class Sensor(IntEnum):
    TEMPERATURE = 1 << 0
    HUMIDITY = 1 << 1
    TEMPERATURE_AND_HUMIDITY = TEMPERATURE | HUMIDITY

class SystemControl(IntEnum):
    DISABLE_LOW_POWER = 0
    ENABLE_LOW_POWER = 1 << 0
    RESET = 1 << 7

class SystemTiming:
    BOOTING = 0.002
    CONVERSION_SINGLE_SHOT = 0.13
    CONVERSION_CONTINUOUS = 0.238

class ENS21x:
    def __init__(self, bus=1, address=0x43):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.solder_correction = 50 * 64 // 1000  # Default 50mK
        self.debug_enabled = False
        
        self.part_id = 0
        self.die_rev = 0
        self.uid = 0
        self.t_data = 0
        self.h_data = 0
        self.t_status = Result.STATUS_I2C_ERROR
        self.h_status = Result.STATUS_I2C_ERROR

    def begin(self):
        self.read_identifiers()
        return self.is_connected()

    def is_connected(self):
        return False  # To be overridden in subclass

    def enable_debugging(self):
        self.debug_enabled = True

    def disable_debugging(self):
        self.debug_enabled = False

    def update(self, delay_seconds=SystemTiming.CONVERSION_CONTINUOUS):
        time.sleep(delay_seconds)
        data = self.read_register(RegisterAddress.T_VAL, 6)
        
        if isinstance(data, Result):
            return data
        
        # Process as big-endian based on sensor's byte order
        t_raw = int.from_bytes(data[:3], byteorder='big')
        h_raw = int.from_bytes(data[3:], byteorder='big')
        
        self.t_status = self.check_data(t_raw)
        self.h_status = self.check_data(h_raw)
        
        if self.t_status == Result.STATUS_OK:
            self.t_data = (t_raw & 0xFFFF) >> 7
        if self.h_status == Result.STATUS_OK:
            self.h_data = (h_raw & 0xFFFF) >> 7
        
        self.debug(f"Update result: T-{self.t_status}, H-{self.h_status}")
        return Result.STATUS_OK

    def single_shot_measure(self, sensor=Sensor.TEMPERATURE_AND_HUMIDITY):
        result = self.write_register(RegisterAddress.SENS_START, sensor)
        if result != Result.STATUS_OK:
            return result
        
        return self.update(SystemTiming.CONVERSION_SINGLE_SHOT)

    def set_low_power(self, enable):
        value = SystemControl.ENABLE_LOW_POWER if enable else SystemControl.DISABLE_LOW_POWER
        return self.write_register(RegisterAddress.SYS_CTRL, value)

    def reset(self):
        result = self.write_register(RegisterAddress.SYS_CTRL, SystemControl.RESET)
        time.sleep(SystemTiming.BOOTING)
        return result

    def read_identifiers(self):
        self.set_low_power(False)
        time.sleep(SystemTiming.BOOTING)
        
        part_id_data = self.read_register(RegisterAddress.PART_ID, 2)
        die_rev_data = self.read_register(RegisterAddress.DIE_REV, 2)
        uid_data = self.read_register(RegisterAddress.UID, 6)
        
        if not isinstance(part_id_data, Result):
            self.part_id = int.from_bytes(part_id_data, byteorder='little')
        if not isinstance(die_rev_data, Result):
            self.die_rev = int.from_bytes(die_rev_data, byteorder='little')
        if not isinstance(uid_data, Result):
            self.uid = int.from_bytes(uid_data, byteorder='little')
        
        self.set_low_power(True)
        # print(f"Identifiers: PID-{self.part_id}, REV-{self.die_rev}, UID-{self.uid}")
        self.debug(f"Identifiers: PID-{self.part_id}, REV-{self.die_rev}, UID-{self.uid}")

    def crc7(self, payload):
        # Corrected CRC-7 implementation matching the sensor's spec
        crc = 0x7F  # Initial value
        polynomial = 0x89 << 9  # Align polynomial with 17-bit payload
        
        for i in range(17):
            if (payload & (1 << (16 - i))):
                crc ^= polynomial
            if crc & 0x10000:
                crc ^= 0x89 << 9
            crc = (crc << 1) & 0x1FFFF  # Maintain 17-bit operations
            
        return (crc >> 10) & 0x7F  # Extract 7-bit CRC

    def check_data(self, data):
        valid = (data >> 16) & 0x01
        crc = (data >> 17) & 0x7F
        payload = data & 0x1FFFF
        
        if self.crc7(payload) == crc:
            return Result.STATUS_OK if valid else Result.STATUS_INVALID
        return Result.STATUS_CRC_ERROR

    def read_register(self, register, length):
        try:
            data = self.bus.read_i2c_block_data(self.address, register, length)
            self.debug(f"Read {register}: {bytes(data)}")
            return data
        except IOError:
            return Result.STATUS_I2C_ERROR

    def write_register(self, register, value):
        try:
            if isinstance(value, (int, IntEnum)):
                self.bus.write_byte_data(self.address, register, value)
            else:
                self.bus.write_i2c_block_data(self.address, register, list(value.to_bytes(2, 'little')))
            self.debug(f"Write {register}: {value}")
            return Result.STATUS_OK
        except IOError:
            return Result.STATUS_I2C_ERROR

    @property
    def temp_kelvin(self):
        return (self.t_data - self.solder_correction) / 64.0

    @property
    def temp_celsius(self):
        return self.temp_kelvin - 273.15

    @property
    def temp_fahrenheit(self):
        return (self.temp_kelvin * 9/5) - 459.67

    @property
    def humidity_percent(self):
        return self.h_data / 512.0

    def debug(self, message):
        if self.debug_enabled:
            print(f"ENS21x DEBUG: {message}")

class ENS215(ENS21x):
    def __init__(self, bus=1, address=0x47):
        super().__init__(bus, address)
    
    def is_connected(self):
        return self.part_id == 0x0215