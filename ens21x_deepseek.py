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
        
        # Process as little-endian 24-bit values
        t_raw = int.from_bytes(data[:3], byteorder='little')
        h_raw = int.from_bytes(data[3:6], byteorder='little')
        
        self.t_status = self.check_data(t_raw)
        self.h_status = self.check_data(h_raw)
        
        if self.t_status == Result.STATUS_OK:
            self.t_data = (t_raw & 0xFFFF) >> 7  # Extract 16-bit temperature data
        if self.h_status == Result.STATUS_OK:
            self.h_data = (h_raw & 0xFFFF) >> 7  # Extract 16-bit humidity data
        
        self.debug(f"t_raw, h_raw: T-{t_raw}, H-{h_raw}")
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
        time.sleep(0.1)  # Add stabilization delay
        
        # Read part ID (16-bit little-endian)
        part_id_data = self.read_register(RegisterAddress.PART_ID, 2)
        if not isinstance(part_id_data, Result):
            self.part_id = int.from_bytes(part_id_data, byteorder='little')
        
        # Read die revision (16-bit little-endian)
        die_rev_data = self.read_register(RegisterAddress.DIE_REV, 2)
        if not isinstance(die_rev_data, Result):
            self.die_rev = int.from_bytes(die_rev_data, byteorder='little')
        
        # Read UID (48-bit little-endian)
        uid_data = self.read_register(RegisterAddress.UID, 6)
        if not isinstance(uid_data, Result):
            self.uid = int.from_bytes(uid_data, byteorder='little')
        
        self.debug(f"Identifiers: PID-{hex(self.part_id)}, REV-{hex(self.die_rev)}, UID-{hex(self.uid)}")
        self.set_low_power(True)

    def crc7(self, payload):
        """Exact replica of the Arduino CRC-7 calculation"""
        CRC7POLY = 0x89
        CRC7IVEC = 0x7F
        DATA7WIDTH = 17
        CRC7WIDTH = 7

        # Align polynomial with data width
        pol = CRC7POLY << (DATA7WIDTH - CRC7WIDTH - 1)  # 0x89 << 9 = 0x4480
        bit = 1 << (DATA7WIDTH - 1)                     # 0x10000
        mask = (1 << (DATA7WIDTH + CRC7WIDTH)) - 1       # 24-bit mask

        # Prepare the value with initial vector
        val = (payload << CRC7WIDTH) | CRC7IVEC

        # Apply polynomial division
        while (bit << CRC7WIDTH) & mask:
            if (bit << CRC7WIDTH) & val:
                val ^= pol << CRC7WIDTH
            bit >>= 1
            pol >>= 1

        # Extract CRC from upper 7 bits
        return (val >> 17) & 0x7F

    def check_data(self, data):
        """Updated data validation matching original logic"""
        data &= 0xFFFFFF  # Ensure 24-bit value
        valid = (data >> 16) & 0x01
        crc_received = (data >> 17) & 0x7F
        payload = data & 0x1FFFF  # Original 17-bit payload

        if self.crc7(payload) == crc_received:
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