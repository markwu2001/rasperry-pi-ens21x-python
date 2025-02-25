import smbus2
import time
from enum import Enum, IntEnum
import struct

class SystemTiming:
    BOOTING = 0.002  # 2ms
    CONVERSION_SINGLE_SHOT = 0.130  # 130ms
    CONVERSION_CONTINUOUS = 0.238  # 238ms

class SystemControl(IntEnum):
    DISABLE_LOW_POWER = 0
    ENABLE_LOW_POWER = 1 << 0
    RESET = 1 << 7

class RegisterAddress(IntEnum):
    PART_ID = 0x00
    DIE_REV = 0x02
    UID = 0x04
    SYS_CTRL = 0x10
    SYS_STAT = 0x11
    SENS_RUN = 0x21
    SENS_START = 0x22
    SENS_STOP = 0x23
    SENS_STAT = 0x24
    T_VAL = 0x30
    H_VAL = 0x33

class Sensor(IntEnum):
    TEMPERATURE = 1 << 0
    HUMIDITY = 1 << 1
    TEMPERATURE_AND_HUMIDITY = TEMPERATURE | HUMIDITY

class Result(Enum):
    STATUS_I2C_ERROR = "i2c-error"
    STATUS_CRC_ERROR = "crc-error"
    STATUS_INVALID = "data-invalid"
    STATUS_OK = "ok"

class ENS21x:
    def __init__(self, bus_number=1, address=0x43):
        self.bus_number = bus_number
        self.address = address
        self.solder_correction = 0
        self.debug_stream = None

        # Rename properties to avoid conflict
        self._part_id = 0
        self._die_rev = 0
        self._uid = 0
        self.t_data = 0
        self.h_data = 0
        self.t_status = Result.STATUS_INVALID
        self.h_status = Result.STATUS_INVALID

        try:
            self.bus = smbus2.SMBus(self.bus_number)
        except FileNotFoundError:
            raise RuntimeError("I2C bus not found")

    def __del__(self):
        if hasattr(self, 'bus'):
            self.bus.close()

    def begin(self):
        self.read_identifiers()
        return self.is_connected()

    def is_connected(self):
        raise NotImplementedError("Subclasses must implement this method")

    # Add property getters
    @property
    def part_id(self):
        return self._part_id

    @property
    def die_rev(self):
        return self._die_rev

    @property
    def uid(self):
        return self._uid

    def enable_debugging(self, debug_stream):
        self.debug_stream = debug_stream

    def disable_debugging(self):
        self.debug_stream = None

    def update(self, delay_ms=SystemTiming.CONVERSION_CONTINUOUS):
        time.sleep(delay_ms / 1000.0)
        buffer = self.read_register(RegisterAddress.T_VAL, 6)
        if len(buffer) != 6:
            return Result.STATUS_I2C_ERROR

        # Process temperature data (first 3 bytes)
        t_val = struct.unpack('<I', bytes(buffer[0:3] + [0]))[0] & 0xFFFFFF
        self.t_status = self.check_data(t_val)
        self.t_data = struct.unpack('<H', bytes(buffer[0:2]))[0]

        # Process humidity data (next 3 bytes)
        h_val = struct.unpack('<I', bytes(buffer[3:6] + [0]))[0] & 0xFFFFFF
        self.h_status = self.check_data(h_val)
        self.h_data = struct.unpack('<H', bytes(buffer[3:5]))[0]

        self._debug("update", Result.STATUS_OK)
        return Result.STATUS_OK

    # Rest of the class remains the same...
    # [Keep all other methods from the original implementation]

    def read_identifiers(self):
        self.set_low_power(False)
        time.sleep(SystemTiming.BOOTING)

        part_id = self.read_register(RegisterAddress.PART_ID, 2)
        self._part_id = struct.unpack('<H', bytes(part_id))[0] if len(part_id) == 2 else 0

        die_rev = self.read_register(RegisterAddress.DIE_REV, 2)
        self._die_rev = struct.unpack('<H', bytes(die_rev))[0] if len(die_rev) == 2 else 0

        uid = self.read_register(RegisterAddress.UID, 8)
        self._uid = struct.unpack('<Q', bytes(uid))[0] if len(uid) == 8 else 0

        self.set_low_power(True)

class ENS215(ENS21x):
    def __init__(self, bus_number=1, address=0x47):
        super().__init__(bus_number, address)
    
    def is_connected(self):
        return self.part_id == 0x0215