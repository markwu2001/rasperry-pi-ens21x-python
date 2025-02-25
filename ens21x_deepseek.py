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

        self.bus = smbus2.SMBus(self.bus_number)

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

    def single_shot_measure(self, sensor=Sensor.TEMPERATURE_AND_HUMIDITY):
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

    def start_continuous_measure(self, sensor=Sensor.TEMPERATURE_AND_HUMIDITY):
        result = self.write_register(RegisterAddress.SENS_RUN, sensor)
        if result == Result.STATUS_OK:
            return self.single_shot_measure(sensor)
        return result

    def stop_continuous_measure(self, sensor=Sensor.TEMPERATURE_AND_HUMIDITY):
        return self.write_register(RegisterAddress.SENS_STOP, sensor)

    def set_low_power(self, enable):
        if enable:
            return self.write_register(RegisterAddress.SYS_CTRL, SystemControl.ENABLE_LOW_POWER)
        else:
            return self.write_register(RegisterAddress.SYS_CTRL, SystemControl.DISABLE_LOW_POWER)

    def reset(self):
        result = self.write_register(RegisterAddress.SYS_CTRL, SystemControl.RESET)
        if result == Result.STATUS_OK:
            time.sleep(SystemTiming.BOOTING)
        return result

    def read_register(self, register, length):
        try:
            msg = smbus2.i2c_msg.write(self.address, [register])
            self.bus.i2c_rdwr(msg)
            msg = smbus2.i2c_msg.read(self.address, length)
            self.bus.i2c_rdwr(msg)
            data = list(msg)
            self._debug("read", data, Result.STATUS_OK)
            return data
        except Exception as e:
            self._debug("read", None, Result.STATUS_I2C_ERROR)
            return []

    def write_register(self, register, data):
        try:
            if not isinstance(data, list):
                data = [data]
            msg = smbus2.i2c_msg.write(self.address, [register] + data)
            self.bus.i2c_rdwr(msg)
            self._debug("write", data, Result.STATUS_OK)
            return Result.STATUS_OK
        except Exception as e:
            self._debug("write", None, Result.STATUS_I2C_ERROR)
            return Result.STATUS_I2C_ERROR

    def read_identifiers(self):
        self.set_low_power(False)
        time.sleep(SystemTiming.BOOTING)

        part_id = self.read_register(RegisterAddress.PART_ID, 2)
        self.part_id = struct.unpack('<H', bytes(part_id))[0] if len(part_id) == 2 else 0

        die_rev = self.read_register(RegisterAddress.DIE_REV, 2)
        self.die_rev = struct.unpack('<H', bytes(die_rev))[0] if len(die_rev) == 2 else 0

        uid = self.read_register(RegisterAddress.UID, 8)
        self.uid = struct.unpack('<Q', bytes(uid))[0] if len(uid) == 8 else 0

        self.set_low_power(True)

    def crc7(self, val):
        crc7poly = 0x89
        crc7width = 7
        crc7ivec = 0x7F
        data7width = 17

        pol = crc7poly << (data7width - crc7width - 1)
        bit = 1 << (data7width - 1)
        val = (val << crc7width) | crc7ivec

        pol <<= crc7width
        bit <<= crc7width

        while bit > (1 << (crc7width - 1)):
            if val & bit:
                val ^= pol
            bit >>= 1
            pol >>= 1

        return (val >> (data7width)) & 0x7F

    def check_data(self, data):
        data &= 0xFFFFFF
        valid = (data >> 16) & 0x01
        stored_crc = (data >> 17) & 0x7F
        payload = data & 0x1FFFF

        computed_crc = self.crc7(payload)
        if computed_crc == stored_crc:
            return Result.STATUS_OK if valid else Result.STATUS_INVALID
        return Result.STATUS_CRC_ERROR

    # Debugging methods
    def _debug(self, func, *args):
        if self.debug_stream is None:
            return

        message = f"ENS21x debug -- {func}"
        if isinstance(args[0], Result):
            message += f" status: {args[0].value}"
        elif isinstance(args[0], (list, bytes)):
            data = ' '.join([f"0x{byte:02X}" for byte in args[0]])
            status = args[1].value if len(args) > 1 else ''
            message += f" {data} status: {status}"
        print(message, file=self.debug_stream)

    # Property accessors
    @property
    def part_id(self):
        return self._part_id

    @property
    def die_rev(self):
        return self._die_rev

    @property
    def uid(self):
        return self._uid

    def set_solder_correction(self, correction=50 * 64 // 1000):
        self.solder_correction = correction

    def get_temp_kelvin(self):
        return (self.t_data - self.solder_correction) / 64.0

    def get_temp_celsius(self):
        return self.get_temp_kelvin() - 273.15

    def get_temp_fahrenheit(self):
        return (9.0 * (self.t_data - self.solder_correction) / 320.0) - 459.67

    def get_humidity_percent(self):
        return self.h_data / 512.0

    def get_absolute_humidity_percent(self):
        temp_c = self.get_temp_celsius()
        rh = self.get_humidity_percent()
        molar_mass = 18.01534
        gas_constant = 8.21447215
        saturation_pressure = 6.1121 * 2.718281828**((17.67 * temp_c) / (temp_c + 243.5))
        absolute_humidity = (saturation_pressure * rh * molar_mass) / ((273.15 + temp_c) * gas_constant)
        return absolute_humidity

class ENS215(ENS21x):
    def __init__(self, bus_number=1, address=0x47):
        super().__init__(bus_number, address)
    
    def is_connected(self):
        return self.part_id == 0x0215