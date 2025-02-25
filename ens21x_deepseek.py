import time
import smbus2

class ENS21x:
    class SystemTiming:
        BOOTING = 2  # Booting time in ms (also after reset, or going to high power)
        CONVERSION_SINGLE_SHOT = 130  # Conversion time in ms for single shot T/H measurement
        CONVERSION_CONTINUOUS = 238  # Conversion time in ms for continuous T/H measurement

    class SystemControl:
        DISABLE_LOW_POWER = 0
        ENABLE_LOW_POWER = 1 << 0
        RESET = 1 << 7

    class RegisterAddress:
        PART_ID = 0x00  # Identifies the part as ENS21x
        DIE_REV = 0x02  # Identifies the die revision version
        UID = 0x04  # Unique identifier
        SYS_CTRL = 0x10  # System configuration
        SYS_STAT = 0x11  # System status
        SENS_RUN = 0x21  # The run mode (single shot or continuous)
        SENS_START = 0x22  # Start measurement
        SENS_STOP = 0x23  # Stop continuous measurement
        SENS_STAT = 0x24  # Sensor status (idle or measuring)
        T_VAL = 0x30  # Temperature readout
        H_VAL = 0x33  # Relative humidity readout

    class Sensor:
        TEMPERATURE = 1 << 0
        HUMIDITY = 1 << 1
        TEMPERATURE_AND_HUMIDITY = TEMPERATURE | HUMIDITY

    class Result:
        STATUS_I2C_ERROR = 4  # There was an I2C communication error, `read`ing the value.
        STATUS_CRC_ERROR = 3  # The value was read, but the CRC over the payload (valid and data) does not match.
        STATUS_INVALID = 2  # The value was read, the CRC matches, but the data is invalid (e.g. the measurement was not yet finished).
        STATUS_OK = 1  # The value was read, the CRC matches, and data is valid.

    def __init__(self, bus, address=0x43):
        self.bus = bus
        self.address = address
        self.solder_correction = 0
        self.part_id = 0
        self.die_rev = 0
        self.uid = 0
        self.t_data = 0
        self.h_data = 0
        self.t_status = self.Result.STATUS_I2C_ERROR
        self.h_status = self.Result.STATUS_I2C_ERROR
        self.debug_stream = None

    def begin(self):
        self.read_identifiers()
        return self.is_connected()

    def is_connected(self):
        return self.part_id == 0x0215

    def enable_debugging(self, debug_stream):
        self.debug_stream = debug_stream

    def disable_debugging(self):
        self.debug_stream = None

    def update(self, ms=SystemTiming.CONVERSION_CONTINUOUS):
        time.sleep(ms / 1000.0)
        buffer = self.read(self.RegisterAddress.T_VAL, 6)
        if buffer:
            self.t_data = int.from_bytes(buffer[:2], byteorder='little')
            self.t_status = self.check_data(int.from_bytes(buffer[:4], byteorder='little'))

            self.h_data = int.from_bytes(buffer[3:5], byteorder='little')
            self.h_status = self.check_data(int.from_bytes(buffer[3:7], byteorder='little'))
            return self.Result.STATUS_OK
        return self.Result.STATUS_I2C_ERROR

    def single_shot_measure(self, sensor=Sensor.TEMPERATURE_AND_HUMIDITY):
        result = self.write(self.RegisterAddress.SENS_START, sensor)
        if result == self.Result.STATUS_OK:
            result = self.update(self.SystemTiming.CONVERSION_SINGLE_SHOT)
            if result == self.Result.STATUS_OK:
                if sensor == self.Sensor.TEMPERATURE:
                    result = self.t_status
                elif sensor == self.Sensor.HUMIDITY:
                    result = self.h_status
                elif sensor == self.Sensor.TEMPERATURE_AND_HUMIDITY:
                    result = self.t_status if self.t_status != self.Result.STATUS_OK else self.h_status
        return result

    def start_continuous_measure(self, sensor=Sensor.TEMPERATURE_AND_HUMIDITY):
        result = self.write(self.RegisterAddress.SENS_RUN, sensor)
        if result == self.Result.STATUS_OK:
            result = self.single_shot_measure(sensor)
        return result

    def stop_continuous_measure(self, sensor=Sensor.TEMPERATURE_AND_HUMIDITY):
        return self.write(self.RegisterAddress.SENS_STOP, sensor)

    def set_low_power(self, enable):
        if enable:
            return self.write(self.RegisterAddress.SYS_CTRL, self.SystemControl.ENABLE_LOW_POWER)
        else:
            return self.write(self.RegisterAddress.SYS_CTRL, self.SystemControl.DISABLE_LOW_POWER)

    def reset(self):
        result = self.write(self.RegisterAddress.SYS_CTRL, self.SystemControl.RESET)
        if result == self.Result.STATUS_OK:
            time.sleep(self.SystemTiming.BOOTING / 1000.0)
        return result

    def get_part_id(self):
        return self.part_id

    def get_die_rev(self):
        return self.die_rev

    def get_uid(self):
        return self.uid

    def set_solder_correction(self, correction=50 * 64 / 1000):
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
        molar_mass_of_water = 18.01534
        universal_gas_constant = 8.21447215
        temp_celsius = self.get_temp_celsius()
        return (6.1121 * pow(2.718281828, (17.67 * temp_celsius) / (temp_celsius + 243.5)) * self.get_humidity_percent() * molar_mass_of_water) / ((273.15 + temp_celsius) * universal_gas_constant)

    def get_data_t(self):
        return self.t_data

    def get_data_h(self):
        return self.h_data

    def get_status_t(self):
        return self.t_status

    def get_status_h(self):
        return self.h_status

    def read(self, address, size):
        try:
            write = smbus2.i2c_msg.write(self.address, [address])
            read = smbus2.i2c_msg.read(self.address, size)
            self.bus.i2c_rdwr(write, read)
            return list(read)
        except:
            return None

    def write(self, address, data):
        try:
            if isinstance(data, int):
                data = [data]
            self.bus.write_i2c_block_data(self.address, address, data)
            return self.Result.STATUS_OK
        except:
            return self.Result.STATUS_I2C_ERROR

    def read_identifiers(self):
        self.set_low_power(False)
        time.sleep(self.SystemTiming.BOOTING / 1000.0)

        self.part_id = int.from_bytes(self.read(self.RegisterAddress.PART_ID, 2), byteorder='little')
        self.die_rev = int.from_bytes(self.read(self.RegisterAddress.DIE_REV, 2), byteorder='little')
        self.uid = int.from_bytes(self.read(self.RegisterAddress.UID, 8), byteorder='little')

        self.set_low_power(True)

    def crc7(self, val):
        crc7width = 7
        crc7poly = 0x89
        crc7ivec = 0x7F
        data7width = 17
        data7mask = ((1 << data7width) - 1)
        data7msb = (1 << (data7width - 1))

        pol = crc7poly << (data7width - crc7width - 1)
        bit = data7msb
        val = val << crc7width
        bit = bit << crc7width
        pol = pol << crc7width
        val |= crc7ivec

        while bit & (data7mask << crc7width):
            if bit & val:
                val ^= pol
            bit >>= 1
            pol >>= 1
        return val

    def check_data(self, data):
        data &= 0xffffff
        valid = (data >> 16) & 0x01
        crc = (data >> 17) & 0x7f
        payload = data & 0x1ffff

        if self.crc7(payload) == crc:
            return self.Result.STATUS_OK if valid == 1 else self.Result.STATUS_INVALID
        return self.Result.STATUS_CRC_ERROR

    def debug(self, msg):
        if self.debug_stream:
            self.debug_stream.write(f"ENS21x debug -- {msg}\n")

    def debug_data(self, msg, data, result):
        if self.debug_stream:
            self.debug_stream.write(f"ENS21x debug -- {msg} {[hex(d) for d in data]} status: {result}\n")
class ENS215(ENS21x):
    def __init__(self, bus_num=1, address=0x47):
        super().__init__(bus_num, address)

    def is_connected(self):
        return self.part_id == 0x0215
    