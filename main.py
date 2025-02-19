import smbus2
import time

class ENS210:
    def __init__(self, bus, address):
        self.bus = smbus2.SMBus(bus)
        self.address = address

        # Check device ID (recommended)
        device_id = self.read_byte(0x00)  # WHO_AM_I register
        if device_id != 0x6A:  # ENS210 ID
            raise RuntimeError("ENS210 not found at address 0x{:02x}".format(address))

    def read_byte(self, register):
        try:
            return self.bus.read_byte_data(self.address, register)
        except Exception as e:
            print(f"Error reading byte from register 0x{register:02x}: {e}")
            return None

    def read_block(self, register, length):
        try:
            return self.bus.read_i2c_block_data(self.address, register, length)
        except Exception as e:
            print(f"Error reading block from register 0x{register:02x}: {e}")
            return None

    def write_byte(self, register, value):
        try:
            self.bus.write_byte_data(self.address, register, value)
            return True
        except Exception as e:
            print(f"Error writing byte 0x{value:02x} to register 0x{register:02x}: {e}")
            return False

    def read_temperature(self):
        """Reads temperature in milli-degrees Celsius."""
        data = self.read_block(0x01, 2)  # TEMP_OUT register (2 bytes)
        if data is None:
            return None

        temperature_raw = (data[0] << 8) | data[1]
        # Datasheet says value is already in milli-degrees C
        return temperature_raw  # Already in milli-degrees Celsius

    def read_humidity(self):
        """Reads relative humidity in milli-percent."""
        data = self.read_block(0x03, 2)  # RH_OUT register (2 bytes)
        if data is None:
            return None
        humidity_raw = (data[0] << 8) | data[1]
        return humidity_raw  # Already in milli-percent RH

    def read_interrupt_status(self):
        return self.read_byte(0x06)

    def close(self):
        self.bus.close()

# Example Usage:
if __name__ == "__main__":
    i2c_bus = 1  # Replace with your I2C bus number
    ens210_address = 0x47  # Default ENS210 address

    try:
        ens210 = ENS210(i2c_bus, ens210_address)

        while True:
            temperature = ens210.read_temperature()
            humidity = ens210.read_humidity()
            interrupt_status = ens210.read_interrupt_status()

            if temperature is not None and humidity is not None and interrupt_status is not None:
                print(f"Temperature: {temperature / 1000.0:.2f} °C")
                print(f"Humidity: {humidity / 1000.0:.2f} %RH")
                print(f"Interrupt Status: 0x{interrupt_status:02x}")
            
            time.sleep(1)

    except RuntimeError as e:
        print(e)  # Print the error from the device ID check
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if 'ens210' in locals(): # Check if ens210 was instantiated
            ens210.close()