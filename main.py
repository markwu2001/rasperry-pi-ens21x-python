import smbus2
import time

# AI Generated brainrot

class TemperatureSensor:
    def __init__(self, bus, address):
        self.bus = smbus2.SMBus(bus)  # Use smbus2 for better I2C support
        self.address = address

    def read_temperature(self):
        """Reads the temperature from the sensor.

        This example assumes a simple sensor that returns a 16-bit
        temperature value (two bytes).  You'll likely need to adapt
        this based on your specific sensor's datasheet.
        """
        try:
            # Read two bytes from the sensor
            data = self.bus.read_i2c_block_data(self.address, 0x48, 2) #0x00 is an example command or register; adapt to your sensor

            # Combine the bytes into a single value (adjust byte order if needed)
            # Example: Assuming high byte first (common for temperature)
            temperature_raw = (data[0] << 8) | data[1]

            # Convert the raw value to temperature (consult your sensor's datasheet)
            # Example: Assuming the sensor returns temperature in units of 0.01 degrees C
            temperature_celsius = temperature_raw / 100.0

            return temperature_celsius

        except Exception as e:
            print(f"Error reading temperature: {e}")
            return None

    def close(self):
        self.bus.close()



# Example usage:
if __name__ == "__main__":
    i2c_bus = 1  # The I2C bus number on your Raspberry Pi (usually 1)
    sensor_address = 0x43 # ENS21x

    sensor = TemperatureSensor(i2c_bus, sensor_address)

    try:
        while True:
            temperature = sensor.read_temperature()
            if temperature is not None:
                print(f"Temperature: {temperature:.2f} °C")

            time.sleep(1)  # Read every second

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        sensor.close() # Important to close the I2C bus