from ens21x_deepseek import ENS215, Sensor, Result
import smbus2

# Initialize I2C bus
try:
    ens = ENS215(bus_number=1)
    if ens.begin():
        ens.single_shot_measure()
        print(f"Temperature: {ens.get_temp_celsius():.2f}°C")
        print(f"Humidity: {ens.get_humidity_percent():.2f}%")
except RuntimeError as e:
    print(f"Error: {e}")