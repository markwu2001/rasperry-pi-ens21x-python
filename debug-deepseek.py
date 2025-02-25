import ens21x_deepseek
import smbus2

# Initialize I2C bus
bus = smbus2.SMBus(1)

# Initialize ENS21x sensor
sensor = ENS215(bus, address=0x47)

# Begin communication with the sensor
if sensor.begin():
    print("Sensor connected successfully!")
    
    # Perform a single shot measurement
    result = sensor.single_shot_measure()
    if result == ENS21x.Result.STATUS_OK:
        print(f"Temperature: {sensor.get_temp_celsius()} °C")
        print(f"Humidity: {sensor.get_humidity_percent()} %RH")
    else:
        print("Measurement failed!")
else:
    print("Failed to connect to the sensor!")