from ens21x_deepseek import ENS215, Sensor, Result
import time

sensor = ENS215(bus=1)
if sensor.begin():
    sensor.enable_debugging()
    
    # Ensure sensor is out of low power mode
    sensor.set_low_power(False)
    time.sleep(0.1)
    
    # Perform measurement
    result = sensor.single_shot_measure()
    if result == Result.STATUS_OK:
        print(f"Temperature: {sensor.temp_celsius:.2f}°C")
        print(f"Humidity: {sensor.humidity_percent:.2f}%")
    else:
        print(f"Measurement failed: {result}")