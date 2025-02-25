from ens21x_deepseek import ENS215, Sensor, Result

sensor = ENS215(bus=1)
if sensor.begin():
    sensor.enable_debugging()
    result = sensor.single_shot_measure()
    if result == Result.STATUS_OK:
        print(f"Temperature: {sensor.temp_celsius:.2f}°C")
        print(f"Humidity: {sensor.humidity_percent:.2f}%")
    else:
        print(f"Measurement failed: {result}")