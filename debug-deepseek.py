from ens21x_deepseek import ENS215, Sensor, Result

# Initialize sensor with debugging
sensor = ENS215(bus_num=1, address=0x47)
sensor.enable_debugging(print)  # Enable debug output to print

if sensor.begin():
    print("Sensor connected!")
    print(f"PART_ID: {hex(sensor.part_id)}")
    print(f"DIE_REV: {hex(sensor.die_rev)}")
    print(f"UID: {hex(sensor.uid)}")
    
    # Perform measurement with status check
    result = sensor.single_shot_measure(Sensor.TEMPERATURE_AND_HUMIDITY)
    print(f"\nMeasurement result: {result.name}")
    
    if result == Result.STATUS_OK:
        print(f"Raw T: {sensor.t_data} (Status: {sensor.t_status.name})")
        print(f"Raw H: {sensor.h_data} (Status: {sensor.h_status.name})")
        print(f"Temperature: {sensor.temperature_celsius:.2f}°C")
        print(f"Humidity: {sensor.humidity_percent:.2f}%")
    else:
        print("Measurement failed. Possible issues:")
        print("- Sensor not properly connected")
        print("- Incorrect I2C address")
        print("- CRC validation failed")
else:
    print("Sensor not found. Check:")
    print("- I2C connection and address")
    print("- Bus number (try 0 for older Pi models)")
    print("- Sensor power supply")