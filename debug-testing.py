import smbus

bus = smbus.SMBus(1)  # Or 0, depending on your setup
address = 0x47      # The I2C address of your sensor

try:
    part_id = bus.read_word_data(address, 0x00)  # Read Part ID
    print(f"Part ID: 0x{part_id:04X}")
except Exception as e:
    print(f"Error: {e}")