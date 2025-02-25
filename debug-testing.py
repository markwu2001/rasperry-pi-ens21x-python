import smbus

bus = smbus.SMBus(1)
address = 0x47

try:
    # Read the Part ID register (0x00) byte by byte
    byte1 = bus.read_byte_data(address, 0x00)
    byte2 = bus.read_byte_data(address, 0x01)

    part_id = (byte2 << 8) | byte1  # Combine the bytes (important: byte order!)

    print(f"Byte 1: 0x{byte1:02X}")
    print(f"Byte 2: 0x{byte2:02X}")
    print(f"Part ID: 0x{part_id:04X}")

except Exception as e:
    print(f"Error: {e}")