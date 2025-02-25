import smbus
import time

class ENS21x:
    def __init__(self, address=0x43, debug=False):
        self.address = address
        self.bus = smbus.SMBus(1)  # Use 1 for /dev/i2c-1 (usually on RPi)
        self.debug = debug
        self.solder_correction = 0
        self.part_id = 0
        self.die_rev = 0
        self.uid = 0
        self.t_data = 0
        self.h_data = 0
        self.t_status = None
        self.h_status = None
        self.read_identifiers()

    def begin(self):
        """Initializes the sensor. Returns True if successful, False otherwise."""
        # No explicit begin needed in Python with smbus.  Identifiers are read in __init__.
        return self.is_connected()

    def is_connected(self):
      """Checks if the device is connected by attempting to read the Part ID"""
      try:
        part_id = self.read_word(0x00) #PART_ID register
        return True
      except Exception as e:
        if self.debug:
          print(f"Connection check failed: {e}")
        return False


    def update(self, ms):
        """Reads temperature and humidity data."""
        time.sleep(ms / 1000.0)  # Convert ms to seconds

        try:
            data = self.bus.read_i2c_block_data(self.address, 0x04, 6) # T_VAL register
            self.t_data = (data[1] << 8) | data[0]
            self.t_status = self.check_data((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0])
            self.h_data = (data[4] << 8) | data[5]
            self.h_status = self.check_data((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2])
            return True # Simulate Result::STATUS_OK
        except Exception as e:
            if self.debug:
                print(f"Error reading data: {e}")
            return False  # Simulate a failed Result

    def single_shot_measure(self, sensor):
        """Performs a single-shot measurement."""
        if sensor not in (0,1,2): #Sensor enum values
          raise ValueError("Invalid sensor value. Use 0 for TEMPERATURE, 1 for HUMIDITY, 2 for TEMPERATURE_AND_HUMIDITY")
        try:
          self.write_byte(0x01, sensor) # SENS_START register
          time.sleep(0.05) # Wait for conversion (adjust if needed)
          self.update(50) #Update after the conversion
          if sensor == 0:
            return self.t_status
          elif sensor == 1:
            return self.h_status
          else:
            return self.t_status if self.t_status != False else self.h_status #Python boolean instead of Result enum
        except Exception as e:
          if self.debug:
            print(f"Error in single_shot_measure: {e}")
          return False

    def start_continuous_measure(self, sensor):
        """Starts continuous measurements."""
        if sensor not in (0,1,2):
          raise ValueError("Invalid sensor value. Use 0 for TEMPERATURE, 1 for HUMIDITY, 2 for TEMPERATURE_AND_HUMIDITY")
        try:
          self.write_byte(0x02, sensor) # SENS_RUN register
          return self.single_shot_measure(sensor)
        except Exception as e:
          if self.debug:
            print(f"Error in start_continuous_measure: {e}")
          return False

    def stop_continuous_measure(self, sensor):
        """Stops continuous measurements."""
        if sensor not in (0,1,2):
          raise ValueError("Invalid sensor value. Use 0 for TEMPERATURE, 1 for HUMIDITY, 2 for TEMPERATURE_AND_HUMIDITY")
        try:
          self.write_byte(0x03, sensor) # SENS_STOP register
          return True
        except Exception as e:
          if self.debug:
            print(f"Error in stop_continuous_measure: {e}")
          return False


    def set_low_power(self, enable):
        """Enables or disables low power mode."""
        try:
          if enable:
              self.write_byte(0x08, 0x01) # SYS_CTRL register, ENABLE_LOW_POWER
          else:
              self.write_byte(0x08, 0x00) # SYS_CTRL register, DISABLE_LOW_POWER
          return True
        except Exception as e:
          if self.debug:
            print(f"Error in set_low_power: {e}")
          return False

    def reset(self):
        """Resets the sensor."""
        try:
          self.write_byte(0x08, 0x04) # SYS_CTRL register, RESET
          time.sleep(0.05)  # Wait for boot
          self.read_identifiers() #Re-read identifiers after reset
          return True
        except Exception as e:
          if self.debug:
            print(f"Error in reset: {e}")
          return False

    def get_part_id(self):
        return self.part_id

    def get_die_rev(self):
        return self.die_rev

    def get_uid(self):
        return self.uid

    def set_solder_correction(self, correction):
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
        # (Simplified) August-Roche-Magnus formula for demonstration
        t = self.get_temp_celsius()
        return (6.1121 * 2.71828 ** ((17.67 * t) / (t + 243.5)) * self.get_humidity_percent() * 18.01534) / ((273.15 + t) * 8.21447215)


    def get_data_t(self):
        return self.t_data

    def get_data_h(self):
        return self.h_data

    def get_status_t(self):
        return self.t_status

    def get_status_h(self):
        return self.h_status

    def read_word(self, register):
        try:
            return self.bus.read_word_data(self.address, register)
        except Exception as e:
            if self.debug:
              print(f"Error reading word: {e}")
            return 0 # Or raise the exception if you prefer

    def write_byte(self, register, value):
        try:
            self.bus.write_byte_data(self.address, register, value)
        except Exception as e:
            if self.debug:
              print(f"Error writing byte: {e}")

    def read_identifiers(self):
        if self.debug:
          print("Reading Identifiers")
        self.set_low_power(False)
        time.sleep(0.05)

        self.part_id = self.read_word(0x00)
        if self.debug:
          print(f"PART_ID: 0x{self.part_id:04X}")

        self.die_rev = self.read_word(0x02)
        if self.debug:
          print(f"DIE_REV: 0x{self.die_rev:04X}")

        # UID is 64 bits, needs special handling

class ENS215(ENS21x): # ENS215 class inheriting from ENS21x
    def __init__(self, address=0x47, debug=False):  # ENS215 uses 0x47 by default
        super().__init__(address, debug)  # Call the parent class's constructor
        # self.debug_prefix = "ENS215 debug -- "  # If you want a different debug prefix

    def is_connected(self):
        """Checks if the device is an ENS215."""
        return self.part_id == 0x0215

    def begin(self, address=None): #Override begin to allow no parameters
      """Initializes the sensor. Returns True if successful, False otherwise."""
      if address is None:
        return super().begin() #Call base class begin with default address
      else:
        return super().begin(address) #Call base class begin with given address

# Example usage (updated):
ens = ENS215(debug=True)  # Create an ENS215 object

if not ens.begin():
    print("ENS215 sensor not found. Check I2C connection and address.")
    exit()

if not ens.is_connected():
    print("Device is not an ENS215. Part ID:", hex(ens.get_part_id()))
    exit()