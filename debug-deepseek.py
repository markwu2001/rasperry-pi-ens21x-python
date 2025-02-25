from ens21x_deepseek import ENS215, ENS21x

ens = ENS215(bus_number=1)
ens.begin()
ens.single_shot_measure()
print(f"Temperature: {ens.get_temp_celsius()}°C")
print(f"Humidity: {ens.get_humidity_percent()}%")