from inav_control import MSPController
import time

msp = MSPController()

for i in range(5):
    voltage = msp.get_battery_voltage()
    print(f"Próba {i+1}: napięcie = {voltage} V")
    time.sleep(1)

msp.close()