# test_yaw_verify.py
from inav_control import MSPController
import time

msp = MSPController()

# Najpierw odczyt bez override
channels = msp.get_rc_channels()
print(f"Przed override — kanał 3 (yaw): {channels[2] if channels else 'brak'}")

# Wyślij yaw = 1560
msp.set_yaw(1560)
time.sleep(0.1)

# Odczyt po wysłaniu
channels = msp.get_rc_channels()
print(f"Po set_yaw(1560) — kanał 3 (yaw): {channels[2] if channels else 'brak'}")

msp.close()