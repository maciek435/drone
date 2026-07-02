from inav_control import MSPController
import time

msp = MSPController()

print("Wysyłam 1560 tylko na kanale 4 (indeks 3)...")

start = time.time()
while time.time() - start < 15:
    channels = [1500] * 18
    channels[3] = 1560  # indeks 3 = kanał 4
    payload = b''
    for ch in channels:
        payload += ch.to_bytes(2, byteorder='little')
    with msp.uart_lock:
        request = msp._build_request(msp.MSP_SET_RAW_RC, payload)
        msp.ser.reset_input_buffer()
        msp.ser.write(request)
    time.sleep(0.05)

print("Koniec")
msp.close()