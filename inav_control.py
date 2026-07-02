# inav_control.py

import serial
import time
import config
import threading


class MSPController:
    MSP_ANALOG = 110
    MSP_SET_RAW_RC = 200
    MSP_RC = 105

    def __init__(self):
        self.ser = serial.Serial(
            config.UART_PORT,
            config.UART_BAUDRATE,
            timeout=0.1
        )
        self.uart_lock = threading.Lock()
        time.sleep(0.5)

    def _build_request(self, command, payload=b''):
        """Buduje ramkę zapytania MSP v1: $M< [size] [cmd] [payload] [checksum]"""
        size = len(payload)
        checksum = size ^ command
        for b in payload:
            checksum ^= b
        return bytes([ord('$'), ord('M'), ord('<'), size, command]) + payload + bytes([checksum])
    
    def _read_response(self, expected_command, timeout=0.5):
        """
        Czyta i parsuje odpowiedź MSP. Zwraca payload (bytes) lub None przy błędzie.
        """
        deadline = time.time() + timeout
        buffer = b''

        while time.time() < deadline:
            chunk = self.ser.read(64)
            if chunk:
                buffer += chunk
                # Szukanie nagłówka odpowiedzi $M>
                idx = buffer.find(b'$M>')
                if idx != -1 and len(buffer) >= idx + 5:
                    size = buffer[idx + 3]
                    cmd = buffer[idx + 4]
                    total_len = idx + 5 + size + 1  # nagłówek+size+cmd + payload + checksum

                    if len(buffer) >= total_len:
                        payload = buffer[idx + 5: idx + 5 + size]
                        if cmd == expected_command:
                            return payload
                        return None
            else:
                time.sleep(0.005)

        return None

    def get_rc_channels(self):
        with self.uart_lock:
            request = self._build_request(self.MSP_RC)
            self.ser.reset_input_buffer()
            self.ser.write(request)

            payload = self._read_response(self.MSP_RC)
        if payload and len(payload) >= 2:
            channels = []
            for i in range(0, len(payload), 2):
                if i + 1 < len(payload):
                    val = int.from_bytes(payload[i:i+2], byteorder='little')
                    channels.append(val)
            return channels
        return None

    def set_yaw(self, yaw_value):
        yaw_value = max(1000, min(2000, int(yaw_value)))

        channels = [1500] * 18
        channels[3] = yaw_value

        payload = b''
        for ch in channels:
            payload +=ch.to_bytes(2, byteorder='little')
        
        with self.uart_lock:
            request = self._build_request(self.MSP_SET_RAW_RC, payload)
            self.ser.reset_input_buffer()
            self.ser.write(request)

    def close(self):
        self.ser.close()