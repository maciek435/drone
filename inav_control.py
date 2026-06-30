# inav_control.py

import serial
import time
import config


class MSPController:
    MSP_ANALOG = 110

    def __init__(self):
        self.ser = serial.Serial(
            config.UART_PORT,
            config.UART_BAUDRATE,
            timeout=0.5
        )
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
                time.sleep(0.01)

        return None

    def get_battery_voltage(self):
        """
        Zwraca napięcie baterii w woltach (float), lub None jeśli brak odpowiedzi.
        MSP_ANALOG payload: [vbat(1B, 0.1V), mAhUsed(2B), rssi(2B), amperage(2B)]
        """
        request = self._build_request(self.MSP_ANALOG)
        self.ser.reset_input_buffer()
        self.ser.write(request)

        payload = self._read_response(self.MSP_ANALOG)
        if payload and len(payload) >= 1:
            vbat_raw = payload[0]
            return round(vbat_raw / 10.0, 1)
        return None

    def close(self):
        self.ser.close()