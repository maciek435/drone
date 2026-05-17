import cv2
from djitellopy import Tello
import threading
import time

class VideoStream:
    def __init__(self, width=240, height=160):
        self.tello = Tello()
        self.frame = None
        self.stopped = False
        self.width = width
        self.height = height

        try:
            self.tello.connect()
            print(f"Tello Battery: {self.tello.get_battery()}")
            self.tello.streamon()
            time.sleep(2)
            self._open_cap()
        except Exception as e:
            print(f"Tello connection error: {e}")

    def _open_cap(self):
        self.cap = cv2.VideoCapture(
            'udp://@0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000&timeout=2000000',
            cv2.CAP_FFMPEG
        )
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(10):
            self.cap.grab()

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            self.cap.grab()
            self.cap.grab()
            ret, frame = self.cap.read()
            if ret and frame is not None:
                self.frame = cv2.resize(frame, (self.width, self.height),
                                        interpolation=cv2.INTER_NEAREST)
            else:
                print("Stream error — reconnecting...")
                self.cap.release()
                time.sleep(0.3)
                self._open_cap()

    def read(self):
        return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.stopped = True
        self.tello.streamoff()
        self.cap.release()