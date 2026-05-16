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
            print(f"Tello Baterry: {self.tello.get_battery()}")
            self.tello.streamon()
            time.sleep(2)
            self.cap = cv2.VideoCapture('udp://@0.0.0.0:11111', cv2.CAP_FFMPEG)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
        except Exception as e:
            print(f"Tello connection error: {e}")

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                resized = cv2.resize(frame, (self.width, self.height))
                self.frame = resized
            else:
                self.cap.release()
                self.cap = cv2.VideoCapture('udp://@0.0.0.0:11111', cv2.CAP_FFMPEG)
                time.sleep(0.5)

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.tello.streamoff()