import threading
import time
import config

class PiVideoStream:
    def __init__(self, width=320, height=240):
        self.width = config.FRAME_WIDTH
        self.height = config.FRAME_HEIGHT
        self.frame = None
        self.stopped = False

        import picamera2
        self.cam = picamera2.Picamera2()

        cfg = self.cam.create_video_configuration(
            main={"format": "BGR888", "size": (self.width, self.height)}
        )

        self.cam.configure(cfg)
        self.cam.start()

        time.sleep(2.0)

        self.frame = self.cam.capture_array()

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        return self

    def _update(self):
        while not self.stopped:
            self.frame = self.cam.capture_array()
    
    def read(self):
        return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.stopped = True
        time.sleep(0.1)
        self.cam.stop()
        