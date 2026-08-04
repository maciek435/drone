# import threading
# import time
# import config
# import cv2

# class PiVideoStream:
#     def __init__(self, width=320, height=240):
#         self.width = config.FRAME_WIDTH
#         self.height = config.FRAME_HEIGHT
#         self.frame = None
#         self.stopped = False

#         import picamera2
#         self.cam = picamera2.Picamera2()

#         cfg = self.cam.create_video_configuration(
#             main={"format": "RGB888", "size": (self.width, self.height)}
#         )

#         self.cam.configure(cfg)
#         self.cam.start()

#         time.sleep(2.0)

#         self.frame = self.cam.capture_array()

#     def start(self):
#         threading.Thread(target=self._update, daemon=True).start()
#         return self

#     def _update(self):
#         while not self.stopped:
#             self.frame = cv2.rotate(self.cam.capture_array(), cv2.ROTATE_180)
    
#     def read(self):
#         return self.frame.copy() if self.frame is not None else None

#     def stop(self):
#         self.stopped = True
#         time.sleep(0.1)
#         self.cam.stop()
        
import os
import time
import cv2
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
from libcamera import Transform
import config


class PiVideoStream:
    def __init__(self):
        os.makedirs(os.path.dirname(config.RECORD_PATH), exist_ok=True)

        self.picam2 = Picamera2()

        main_stream = {"size": (config.RECORD_WIDTH, config.RECORD_HEIGHT)}
        lores_stream = {"size": (config.FRAME_WIDTH, config.FRAME_HEIGHT), "format": "YUV420"}

        video_config = self.picam2.create_video_configuration(
            main=main_stream,
            lores=lores_stream,
            encode="lores",
            transform=Transform(hflip=1, vflip=1)
        )
        self.picam2.configure(video_config)

        self.recording = False

    def start(self):
        self.picam2.start()
        return self

    def start_recording(self):
        if not self.recording:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(os.path.dirname(config.RECORD_PATH), f"nagranie_{timestamp}.h264")
            self.encoder = H264Encoder(bitrate=config.RECORD_BITRATE)   # <- SWIEZY enkoder
            self.output = FileOutput(path)                              # <- SWIEZY output
            self.picam2.start_encoder(self.encoder, self.output)
            self.recording = True
            print(f"[RECORDING] start: {path}")

    def stop_recording(self):
        if self.recording:
            self.picam2.stop_encoder(self.encoder)
            self.recording = False
            print("[RECORDING] stop")

    def read(self):
        yuv = self.picam2.capture_array("lores")
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)