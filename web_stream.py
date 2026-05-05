import cv2
from flask import Flask, Response, render_template
import mediapipe as mp
import time
import threading

app = Flask(__name__)

class VideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 240)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 160)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while True:
            if self.stopped: return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True


vs = VideoStream().start()

mp_pose = mp.solutions.pose

pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=0,
    smooth_landmarks=False, 
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

def gen_frames():
    p_time = 0
    while True:
        frame = vs.read()
        if frame is None: continue
        
        img = frame.copy()
        
        rgb_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb_frame)

        if results.pose_landmarks:
            lm = results.pose_landmarks.landmark
            h, w, _ = img.shape
            try:
                # Wybieramy tylko kluczowe punkty
                pts = {
                    11: (int(lm[11].x * w), int(lm[11].y * h)), # r_sh
                    12: (int(lm[12].x * w), int(lm[12].y * h)), # l_sh
                    23: (int(lm[23].x * w), int(lm[23].y * h)), # r_hi
                    24: (int(lm[24].x * w), int(lm[24].y * h))  # l_hi
                }

                # Rysowanie tułowia
                cv2.line(img, pts[11], pts[12], (0, 255, 0), 2)
                cv2.line(img, pts[11], pts[23], (0, 255, 0), 2)
                cv2.line(img, pts[12], pts[24], (0, 255, 0), 2)
                cv2.line(img, pts[23], pts[24], (0, 255, 0), 2)

                cx = int((pts[11][0] + pts[12][0]) / 2)
                cy = int((pts[11][1] + pts[23][1]) / 2)
                cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)
            except: pass

        # FPS
        c_time = time.time()
        fps = 1 / (c_time - p_time)
        p_time = c_time
        cv2.putText(img, f"FPS: {int(fps)}", (10, 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Kompresja
        ret, buffer = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def home(): return render_template("index.html")

@app.route('/video_feed')
def video_feed(): return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)