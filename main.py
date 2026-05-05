import cv2
import time
import json
from flask import Flask, Response, render_template
from camera import VideoStream
from regulator import EMAFilter, PID
from vision import PoseDetector

app = Flask(__name__)
vs = VideoStream().start()
detector = PoseDetector()

center_x, center_y = 120, 80
initialized = False

telemetry_data = {"err_x": 0, "err_y": 0, "ctrl_x": 0, "ctrl_y": 0}

filter_x = EMAFilter(alpha=0.2)
filter_y = EMAFilter(alpha=0.2)

pid_x = None
pid_y = None



def gen_frames():
    global telemetry_data, pid_x, pid_y, initialized
    p_time = 0

    while True:
        frame = vs.read()
        if frame is None: continue
        img = frame.copy()

        h, w, _ = img.shape
        c_x, c_y = w // 2, h // 2

        if not initialized:
            pid_x = PID(kp=0.6, ki=0.01, kd=0.1, setpoint=c_x)
            pid_y = PID(kp=0.6, ki=0.01, kd=0.1, setpoint=c_y)
            initialized = True

        cv2.line(img, (c_x - 10, c_y), (c_x + 10, c_y), (255, 255, 255), 1)
        cv2.line(img, (c_x, c_y - 10), (c_x, c_y + 10), (255, 255, 255), 1)

        # Detekcja
        cx, cy, pts = detector.find_torso(img)

        if cx is not None and initialized:
            # Filtrowanie i PID
            s_cx = filter_x.apply(cx)
            s_cy = filter_y.apply(cy)
            
            ctrl_x = pid_x.update(s_cx)
            ctrl_y = pid_y.update(s_cy)

            telemetry_data = {
                "err_x": int(c_x - s_cx),
                "err_y": int(c_y - s_cy),
                "ctrl_x": round(ctrl_x, 1),
                "ctrl_y": round(ctrl_y, 1)
            }

            # Rysowanie (wizualizacja)
            cv2.line(img, pts[11], pts[12], (0, 255, 0), 2)
            cv2.line(img, pts[11], pts[23], (0, 255, 0), 2)
            cv2.line(img, pts[12], pts[24], (0, 255, 0), 2)
            cv2.line(img, pts[23], pts[24], (0, 255, 0), 2)
            
            cv2.circle(img, (int(s_cx), int(s_cy)), 5, (0, 0, 255), -1)

        # FPS
        fps = 1 / (time.time() - p_time)
        p_time = time.time()
        cv2.putText(img, f"FPS: {int(fps)}", (10, 25), 1, 1, (0, 255, 0), 2)

        ret, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 65])
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def home(): return render_template("index.html")

@app.route('/video_feed')
def video_feed(): return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry')
def telemetry(): return Response(json.dumps(telemetry_data), mimetype='application/json')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)