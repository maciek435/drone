import cv2
import time
import json
import threading
from flask import Flask, Response, render_template
from regulator import KalmanLite, Regulator, DistanceRegulator
from vision import PoseDetector
import config
from pi_camera import PiVideoStream
from inav_control import MSPController

msp = MSPController()
app = Flask(__name__)
vs = PiVideoStream().start()
detector = PoseDetector()


filter_x = KalmanLite(
    process_noise=config.KALMAN_PROCESS_NOISE_XY,
    measurement_noise=config.KALMAN_MEASUREMENT_NOISE_XY
)
filter_y = KalmanLite(
    process_noise=config.KALMAN_PROCESS_NOISE_XY,
    measurement_noise=config.KALMAN_MEASUREMENT_NOISE_XY
)
# filter_z = KalmanLite(process_noise=0.1, measurement_noise=3.0) #0.05 5.0
reg_x = Regulator(kp=config.REG_X_KP, kd=config.REG_X_KD, max_output=config.REG_X_MAX_OUTPUT)
# reg_y = Regulator(kp=0.5, kd=0.3, max_output=40) #50
# reg_z = DistanceRegulator(kp=1.5, max_jump=15, max_output=50)

telemetry_data = {
    "err_x": 0, 
    "err_y": 0,
    "batt": "--",
    "detected": False, 
    "follow": False
}

# last_filtered_height = 0
latest_detection = {"cx": None, "cy": None, "h_tors": None, "pts": None}
detection_lock = threading.Lock()

latest_filtered = {"cx": None, "cy": None}
filtered_lock = threading.Lock()

latest_baterry = {"voltage": None}
# battery_lock = threading.Lock()

follow_active = False
follow_lock = threading.Lock()


def switch_worker():
    global follow_active
    while True:
        channels = msp.get_rc_channels()
        if channels and len(channels) >= 8:
            switch_val = channels[7]
            with follow_lock:
                follow_active = switch_val > 1700
        time.sleep(0.1)

def detection_worker():
    while True:
        frame = vs.read()
        if frame is None:
            time.sleep(0.01)
            continue
        cx, cy, h_tors, pts = detector.find_torso(frame)
        with detection_lock:
            latest_detection["cx"] = cx
            latest_detection["cy"] = cy
            latest_detection["h_tors"] = h_tors
            latest_detection["pts"] = pts



def flight_worker():
    while True:
        with follow_lock:
                active = follow_active
        

        if active:
            with filtered_lock:
                s_cx = latest_filtered["cx"]


            if s_cx is not None:
                frame_center_x = config.FRAME_WIDTH // 2
                error_x = s_cx - frame_center_x

                if abs(error_x) < config.DEADZONE_W:
                    msp.set_yaw(1500)
                else:
                    correction = reg_x.update(error_x)
                    yaw = int(1500 + correction)
                    msp.set_yaw(yaw)
            else:
                reg_x.reset()
                msp.set_yaw(1500)

        time.sleep(0.05)


def gen_frames():
    global telemetry_data
    p_time = 0

    while True:
        frame = vs.read()
        if frame is None:
            time.sleep(0.01)
            continue

        # with battery_lock:
        #     batt = latest_baterry["voltage"]
        # batt_str = f"{batt}V" if batt is not None else "--"
        batt_str = "--"

        with follow_lock:
            follow = follow_active

        img = frame
        h, w, _ = img.shape
        c_x, c_y = w // 2, h // 2

        cv2.line(img, (c_x - 10, c_y), (c_x + 10, c_y), (255, 255, 255), 1)
        cv2.line(img, (c_x, c_y - 10), (c_x, c_y + 10), (255, 255, 255), 1)

        cv2.rectangle(
            img,
            (c_x - config.DEADZONE_W, c_y - config.DEADZONE_H),
            (c_x + config.DEADZONE_W, c_y + config.DEADZONE_H),
            (0, 165, 255), 1
        )

        with detection_lock:
            cx = latest_detection["cx"]
            cy = latest_detection["cy"]
            h_tors = latest_detection["h_tors"]
            pts = latest_detection["pts"]

        if cx is not None:
            s_cx = filter_x.apply(cx)
            s_cy = filter_y.apply(cy)

            with filtered_lock:
                latest_filtered["cx"] = s_cx
                latest_filtered["cy"] = s_cy

            telemetry_data = {
                "err_x": int(c_x - s_cx),
                "err_y": int(c_y - s_cy),
                "batt": batt_str,
                "detected": True,
                "follow": follow
            }

            if pts:
                cv2.line(img, pts[11], pts[12], (0, 255, 0), 2)
                cv2.line(img, pts[11], pts[23], (0, 255, 0), 2)
                cv2.line(img, pts[12], pts[24], (0, 255, 0), 2)
                cv2.line(img, pts[23], pts[24], (0, 255, 0), 2)

            cv2.circle(img, (int(s_cx), int(s_cy)), 5, (0, 0, 255), -1)
        else:
            with filtered_lock:
                latest_filtered["cx"] = None
                latest_filtered["cy"] = None

            telemetry_data = {
                "err_x": 0, "err_y": 0,
                "batt": batt_str,
                "detected": False,
                "follow": follow
            }
            filter_x.reset()
            filter_y.reset()

        fps = 1 / (time.time() - p_time + 0.0001)
        p_time = time.time()
        cv2.putText(img, f"FPS: {int(fps)}", (10, 25), 1, 1, (0, 255, 0), 2)

        ret, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 25])
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        time.sleep(0.033)

threading.Thread(target=detection_worker, daemon=True).start()
# threading.Thread(target=battery_worker, daemon=True).start()
threading.Thread(target=switch_worker, daemon=True).start()
threading.Thread(target=flight_worker, daemon=True).start()

@app.route('/')
def home(): return render_template("index.html")

@app.route('/video_feed')
def video_feed(): return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry')
def telemetry(): return Response(json.dumps(telemetry_data), mimetype='application/json')

if __name__ == '__main__':
    app.run(
        host=config.FLASK_HOST,
        port=config.FLASK_PORT,
        threaded=True,
        use_reloader=False
    )