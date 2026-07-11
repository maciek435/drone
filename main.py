import cv2
import time
import json
import threading
import config
from flask import Flask, Response, render_template
from tello_camera import VideoStream
from regulator import KalmanLite, Regulator, DistanceRegulator, KalmanCV1D
from vision import PoseDetector
from tracker import HybridBodyTracker


app = Flask(__name__)
vs = VideoStream().start()
detector = PoseDetector()

last_extra = {"h_tors": None, "pts": None}

def detect_fn(frame):
    result = detector.find_torso(frame)

    if result is None:
        return None
    
    last_extra["h_tors"] = result["h_tors"]
    last_extra["pts"] = result["pts"]

    return (result["cx"], result["cy"], result["bbox"])

tracker = HybridBodyTracker(
    detect_fn,
    detect_every_n=config.TRACKER_DETECT_EVERY_N,
    gate_radius=config.TRACKER_GATE_RADIUS,
    confirm_frames=config.TRACKER_CONFIRM_FRAMES,
    max_misses=config.TRACKER_MAX_MISSES,
    q=config.TRACKER_KALMAN_Q,
    r=config.TRACKER_KALMAN_R,
)

filter_z = KalmanLite(process_noise=config.FILTER_Z_PROCESS_NOISE, measurement_noise=config.FILTER_Z_MEASUREMENT_NOISE)
reg_x = Regulator(kp=config.REG_X_KP, kd=config.REG_X_KD, max_output=config.REG_X_MAX_OUTPUT) 
reg_y = Regulator(kp=config.REG_Y_KP, kd=config.REG_Y_KD, max_output=config.REG_Y_MAX_OUTPUT) 
reg_z = DistanceRegulator(kp=config.REG_Z_KP, max_jump=config.REG_Z_MAX_JUMP, max_output=config.REG_Z_MAX_OUTPUT)

target_angle_x = 0
target_angle_y = 0
target_speed_z = 0
telemetry_data = {"err_x": 0, "err_y": 0, "ctrl_x": 0, "ctrl_y": 0}
running = True
follow_active = False
last_filtered_height = 0

latest_track = {"cx": None, "cy": None, "locked": False}
track_lock = threading.Lock()

cached_battery = 0
last_battery_time = 0

def flight_worker():
    global target_angle_x, target_angle_y, target_speed_z, running, follow_active

    while running:
        if follow_active and target_angle_x != 0:
            yaw_speed = int(target_angle_x)
            if abs(yaw_speed) < config.DEADZONE:
                yaw_speed = 0
            up_down_speed = int(target_angle_y)
            if abs(up_down_speed) < config.DEADZONE:
                up_down_speed = 0
            fwd_speed = int(target_speed_z)
            # if abs(fwd_speed) < config.DEADZONE:
            #     fwd_speed = 0

            # vs.tello.send_rc_control(0, fwd_speed, up_down_speed, yaw_speed)
            vs.tello.send_rc_control(0, 0, 0, yaw_speed)
        else:
            if vs.tello.is_flying:
                vs.tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.05)

def tracking_worker():
    while True:
        frame = vs.read()
        if frame is None:
            time.sleep(0.01)
            continue
        
        cx, cy, locked = tracker.update(frame)
        with track_lock:
            latest_track["cx"] = cx
            latest_track["cy"] = cy
            latest_track["locked"] = locked


def gen_frames():
    global telemetry_data, target_angle_x, target_angle_y, last_filtered_height, target_speed_z
    global cached_battery, last_battery_time

    while True:
        frame = vs.read()
        if frame is None:
            time.sleep(0.01)
            continue

        img = frame
        h, w, _ = img.shape
        c_x, c_y = w // 2, h // 2

        cv2.line(img, (c_x - 10, c_y), (c_x + 10, c_y), (255, 255, 255), 1)
        cv2.line(img, (c_x, c_y - 10), (c_x, c_y + 10), (255, 255, 255), 1)

        with track_lock:
            cx = latest_track["cx"]
            cy = latest_track["cy"]
            locked = latest_track["locked"]
            h_tors = last_extra["h_tors"]
            pts = last_extra["pts"]
            

        if locked:
            s_cx = cx
            s_cy = cy
            s_hz = filter_z.apply(h_tors)
            last_filtered_height = s_hz

            target_angle_x = reg_x.update(s_cx - c_x)
            target_angle_y = reg_y.update(c_y - s_cy)
            target_speed_z = reg_z.update(s_hz)
            err_z = (reg_z.target_height - s_hz) if reg_z.target_height else 0

            now = time.time()
            if now - last_battery_time > 1.0:
                cached_battery = vs.tello.get_battery()
                last_battery_time = now

            telemetry_data = {
                "err_x": int(c_x - s_cx),
                "err_y": int(c_y - s_cy),
                "err_z": int(err_z),
                "ctrl_x": round(target_angle_x, 1),
                "ctrl_y": round(target_angle_y, 1),
                "ctrl_z": round(target_speed_z, 1),
                "batt": cached_battery
            }

            if pts:
                cv2.line(img, pts[11], pts[12], (0, 255, 0), 2)
                cv2.line(img, pts[11], pts[23], (0, 255, 0), 2)
                cv2.line(img, pts[12], pts[24], (0, 255, 0), 2)
                cv2.line(img, pts[23], pts[24], (0, 255, 0), 2)

            cv2.circle(img, (int(s_cx), int(s_cy)), 5, (0, 0, 255), -1)
        else:
            target_angle_x = 0
            target_angle_y = 0
            target_speed_z = 0
            reg_x.reset()
            reg_y.reset()

        ret, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 25])
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        time.sleep(0.033)

threading.Thread(target=tracking_worker, daemon=True).start()
threading.Thread(target=flight_worker, daemon=True).start()

@app.route('/')
def home(): return render_template("index.html")

@app.route('/video_feed')
def video_feed(): return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry')
def telemetry(): return Response(json.dumps(telemetry_data), mimetype='application/json')

@app.route('/takeoff')
def takeoff():
    try:
        vs.tello.takeoff()
        time.sleep(1)
        return json.dumps({"status": "success", "message": "Takeoff initiated"})
    except Exception as e:
        return json.dumps({"status": "error", "message": str(e)})

@app.route('/land')
def land():
    try:
        vs.tello.land()
        return json.dumps({"status": "success", "message": "Landing initiated"})
    except Exception as e:
        return json.dumps({"status": "error", "message": str(e)})

@app.route('/toggle_follow')
def toggle_follow():
    global follow_active, last_filtered_height
    follow_active = not follow_active
    if follow_active:
        reg_z.set_reference(last_filtered_height)
        tracker.reset()
        print(f"Śledzenie aktywne! Cel odległości: {last_filtered_height}px")
    else:
        tracker.reset() 
    
    return json.dumps({"status": "ok", "active": follow_active})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)