import cv2
import time
import json
import threading
import config
from flask import Flask, Response, render_template
from regulator import KalmanLite, Regulator, DistanceRegulator, KalmanCV1D
from vision import PoseDetector
from pi_camera import PiVideoStream
from inav_control import MSPController
from tracker import HybridBodyTracker
from gimbal import ServoGimbal
import board
from tof_sensors import ToFArray
from safety import SafetyGuard, ToFObstacleGuard



msp = MSPController()
app = Flask(__name__)
vs = PiVideoStream().start()
detector = PoseDetector()
gimbal = ServoGimbal(
    pin=config.GIMBAL_SERVO_PIN,
    reversed=True
)

tof_array = ToFArray(
    xshut_pins=[board.D17, board.D27, board.D22],  # dostosuj do config.TOF_XSHUT_PINS
    addresses=config.TOF_I2C_ADDRESSES,
    distance_mode=config.TOF_DISTANCE_MODE,
    timing_budget=config.TOF_TIMING_BUDGET_MS
)
tof_guard = ToFObstacleGuard(stop_cm=config.TOF_STOP_CM)

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

safety_guard = SafetyGuard(min_htors_px=config.MIN_HTORS_PX, max_misses=10)

# filter_z = KalmanLite(process_noise=config.FILTER_Z_PROCESS_NOISE, measurement_noise=config.FILTER_Z_MEASUREMENT_NOISE)
filter_z = KalmanCV1D(q=config.FILTER_Z_PROCESS_NOISE, r=config.FILTER_Z_MEASUREMENT_NOISE)
reg_x = Regulator(kp=config.REG_X_KP, kd=config.REG_X_KD, max_output=config.REG_X_MAX_OUTPUT) 
reg_y = Regulator(kp=config.REG_Y_KP, kd=config.REG_Y_KD, max_output=config.REG_Y_MAX_OUTPUT) 
reg_z = DistanceRegulator(kp=config.REG_Z_KP, max_jump=config.REG_Z_MAX_JUMP, max_output=config.REG_Z_MAX_OUTPUT)

target_angle_x = 0
target_angle_y = 0
target_speed_z = 0

telemetry_data = {
    "err_x": 0, 
    "err_y": 0,
    "batt": "--",
    "detected": False, 
    "follow": False
}

running = True
follow_active = False
follow_lock = threading.Lock()  
last_filtered_height = 0
last_update_time = 0
last_tracker_update_time = 0
was_locked = False
gimbal_only_active = False

latest_track = {"cx": None, "cy": None, "locked": False}
track_lock = threading.Lock()

def is_plausible_height(measured, predicted, threshold):
    return abs(measured-predicted) <= threshold

def get_switch_state(switch_val):
    if switch_val > 1700:
        return "FULL"      
    elif switch_val > 1300:
        return "OFF" 
    else:
        return "GIMBAL_ONLY"       

def switch_worker():
    global follow_active, gimbal_only_active, last_filtered_height
    prev_state = "OFF"

    while True:
        channels = msp.get_rc_channels()
        if channels and len(channels) >= 8:
            switch_val = channels[7]
            new_state = get_switch_state(switch_val)

            with follow_lock:
                follow_active = (new_state == "FULL")
                gimbal_only_active = (new_state in ("FULL", "GIMBAL_ONLY"))

            if new_state == "FULL" and prev_state != "FULL":
                reg_z.set_reference(last_filtered_height)
                tracker.reset()
                print(f"[SWITCH] FULL follow AKTYWNE")
            elif new_state == "OFF" and prev_state != "OFF":
                tracker.reset()
                print(f"[SWITCH] OFF")
            elif new_state == "GIMBAL_ONLY" and prev_state == "OFF":
                tracker.reset()
                print(f"[SWITCH] GIMBAL_ONLY aktywne")

            prev_state = new_state
        time.sleep(0.1)

def flight_worker():
    global target_angle_x, target_angle_y, target_speed_z, running, follow_active

    while running:
        with follow_lock:
            active = follow_active

        stale = (time.time() - last_update_time) > config.MAX_DATA_AGE_S
        stale = stale or (time.time() - last_tracker_update_time) > config.MAX_DATA_AGE_S
        
        if active and not stale and target_angle_x != 0:
            yaw_offset = int(target_angle_x)
            if abs(yaw_offset) < config.DEADZONE_W:
                reg_x.reset()
                yaw_offset = 0

            # updown_offset = int(target_angle_y)
            # if abs(updown_offset) < config.DEADZONE_H:
            #     reg_y.reset()
            #     updown_offset = 0

            with track_lock:
                h_tors = last_extra["h_tors"]

            fwd_offset = int(target_speed_z)
            fwd_offset = safety_guard.clamp_forward_speed(fwd_offset, h_tors)

            try:
                dist_center, dist_left, dist_right = tof_array.read_all_cm()
            except Exception as e:
                print(f"[TOF] blad odczytu: {e}")
                dist_center, dist_left, dist_right = None, None, None

            if tof_guard.should_block(dist_center, dist_left, dist_right):
                fwd_offset = 0

            # throttle_pwm = 1500 + updown_offset
            yaw_pwm = 1500 + yaw_offset
            pitch_pwm = 1500 + fwd_offset
            
            

            msp.set_rc(yaw=yaw_pwm, pitch=pitch_pwm, roll=1500, throttle=1500)
            
        else:
            reg_x.reset()
            reg_y.reset()
            reg_z.reset()
            msp.set_rc(yaw=1500, pitch=1500, roll=1500, throttle=1500)
       
        time.sleep(0.05)

def tracking_worker():
    global last_tracker_update_time, target_angle_x, target_angle_y, target_speed_z
    global last_filtered_height, telemetry_data, last_update_time
    global was_locked

    while True:
        frame = vs.read()
        if frame is None:
            time.sleep(0.01)
            continue

        last_update_time = time.time()
        h, w, _ = frame.shape
        c_x, c_y = w // 2, h // 2

        try:
            cx, cy, locked, h_est = tracker.update(frame)
            last_tracker_update_time = time.time()
        except Exception as e:
            print(f"[TRACKING_WORKER] blad: {e}")
            continue

        with track_lock:
            latest_track["cx"] = cx
            latest_track["cy"] = cy
            latest_track["locked"] = locked
            latest_track["h_est"] = h_est

        h_tors = last_extra["h_tors"]

        if locked:
            s_cx, s_cy = cx, cy
            if locked and not was_locked:
                filter_z.init(h_tors)
                s_hz = filter_z.x
            else:
                filter_z.predict()
                pred_h = filter_z.x
                if h_tors is not None and is_plausible_height(h_tors, pred_h, config.HEIGHT_GATE_THRESHOLD):
                    filter_z.correct(h_tors)
                s_hz = filter_z.x

            was_locked = locked
            last_filtered_height = s_hz

            target_angle_x = reg_x.update(s_cx - c_x)
            target_angle_y = reg_y.update(c_y - s_cy)

            gimbal_offset = target_angle_y
            if abs(gimbal_offset) < config.DEADZONE_H:
                gimbal_offset = 0
                reg_y.reset()

            with follow_lock:
                active = follow_active
                gimbal_active = gimbal_only_active

            try:
                if gimbal_active:
                    gimbal.set_offset(gimbal_offset)
                else:
                    gimbal.set_offset(0)
            except Exception as e:
                print(f"[GIMBAL] blad: {e}")

            target_speed_z = reg_z.update(s_hz)
            err_z = (reg_z.target_height - s_hz) if reg_z.target_height else 0

            telemetry_data = {
                "err_x": int(c_x - s_cx),
                "err_y": int(c_y - s_cy),
                "err_z": int(err_z),
                "ctrl_x": round(target_angle_x, 1),
                "ctrl_y": round(target_angle_y, 1),
                "ctrl_z": round(target_speed_z, 1),
                "batt": "--"
            }
        else:
            target_angle_x = 0
            target_angle_y = 0
            target_speed_z = 0
            reg_x.reset()
            reg_y.reset()
            reg_z.reset()

            try:
                gimbal.set_offset(0)
            except Exception as e:
                print(f"[GIMBAL] blad: {e}")

            telemetry_data = {
                "err_x": 0, "err_y": 0, "err_z": 0,
                "ctrl_x": 0, "ctrl_y": 0, "ctrl_z": 0,
                "batt": "--"
            }


def gen_frames():
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

        pts = last_extra["pts"]

        if locked:
            if pts:
                cv2.line(img, pts[11], pts[12], (0, 255, 0), 2)
                cv2.line(img, pts[11], pts[23], (0, 255, 0), 2)
                cv2.line(img, pts[12], pts[24], (0, 255, 0), 2)
                cv2.line(img, pts[23], pts[24], (0, 255, 0), 2)

            cv2.circle(img, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        ret, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 25])
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        time.sleep(0.033)


threading.Thread(target=tracking_worker, daemon=True).start()
threading.Thread(target=flight_worker, daemon=True).start()
threading.Thread(target=switch_worker, daemon=True).start()


@app.route('/')
def home(): return render_template("index.html")

@app.route('/video_feed')
def video_feed(): return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry')
def telemetry(): return Response(json.dumps(telemetry_data), mimetype='application/json')

if __name__ == '__main__':
    from waitress import serve
    serve(app, host=config.FLASK_HOST, port=config.FLASK_PORT)