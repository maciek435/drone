import cv2
import mediapipe as mp
import config

class PoseDetector:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=config.MP_MODEL_COMPLEXITY,
            smooth_landmarks=False,
            min_detection_confidence=config.MP_DETECTION_CONFIDENCE,
            min_tracking_confidence=config.MP_TRACKING_CONFIDENCE
        )
        self.last_h_tors = None

    def find_torso(self, frame, bbox_margin=0.25):
        h, w, _ = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb_frame)

        if results.pose_landmarks:
            lm = results.pose_landmarks.landmark

            key_points = [lm[11], lm[12], lm[23], lm[24]]
            if any(p.visibility < 0.5 for p in key_points):
                return None

            pts = {
                11: (int(lm[11].x * w), int(lm[11].y * h)),
                12: (int(lm[12].x * w), int(lm[12].y * h)),
                23: (int(lm[23].x * w), int(lm[23].y * h)),
                24: (int(lm[24].x * w), int(lm[24].y * h))
            }

            cx = int((pts[11][0] + pts[12][0]) / 2)
            cy = int((pts[11][1] + pts[23][1]) / 2)

            h_left = pts[23][1] - pts[11][1]
            h_right = pts[24][1] - pts[12][1]
            h_tors = (h_left + h_right) / 2

            if h_tors <= 0:
                return None

            xs = [p[0] for p in pts.values()]
            ys = [p[1] for p in pts.values()]
            x1, x2 = min(xs), max(xs)
            y1, y2 = min(ys), max(ys)

            bw = x2 - x1
            bh = y2 - y1

            # zakomentowane: -----------------
            # if bw < 5 or bh < 5:
            #     return None
            # ----------------------------------

            bbox = (x1, y1, bw, bh)

            return {"cx": cx, "cy": cy, "h_tors": h_tors, "pts": pts, "bbox": bbox}        
        else:
            return None