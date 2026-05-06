import cv2
import mediapipe as mp

class PoseDetector:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            smooth_landmarks=False,
            min_detection_confidence=0.5
        )

    def find_torso(self, frame):
        h, w, _ = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb_frame)
        
        if results.pose_landmarks:
            lm = results.pose_landmarks.landmark
            try:
                pts = {
                    11: (int(lm[11].x * w), int(lm[11].y * h)),
                    12: (int(lm[12].x * w), int(lm[12].y * h)),
                    23: (int(lm[23].x * w), int(lm[23].y * h)),
                    24: (int(lm[24].x * w), int(lm[24].y * h))
                }
                cx = int((pts[11][0] + pts[12][0]) / 2)
                cy = int((pts[11][1] + pts[23][1]) / 2)
                return cx, cy, pts
            except:
                return None, None, None
        return None, None, None