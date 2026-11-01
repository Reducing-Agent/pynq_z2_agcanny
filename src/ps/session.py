"""Shared camera and gesture processing for demo and accuracy evaluation."""
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import gesture as g

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"


def _env_int(name, default):
    try:
        return int(os.environ[name])
    except (KeyError, ValueError):
        return default


def _env_float(name, default):
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


# 人脸检测很贵：PC 上 1.49 ms，按实测 PC->板子约 20x 放大，板上约 30 ms/帧，
# 每帧都跑的话光这一项就把帧率压到 33 fps。
# 人不会在 0.2 秒内把脸挪到别处，所以没必要每帧都找。
FACE_EVERY = _env_int("FACE_EVERY", 6)        # 每 6 帧重新找一次脸（约 0.2 s）
FACE_SCALE = _env_float("FACE_SCALE", 0.5)    # 在半尺寸图上找，快约 4 倍
                                              # 想关掉这个优化：FACE_EVERY=1 FACE_SCALE=1


def open_camera(index):
    # Camera property values are backend dependent. Do not force Windows exposure
    # values onto Linux or claim that a zero readback proves a setting is supported.
    names = ("CAP_DSHOW", "CAP_MSMF", "CAP_ANY") if os.name == "nt" else ("CAP_V4L2", "CAP_ANY")
    for name in names:
        backend = getattr(cv2, name, None)
        if backend is None:
            continue
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            # 2026-10-28 probe_camera.py 实测（C270 + PYNQ-Z2，每项 3 秒）：
            #     640x480 默认 YUYV   29.9 fps
            #     640x480 MJPEG      30.0 fps   ← 最佳，采用
            #     640x480 BUFFERSIZE=1 15.0 fps ← 坑！单缓冲时 V4L2 要等用户
            #                                      取走帧才开始下一次曝光，
            #                                      等于把流水线砍半。**不要设**
            #   MJPEG 的额外好处：USB 带宽从 640*480*2 B 降到约 1/5，
            #   给 PL 的 HP 口留出余量（我们刚因为带宽墙折腾过一整轮）。
            #   Windows 上不设 fourcc：CAP_DSHOW 下行为不同，且 PC 不是瓶颈。
            if os.name != "nt":
                cap.set(cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc(*"MJPG"))
                print("[camera] index=%s backend=%s fourcc=MJPG" % (index, name))
            else:
                print("[camera] index=%s backend=%s" % (index, name))
            return cap
        cap.release()
    raise RuntimeError("Cannot open camera %s. Close other camera apps or try --camera 1." % index)


def read_frame(cap):
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Camera opened but no frame received. Check camera permissions and connection.")
    if frame.shape[:2] != (480, 640):
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
    return cv2.flip(frame, 1)


class GestureSession:
    def __init__(self, boost=None, face=False):
        self.boost = True
        self.face = face
        self.calibrated = False
        self.calibration_status = {"ok": False, "reason": "not_started"}
        self.face_detector = None
        self._face_load_attempted = False
        self.faces = []
        self._face_age = FACE_EVERY      # 让第一次 detect 一定去找脸
        self.restore_settings()
        if boost is not None:
            self.boost = boost

    def restore_settings(self):
        path = RESULTS / "calibration.json"
        if not path.exists():
            return
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            g.apply_strict(float(data.get("strict", 1.0)))
            self.boost = bool(data.get("boost", True))
            # Do not silently reuse old skin thresholds. Start with the default
            # range; the user can explicitly accept it or calibrate this session.
        except (OSError, ValueError, TypeError) as exc:
            print("[settings] ignored invalid saved settings:", exc)

    def settings(self):
        return {"strict": g.STRICT, "boost": self.boost, "face": self.face,
                "skin_low": g.SKIN_LOW.tolist(), "skin_high": g.SKIN_HIGH.tolist(),
                "detect_size": [g.DETECT_W, g.DETECT_H], "calibrated": self.calibrated,
                "face_detector_state": self.face_state(),
                # 2026-10-18：加入"像不像手"五道形状判据，治"没手却数出手指"。
                # 现场用这个字符串确认板上跑的是哪一版，别拿错文件。
                "recognizer_revision": "2026-10-18-false-positive-guard"}

    def face_state(self):
        if not self.face:
            return "off"
        if self.face_detector is not None:
            return "ready"
        return "unavailable" if self._face_load_attempted else "not_loaded"

    def face_label(self):
        state = self.face_state()
        if state == "ready":
            return "ON (%d found)" % len(self.faces)
        return {"off": "OFF", "not_loaded": "LOADING", "unavailable": "UNAVAILABLE"}[state]

    def annotate_faces(self, image):
        # Only call with a display copy. Counting still uses the untouched image.
        if self.face:
            sx, sy = image.shape[1] / g.DETECT_W, image.shape[0] / g.DETECT_H
            for x, y, w, h in self.faces:
                cv2.rectangle(image, (round(x * sx), round(y * sy)),
                              (round((x + w) * sx), round((y + h) * sy)), (50, 60, 255), 2)

    def save_settings(self):
        RESULTS.mkdir(parents=True, exist_ok=True)
        (RESULTS / "calibration.json").write_text(
            json.dumps(self.settings(), ensure_ascii=False, indent=2), encoding="utf-8")

    def prepare(self, frame):
        small = cv2.resize(frame, (g.DETECT_W, g.DETECT_H), interpolation=cv2.INTER_AREA)
        if self.boost:
            small = g.apply_gamma(small, g.gamma_for(g.measure_brightness(small)))
            small = g.apply_contrast(small, g.contrast_fix_params(small))
        return small

    def calibrate(self, prepared):
        # Pass an untouched processing image, never one with drawn labels/boxes.
        self.calibrated = bool(g.calibrate_skin(prepared))
        self.calibration_status = dict(g.LAST_CALIBRATION)
        if self.calibrated:
            self.save_settings()
        return self.calibrated

    def calibration_hint(self):
        reason = self.calibration_status.get("reason", "not_started")
        return {
            "not_started": "Cover the yellow box with your palm, then click CALIBRATE.",
            "sample_too_gray_or_dark": "Calibration FAILED: sample too gray/dark. Check DEBUG; if the hand is clear, click GESTURE ON/OFF.",
            "palm_does_not_cover_box": "Calibration FAILED: sample colors vary too much. Cover the yellow box with your palm and retry.",
            "ok": "Calibration OK. Show 1 / 2 / 3 fingers and hold steady.",
        }.get(reason, "Please calibrate again.")

    def detect(self, prepared, debug=False):
        mask = g.make_skin_mask(prepared)
        canvas = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if debug else None
        if self.face:
            if not self._face_load_attempted:
                self._face_load_attempted = True
                self.face_detector = g.load_face_detector()
            # 每 FACE_EVERY 帧才真跑一次检测，中间几帧沿用上一次的脸框。
            # （缩小检测的活儿在 gesture.detect_faces 内部做，
            #   这里拿到的框已经是检测尺寸坐标，不要在这里再缩放。）
            if self._face_age >= FACE_EVERY:
                self._face_age = 0
                self.faces = [tuple(map(int, box))
                              for box in g.detect_faces(prepared, self.face_detector)]
            self._face_age += 1
        else:
            self.faces = []
        n, canvas, detail = g.count_fingers(mask, canvas, self.faces)
        detail.update(face_count=len(self.faces), face_detector_state=self.face_state())
        if debug:
            self.annotate_faces(canvas)
            # Put labels outside the mask so fingertip/contour marks remain visible.
            panel = np.zeros((g.DETECT_H + 86, g.DETECT_W, 3), np.uint8)
            panel[:g.DETECT_H] = canvas
            label(panel, "fingers: %d | %s" % (n, detail.get("reason", "unknown")),
                  g.DETECT_H + 20, scale=0.43, x=5)
            label(panel, "area: %.0f | tips: %d | gaps: %d" % (
                detail.get("area", 0), detail["tips"], detail["gaps"]),
                  g.DETECT_H + 43, scale=0.4, x=5)
            label(panel, "Face guard: " + self.face_label(), g.DETECT_H + 67, scale=0.42, x=5)
            canvas = panel
        return n, canvas, detail

    def adjust(self, key):
        if key in (ord('+'), ord('=')):
            g.apply_strict(g.STRICT + 0.15)
        elif key == ord('-'):
            g.apply_strict(g.STRICT - 0.15)
        elif key == ord('b'):
            self.boost = not self.boost
            self.calibrated = False
            print("[gesture] brightness setting changed; press c to recalibrate")
        elif key == ord('f'):
            self.face = not self.face
            self.faces = []
            self._face_age = FACE_EVERY      # 重新打开时立刻找一次
            if self.face and self.face_detector is None:
                self._face_load_attempted = False
        else:
            return False
        self.save_settings()
        return True


def label(image, text, y, color=(235, 235, 235), scale=0.55, x=12):
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv2.LINE_AA)


def calibration_box(image):
    x1, y1, x2, y2 = g.cal_frame_box(image)
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 220, 255), 2)


def utc_stamp():
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
