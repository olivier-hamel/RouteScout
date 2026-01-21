# importing necessary libraries
import cv2 as cv
import time
import os
import json
from collections import defaultdict
import numpy as np
import platform
import argparse
import glob
import urllib.request
import urllib.error
import threading


# Pothole tracker class
class PotholeTracker:
    def __init__(self, max_disappeared=5, distance_threshold=50):
        self.next_id = 0
        self.objects = {}  # id -> centroid
        self.disappeared = defaultdict(int)  # id -> frame count
        self.bboxes = {}  # id -> bounding box
        self.max_disappeared = max_disappeared
        self.distance_threshold = distance_threshold
        self.exited_potholes = []  # Store potholes that exited

    def register(self, centroid, bbox):
        """Register a new pothole with next available ID"""
        self.objects[self.next_id] = centroid
        self.bboxes[self.next_id] = bbox
        self.disappeared[self.next_id] = 0
        self.next_id += 1

    def deregister(self, object_id, frame_number, timestamp, exit_line_y):
        """Remove pothole and record exit information"""
        if object_id in self.bboxes:
            x, y, w, h = self.bboxes[object_id]
            centroid_x = x + w // 2

            # Check if exited at or past the exit line
            if y + h >= exit_line_y:
                exit_info = {
                    'id': int(object_id),
                    'x_coordinate': int(centroid_x),
                    'timestamp': float(timestamp),
                    'frame_number': int(frame_number)
                }
                self.exited_potholes.append(exit_info)
                print(
                    f"[EXIT] Pothole ID {object_id} exited at x={centroid_x}, frame={frame_number}, time={timestamp:.2f}s")

        del self.objects[object_id]
        del self.disappeared[object_id]
        del self.bboxes[object_id]

    def update(self, detections, frame_number, timestamp, exit_line_y):
        """Update tracked potholes with new detections"""
        if len(detections) == 0:
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id, frame_number, timestamp, exit_line_y)
            return self.objects

        input_centroids = []
        input_bboxes = []
        for (x, y, w, h) in detections:
            cX = x + w // 2
            cY = y + h // 2
            input_centroids.append((cX, cY))
            input_bboxes.append((x, y, w, h))

        if len(self.objects) == 0:
            for i in range(len(input_centroids)):
                self.register(input_centroids[i], input_bboxes[i])
        else:
            object_ids = list(self.objects.keys())
            object_centroids = list(self.objects.values())

            D = np.zeros((len(object_centroids), len(input_centroids)))
            for i, obj_centroid in enumerate(object_centroids):
                for j, input_centroid in enumerate(input_centroids):
                    D[i, j] = np.linalg.norm(np.array(obj_centroid) - np.array(input_centroid))

            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            used_rows = set()
            used_cols = set()

            for (row, col) in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue
                if D[row, col] > self.distance_threshold:
                    continue

                object_id = object_ids[row]
                self.objects[object_id] = input_centroids[col]
                self.bboxes[object_id] = input_bboxes[col]
                self.disappeared[object_id] = 0

                used_rows.add(row)
                used_cols.add(col)

            unused_rows = set(range(D.shape[0])) - used_rows
            for row in unused_rows:
                object_id = object_ids[row]
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id, frame_number, timestamp, exit_line_y)

            unused_cols = set(range(D.shape[1])) - used_cols
            for col in unused_cols:
                self.register(input_centroids[col], input_bboxes[col])

        return self.objects


# Global variables for Flask access
exited_potholes_data = []
tracker = None

# reading label name from obj.names file
class_name = []
with open(os.path.join("project_files", 'obj.names'), 'r') as f:
    class_name = [cname.strip() for cname in f.readlines()]

# importing model weights and config file
net1 = cv.dnn.readNet('project_files/yolov4_tiny.weights', 'project_files/yolov4_tiny.cfg')

# DNN backend/target are configured after CLI parsing (auto/OpenCL/CPU).

model1 = cv.dnn_DetectionModel(net1)
model1.setInputParams(size=(640, 480), scale=1 / 255, swapRB=True)


class _LatestFrameCapture:
    """Continuously grabs frames and retains only the most recent.

    This prevents large latency when processing is slower than camera FPS.
    """

    def __init__(self, cap: cv.VideoCapture):
        self._cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._frame_time = 0.0
        self._stopped = False
        self._thread = threading.Thread(target=self._reader, name='latest-frame-capture', daemon=True)

    def start(self) -> "_LatestFrameCapture":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stopped = True
        try:
            self._thread.join(timeout=1.0)
        except Exception:
            pass

    def read_latest(self):
        with self._lock:
            return self._frame, self._frame_time

    def _reader(self) -> None:
        while not self._stopped:
            ok, frame = self._cap.read()
            if not ok:
                # Avoid busy spinning if the camera stalls.
                time.sleep(0.01)
                continue
            with self._lock:
                self._frame = frame
                self._frame_time = time.time()

def _try_open_camera(source, backend, width, height):
    cap = cv.VideoCapture(source, backend)
    if cap.isOpened():
        if width is not None:
            cap.set(cv.CAP_PROP_FRAME_WIDTH, int(width))
        if height is not None:
            cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(height))
    return cap


def _list_linux_video_devices():
    return sorted(glob.glob('/dev/video*'))


parser = argparse.ArgumentParser(description='Pothole detection and tracking (webcam)')
parser.add_argument('--camera', '--camera-index', dest='camera_index', type=int, default=0,
                    help='Camera index to open (default: 0)')
parser.add_argument('--device', dest='device', default=None,
                    help='Video device path (Linux), e.g. /dev/video2. Overrides --camera.')
parser.add_argument('--width', dest='width', type=int, default=640, help='Capture width (default: 640)')
parser.add_argument('--height', dest='height', type=int, default=480, help='Capture height (default: 480)')
parser.add_argument('--net-width', dest='net_width', type=int, default=640,
                    help='Neural net input width (default: 640). Lower can be faster (e.g. 416 or 320).')
parser.add_argument('--net-height', dest='net_height', type=int, default=480,
                    help='Neural net input height (default: 480). Lower can be faster (e.g. 416 or 320).')
parser.add_argument('--headless', '--no-gui', dest='headless', action='store_true',
                    help='Disable OpenCV window display (prints tracking info to terminal instead)')
parser.add_argument('--mjpeg', action='store_true',
                    help='Request MJPEG from the camera (often reduces CPU + latency on webcams)')
parser.add_argument('--buffer-size', dest='buffer_size', type=int, default=1,
                    help='Best-effort camera buffer size (default: 1). Higher can increase lag.')
parser.add_argument('--drop-frames', dest='drop_frames', type=int, default=0,
                    help='Before processing each frame, grab+discard N frames to reduce lag (default: 0).')
parser.add_argument('--threaded-capture', dest='threaded_capture', action='store_true',
                    help='Use a background thread to always process the newest frame (recommended).')
parser.add_argument('--no-threaded-capture', dest='threaded_capture', action='store_false',
                    help='Disable threaded capture (may increase latency if inference is slow).')
parser.set_defaults(threaded_capture=True)
parser.add_argument('--save-video', action='store_true',
                    help='Save annotated output video to pothole_results/result.avi (slower)')
parser.add_argument('--print-every', dest='print_every', type=int, default=5,
                    help='When headless, print tracking info every N frames (default: 5)')
parser.add_argument('--profile', dest='profile', action='store_true',
                    help='Print DNN timing + detection counts in headless mode (use with --print-every)')
parser.add_argument('--min-score', dest='min_score', type=float, default=float(os.getenv('POTHOLE_MIN_SCORE', '0.7')),
                    help='Minimum detection confidence to keep (default: 0.7). Lower if missing detections under load.')
parser.add_argument('--max-area-frac', dest='max_area_frac', type=float, default=float(os.getenv('POTHOLE_MAX_AREA_FRAC', '0.1')),
                    help='Drop boxes larger than this fraction of the frame area (default: 0.1).')
parser.add_argument('--exit-line-offset', dest='exit_line_offset', type=int, default=int(os.getenv('POTHOLE_EXIT_LINE_OFFSET', '80')),
                    help='Exit line offset from bottom in pixels (default: 80).')
parser.add_argument('--list-cameras', action='store_true',
                    help='List /dev/video* (Linux) and exit')
parser.add_argument('--probe-cameras', action='store_true',
                    help='Try each /dev/video* (Linux) and report which ones return frames')
parser.add_argument('--server-url', dest='server_url', default=os.getenv('POTHOLE_SERVER_URL', ''),
                    help='Base URL of the Flask server, e.g. http://192.168.1.50:5000 (or set POTHOLE_SERVER_URL)')
parser.add_argument('--server-timeout', dest='server_timeout', type=float,
                    default=float(os.getenv('POTHOLE_SERVER_TIMEOUT', '2.0')),
                    help='Timeout (seconds) for POSTing exited potholes to the server (default: 2.0)')
parser.add_argument('--send-exited', dest='send_exited', action='store_true',
                    help='POST newly-exited potholes to <server-url>/api/potholes')

# Acceleration / performance knobs
parser.add_argument('--dnn-target', dest='dnn_target', default=os.getenv('POTHOLE_DNN_TARGET', 'auto'),
                    choices=['auto', 'cpu', 'opencl', 'opencl-fp16'],
                    help=(
                        "DNN target device. 'auto' tries OpenCL (GPU) then falls back to CPU. "
                        "On Raspberry Pi, GPU acceleration is only available if OpenCV has OpenCL support."))
parser.add_argument('--opencv-threads', dest='opencv_threads', type=int, default=int(os.getenv('POTHOLE_OPENCV_THREADS', '0')),
                    help='Set OpenCV thread count (0 = default). On Pi, try 2-4 for better throughput.')
args = parser.parse_args()

# If running over SSH or without a GUI, OpenCV windows won't work.
# Auto-switch to headless in that case to avoid "nothing happens" confusion.
if not getattr(args, 'headless', False):
    if platform.system() == 'Linux' and not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
        args.headless = True
        print('No DISPLAY/WAYLAND detected; enabling --headless mode.')


def _configure_dnn_acceleration(net: cv.dnn_Net, dnn_target: str, opencv_threads: int) -> tuple[str, str]:
    """Configure OpenCV DNN acceleration.

    Returns:
        (backend_label, target_label)
    """
    # General OpenCV performance flags.
    try:
        cv.setUseOptimized(True)
    except Exception:
        pass

    if isinstance(opencv_threads, int) and opencv_threads > 0:
        try:
            cv.setNumThreads(int(opencv_threads))
        except Exception:
            pass

    # DNN backend is OpenCV unless CUDA/other backends are explicitly added.
    try:
        net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
    except Exception:
        # Very old OpenCV builds.
        pass

    def _set_cpu() -> tuple[str, str]:
        net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)
        return ('opencv', 'cpu')

    def _set_opencl(fp16: bool) -> tuple[str, str] | None:
        if not hasattr(cv, 'ocl'):
            return None
        try:
            if not cv.ocl.haveOpenCL():
                return None
            cv.ocl.setUseOpenCL(True)
        except Exception:
            return None

        # Some builds expose OpenCL targets but still execute on CPU; still harmless.
        if fp16 and hasattr(cv.dnn, 'DNN_TARGET_OPENCL_FP16'):
            net.setPreferableTarget(cv.dnn.DNN_TARGET_OPENCL_FP16)
            return ('opencv', 'opencl-fp16')
        if hasattr(cv.dnn, 'DNN_TARGET_OPENCL'):
            net.setPreferableTarget(cv.dnn.DNN_TARGET_OPENCL)
            return ('opencv', 'opencl')
        return None

    requested = (dnn_target or 'auto').strip().lower()
    if requested == 'cpu':
        return _set_cpu()
    if requested == 'opencl':
        chosen = _set_opencl(fp16=False)
        if chosen is None:
            print('OpenCL requested but not available; falling back to CPU.')
            return _set_cpu()
        return chosen
    if requested == 'opencl-fp16':
        chosen = _set_opencl(fp16=True)
        if chosen is None:
            print('OpenCL-FP16 requested but not available; falling back to CPU.')
            return _set_cpu()
        return chosen

    # auto
    chosen = _set_opencl(fp16=True)
    if chosen is None:
        chosen = _set_opencl(fp16=False)
    if chosen is None:
        chosen = _set_cpu()
    return chosen

def _require_multiple_of_32(name: str, value: int) -> None:
    if int(value) <= 0 or (int(value) % 32) != 0:
        raise SystemExit(
            f"ERROR: --{name.replace('_', '-')} must be a positive multiple of 32 for YOLO (got {value}). "
            f"Try e.g. --net-width 320 --net-height 256 (or 416x320)."
        )

_require_multiple_of_32('net_width', args.net_width)
_require_multiple_of_32('net_height', args.net_height)

# Configure DNN input size after CLI parsing
model1.setInputParams(size=(args.net_width, args.net_height), scale=1 / 255, swapRB=True)

backend_label, target_label = _configure_dnn_acceleration(net1, args.dnn_target, args.opencv_threads)
print(f"DNN acceleration: backend={backend_label} target={target_label} (opencv_threads={args.opencv_threads})")

if args.list_cameras:
    if platform.system() == 'Linux':
        devices = _list_linux_video_devices()
        print('Detected video devices:')
        for d in devices:
            print(' ', d)
    else:
        print('Camera listing is only supported on Linux in this script.')
    raise SystemExit(0)

if args.probe_cameras:
    if platform.system() != 'Linux':
        print('Camera probing is only supported on Linux in this script.')
        raise SystemExit(0)

    devices = _list_linux_video_devices()
    if not devices:
        print('No /dev/video* devices found.')
        raise SystemExit(1)

    print('Probing cameras (opening + reading 1 frame):')
    working = []
    for dev in devices:
        cap_probe = _try_open_camera(dev, cv.CAP_V4L2, args.width, args.height)
        if not cap_probe.isOpened():
            print(f'  {dev}: cannot open')
            continue

        ok, frame = cap_probe.read()
        cap_probe.release()

        if ok and frame is not None:
            shape = getattr(frame, 'shape', None)
            print(f'  {dev}: OK frame={shape}')
            working.append(dev)
        else:
            print(f'  {dev}: opened but no frame')

    if working:
        print('\nUse one of these, for example:')
        print(f'  python3 camera_video.py --device {working[0]}')
    raise SystemExit(0)

# defining the video source
# On Linux, prefer V4L2 to avoid GStreamer-related loader issues.
backend = cv.CAP_V4L2 if platform.system() == 'Linux' else cv.CAP_ANY

source = args.device if args.device else int(args.camera_index)
cap = _try_open_camera(source, backend, args.width, args.height)

if not cap.isOpened() and args.device is None:
    print("Error: Could not open camera. Trying camera index 1...")
    cap = _try_open_camera(1, backend, args.width, args.height)

if not cap.isOpened():
    if args.device:
        print(f"Error: Could not open camera device '{args.device}'.")
    else:
        print(f"Error: Could not open camera index {args.camera_index} (or fallback index 1).")
    print("Tip: On Linux, try: python3 camera_video.py --list-cameras")
    exit()

cap.set(cv.CAP_PROP_FRAME_WIDTH, args.width)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, args.height)

# Low-latency tweaks (best-effort; not all backends support these).
try:
    if hasattr(cv, 'CAP_PROP_BUFFERSIZE'):
        cap.set(cv.CAP_PROP_BUFFERSIZE, int(args.buffer_size))
except Exception:
    pass

if args.mjpeg:
    try:
        cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
    except Exception:
        pass

width = int(cap.get(3))
height = int(cap.get(4))

# Exit line placement (raise it to avoid tiny boxes disappearing near the bottom)
exit_line_y = max(10, height - int(args.exit_line_offset))

# Initialize tracker
tracker = PotholeTracker(max_disappeared=5, distance_threshold=80)

result_path = "pothole_results"
os.makedirs(result_path, exist_ok=True)

result = None
if args.save_video:
    result = cv.VideoWriter(
        os.path.join(result_path, 'result.avi'),
        cv.VideoWriter_fourcc(*'XVID'),
        10, (width, height)
    )

# Parameters
starting_time = time.time()
Conf_threshold = 0.5
NMS_threshold = 0.4
frame_counter = 0


def _join_url(base: str, path: str) -> str:
    base = (base or '').rstrip('/')
    path = '/' + (path or '').lstrip('/')
    return base + path


def _post_json(url: str, payload: dict, timeout_s: float = 2.0) -> None:
    data = json.dumps(payload).encode('utf-8')
    req = urllib.request.Request(
        url,
        data=data,
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        # Drain response body to reuse connections when possible.
        resp.read()

print("Camera opened successfully!")
print("Running pothole detection and tracking...")
if args.headless:
    print("Headless mode: printing tracks to terminal (Ctrl+C to stop)")
else:
    print("Press 'q' to quit")

# Main detection loop
server_url = (args.server_url or '').strip()
send_exited = bool(args.send_exited or server_url)
server_endpoint = _join_url(server_url, '/api/potholes') if server_url else ''
sent_exited_count = 0
last_send_attempt_at = 0.0

if send_exited:
    if server_endpoint:
        print(f"Publishing exited potholes to {server_endpoint} (timeout={float(args.server_timeout)}s)")
    else:
        print("Publishing enabled but no --server-url provided; will not POST.")

capture = None
if args.threaded_capture:
    capture = _LatestFrameCapture(cap).start()

while True:
    if capture is None:
        if args.drop_frames > 0:
            for _ in range(int(args.drop_frames)):
                cap.grab()
        ret, frame = cap.read()
        frame_read_at = time.time()
    else:
        frame, frame_read_at = capture.read_latest()
        ret = frame is not None

    if not ret:
        # With threaded capture, the first few iterations may happen before the
        # capture thread has produced a frame.
        time.sleep(0.001)
        continue

    # Avoid drawing onto a frame that might be replaced while we process.
    if not args.headless or args.save_video:
        frame = frame.copy()

    frame_counter += 1
    current_time = time.time() - starting_time

    # Detect potholes
    dnn_t0 = time.perf_counter()
    classes, scores, boxes = model1.detect(frame, Conf_threshold, NMS_threshold)
    dnn_ms = (time.perf_counter() - dnn_t0) * 1000.0

    raw_count = 0
    try:
        raw_count = int(len(boxes))
    except Exception:
        raw_count = 0

    # Filter high-confidence detections
    valid_detections = []
    for (classid, score, box) in zip(classes, scores, boxes):
        if float(score) >= float(args.min_score):
            x, y, w, h = box
            recarea = w * h
            area = width * height
            # Filter by size and position; keep boxes above exit line to avoid vanishing early
            if (recarea / area) <= float(args.max_area_frac) and y < exit_line_y - 10:
                valid_detections.append((x, y, w, h))

    # Update tracker
    objects = tracker.update(valid_detections, frame_counter, current_time, exit_line_y)

    fps = frame_counter / current_time if current_time > 0 else 0
    capture_delay_ms = max(0.0, (time.time() - frame_read_at) * 1000.0)

    if args.headless:
        if args.print_every > 0 and (frame_counter % args.print_every == 0):
            tracks = []
            for object_id, centroid in objects.items():
                x, y, w, h = tracker.bboxes[object_id]
                tracks.append(
                    f"{int(object_id)}:(cx={int(centroid[0])},cy={int(centroid[1])},x={int(x)},y={int(y)},w={int(w)},h={int(h)})"
                )

            profile_str = ''
            if args.profile:
                profile_str = (
                    f" dnn={dnn_ms:.0f}ms raw={raw_count} kept={len(valid_detections)}"
                    f" min_score={float(args.min_score):.2f} max_area_frac={float(args.max_area_frac):.2f}"
                )
            print(
                f"frame={frame_counter} t={current_time:.2f}s fps={fps:.2f} delay={capture_delay_ms:.0f}ms "
                f"active={len(objects)} exited={len(tracker.exited_potholes)} "
                + ("tracks=[" + ", ".join(tracks) + "]" if tracks else "tracks=[]")
                + profile_str
            )
    else:
        # Draw tracking information
        for object_id, centroid in objects.items():
            x, y, w, h = tracker.bboxes[object_id]

            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.putText(frame, f"ID: {object_id}", (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv.circle(frame, (int(centroid[0]), int(centroid[1])), 4, (0, 255, 0), -1)

        # Draw exit line
        cv.line(frame, (0, exit_line_y), (width, exit_line_y), (0, 0, 255), 2)
        cv.putText(frame, "EXIT LINE", (10, max(15, exit_line_y - 10)), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display FPS and stats
        cv.putText(frame, f'FPS: {fps:.2f}  delay: {capture_delay_ms:.0f}ms', (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(frame, f'Active: {len(objects)} | Exited: {len(tracker.exited_potholes)}',
                   (20, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv.imshow('Pothole Detection & Tracking - Press Q to quit', frame)

    if result is not None:
        result.write(frame)

    exited_potholes_data = tracker.exited_potholes.copy()

    # Send newly-exited potholes to the server (best-effort).
    if send_exited and server_endpoint:
        total_exited = len(tracker.exited_potholes)
        if total_exited > sent_exited_count:
            now_t = time.time()
            # Simple backoff: attempt at most once per second.
            if now_t - last_send_attempt_at >= 1.0:
                last_send_attempt_at = now_t
                batch = tracker.exited_potholes[sent_exited_count:]
                payload = {
                    'source': 'pothole-detector',
                    'potholes': batch,
                }
                try:
                    _post_json(server_endpoint, payload, timeout_s=float(args.server_timeout))
                    sent_exited_count = total_exited
                    if args.headless:
                        print(f"[send] posted {len(batch)} exited pothole(s) to {server_endpoint}")
                except Exception as e:
                    # Keep running detection even if network fails.
                    if args.headless:
                        print(f"[send] failed to POST to {server_endpoint}: {e}")

    if not args.headless:
        key = cv.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            break

print(f"\n=== Session Summary ===")
print(f"Total frames processed: {frame_counter}")
print(f"Total potholes exited: {len(tracker.exited_potholes)}")
print(f"\nExited potholes data:")
for exit_data in tracker.exited_potholes:
    print(f"  ID {exit_data['id']}: x={exit_data['x_coordinate']}, "
          f"frame={exit_data['frame_number']}, time={exit_data['timestamp']:.2f}s")

# Save exit data to file
with open(os.path.join(result_path, 'exited_potholes.json'), 'w') as f:
    json.dump(tracker.exited_potholes, f, indent=2)

print(f"\nResults saved to '{result_path}/' folder")

if capture is not None:
    capture.stop()
cap.release()
if result is not None:
    result.release()
if not args.headless:
    cv.destroyAllWindows()
