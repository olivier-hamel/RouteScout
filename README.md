
# RouteScoute

## The problem 
Potholes aren’t just annoying, they’re expensive and surprisingly hard to manage at city scale.

- Drivers hit potholes before anyone reports them.
- Reports (311 apps, phone calls, social posts) are noisy, duplicated, and rarely include precise location/severity.
- Maintenance becomes reactive: crews get dispatched after damage is widespread, not when early intervention is cheapest.

Our solution: **turn everyday driving into passive road inspection.**

Imagine a small camera + compute module mounted on vehicles that already drive through every neighborhood (city cars, buses, service fleets or even opt-in personal cars). As the vehicle drives, the system runs pothole detection and uploads a continuously updated “road damage layer” so the city can:

- see where potholes are forming (not weeks later)
- prioritize repairs by frequency/severity
- plan routes for maintenance crews with real data

In a real deployment, those detections would be geo-tagged with GPS and aligned to a city map.

## Why a robot (24h hackathon proof-of-concept)
In 24 hours, we can’t mount hardware on a real car, drive around Montreal, and build a fleet-scale data pipeline.

So we built a **miniature RouteScout rover** as a proof-of-concept that exercises the *same core loop* end-to-end:

**camera → detect pothole → attach location estimate → send to backend → visualize on a live dashboard**.

Because we were indoors at McGill (and GPS was unreliable), we couldn’t use the normal “car stack” for positioning. Instead, we used **LiDAR-based SLAM** to localize the rover and build a map in real time.

Conceptually, the only thing that changed was the pose source:

- Real car version: **pose from GPS/INS + wheel odometry**

- Hackathon rover: **pose from LiDAR SLAM + wheel odometry + IMU**

Everything downstream stayed the same: detections become “events,” events get streamed to the backend, and the dashboard updates live.

Same idea, smaller sandbox.

## System architecture (data flow)

```
┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                           LAPTOP (LAN server)                                                │
│                                                                                                              │
│  ┌───────────────────────────────┐        HTTP (poll + teleop)       ┌───────────────────────────────────┐   │
│  │ React dashboard (Vite)        │ <───────────────────────────────► │ Flask backend                     │   │
│  │  - polls:                     │                                   │  - REST API for dashboard         │   │
│  │    GET /api/ros/map           │                                   │  - Receives vision potholes       │   │
│  │    GET /api/ros/robot_pose_2d │                                   │  - Bridges Web <-> ROS (rosbridge)│   │
│  │    GET /api/potholes          │                                   │                                   │   │
│  │  - teleop:                    │                                   │  endpoints used:                  │   │
│  │    POST /api/ros/cmd_vel/*    │                                   │   - GET  /api/ros/map             │   │
│  └───────────────────────────────┘                                   │   - GET  /api/ros/robot_pose_2d   │   │
│                                                                      │   - GET  /api/potholes            │   │
│                                                                      │   - POST /api/ros/cmd_vel/*       │   │
│                                                                      └────────▲──────▲───────────────────┘   │
│                                                      HTTP POST /api/potholes  │      │ rosbridge WebSocket   │
└───────────────────────────────────────────────────────────────────────────────┼──────┼───────────────────────┘
                    ┌─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─┘      │ 
                    |                         ┌────────────────────────────────────────┴───────┐
                    │                         │        ROS 2 graph (topics + transforms)       │
                    │                         │   /map   /odom   /tf   /cmd_vel                │
                    │                         └───────────────▲───────────────────────▲────────┘
                    |                                         │ DDS / ROS 2           │ DDS / ROS 2
                    │                                         │                       │
┌───────────────────│─────────────────────────────────────────┴───────────────────────┴────────────────────────┐
│                   │         RASPBERRY PI (robot compute: vision + ROS 2 SLAM)                                │
│                   │                                                                                          │
│  ┌────────────────┴─────────────────────┐                     ┌───────────────────────────────────────────┐  │
│  │ Camera + YOLO (Python/OpenCV)        │                     │ ROS 2 + SLAM / localization               │  │
│  │  - detects potholes                  │                     │  - publishes: /map                        │  │
│  │  - sends detections to server        │                     │  - uses: /odom + /tf                      │  │
│  └───────────────┬──────────────────────┘                     │  - consumes: /cmd_vel (robot motion)      │  │
│                  │                                            └───────────────────▲───────────────────────┘  │
│                  ▼                                                                │                          │
│        (to Laptop Flask backend)                                                  │                          │
└───────────────────────────────────────────────────────────────────────────────────│──────────────────────────┘
                                                                                    │
                                                                                    │  UART (ROS 2 / DDS)
┌───────────────────────────────────────────────────────────────────────────────────│──────────────────────────┐
│                                ESP32 firmware (ESP-IDF + FreeRTOS)                │                          │
│  ┌───────────────────────────────────────────┐               ┌────────────────────┴─────────────────┐        │
│  │  micro-ROS client                         │               │ micro-ROS agent (host)               │        │
│  │  - subscribes: /cmd_vel                   │               │  - bridges MCU micro-ROS <-> ROS 2   │        │
│  │  - publishes:  /odom, /tf                 │               │    graph (DDS side)                  │        │
│  │  - publishes:  /odom, /tf                 │               └──────────────────────────────────────┘        │
│  └───────────────────────────────────────────┘                                                               │
│                                                                                                              │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

## How we built it
We built RouteScout as a full stack spanning embedded firmware, FreeRTOS, ROS 2, computer vision, and a web UI.

### 1) Real-time firmware (ESP32 + ESP-IDF/FreeRTOS)
The firmware lives in `robot-firmware/` and is built with FreeRTOS. Key pieces:

- **Motor interface over I2C** using a Yahboom motor controller. The motor controller provides **encoder feedback** and uses its **internal speed PID** when we send speed commands.
- **Differential/skid-steer mixing**: we accept Twist-style commands \\((v, \omega)\\) and compute left/right track velocities:

  $$v_L = v - \omega\,\frac{W}{2},\qquad v_R = v + \omega\,\frac{W}{2}$$

  Those are mapped to the 4 wheels (left pair = \\(v_L\\), right pair = \\(v_R\\)).

- **IMU sampling + sensor fusion** with an MPU6050.
  - The IMU task is interrupt-driven to keep sampling responsive.
  - We run a **Madgwick filter** to produce roll/pitch/yaw.

- **micro-ROS Nav2 bridge over UART**.
  - Subscribes to `cmd_vel`.
  - Publishes `odom` and `tf` so higher-level ROS tools can consume the base motion.

- **Heading correction PID (IMU-based yaw hold)**.
  Our lightweight cardboard chassis means wheel alignment isn’t perfect, so the robot naturally drifts even when commanded to go straight. The firmware includes a yaw PID that:
  - tracks a target yaw when the command is “mostly forward”,
  - injects a small corrective $\omega$ into the motion command.

This dramatically improves odometry consistency and (in practice) makes mapping behave better.

### 2) Odometry tuned for a skid-steer base
Skid-steer/differential models are sensitive to slip, especially during turns. Our odometry integrator computes linear velocity from left/right wheel angular velocity, and can **fuse IMU yaw** so heading stays stable even if the wheels slip.

Implementation detail: wheel speeds are computed from encoder deltas and converted into $\mathrm{rad/s}$, then into $\mathrm{m/s}$ using the wheel radius; pose integrates forward in 2D and publishes as `nav_msgs/Odometry`.

### 3) ROS 2 mapping + visualization
On the ROS side, we operate in a standard 2D navigation/mapping shape:

- `odom` + `tf` are provided by the micro-ROS base.
- A mapping node (e.g., SLAM Toolbox + a 2D LiDAR driver) produces:
  - `/map` (`nav_msgs/OccupancyGrid`)
  - TF that aligns `map -> odom`

The web UI reads `/map` and a computed **robot pose in map frame** for a clean RViz-like “you are here” view.

### 4) Computer vision: YOLOv4-tiny via OpenCV DNN
The pothole detector lives in `camera/camera_video.py` and uses:

- **YOLOv4-tiny** (`project_files/yolov4_tiny.cfg` + `project_files/yolov4_tiny.weights`)
- OpenCV’s `cv.dnn_DetectionModel` API.

Pipeline:

1. **Capture** from a webcam (Linux `/dev/video*` supported).
2. **Low-latency frame handling**: a background capture thread retains only the *latest* frame so inference doesn’t build up seconds of lag.
3. **Preprocess**: resize to the configured net size (must be a multiple of 32), scale by `1/255`, swap BGR→RGB.
4. **Inference + NMS** with configurable thresholds.
5. **Post-filtering**:
   - drop detections below a confidence threshold (`--min-score`, default 0.7)
   - drop giant boxes (`--max-area-frac`) to reduce obvious false positives
6. **Tracking**: a simple centroid-distance tracker assigns stable IDs across frames.
7. **Event extraction**: when a tracked pothole crosses a configurable “exit line” near the bottom of the image, we record an “exited pothole” event.

#### How detections reach the map
When the camera script records new exited potholes, it **POSTs them to the backend** (`POST /api/potholes`). The backend snapshots the robot pose at ingest time and stores it alongside the event.

This gives us a “pothole marker on the map” that is consistent and useful for triage, even before full camera-to-map projection is implemented.

### 5) Web stack: Flask backend + React dashboard
The web stack is:

- Backend: **Flask** (`backend/app.py`)
- ROS integration: **roslibpy** connecting to a **rosbridge websocket** server (`backend/ros_bridge.py`)
- Frontend: **React + Vite + Material UI** (`frontend/`)

#### Backend responsibilities
- **Teleop API**: endpoints like `POST /api/ros/cmd_vel/hold` and `/release`.
  - The backend can continuously publish to `/cmd_vel` at a fixed rate (configurable) so bases that expect steady commands don’t time out.
- **ROS data caching**: subscribes to `/map`, `/odom`, and optionally `/tf` + `/tf_static` .
- **2D pose helper**: computes robot pose in the `map` frame when TF is available (`GET /api/ros/robot_pose_2d`).
- **Pothole ingestion**: accepts pothole event batches (`POST /api/potholes`) and serves them to the UI (`GET /api/potholes`).

#### Frontend responsibilities
- **Live 2D map rendering**: the occupancy grid is converted into a canvas image for fast drawing.
- **Robot overlay**: a heading arrow drawn from the latest map-frame pose.
- **Pothole overlay**: markers drawn at the stored pose snapshots.
- **Teleop controls**: press/tap controls that latch commands and release-to-stop.

## Challenges we ran into
- **Networking + ROS 2 reality**: DDS/UDP discovery on eduroam is chaotic. A dedicated LAN (router/AP) makes demos *way* more stable.
  So we flashed a router with **OpenWRT** and made a dedicated local LAN:
  - consistent IP addressing
  - fewer multicast / UDP headaches
  - easier multi-device debugging (Pi + laptop + robot all on the same network)
- **SLAM tuning + motion consistency**: small drift or inconsistent wheel motion shows up immediately in mapping. IMU yaw stabilization + a yaw hold PID were big wins.
- **Real-time perception**: even a “tiny” YOLO can lag on embedded compute. Threaded capture + lightweight post-filtering kept latency reasonable.
- **Hardware chaos**: motor mounts, wiring, and power regulation always find a way to fail at the worst moment.

## Accomplishments that we're proud of ... in no particle order ...
1) We made it out alive!
- We made a beautiful robot
- The robot worked!
- We all made new friends

In all seriousness though, we are very proud of what we made. Our fully self-contained robot is successfully able to detect potholes, map them accurately, and communicate the data live. We also created a system that could theoretically feed this information to autonomous vehicles and the government, which under no exaggeration, litterly makes the world a better place!

## What we learned
We came into this competition with 0 practical knowledge of SLAM, ROS2 and LiDAR. We walked out with a very good understanding of how these concepts are applied in robotics.

We also gained hands-on experience with, sensor fusion, real-time robotics, and embedded programming. Beyond technical skills, we learned the importance of designing for both humans and machines, balancing reliability with playfulness in a real-world problem.

## What's next for The HoleyMoley RouteScout
The sky is limit! Well not really, unless we start building roads in the sky. But HoleyMoley is a very practical and efficient solutions for governments to identify immediate road maintenance needs. At the same time, the data collected by HoleyMoley can also be used to advance and existing automonous vehicle systems, leading to safer and smoother rides.



# React + Flask (Dev Setup)

## Quick start (recommended)

First-time setup:

```bash
cd backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Then, from the repo root:

```bash
npm install
npm run dev
```

Open the Vite URL printed in the terminal (usually `http://127.0.0.1:5173/`).

## Backend (Flask)

```bash
cd backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python app.py
```

Backend runs at `http://127.0.0.1:5000`.

### One-command start (Flask + rosbridge)

If you want the backend to start rosbridge + Flask together:

```bash
cd backend
chmod +x start_with_rosbridge.sh
./start_with_rosbridge.sh
```

This will run rosbridge using [backend/rosbridge_params.yaml](backend/rosbridge_params.yaml) (Jazzy-safe) and then start the Flask server with `ENABLE_ROSBRIDGE=true`.

## ROS2 (optional)

The Flask backend can connect to ROS2 topics via a rosbridge websocket server.

1) Install + start rosbridge in your ROS2 environment.

Install (Ubuntu apt, Jazzy):

```bash
sudo apt update
sudo apt install ros-jazzy-rosbridge-server
```

Start rosbridge:

On ROS2 Jazzy, `rosbridge_websocket` may crash if `delay_between_messages` is set as an integer (`0`) instead of a float (`0.0`). This repo includes a params file with the correct type.

```bash
source /opt/ros/jazzy/setup.bash

# Reliable on Jazzy (uses the float-typed workaround param):
ros2 run rosbridge_server rosbridge_websocket --ros-args --params-file backend/rosbridge_params.yaml

# Alternatively, if your system's launch file works for you:
# ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

2) Enable the integration when running Flask:

```bash
cd backend
source .venv/bin/activate
pip install -r requirements.txt

export ENABLE_ROSBRIDGE=true
export ROSBRIDGE_HOST=127.0.0.1
export ROSBRIDGE_PORT=9090
export ROSBRIDGE_TOPIC=/chatter
export ROSBRIDGE_MESSAGE_TYPE=std_msgs/msg/String

# Optional: keep publishing /cmd_vel at a steady rate (useful if your base
# controller expects 10-20Hz commands). When idle, the backend will publish zeros.
export ROSBRIDGE_CMD_VEL_STREAM=true
export ROSBRIDGE_CMD_VEL_STREAM_HZ=20
export ROSBRIDGE_CMD_VEL_IDLE_TIMEOUT_S=0.25

# Map + odom + TF (RViz-style 2D view)
export ROSBRIDGE_MAP_TOPIC=/map
export ROSBRIDGE_MAP_MESSAGE_TYPE=nav_msgs/msg/OccupancyGrid
export ROSBRIDGE_ODOM_TOPIC=/odom
export ROSBRIDGE_ODOM_MESSAGE_TYPE=nav_msgs/msg/Odometry
export ROSBRIDGE_TF_TOPIC=/tf
export ROSBRIDGE_TF_MESSAGE_TYPE=tf2_msgs/msg/TFMessage
export ROSBRIDGE_TF_STATIC_TOPIC=/tf_static
export ROSBRIDGE_TF_STATIC_MESSAGE_TYPE=tf2_msgs/msg/TFMessage

python app.py
```

Endpoints:
- `GET /api/ros/status`
- `GET /api/ros/latest`
- `GET /api/ros/map` (latest `/map` OccupancyGrid)
- `GET /api/ros/odom` (latest `/odom` Odometry)
- `GET /api/ros/tf` (latest `/tf` + `/tf_static`)
- `GET /api/ros/robot_pose_2d` (robot pose in map frame when TF available)
- `POST /api/ros/publish` with body like `{ "message": { "data": "hello" } }`
- `POST /api/ros/cmd_vel` with body like `{ "linear_x": 0.1, "angular_z": 0.0 }`
- `POST /api/ros/cmd_vel/hold` (press-and-hold) with body like `{ "linear_x": 0.1, "angular_z": 0.0 }`
- `POST /api/ros/cmd_vel/release` (release to stop)

Performance knobs (useful if `/cmd_vel` feels laggy):
- Disable heavy subscriptions: `ROSBRIDGE_SUBSCRIBE_MAP=false`, `ROSBRIDGE_SUBSCRIBE_TF=false`, etc.
    This reduces traffic over the rosbridge websocket.

### Expected ROS2 topics

For the 2D map UI to align like RViz, you typically need:

- `/map` (nav_msgs/OccupancyGrid)
- `/odom` (nav_msgs/Odometry)
- `/tf` and `/tf_static` (tf2_msgs/TFMessage), providing at least `map -> odom`

If TF is missing, the UI will still draw the robot pose from `/odom` but will show a warning that it may not align with the map.

## Frontend (React + Vite)

```bash
cd frontend
npm install
npm run dev
```

Frontend runs at `http://127.0.0.1:5173`.

### API calls

The frontend calls `/api/health`. In dev, Vite proxies `/api` to the Flask backend.


# Start the camera

./run_webcam.sh --headless --device /dev/video0 --dnn-target opencl-fp16 --net-width 320 --net-height 256 --print-every 10 --send-exited --server-url http://192.168.1.124:5000


# If you don't see the topics
ros2 daemon stop

pkill -f _ros2_daemon || true
sudo pkill -f _ros2_daemon || true

rm -rf ~/.ros/ros2cli

ros2 daemon start

ros2 topic list
ros2 node list