# Multi-LiDAR Perception Pipeline (ROS 2)

Real-time fusion, object detection and tracking for four SICK LiDAR sensors on the research vessel **NOVA**, built with ROS 2 Jazzy, C++ (PCL) and Python.

Developed as a project in the Mechatronics Lab at the University of Duisburg-Essen. The goal was a reproducible, modular perception pipeline for a fixed-mounted multi-LiDAR setup that:

- fuses four point clouds into one common frame using TF,
- detects objects with two different clustering approaches (PCL in C++, DBSCAN in Python), on each sensor and on the fused cloud,
- tracks detected objects over time with a Kalman filter,
- measures processing time and detection counts so the configurations can be compared.

<!-- TODO: add an RViz screenshot or GIF of the fused cloud with bounding boxes and tracks here -->

---

## Pipeline overview

```
 /front/cloud ─┐
 /right/cloud ─┤                                   ┌─> object_detector_pcl (C++) ─┐
 /back/cloud  ─┼─> cloud_fusion ─> /nova/cloud_fused┤                              ├─> object_tracker ─> /tracks/*
 /left/cloud  ─┘   (TF + voxel dedup)              └─> cloud_detector (DBSCAN)  ──┘
                                                          │
                                                          └─> metrics_csv_logger ─> results/*.csv
```

| Stage | Node | Language | What it does |
|---|---|---|---|
| Frames | `robot_state_publisher` + `lidar_description` | URDF/Xacro | Fixed sensor frames on the vessel (`ship.urdf.xacro`) |
| Fusion | `cloud_fusion` | Python | Transforms all four clouds into `base_link`, drops stale clouds (`max_age_sec`), voxel de-duplication of overlapping regions, publishes `/nova/cloud_fused` |
| Detection (PCL) | `object_detector_pcl` | C++ | Voxel downsampling, statistical outlier removal, RANSAC plane removal, Euclidean clustering, bounding boxes as `MarkerArray` |
| Detection (DBSCAN) | `cloud_detector` | Python | Optional voxel downsampling, DBSCAN clustering (scikit-learn), bounding boxes as `MarkerArray` |
| Tracking | `object_tracker` | Python | Hungarian data association, one constant-velocity Kalman filter per track, stable track IDs and velocity output |
| Metrics | `metrics_csv_logger` | Python | Logs processing time, input points and detections per sensor/configuration to CSV |
| Tuning | `tuner/param_tuner.py` | Python | Local web UI to change DBSCAN/PCL parameters live (via `ros2 param set`) and save them back to YAML |

Helper nodes: `cloud_filter` (range/height filter, plus voxel filter in C++), `cloud_monitor` (C++/Python topic statistics), `plane_remove_o3d` (Open3D RANSAC plane removal).

---

## Results

Recorded with the real sensors; 11,073 metric samples in total. Values are means with standard deviation.

**Processing time per frame (ms)**

| Input | PCL (C++) | DBSCAN (Python) |
|---|---|---|
| Fused cloud | 135.4 ± 24.2 | 156.3 ± 28.1 |
| Front | 123.8 ± 22.9 | 80.8 ± 15.4 |
| Right | 117.3 ± 20.4 | 88.0 ± 16.6 |
| Left | 139.4 ± 24.3 | 90.1 ± 15.3 |
| Back | 98.2 ± 13.6 | 68.9 ± 11.8 |

**Detections per frame**

| Input | PCL (C++) | DBSCAN (Python) |
|---|---|---|
| Fused cloud | 75.2 ± 10.4 | 31.2 ± 4.3 |
| Single sensor | 27–35 | 10–12 |

![Processing time by sensor and configuration](results/plots/proc_time_boxplot.png)
![Average detections by sensor and configuration](results/plots/detections_barplot.png)

Notes:
- Point and detection counts depend on each detector's preprocessing and tuned parameters, so the numbers compare the two tuned configurations rather than the bare algorithms.
- All plots are in [`results/plots/`](results/plots/), the full summary in [`metrics_summary.txt`](results/plots/metrics_summary.txt), and the raw logs in `results/*.csv`. Plots are generated with `visualize_lidar_metrics.py` (install its extra packages with `pipenv run pip install -r visualization_requirements.txt`).

---

## Repository structure

```
lidar-ros2/
├── src/
│   ├── lidar_py_pkg/        # Python nodes: fusion, DBSCAN detector, tracker, metrics logger, utilities
│   ├── lidar_cpp_pkg/       # C++ nodes: PCL detector, filter, monitor
│   ├── lidar_description/   # URDF/Xacro with the sensor frames
│   ├── lidar_bringup/       # launch files, YAML configs, RViz configs (rviz/)
│   └── sick_scan_xd/        # SICK driver (git submodule, needed for live sensors)
├── tuner/                   # web-based live parameter tuner
├── results/                 # metric logs (CSV) and plots
├── bags/                    # place for ros2 bag recordings (not tracked in git)
├── Pipfile, Pipfile.lock    # Python dependencies (pipenv)
└── .envrc                   # optional direnv setup
```

---

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12
- colcon, rosdep, PCL (system packages)
- pipenv (installed via pipx); direnv is optional
- `sick_scan_xd` submodule for live sensor data

## Installation

```bash
# ROS 2 build tools
sudo apt update
sudo apt install -y git build-essential python3-colcon-common-extensions python3-rosdep pipx
pipx ensurepath && source ~/.bashrc
pipx install pipenv
sudo rosdep init || true
rosdep update

# Clone with submodules
git clone --recurse-submodules https://github.com/atlasvoxel/lidar-ros2.git ~/lidar_ws
cd ~/lidar_ws
# if you forgot --recurse-submodules:
git submodule update --init --recursive

# Python environment (venv inside the project)
export PIPENV_VENV_IN_PROJECT=1
pipenv --python 3.12
pipenv install            # installs from Pipfile.lock

# ROS dependencies and build
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
pipenv run colcon build --symlink-install
source install/setup.bash
```

Optional: with direnv installed, run `direnv allow` once in `~/lidar_ws`. The included `.envrc` then sources ROS 2, the pipenv environment and the workspace overlay automatically.

Sanity check:

```bash
pipenv run ros2 pkg executables lidar_py_pkg
pipenv run ros2 pkg executables lidar_cpp_pkg
```

## Usage

All launch files are in `src/lidar_bringup/launch/`. Each starts the sensor input, the frames, the processing nodes, metrics logging and RViz.

| Launch file | Purpose |
|---|---|
| `live_sensor.launch.xml` | Live sensors, fusion, PCL and DBSCAN detection side by side |
| `pcl_fused.launch.xml` / `pcl_per_sensor.launch.xml` | PCL detector on the fused cloud / on each sensor, with metrics logging |
| `dbscan_fused.launch.xml` / `dbscan_per_sensor.launch.xml` | DBSCAN detector on the fused cloud / on each sensor, with metrics logging |
| `live_tracking.launch.xml` | Fusion, both detectors and Kalman tracking |
| `bag_data.launch.xml` | Replay a recorded bag instead of live sensors (both detectors) |

```bash
pipenv run ros2 launch lidar_bringup pcl_fused.launch.xml
pipenv run ros2 launch lidar_bringup live_tracking.launch.xml
pipenv run ros2 launch lidar_bringup bag_data.launch.xml bag_path:=/path/to/your/bag
```

Live parameter tuning (while the pipeline is running):

```bash
pipenv run python3 tuner/param_tuner.py   # then open http://localhost:5000
```

Parameters live in `src/lidar_bringup/config/` (`fusion.yaml`, `detector_cpp.yaml`, `dbscan_fused.yaml`, `tracker.yaml`). To track DBSCAN detections instead of PCL, set `input_topic: /detections/dbscan/fused` in `tracker.yaml`.
