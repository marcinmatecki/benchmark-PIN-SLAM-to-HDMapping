
# PIN-SLAM to HDMapping simplified instruction

## Step 1 (prepare data)
Download the dataset `kitti_seq05_ros1.bag` by clicking [link](https://huggingface.co/datasets/kubchud/kitti_to_ros/resolve/main/kitti_seq05_ros1.bag) (it is part of [kitti_seq](https://github.com/Jakubach/kitti_to_ros)).

### Extract the dataset

Folder `kitti_seq05_ros1.bag`.

```shell
unzip kitti_seq05_ros1.bag
```
After extraction, the folder name will be `kitti_seq05_ros1.bag`  is an input for further calculations. (without the `.zip` extension).

It should be located in `~/hdmapping-benchmark/data`.

## Step 2 — Build the Docker image

```bash
docker build -t pin-slam_cuda .
```

This installs:
- CUDA 11.8 runtime (Ubuntu 22.04) + PyTorch 2.1
- PIN-SLAM (from the `src/PIN_SLAM` submodule) and its Python dependencies
- `rosbags` (bag reading without ROS) and `laspy`/`lazrs` (LAZ writing)

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-pin-slam.sh
TOPIC=/your/pointcloud_topic ./docker_session_run-pin-slam.sh /path/to/input.bag /path/to/output/dir
```

Optional environment variables:

| Variable | Default | Meaning |
|----------|---------|---------|
| `CONFIG` | `config/lidar_slam/run.yaml` | PIN-SLAM config file (inside the PIN_SLAM repo) |
| `TOPIC` | `/velodyne_points` | point cloud topic in the bag |
| `VIS` | `1` | show PIN-SLAM's live 3D viewer (auto-off without `DISPLAY`) |
| `CPU_ONLY` | `1` | `1` = CPU only (very slow), `0` = use the GPU |
| `POINT_SKIP` | `1` | converter keeps every Nth point |

## Step 4 (Open and visualize data)
Expected data should appear in ~/hdmapping-benchmark/data/output_hdmapping-PIN-SLAM
Use tool [multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping) to open session.json from ~/hdmapping-benchmark/data/output_hdmapping-PIN-SLAM.

You should see following data

lio_initial_poses.reg

poses.reg

scan_lio_*.laz

session.json

trajectory_lio_*.csv

## Contact email
januszbedkowski@gmail.com

## Modifications to PIN-SLAM

This benchmark uses [PIN-SLAM](https://github.com/PRBonn/PIN_SLAM)
(upstream commit `<COMMIT>`) with a small set of additions.
**The estimation pipeline (odometry, implicit neural mapping, loop detection, pose graph optimisation) is unchanged**;
the changes only export loop closure data for evaluation.

All changes are contained in [`patches/pin-slam.patch`](patches/pin-slam.patch)
and are applied automatically during `docker build`.

### What was added

| File | Change |
|---|---|
| `utils/pgo.py` | `PoseGraphManager` creates the loop output folder and `loop_constraints.csv`; new methods `save_loop_from_gtsam()` (computes the relative pose of a loop from the optimised pose graph and saves the loop clouds) and `save_pcd_binary()` (binary PCD writer) |
| `pin_slam.py` | After every accepted loop closure and pose graph optimisation, calls `save_loop_from_gtsam()` with the frame indices, frame timestamps from the bag, the current scan and the local neural point map; prints frame and loop timestamps to the log |
| `pin_slam_ros.py` | Same call for the ROS mode (not used by this benchmark, which runs `pin_slam.py` offline) |

### Loop closure export

For every loop closure that passes registration and is added to the pose graph
(`pin_slam.py`, after `pgm.optimize_pose_graph()`), the following is written to
`/output/loop_clouds/` and `/output/loop_clouds_global/` in the container
(`loop_clouds/` and `loop_clouds_global/` in the output directory on the host):

**`loop_clouds/loop_constraints.csv`** – one row per accepted loop closure:

| Column | Description |
|---|---|
| `current_id` | index of the current frame |
| `history_id` | index of the matched historical frame |
| `current_time` | timestamp of the current frame from the bag [s] |
| `history_time` | timestamp of the historical frame from the bag [s] |
| `tx, ty, tz` | relative translation history → current [m] |
| `roll, pitch, yaw` | relative rotation history → current [rad], ZYX convention |

The relative pose is computed from the **pose graph after optimisation**:
`T_history_current = inv(T_world_history) · T_world_current`.

**Point clouds (PCD, binary, fields `x y z intensity`):**

In the historical frame coordinate system (`loop_clouds/`):
- `loop_<current_id>_current_<history_id>.pcd` – current scan (after PIN-SLAM preprocessing),
- `loop_<current_id>_history_<history_id>.pcd` – local neural point map at the time of the loop closure.

In the world coordinate system (`loop_clouds_global/`):
- `loop_<current_id>_current_<history_id>_global.pcd`,
- `loop_<current_id>_history_<history_id>_global.pcd`.

> Note: the history cloud contains PIN-SLAM **neural points** (the positions of the
> implicit map features), not a raw LiDAR scan.

### Notes

- `current_id` / `history_id` are PIN-SLAM frame indices; use `current_time` /
  `history_time` to match loop closures to the trajectory and the ground truth.
- `loop_constraints.csv` is created only if it does not exist and rows are appended.
  Remove the `loop_clouds/` folder before running the pipeline again, otherwise
  results of several runs are mixed in one file.
- The output paths `/output/loop_clouds` and `/output/loop_clouds_global` are hard-coded
  in `utils/pgo.py`.
- Loop data is written synchronously after each pose graph optimisation. This does not
  affect the estimated trajectory, but adds I/O time and verbose debug output to the log.