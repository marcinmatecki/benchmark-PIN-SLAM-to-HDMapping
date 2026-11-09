#!/bin/bash
# Runs PIN-SLAM on a bag and converts the result to an HDMapping session.
#
# Unlike the ROS-based benchmarks (roscore/tmux/rosbag record), PIN-SLAM runs
# OFFLINE: its built-in dataloader reads the ROS1 .bag file (or ROS2 bag
# directory) directly and writes TUM-format pose files stamped with sensor
# time. The converter then rebuilds the dense world-frame cloud from the input
# bag + poses and writes the HDMapping session. Live progress is shown in
# PIN-SLAM's own 3D viewer (instead of RViz).
#
# Usage:
#   ./docker_session_run-pin-slam.sh <input.bag | ros2_bag_dir> <output_dir>
#
# Environment variables:
#   CONFIG     - PIN-SLAM config inside the repo (default: config/lidar_slam/run.yaml)
#   TOPIC      - point cloud topic in the bag (default: /livox/pointcloud)
#   VIS        - 1 = show PIN-SLAM 3D viewer (default: 1; auto-off without DISPLAY)
#   CPU_ONLY   - 1 = run on CPU (very slow; default 0 = CUDA GPU)
#   POINT_SKIP - converter keeps every Nth point (default 1 = all)

set -e

IMAGE_NAME='pin-slam_cuda'
OUTPUT_NAME='output_hdmapping-PIN-SLAM'

CONFIG="${CONFIG:-config/lidar_slam/run.yaml}"
TOPIC="${TOPIC:-/velodyne_points}"
VIS="${VIS:-1}"
CPU_ONLY="${CPU_ONLY:-1}"
POINT_SKIP="${POINT_SKIP:-1}"

if [[ $# -lt 2 ]]; then
  if command -v zenity >/dev/null 2>&1; then
    DATASET_HOST_PATH=$(zenity --file-selection --title="Select input bag (.bag file or ROS2 bag directory)")
    OUTPUT_HOST_DIR=$(zenity --file-selection --directory --title="Select output directory")
  else
    echo "Usage: $0 <input.bag | ros2_bag_dir> <output_dir>"
    echo ""
    echo "  CONFIG     - PIN-SLAM config (default: config/lidar_slam/run.yaml)"
    echo "  TOPIC      - point cloud topic (default: /livox/pointcloud)"
    echo "  VIS        - 1 = live 3D viewer (default: 1)"
    echo "  CPU_ONLY   - 1 = CPU only, very slow (default: 0)"
    echo "  POINT_SKIP - keep every Nth point in converter (default: 1)"
    exit 1
  fi
else
  DATASET_HOST_PATH=$(realpath "$1")
  OUTPUT_HOST_DIR=$(realpath "$2")
fi

if [[ ! -e "$DATASET_HOST_PATH" ]]; then
  echo "Error: input $DATASET_HOST_PATH does not exist"
  exit 1
fi
mkdir -p "$OUTPUT_HOST_DIR"

DATASET_DIR=$(dirname "$DATASET_HOST_PATH")
DATASET_BASE=$(basename "$DATASET_HOST_PATH")

mkdir -p "$OUTPUT_HOST_DIR/pin_experiments"
mkdir -p "$OUTPUT_HOST_DIR/loop_clouds"
# GPU: prefer the nvidia runtime (works in both legacy and CDI-mode toolkit
# configs, where the --gpus hook path is rejected); fall back to --gpus all,
# then to CPU (PIN-SLAM on CPU is very slow — expect it only for tiny bags).
if [[ "$CPU_ONLY" == "1" ]]; then
  GPU_ARGS=""
elif docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; then
  GPU_ARGS="--runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility"
elif docker run --rm --gpus all ubuntu:22.04 true >/dev/null 2>&1; then
  GPU_ARGS="--gpus all"
else
  GPU_ARGS=""
  CPU_ONLY=1
  echo "WARNING: no working NVIDIA docker runtime detected — running on CPU (very slow)."
fi

# Viewer needs X11.
VIS_ARGS=""
if [[ "$VIS" == "1" && -n "$DISPLAY" ]]; then
  VIS_FLAG="-v"
  xhost +local:docker >/dev/null 2>&1 || true
else
  VIS_FLAG=""
fi

echo "Input bag       : $DATASET_HOST_PATH"
echo "Output dir      : $OUTPUT_HOST_DIR/$OUTPUT_NAME"
echo "PIN-SLAM config : $CONFIG"
echo "Cloud topic     : $TOPIC"
echo "Viewer          : ${VIS_FLAG:-off}"
echo "CPU only        : $CPU_ONLY"

TMUX_SESSION="pinslam"


docker run -it --rm \
    --user 1000:1000 \
    --shm-size=8g \
    $GPU_ARGS \
    -e DISPLAY="$DISPLAY" \
    -e XDG_RUNTIME_DIR=/tmp/xdg \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$DATASET_DIR":/data \
    -v "$OUTPUT_HOST_DIR":/output \
    -v "$OUTPUT_HOST_DIR/pin_experiments":/tmp/pin_experiments \
    "$IMAGE_NAME" bash -c "

set -e

cd /pin_ws/PIN_SLAM

TMUX_SESSION='$TMUX_SESSION'

# ------------------------------------------------------------
# EXISTING TMUX SESSION CHECK
# ------------------------------------------------------------

if tmux has-session -t \"\$TMUX_SESSION\" 2>/dev/null; then
    echo \"[tmux] session \$TMUX_SESSION already exists\"
    echo \"[tmux] attach with: tmux attach -t \$TMUX_SESSION\"
    exit 1
fi

# ------------------------------------------------------------
# CREATE TMUX SESSION
# ------------------------------------------------------------

tmux new-session -d -s \"\$TMUX_SESSION\" -n pinslam

# ============================================================
# PANEL 1: PIN-SLAM
# ============================================================

tmux send-keys -t \"\$TMUX_SESSION\":0 '
set -e

cd /pin_ws/PIN_SLAM

echo \"======================================\"
echo \"        PIN-SLAM START\"
echo \"======================================\"
echo
date
echo

CPU_FLAG=\"\"

if [[ \"'"$CPU_ONLY"'\" == \"1\" ]]; then
    CPU_FLAG=\"-c\"
fi

echo \"[pin-slam] CPU flag: \$CPU_FLAG\"
echo \"[pin-slam] config: '"$CONFIG"'\"
echo \"[pin-slam] topic : '"$TOPIC"'\"
echo \"[pin-slam] bag   : /data/'"$DATASET_BASE"'\"
echo

python3 pin_slam.py \
    '"$CONFIG"' \
    rosbag \
    '"$TOPIC"' \
    -d \
    -i \"/data/'"$DATASET_BASE"'\" \
    -o /tmp/pin_experiments \
    \$CPU_FLAG \
    '"$VIS_FLAG"' || true

echo
echo \"[pin-slam] finished\"
date
echo

RUN_DIR=\$(ls -td /tmp/pin_experiments/*/ | head -1)

echo \"[pin-slam] Run dir: \$RUN_DIR\"
echo

tmux wait-for -S PINSLAM_DONE

echo \"[pin-slam] PINSLAM_DONE sent\"
' C-m

# ============================================================
# PANEL 2: CONVERTER
# ============================================================

tmux split-window -v -t \"\$TMUX_SESSION\":0

tmux send-keys -t \"\$TMUX_SESSION\":0.1 '
set -e

echo \"======================================\"
echo \"        CONVERTER\"
echo \"======================================\"
echo

echo \"[converter] waiting for PIN-SLAM...\"

tmux wait-for PINSLAM_DONE

echo
echo \"[converter] PIN-SLAM finished\"
echo

RUN_DIR=\$(ls -td /tmp/pin_experiments/*/ | head -1)

echo \"[converter] Run dir: \$RUN_DIR\"
echo
echo \"[converter] start\"
date
echo

python3 /pin_ws/converter/pinslam_to_hdmapping.py \
    --bag \"/data/'"$DATASET_BASE"'\" \
    --run-dir \"\$RUN_DIR\" \
    --topic '"$TOPIC"' \
    --out \"/output/'"$OUTPUT_NAME"'\" \
    --point-skip '"$POINT_SKIP"'

echo
echo \"[converter] copying *_tum.txt\"

cp -v \"\$RUN_DIR\"/*_tum.txt \
    \"/output/'"$OUTPUT_NAME"'/\"

echo
echo \"[converter] fixing permissions\"

chmod -R a+rw \
    \"/output/'"$OUTPUT_NAME"'\"

pkill -f tmux
echo
echo \"======================================\"
echo \"       CONVERTER FINISHED\"
echo \"======================================\"
echo
date
echo

tmux wait-for -S CONVERTER_DONE
' C-m

# ============================================================
# WINDOW 1: CONTROL
# ============================================================

tmux new-window -t \"\$TMUX_SESSION\":1 -n control

tmux send-keys -t \"\$TMUX_SESSION\":1 '
echo \"======================================\"
echo \"      PIN-SLAM + CONVERTER\"
echo \"======================================\"
echo

echo \"PIN-SLAM:\"
echo \"  tmux attach -t '"$TMUX_SESSION"'\"
echo

echo \"[control] Loop corrected watcher started\"

(
    while true; do
        if tmux capture-pane -p -S - -t \"\$TMUX_SESSION\":0.0 2>/dev/null | grep -Fq \"Loop corrected:\"; then
            echo
            echo \"[control] FOUND: Loop corrected:\"

                
            sleep 10

            echo \"[control] killing python3 pin_slam.py\"

            pkill -f \"python3 pin_slam.py\" 2>/dev/null || true

            echo \"[control] PIN-SLAM process killed\"
            break
        fi

        sleep 1
    done
) &

echo \"[control] watcher running\"
echo
echo \"Waiting for converter...\"

tmux wait-for CONVERTER_DONE

echo
echo \"======================================\"
echo \"        PIPELINE FINISHED\"
echo \"======================================\"
echo

echo \"Results:\"
echo \"/output/'"$OUTPUT_NAME"'\"
echo

echo \"Run directory:\"
echo \"\$(ls -td /tmp/pin_experiments/*/ | head -1)\"
echo
' C-m

# ------------------------------------------------------------
# INFO
# ------------------------------------------------------------

echo
echo \"======================================\"
echo \" TMUX SESSION: \$TMUX_SESSION\"
echo \"======================================\"
echo
echo \"Attach:\"
echo \"  tmux attach -t \$TMUX_SESSION\"
echo
echo \"Windows:\"
echo \"  0 = pinslam\"
echo \"  1 = control\"
echo
echo \"In window 0:\"
echo \"  pane 0 = PIN-SLAM\"
echo \"  pane 1 = converter\"
echo

# ------------------------------------------------------------
# ATTACH
# ------------------------------------------------------------

tmux attach -t \"\$TMUX_SESSION\"

"