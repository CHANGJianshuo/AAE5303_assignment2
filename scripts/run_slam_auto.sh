#!/bin/bash
###############################################################################
# Automated ORB-SLAM3 Single Run with Map-Count Monitoring
#
# Runs SLAM inside the orbslam3_ros1 container, monitors for tracking loss
# (new map creation, map count >= 2). If tracking is lost, kills and retries.
# On success, saves outputs and runs evaluation + visualization.
#
# Usage (from host):
#   bash scripts/run_slam_auto.sh
###############################################################################

set -euo pipefail

CONTAINER="orbslam3_ros1"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAX_RETRIES=20
RETRY=0

echo "=========================================="
echo " ORB-SLAM3 Automated Runner"
echo "=========================================="

while true; do
    RETRY=$((RETRY + 1))
    if [ "$RETRY" -gt "$MAX_RETRIES" ]; then
        echo "ERROR: Exceeded $MAX_RETRIES retries. Giving up."
        exit 1
    fi

    echo ""
    echo ">>> Attempt #$RETRY"

    # Full cleanup: kill everything including roscore, then restart
    docker exec "$CONTAINER" pkill -9 -f Mono_Compressed 2>/dev/null || true
    sleep 1
    docker exec "$CONTAINER" pkill -9 -f "rosbag play" 2>/dev/null || true
    sleep 1
    docker exec "$CONTAINER" pkill -9 -f rosmaster 2>/dev/null || true
    docker exec "$CONTAINER" pkill -9 -f roscore 2>/dev/null || true
    sleep 2

    # Fresh roscore
    docker exec -d "$CONTAINER" bash -c 'source /opt/ros/noetic/setup.bash && roscore' 2>/dev/null
    sleep 5

    # Clean old trajectory files
    docker exec "$CONTAINER" rm -f /root/ORB_SLAM3/CameraTrajectory.txt /root/ORB_SLAM3/KeyFrameTrajectory.txt

    LOG_FILE="/tmp/slam_run_${RETRY}.log"
    > "$LOG_FILE"

    # Launch SLAM (detached)
    docker exec -d "$CONTAINER" bash -c '
        source /opt/ros/noetic/setup.bash
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ORB_SLAM3/Examples_old/ROS
        cd /root/ORB_SLAM3
        rosrun ORB_SLAM3 Mono_Compressed \
            Vocabulary/ORBvoc.txt \
            Examples/Monocular/HKisland_Mono.yaml \
            /camera/image_raw/compressed:=/left_camera/image/compressed \
            > /tmp/slam_output.log 2>&1
    '
    sleep 12

    if ! docker exec "$CONTAINER" pgrep -f Mono_Compressed &>/dev/null; then
        echo ">>> SLAM failed to start, retrying..."
        continue
    fi

    # Launch rosbag (detached)
    docker exec -d "$CONTAINER" bash -c '
        source /opt/ros/noetic/setup.bash
        rosbag play /root/ORB_SLAM3/data/HKisland_GNSS03.bag \
            --topics /left_camera/image/compressed -r 0.3 \
            > /tmp/rosbag_output.log 2>&1
    '
    sleep 3

    echo ">>> Monitoring for tracking loss (map count >= 2)..."
    FAILED=false
    ROSBAG_DONE=false

    while true; do
        docker exec "$CONTAINER" cat /tmp/slam_output.log > "$LOG_FILE" 2>/dev/null || true

        MC=$(grep -c "New Map created" "$LOG_FILE" 2>/dev/null || true)
        MC=$(echo "${MC:-0}" | tr -d '[:space:]')

        if [ "$MC" -ge 2 ] 2>/dev/null; then
            echo ">>> DETECTED: Map count = $MC (>= 2). Tracking lost!"
            FAILED=true
            docker exec "$CONTAINER" pkill -9 -f Mono_Compressed 2>/dev/null || true
            docker exec "$CONTAINER" pkill -9 -f "rosbag play" 2>/dev/null || true
            sleep 3
            break
        fi

        if ! docker exec "$CONTAINER" pgrep -f "rosbag play" &>/dev/null; then
            if [ "$ROSBAG_DONE" = false ]; then
                echo ">>> Rosbag finished, waiting for SLAM to wrap up..."
                ROSBAG_DONE=true
                sleep 8
                docker exec "$CONTAINER" bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /Mono 2>/dev/null' || true
                sleep 5
                docker exec "$CONTAINER" pkill Mono_Compressed 2>/dev/null || true
                sleep 3
                docker exec "$CONTAINER" cat /tmp/slam_output.log > "$LOG_FILE" 2>/dev/null || true
                break
            fi
        fi
        sleep 5
    done

    if [ "$FAILED" = true ]; then
        echo ">>> Attempt #$RETRY failed (tracking lost). Retrying..."
        continue
    fi

    # Final map count check
    FINAL_MC=$(grep -c "New Map created" "$LOG_FILE" 2>/dev/null || true)
    FINAL_MC=$(echo "${FINAL_MC:-0}" | tr -d '[:space:]')
    if [ "$FINAL_MC" -ge 2 ] 2>/dev/null; then
        echo ">>> Attempt #$RETRY had $FINAL_MC maps. Retrying..."
        continue
    fi

    TRAJ_EXISTS=$(docker exec "$CONTAINER" bash -c '[ -f /root/ORB_SLAM3/CameraTrajectory.txt ] && echo "yes" || echo "no"')
    if [ "$TRAJ_EXISTS" != "yes" ]; then
        echo ">>> No CameraTrajectory.txt generated. Retrying..."
        continue
    fi

    echo ">>> Attempt #$RETRY SUCCEEDED! Map count = $FINAL_MC"
    break
done

# Save results
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RUN_DIR="${PROJECT_DIR}/run_${TIMESTAMP}"
echo ">>> Creating run folder: $RUN_DIR"
mkdir -p "${RUN_DIR}/output" "${RUN_DIR}/figures" "${RUN_DIR}/scripts"

cp "${PROJECT_DIR}/scripts/evaluate_vo_accuracy.py" "${RUN_DIR}/scripts/"
cp "${PROJECT_DIR}/scripts/generate_report_figures.py" "${RUN_DIR}/scripts/" 2>/dev/null || true

docker exec "$CONTAINER" cat /root/ORB_SLAM3/CameraTrajectory.txt > "${RUN_DIR}/output/CameraTrajectory.txt"
docker exec "$CONTAINER" cat /root/ORB_SLAM3/KeyFrameTrajectory.txt > "${RUN_DIR}/output/KeyFrameTrajectory.txt" 2>/dev/null || true
docker exec "$CONTAINER" cat /root/ORB_SLAM3/ground_truth.txt > "${RUN_DIR}/output/ground_truth.txt"
cp "$LOG_FILE" "${RUN_DIR}/output/slam_run.log"

# Evaluate
echo ">>> Running evaluation..."
docker exec "$CONTAINER" bash -c "
    cd /root/AAE5303_assignment2_orbslam3_demo-
    python3 scripts/evaluate_vo_accuracy.py \
        --groundtruth /root/ORB_SLAM3/ground_truth.txt \
        --estimated /root/ORB_SLAM3/CameraTrajectory.txt \
        --workdir /tmp/aae5303_eval \
        --json-out /tmp/aae5303_eval/evaluation_report.json
" 2>&1 | tee "${RUN_DIR}/output/evaluation_output.txt"

docker exec "$CONTAINER" cat /tmp/aae5303_eval/evaluation_report.json > "${RUN_DIR}/output/evaluation_report.json" 2>/dev/null || true

# Visualize
echo ">>> Generating figures..."
docker exec "$CONTAINER" bash -c "
    cd /root/AAE5303_assignment2_orbslam3_demo-
    python3 scripts/generate_report_figures.py \
        --gt /root/ORB_SLAM3/ground_truth.txt \
        --est /root/ORB_SLAM3/CameraTrajectory.txt \
        --evo-ape-zip /tmp/aae5303_eval/ate.zip \
        --out /tmp/aae5303_eval/trajectory_evaluation.png \
        --title-suffix 'HKisland_GNSS03'
" 2>&1

docker exec "$CONTAINER" cat /tmp/aae5303_eval/trajectory_evaluation.png > "${RUN_DIR}/figures/trajectory_evaluation.png" 2>/dev/null || true
for f in ate.zip rpe_trans.zip rpe_rot.zip; do
    docker exec "$CONTAINER" cat "/tmp/aae5303_eval/$f" > "${RUN_DIR}/output/$f" 2>/dev/null || true
done

echo ""
echo "=========================================="
echo " DONE! Run folder: $RUN_DIR"
echo "=========================================="
cat "${RUN_DIR}/output/evaluation_report.json" 2>/dev/null
