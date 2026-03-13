#!/bin/bash
###############################################################################
# Batch ORB-SLAM3 Runner with Map-Count Monitoring
#
# Runs SLAM N times, records success/failure and metrics for each run.
# Each run gets a full roscore restart to prevent message residue issues.
# Outputs a CSV summary at the end.
#
# Usage (from host):
#   bash scripts/run_slam_batch.sh [NUM_RUNS]
#   bash scripts/run_slam_batch.sh 10
###############################################################################

set -uo pipefail

CONTAINER="orbslam3_ros1"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TOTAL_RUNS=${1:-5}
SUMMARY_FILE="${PROJECT_DIR}/batch_summary.csv"

echo "run,tracking_lost,map_count,ate_rmse_m,rpe_trans_drift,rpe_rot_drift,completeness_pct,matched_poses,gt_poses,scale,run_folder" > "$SUMMARY_FILE"

full_cleanup() {
    docker exec "$CONTAINER" pkill -9 -f Mono_Compressed 2>/dev/null || true
    sleep 1
    docker exec "$CONTAINER" pkill -9 -f "rosbag play" 2>/dev/null || true
    sleep 1
    docker exec "$CONTAINER" pkill -9 -f rosmaster 2>/dev/null || true
    docker exec "$CONTAINER" pkill -9 -f roscore 2>/dev/null || true
    sleep 2

    # Verify clean
    local remaining
    remaining=$(docker exec "$CONTAINER" ps aux 2>/dev/null | grep -E "Mono_Compressed|rosbag" | grep -v grep | wc -l)
    if [ "$remaining" -gt 0 ]; then
        docker exec "$CONTAINER" pkill -9 -f Mono_Compressed 2>/dev/null || true
        docker exec "$CONTAINER" pkill -9 -f rosbag 2>/dev/null || true
        sleep 3
    fi

    # Fresh roscore
    docker exec -d "$CONTAINER" bash -c 'source /opt/ros/noetic/setup.bash && roscore' 2>/dev/null
    sleep 5
}

for RUN_NUM in $(seq 1 $TOTAL_RUNS); do
    echo ""
    echo "################################################################"
    echo " RUN $RUN_NUM / $TOTAL_RUNS"
    echo "################################################################"

    full_cleanup

    docker exec "$CONTAINER" rm -f /root/ORB_SLAM3/CameraTrajectory.txt /root/ORB_SLAM3/KeyFrameTrajectory.txt

    LOG_FILE="/tmp/slam_batch_run${RUN_NUM}.log"
    > "$LOG_FILE"

    # Launch SLAM
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
        echo ">>> RUN #$RUN_NUM: SLAM failed to start"
        echo "$RUN_NUM,START_FAIL,0,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A" >> "$SUMMARY_FILE"
        continue
    fi

    # Launch rosbag
    docker exec -d "$CONTAINER" bash -c '
        source /opt/ros/noetic/setup.bash
        rosbag play /root/ORB_SLAM3/data/HKisland_GNSS03.bag \
            --topics /left_camera/image/compressed -r 0.3 \
            > /tmp/rosbag_output.log 2>&1
    '
    sleep 3

    echo ">>> Monitoring run #$RUN_NUM..."
    TRACKING_LOST=false
    ROSBAG_DONE=false

    while true; do
        docker exec "$CONTAINER" cat /tmp/slam_output.log > "$LOG_FILE" 2>/dev/null || true

        MC=$(grep -c "New Map created" "$LOG_FILE" 2>/dev/null || true)
        MC=$(echo "${MC:-0}" | tr -d '[:space:]')

        if [ "$MC" -ge 2 ] 2>/dev/null; then
            echo ">>> RUN #$RUN_NUM: TRACKING LOST (map count=$MC)"
            TRACKING_LOST=true
            docker exec "$CONTAINER" pkill -9 -f Mono_Compressed 2>/dev/null || true
            docker exec "$CONTAINER" pkill -9 -f "rosbag play" 2>/dev/null || true
            sleep 3
            break
        fi

        if ! docker exec "$CONTAINER" pgrep -f "rosbag play" &>/dev/null; then
            if [ "$ROSBAG_DONE" = false ]; then
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

    FINAL_MAP_COUNT=$(grep -c "New Map created" "$LOG_FILE" 2>/dev/null || true)
    FINAL_MAP_COUNT=$(echo "${FINAL_MAP_COUNT:-0}" | tr -d '[:space:]')

    if [ "$FINAL_MAP_COUNT" -ge 2 ] 2>/dev/null; then
        TRACKING_LOST=true
    fi

    if [ "$TRACKING_LOST" = true ]; then
        echo ">>> RUN #$RUN_NUM: FAILED (maps=$FINAL_MAP_COUNT)"
        echo "$RUN_NUM,YES,$FINAL_MAP_COUNT,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A" >> "$SUMMARY_FILE"
        continue
    fi

    TRAJ_EXISTS=$(docker exec "$CONTAINER" bash -c '[ -f /root/ORB_SLAM3/CameraTrajectory.txt ] && echo "yes" || echo "no"')
    if [ "$TRAJ_EXISTS" != "yes" ]; then
        echo ">>> RUN #$RUN_NUM: FAILED (no trajectory)"
        echo "$RUN_NUM,NO_TRAJ,$FINAL_MAP_COUNT,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A" >> "$SUMMARY_FILE"
        continue
    fi

    echo ">>> RUN #$RUN_NUM: SUCCESS. Saving & evaluating..."

    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    RUN_DIR="${PROJECT_DIR}/run_${TIMESTAMP}"
    mkdir -p "${RUN_DIR}/output" "${RUN_DIR}/figures" "${RUN_DIR}/scripts"

    cp "${PROJECT_DIR}/scripts/evaluate_vo_accuracy.py" "${RUN_DIR}/scripts/"
    cp "${PROJECT_DIR}/scripts/generate_report_figures.py" "${RUN_DIR}/scripts/" 2>/dev/null || true

    docker exec "$CONTAINER" cat /root/ORB_SLAM3/CameraTrajectory.txt > "${RUN_DIR}/output/CameraTrajectory.txt"
    docker exec "$CONTAINER" cat /root/ORB_SLAM3/KeyFrameTrajectory.txt > "${RUN_DIR}/output/KeyFrameTrajectory.txt" 2>/dev/null || true
    docker exec "$CONTAINER" cat /root/ORB_SLAM3/ground_truth.txt > "${RUN_DIR}/output/ground_truth.txt"
    cp "$LOG_FILE" "${RUN_DIR}/output/slam_run.log"

    EVAL_OUTPUT=$(docker exec "$CONTAINER" bash -c "
        cd /root/AAE5303_assignment2_orbslam3_demo-
        python3 scripts/evaluate_vo_accuracy.py \
            --groundtruth /root/ORB_SLAM3/ground_truth.txt \
            --estimated /root/ORB_SLAM3/CameraTrajectory.txt \
            --workdir /tmp/aae5303_eval \
            --json-out /tmp/aae5303_eval/evaluation_report.json
    " 2>&1)
    echo "$EVAL_OUTPUT"
    echo "$EVAL_OUTPUT" > "${RUN_DIR}/output/evaluation_output.txt"

    docker exec "$CONTAINER" cat /tmp/aae5303_eval/evaluation_report.json > "${RUN_DIR}/output/evaluation_report.json" 2>/dev/null || true

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

    ATE_RMSE=$(python3 -c "import json; d=json.load(open('${RUN_DIR}/output/evaluation_report.json')); print(f\"{d['ate_rmse_m']:.6f}\")" 2>/dev/null || echo "N/A")
    RPE_TRANS=$(python3 -c "import json; d=json.load(open('${RUN_DIR}/output/evaluation_report.json')); print(f\"{d['rpe_trans_drift_m_per_m']:.6f}\")" 2>/dev/null || echo "N/A")
    RPE_ROT=$(python3 -c "import json; d=json.load(open('${RUN_DIR}/output/evaluation_report.json')); print(f\"{d['rpe_rot_drift_deg_per_100m']:.6f}\")" 2>/dev/null || echo "N/A")
    COMPLETENESS=$(python3 -c "import json; d=json.load(open('${RUN_DIR}/output/evaluation_report.json')); print(f\"{d['completeness_pct']:.2f}\")" 2>/dev/null || echo "N/A")
    MATCHED=$(python3 -c "import json; d=json.load(open('${RUN_DIR}/output/evaluation_report.json')); print(d['matched_poses'])" 2>/dev/null || echo "N/A")
    GT_POSES=$(python3 -c "import json; d=json.load(open('${RUN_DIR}/output/evaluation_report.json')); print(d['gt_poses'])" 2>/dev/null || echo "N/A")
    SCALE=$(echo "$EVAL_OUTPUT" | grep "Scale correction" | awk '{print $NF}' | head -1)
    SCALE=${SCALE:-N/A}

    echo "$RUN_NUM,NO,$FINAL_MAP_COUNT,$ATE_RMSE,$RPE_TRANS,$RPE_ROT,$COMPLETENESS,$MATCHED,$GT_POSES,$SCALE,$RUN_DIR" >> "$SUMMARY_FILE"
    echo ">>> RUN #$RUN_NUM: ATE=$ATE_RMSE, Completeness=$COMPLETENESS%"
done

echo ""
echo "################################################################"
echo " ALL $TOTAL_RUNS RUNS COMPLETE"
echo "################################################################"
echo "Summary: $SUMMARY_FILE"
cat "$SUMMARY_FILE"
