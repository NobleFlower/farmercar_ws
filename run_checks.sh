#!/usr/bin/env bash
set -e
cd /home/tree/farmercar_ws
source devel/setup.bash
python3 src/apriltag_servo/test/test_control.py
python3 src/tricycle_controller/test/test_kinematics.py
python3 src/velocity_adapter/test/test_mapping.py
python3 src/vehicle_controller/test/test_wire.py
for task_spec in 'apriltag_servo faults.test' 'tricycle_controller controller.test' 'velocity_adapter adapter.test' 'camera_manager geometry_preview.test' 'vehicle_controller bridge.test' 'vehicle_controller serial_transport.test' 'camera_manager control_pipeline.test'; do
  read -r task_package task_test <<< "$task_spec"
  task_log=/tmp/farmercar_final_$task_package.$task_test.log
  rostest "$task_package" "$task_test" > "$task_log" 2>&1 || { tail -n 45 "$task_log"; exit 1; }
  tail -n 9 "$task_log"
done
