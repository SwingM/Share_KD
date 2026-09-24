#!/bin/bash
export ROS_DOMAIN_ID=97
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>auto</NetworkInterfaceAddress><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery><Transport><SharedMemory>disable</SharedMemory></Transport></Domain></CycloneDDS>'
export GAZEBO_MODEL_PATH="/home/ubuntu/go2_target_seek_delivery/QY_MODEL/models:/home/ubuntu/go2_target_seek_delivery/KD_MODEL/models:$HOME/.gazebo/models"
export DELIVERY_ROOT=/home/ubuntu/go2_target_seek_delivery
export ONNXRUNTIME=/home/ubuntu/go2_target_seek_delivery/onnxruntime-linux-x64-1.16.3/lib
export LD_LIBRARY_PATH=/home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/lib:$ONNXRUNTIME:$LD_LIBRARY_PATH
export PYTHONPATH=/home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/lib/rl_sar:$PYTHONPATH
export CACHEDAssetsPath=/home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/share/rl_sar/resource/go2
export PATH=/usr/bin:/home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/bin:$PATH

source /opt/ros/humble/setup.bash
source /home/ubuntu/go2_target_seek_delivery/go2_ws_v2/install/setup.bash
source /home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/setup.bash

bash /home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/scripts/go2_cmdvel_three.sh use_camera:=false
