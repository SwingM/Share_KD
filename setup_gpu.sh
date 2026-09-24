#!/bin/bash
# GPU rendering setup for Gazebo on NVIDIA A800
# Run this BEFORE starting any gazebo simulation:
#   source /home/ubuntu/go2_target_seek_delivery/setup_gpu.sh

# NVIDIA GPU rendering
export NVIDIA_VISIBLE_DEVICES=all
export NVIDIA_DRIVER_CAPABILITIES=all

# Force GPU rendering for Gazebo (not software rendering)
export LIBGL_ALWAYS_SOFTWARE=0
export GPU_FORCE_NFB4_DMA=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# CUDA paths (CUDA 11.8)
export CUDA_HOME=/usr/local/cuda-11.8
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# Gazebo model path (DO NOT set GAZEBO_RESOURCE_PATH - it crashes gazebo)
export GAZEBO_MODEL_PATH="/home/ubuntu/go2_target_seek_delivery/QY_MODEL/models:/home/ubuntu/go2_target_seek_delivery/KD_MODEL/models:$HOME/.gazebo/models"

# Ensure gazebo uses GPU
export GAZEBO_RENDERING_ENGINE=Ogre2

echo "GPU rendering environment configured for NVIDIA A800"
echo "Gazebo will use GPU acceleration"
