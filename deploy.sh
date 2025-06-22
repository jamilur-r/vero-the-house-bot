#!/bin/bash

# === CONFIG ===
PI_USER="vero"
PI_HOST="vero.local"
REMOTE_WS_DIR="/home/vero/projects/vero_ws"

echo "📦 Deploying ROS 2 workspace to ${PI_USER}@${PI_HOST}..."

# Step 1: Create workspace directory on Pi (if it doesn't exist)
echo "📁 Ensuring workspace directory exists on Pi..."
ssh ${PI_USER}@${PI_HOST} "mkdir -p ${REMOTE_WS_DIR}/src"

# Step 2: Sync source folder
echo "🔄 Syncing source files..."
rsync -avz --delete ~/projects/vero_ws/src/ ${PI_USER}@${PI_HOST}:${REMOTE_WS_DIR}/src/

# Step 3: SSH into Pi and build
echo "🛠 Building on Pi..."
ssh ${PI_USER}@${PI_HOST} << EOF
  set -e
  source /opt/ros/humble/setup.bash
  cd ${REMOTE_WS_DIR}
  colcon build
  echo "✅ Build complete on Pi."
EOF

echo "🚀 Deployment done. You can now run nodes via SSH or systemd."
