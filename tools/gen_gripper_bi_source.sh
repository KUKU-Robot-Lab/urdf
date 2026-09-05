#!/usr/bin/env bash
# Generate generated/source/openarm_gripper_bi.urdf: the stock OpenArm v10 dual-arm
# body with the vendor 2-finger gripper (openarm_hand) on BOTH arms.
#
# The vendor xacro (vendor/openarm_description/urdf/robot/v10.urdf.xacro,
# bimanual:=true ee_type:=openarm_hand) already emits the exact link/joint
# names the RL generator maps (openarm_{side}_hand, {side}_openarm_hand_joint,
# openarm_{side}_finger_joint1/2 ...), so no local assembly xacro is needed.
# Requires ROS 2 humble (xacro) and the colcon install/ of this workspace.
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${WS}/generated/source/openarm_gripper_bi.urdf"
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u
xacro "${WS}/vendor/openarm_description/urdf/robot/v10.urdf.xacro" \
  bimanual:=true ee_type:=openarm_hand hand:=true > "${OUT}.tmp"
# name the source after the assembly (the vendor xacro hard-codes "openarm")
sed -i '0,/<robot name="openarm"/s//<robot name="openarm_gripper_bi"/' "${OUT}.tmp"
mv "${OUT}.tmp" "${OUT}"
echo "generated ${OUT}"
