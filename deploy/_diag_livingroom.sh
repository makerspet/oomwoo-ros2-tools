#!/usr/bin/env bash
# Throwaway diagnostic: does the stock living_room render its furniture meshes
# for the headless GPU-LiDAR? Spawns the robot just north of the Sofa (.obj mesh)
# with the south wall (a box) behind it, samples one /scan, reports per-sector
# min range. If the sofa mesh renders, SOUTH ~= 0.5 m; if phantom, SOUTH ~= 2 m
# (beam passes through to the wall). Walls returning at all = boxes render (the
# positive control). Also greps the gz log for mesh-load failures.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_LOCALHOST_ONLY=1

WORLD=${WORLD:-$(ros2 pkg prefix kaiaai_gazebo)/share/kaiaai_gazebo/worlds/living_room.world}
X=${X:-0.394}; Y=${Y:--0.3}; YAW=${YAW:-0.0}
LAUNCH=$HOME/oomwoo-dev/deploy/diag_livingroom.launch.py
GZLOG=/tmp/gz_diag.log

pkill -KILL -f "gz sim" 2>/dev/null || true
pkill -KILL -f "ros2 launch" 2>/dev/null || true
pkill -KILL -f parameter_bridge 2>/dev/null || true
sleep 2

echo "[diag] world=$WORLD  spawn=($X,$Y,yaw=$YAW)"
ros2 launch "$LAUNCH" world:="$WORLD" x_pose:="$X" y_pose:="$Y" yaw:="$YAW" > "$GZLOG" 2>&1 &
LP=$!
trap 'kill -INT $LP 2>/dev/null; sleep 2; pkill -KILL -f "gz sim" 2>/dev/null; pkill -KILL -f "ros2 launch" 2>/dev/null || true' EXIT

# render scene under llvmpipe loads every furniture mesh -> slow; give it time
sleep 35

python3 - <<'PY'
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class S(Node):
    def __init__(self):
        super().__init__('scan_diag')
        self.msg=None
        self.create_subscription(LaserScan,'scan',self.cb,qos_profile_sensor_data)
    def cb(self,m): self.msg=m

rclpy.init(); n=S()
for _ in range(300):
    rclpy.spin_once(n,timeout_sec=0.1)
    if n.msg: break
m=n.msg
if m is None:
    print('[diag] NO /scan received'); rclpy.shutdown(); raise SystemExit(0)
r=list(m.ranges); N=len(r)
fin=[x for x in r if math.isfinite(x) and x>0]
print(f'[diag] beams={N} finite={len(fin)} ({100*len(fin)/N:.0f}%) '
      f'min={min(fin):.2f} max={max(fin):.2f}')
def sect(name, ctr, half=math.radians(20)):
    vals=[x for i,x in enumerate(r)
          if math.isfinite(x) and x>0
          and abs(math.atan2(math.sin(m.angle_min+i*m.angle_increment-ctr),
                             math.cos(m.angle_min+i*m.angle_increment-ctr)))<=half]
    v=min(vals) if vals else float('nan')
    print(f'  {name:>6} ({math.degrees(ctr):+4.0f} deg): min={v:.2f}  n={len(vals)}')
print('[diag] nearest obstacle per direction (robot frame, 0deg=+x fwd):')
sect('SOUTH', -math.pi/2)   # toward Sofa .obj mesh  (visible ~0.5, phantom ~2.1)
sect('EAST',  0.0)
sect('NORTH', math.pi/2)    # toward TableMarble/TVStand/wall
sect('WEST',  math.pi)      # toward MiniSofa .obj mesh / wall
rclpy.shutdown()
PY

echo
echo "===== gz mesh-load lines (deduped, color-stripped) ====="
sed -E "s/\x1b\[[0-9;]*m//g" "$GZLOG" 2>/dev/null \
  | grep -iE "OBJLoader|Collada|MeshManager|Unable to find|could not|failed to load|Ogre.*Exception" \
  | sort | uniq -c | sort -rn | head -40
echo "===== end ====="
