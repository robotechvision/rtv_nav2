# Copyright (c) 2021 RoboTech Vision
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

def pathCallback(path):
    global pub
    poses = PoseArray()
    poses.header = path.header
    poses.poses = [p.pose for p in path.poses]
    pub.publish(poses)

def main():
    global pub
    # Start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')
    path_qos = QoSProfile(
          durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
    node.create_subscription(Path, "/plan", pathCallback, path_qos)
    pub = node.create_publisher(PoseArray, "/plan_poses", 10)
    rclpy.spin(node)

if __name__ == '__main__':
    main()