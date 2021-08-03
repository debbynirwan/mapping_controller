"""Random Bounce
Description:
    This ros2 node once commanded to start will control the robot
    to bounce until commanded to stop.
License:
    Copyright 2021 Debby Nirwan
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""
import rclpy
import threading
import random
import copy
import yaml
import os

from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import degrees, radians
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

MILLISECONDS = 0.001
ONE_SECOND_PERIOD = 5
ROBOT_DIAMETER = 0.07
ROBOT_RADIUS = ROBOT_DIAMETER / 2.0

TOF_MAX_READING = 2.000
TOF_MIN_READING = 0.005

IR_MAX_READING = 0.070
IR_MIN_READING = 0.005


class RandomBounce(Node):

    def __init__(self):
        super().__init__('random_bounce')
        self._velocity_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan',
                                 self.laser_scan_callback, 1)
        self.srv = self.create_service(SetBool, '/start_stop_bouncing',
                                       self.start_stop)
        self.timer = self.create_timer(200 * MILLISECONDS, self.timer_callback)
        self.velocity_lock = threading.Lock()
        self.velocity = Twist()
        self.prev_velocity = Twist()
        self.time = 0
        self.turn_right = True
        self.change_turn = False
        self.enabled = False
        self.speed_zero = Vector3(x=0.0, y=0.0, z=0.0)
        self.twist_zero = Twist(linear=self.speed_zero,
                                angular=self.speed_zero)
        self.stopped = False
        self.config = None
        package_dir = get_package_share_directory('mapping_controller')
        cfg_file = 'resource/random_bounce.yaml'
        with open(os.path.join(package_dir, cfg_file)) as cfg_file:
            self.config = yaml.load(cfg_file, Loader=yaml.FullLoader)

    def start_stop(self, request, response):
        self.velocity_lock.acquire()
        self.enabled = request.data
        self.velocity_lock.release()
        response.success = True

        return response

    def timer_callback(self):
        self.velocity_lock.acquire()
        velocity = self.twist_zero
        if self.enabled:
            velocity = copy.deepcopy(self.velocity)
        else:
            self.time = 0
        self.velocity_lock.release()

        if (velocity != self.prev_velocity):
            self._velocity_cmd.publish(velocity)
            self.prev_velocity = copy.deepcopy(velocity)
            if velocity == self.twist_zero:
                self.stopped = True
            else:
                self.stopped = False

        if self.time != 0:
            self.time -= 1

    def laser_scan_callback(self, msg):
        sensor_angle = msg.angle_min
        for laser_range in msg.ranges:
            tof = (round(degrees(sensor_angle)) == 0)
            if tof:
                max = TOF_MAX_READING + ROBOT_RADIUS
                min = TOF_MIN_READING + ROBOT_RADIUS
            else:
                max = IR_MAX_READING + ROBOT_RADIUS
                min = IR_MIN_READING + ROBOT_RADIUS
            if laser_range <= max and laser_range >= min:
                distance = abs(laser_range - ROBOT_RADIUS)
                if (distance <= self.config['object_distance']) and \
                    (abs(round(degrees(sensor_angle))) <=
                     self.config['object_angle']):
                    if self.time == 0:
                        self.velocity_lock.acquire()
                        if not self.stopped:
                            self.velocity = copy.deepcopy(self.twist_zero)
                        else:
                            self.velocity.linear = Vector3(x=0.0, y=0.0, z=0.0)
                            angular_spd = radians(self.config['angular_speed'])
                            self.velocity.angular = Vector3(x=0.0,
                                                            y=0.0,
                                                            z=angular_spd)
                            if not self.turn_right:
                                self.velocity.angular = Vector3(x=0.0,
                                                                y=0.0,
                                                                z=-angular_spd)
                            self.time = self.config['rotation_in_second'] * \
                                ONE_SECOND_PERIOD
                            self.change_turn = random.choice([True, False])
                        self.velocity_lock.release()
                    return
            sensor_angle += msg.angle_increment

        # not blocked
        if self.time == 0:
            self.velocity_lock.acquire()
            self.velocity.linear = Vector3(x=self.config['linear_speed'],
                                           y=0.0, z=0.0)
            self.velocity.angular = Vector3(x=0.0, y=0.0, z=0.0)
            self.velocity_lock.release()
            if self.change_turn:
                self.change_turn = False
                self.turn_right = random.choice([True, False])


def main(args=None):
    rclpy.init(args=args)
    random_bounce = RandomBounce()
    rclpy.spin(random_bounce)
    random_bounce.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
