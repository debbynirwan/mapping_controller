"""Mission Controller
Description:
    This ros2 node commands the robot bounce until timeout and save
    the map to the given path.
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
import numpy
import cv2
import copy
import os

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool
from pathlib import Path

UNOBSERVED = -1
UNOCCUPIED = 0
MINUTE = 60
DEFAULT_MAPPING_TIME = 5
MAX_CONFIDENCE = 100.0
MIN_LIKELIHOOD_THRES = 50
PNG_OCCUPIED = 0.0
PNG_UNOCCUPIED = 255.0
PNG_UNOBSERVED = 128.0


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        self._start_bouncing_srv = self.create_client(SetBool,
                                                      '/start_stop_bouncing')
        self._use_probability = self.declare_parameter('use_probability',
                                                       False)
        self._use_mapper = self.declare_parameter('mapper', False)
        self._mapping_time = self.declare_parameter('mapping_time',
                                                    DEFAULT_MAPPING_TIME)
        default_path = str(Path.home())
        self._path = self.declare_parameter('path', default_path)
        self._bounce = True
        self._map_lock = threading.Lock()
        self._prob_map_lock = threading.Lock()
        self._occ_cache = []
        self._occ_cache_prob = []
        self._occ_cache_width = 0
        self._occ_cache_width_prob = 0

        if self._use_probability.value:
            self.create_subscription(OccupancyGrid, '/prob_map',
                                     self.occupancy_prob_grid_callback, 1)
        if self._use_mapper.value:
            self.create_subscription(OccupancyGrid, '/map',
                                     self.occupancy_grid_callback, 1)
        self.timer = self.create_timer(self._mapping_time.value * MINUTE,
                                       self.timer_callback)

        while not self._start_bouncing_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetBool.Request()
        request.data = self._bounce
        self._bounce = not self._bounce
        self._start_bouncing_srv.call_async(request)

    def timer_callback(self):
        request = SetBool.Request()
        request.data = self._bounce
        self._bounce = not self._bounce
        self._start_bouncing_srv.call_async(request)

        map_to_save_prob = []
        map_to_save_max_likelihood = []
        if self._use_probability.value:
            for occ in self._occ_cache_prob:
                occ_reversed = MAX_CONFIDENCE - occ
                confidence_reversed = int(occ_reversed/MAX_CONFIDENCE
                                          * PNG_UNOCCUPIED)
                map_to_save_prob.append(confidence_reversed)
                if confidence_reversed < MIN_LIKELIHOOD_THRES:
                    map_to_save_max_likelihood.append(PNG_OCCUPIED)
                else:
                    map_to_save_max_likelihood.append(confidence_reversed)

            prob_file = os.path.join(self._path.value, 'prob_map.png')
            self.get_logger().info(f"saving prob map to: {prob_file}")
            cv2.imwrite(prob_file,
                        numpy.reshape(numpy.array(map_to_save_prob),
                                      (-1, self._occ_cache_width_prob)))

            max_likelihood_file = os.path.join(self._path.value,
                                               'prob_map_max_likelihood.png')
            self.get_logger().info(
                f"saving max likelihood map to: {max_likelihood_file}")
            cv2.imwrite(max_likelihood_file,
                        numpy.reshape(numpy.array(map_to_save_max_likelihood),
                                      (-1, self._occ_cache_width_prob)))

        map_to_save = []
        if self._use_mapper.value:
            for occ in self._occ_cache:
                if occ == UNOBSERVED:
                    map_to_save.append(int(PNG_UNOBSERVED))
                elif occ == UNOCCUPIED:
                    map_to_save.append(int(PNG_UNOCCUPIED))
                else:
                    map_to_save.append(int(PNG_OCCUPIED))
            map_file = os.path.join(self._path.value, 'map.png')
            self.get_logger().info(f"saving map to: {map_file}")
            cv2.imwrite(map_file,
                        numpy.reshape(numpy.array(map_to_save),
                                      (-1, self._occ_cache_width)))

        self.timer.cancel()
        self.get_logger().info("mapping completed")

    def occupancy_grid_callback(self, msg):
        self._map_lock.acquire()
        self._occ_cache = copy.deepcopy(msg.data)
        self._occ_cache_width = msg.info.width
        self._map_lock.release()

    def occupancy_prob_grid_callback(self, msg):
        self._prob_map_lock.acquire()
        self._occ_cache_prob = copy.deepcopy(msg.data)
        self._occ_cache_width_prob = msg.info.width
        self._prob_map_lock.release()


def main(args=None):
    rclpy.init(args=args)
    mission_controller = MissionController()
    rclpy.spin(mission_controller)
    mission_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
