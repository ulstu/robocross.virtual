import rclpy
import numpy as np
import traceback
import cv2
import os
import math
import time
import yaml
import threading
import matplotlib.pyplot as plt
import sensor_msgs.msg
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import PointStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import controller_url_prefix
from PIL import Image
from .lib.timeit import timeit
from .lib.orientation import euler_from_quaternion
from .lib.map_server import start_web_server, MapWebServer
from .lib.world_model import WorldModel
from robot_interfaces.srv import PoseService
from robot_interfaces.msg import EgoPose

SENSOR_DEPTH = 40

class NodeEgoController(Node):
    def __init__(self):
        try:
            super().__init__('node_ego_controller')
            self._logger.info(f'Node Ego Started')
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE

            self.__world_model = WorldModel()
            self.__ws = None
            
            package_dir = get_package_share_directory("webots_ros2_suv")

            self.create_subscription(Odometry, '/odom', self.__on_odom_message, qos)
            self.create_subscription(sensor_msgs.msg.Image, '/vehicle/camera/image_color', self.__on_image_message, qos)
            self.create_subscription(sensor_msgs.msg.PointCloud2, "/lidar", self.__on_lidar_message, qos)

            self.__ackermann_publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 1)
            
            self.start_web_server()

        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def start_web_server(self):
        self.__ws = MapWebServer(log=self._logger.info)
        threading.Thread(target=start_web_server, args=[self.__ws]).start()

    def __on_lidar_message(self, data):
        pass

    def __on_range_image_message(self, data):
        image = np.frombuffer(data.data, dtype="float32").reshape((data.height, data.width, 1))
        image[image == np.inf] = SENSOR_DEPTH
        image[image == -np.inf] = 0
        
        range_image = image / SENSOR_DEPTH

    def drive(self):
        self.__world_model.command_message.speed = 20.0
        self.__world_model.command_message.steering_angle = 0.0

        self.__ackermann_publisher.publish(self.__world_model.command_message)

    #@timeit
    def __on_image_message(self, data):
        image = data.data
        image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_RGBA2RGB))

        self.__world_model.rgb_image = np.asarray(analyze_image)

        t1 = time.time()
        # TODO: put your code here
        t2 = time.time()
        
        delta = t2 - t1
        fps = 1 / delta if delta > 0 else 100
        self._logger.info(f"Current FPS: {fps}")

        pos = self.__world_model.get_current_position()

        self.drive()

        if self.__ws is not None:
            self.__ws.update_model(self.__world_model)

    def __on_odom_message(self, data):
            roll, pitch, yaw = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
            lat, lon, orientation = self.__world_model.coords_transformer.get_global_coords(data.pose.pose.position.x, data.pose.pose.position.y, yaw)
            self.__world_model.update_car_pos(lat, lon, orientation)
            if self.__ws is not None:
                self.__ws.update_model(self.__world_model)

def main(args=None):
    try:
        rclpy.init(args=args)
        path_controller = NodeEgoController()
        rclpy.spin(path_controller)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
