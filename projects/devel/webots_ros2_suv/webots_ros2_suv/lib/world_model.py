import rclpy
import os
import cv2
import pathlib
import yaml
import numpy as np
import math
from ament_index_python.packages import get_package_share_directory
from .car_model import CarModel
from .coords_transformer import CoordsTransformer
from ackermann_msgs.msg import AckermannDrive
from webots_ros2_suv.lib.map_utils import is_point_in_polygon, calc_dist_point
from typing import List

class WorldModel(object):
    '''
    Класс, моделирующий параметры внешней среды в привязке к глобальной карте.
    '''
    def __init__(self):
        self.__car_model = CarModel()
        self.coords_transformer = CoordsTransformer()
        
        self.path = None                    # спланированный путь в координатах BEV
        self.gps_path = None                # спланированный путь в глобальных координатах
        self.rgb_image = None               # цветное изображение с камеры
        self.range_image = None             # изображение с камеры глубины
        self.point_cloud = None             # облако точек от лидара
        self.command_message = AckermannDrive() # Сообщение типа AckermanDrive для движения автомобиля
    
    def load_map(self, mapyaml):
        self.global_map = []
        for f in mapyaml['features']:
            self.global_map.append({
                'name': f['properties']['id'].replace('_point', ''),
                'type': f['geometry']['type'],
                'coordinates': f['geometry']['coordinates'],
                'seg_num': f['properties'].get('seg_num', 0)
            })

    def get_current_position(self):
        return self.__car_model.get_position()

    def fill_params(self):
        pos = self.__car_model.get_position()
        self.params["cur_point"] = self.cur_path_point
        self.params["cur_path_segment"] = self.cur_path_segment
        self.params['lat'] = pos[0]
        self.params['lon'] = pos[1]
        self.params['angle'] = pos[2]

    def draw_scene(self, log=print):
        pass

    def get_speed(self):
        return self.__car_model.get_speed()
    
    def set_speed(self, speed):
        self.__car_model.set_speed(speed)

    def update_car_pos(self, lat, lon, orientation):        
        self.__car_model.update(lat=lat, lon=lon, orientation=orientation)

    def get_current_zones(self):
        lat, lon, o = self.get_current_position()
        zones = []
        for p in self.global_map:
            if p['type'] == 'Polygon':
                if is_point_in_polygon(lat, lon, p['coordinates'][0]): # and self.cur_path_point > 2:
                    zones.append(p)
        return zones
