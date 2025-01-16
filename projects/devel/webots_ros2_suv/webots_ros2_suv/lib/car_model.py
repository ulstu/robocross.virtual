import rclpy
from ackermann_msgs.msg import AckermannDrive


class CarModel(object):
    '''
    Класс, моделирующий параметры внешней среды в привязке к локальной карте.
    В случае использования симулятора используются значения из симулятора
    Для реального автомобиля параметры получаются из подсистемы навигации
    '''
    def __init__(self):
        self.__speed = 0
        self.__lat = 0
        self.__lon = 0
        self.__orientation = 0
        self.path = None                    # спланированный путь в координатах BEV
        self.gps_path = None                # спланированный путь в глобальных координатах
        self.rgb_image = None               # цветное изображение с камеры
        self.range_image = None             # изображение с камеры глубины
        self.point_cloud = None             # облако точек от лидара
        self.command_message = AckermannDrive() # Сообщение типа AckermanDrive для движения автомобиля


    def update(self, speed=None, lat=None, lon=None, orientation=None):
        if speed:
            self.__speed = speed
        if lat:
            self.__lat = lat
        if lon:
            self.__lon = lon
        if orientation:
            self.__orientation = orientation

    def get_position(self):
        return self.__lat, self.__lon, self.__orientation
    
    def set_speed(self, speed):
        self.__speed = speed

    def get_speed(self):
        return self.__speed
    
