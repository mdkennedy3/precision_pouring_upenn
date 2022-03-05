# This file stores the location of the data directory
import platform
import rospkg




data_location = '/home/kuka_demo/ws_kuka/src/'

if platform.system() == 'Linux':
    rospack = rospkg.RosPack()
    data_location = rospack.get_path('pouring_unknown_geom') +'/'