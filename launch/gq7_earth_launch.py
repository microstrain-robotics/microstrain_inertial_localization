import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_localization'

_GQ7_LAUNCH_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'launch', 'gq7_launch.py')

_GQ7_EARTH_PARAMS_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'config', 'earth', 'gq7.yml')

def generate_launch_description():

  launch_description = []

  gq7_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(_GQ7_LAUNCH_FILE),
    launch_arguments={
      'params_file': _GQ7_EARTH_PARAMS_FILE
    }.items()
  )
  launch_description.append(gq7_launch)

  return LaunchDescription(launch_description)
