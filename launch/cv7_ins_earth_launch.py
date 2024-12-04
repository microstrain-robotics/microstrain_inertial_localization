import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_localization'

_CV7_INS_LAUNCH_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'launch', 'cv7_ins_launch.py')

_CV7_INS_EARTH_PARAMS_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'config', 'earth', 'cv7_ins.yml')

def generate_launch_description():

  launch_description = []

  cv7_ins_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(_CV7_INS_LAUNCH_FILE),
    launch_arguments={
      'params_file': _CV7_INS_EARTH_PARAMS_FILE
    }.items()
  )
  launch_description.append(cv7_ins_launch)

  return LaunchDescription(launch_description)
