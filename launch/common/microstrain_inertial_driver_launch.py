import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions.node import ParameterFile

from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_localization'

_EMPTY_PARAMS_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'config', 'common', 'empty.yml')

def generate_launch_description():

  launch_description = []

  arguments = [
    # TODO: Put some optional arguments in here

    DeclareLaunchArgument('params_file', default_value=_EMPTY_PARAMS_FILE, description='Path to file that contains parameters for this use-case'),
  ]
  launch_description.extend(arguments)

  # Manually configure the node instead of using the launch file so we can use the arguments
  microstrain_node = Node(
    package    = "microstrain_inertial_driver",
    executable = "microstrain_inertial_driver_node",
    name       = "microstrain_inertial_driver",
    namespace  = '',
    output     = 'screen',
    parameters = [
      ParameterFile(LaunchConfiguration('params_file'), allow_substs=True),
    ]
  )
  launch_description.append(microstrain_node)

  return LaunchDescription(launch_description)
  

 
 
