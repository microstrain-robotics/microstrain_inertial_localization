import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.actions.node import ParameterFile

from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_localization'

_EMPTY_PARAMS_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'config', 'common', 'empty.yml')

def generate_launch_description():

  launch_description = []

  arguments = [
    # These will be passed directly to the params file
    DeclareLaunchArgument('ublox_f9p_port', default_value='/dev/ttyACM1', description='The port that the ublox F9P is connected to'),

    # Allow the includer to specify whatever parameter file they want
    DeclareLaunchArgument('params_file', default_value=_EMPTY_PARAMS_FILE, description='Path to file that contains user defined parameters'),
  ]
  launch_description.extend(arguments)

  ublox_f9p_node = Node(
    package = 'ublox_gps',
    executable = 'ublox_gps_node',
    name = 'ublox_f9p',
    namespace = '',
    output = 'screen',
    remappings = [
      # Remap the fix and fix velocity topics so they will get sent to the CV7-INS
      ('fix', 'ext/llh_position'),
      ('fix_velocity', 'ext/velocity_enu'),

      # Some newer versions of the driver need to have the name of the node in the topic name
      ('ublox_f9p/fix', 'ext/llh_position'),
      ('ublox_f9p/fix_velocity', 'ext/velocity_enu'),

      # Looks like the RTCM topics are forced into the global namespace. Fix that here
      ('/rtcm', 'rtcm'),
    ],
    parameters = [
      ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)
    ]
  )
  launch_description.append(ublox_f9p_node)

  return LaunchDescription(launch_description)
  

 
 
