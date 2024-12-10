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
    DeclareLaunchArgument('ntrip_host',       default_value='20.185.11.35', description='The host name or IP of the NTRIP caster you want to connect to'),
    DeclareLaunchArgument('ntrip_port',       default_value='2101',         description='The port of the NTRIP caster you want to connect to'),
    DeclareLaunchArgument('ntrip_mountpoint', default_value='VRS_RTCM3',    description='The mountpoint on the NTRIP caster you want to connec to'),
    DeclareLaunchArgument('ntrip_username',   default_value='user',         description='Username to use to authenticate with the NTRIP caster'),
    DeclareLaunchArgument('ntrip_password',   default_value='pass',         description='Password to use to authenticate with the NTRIP caster'),
    DeclareLaunchArgument('ntrip_ssl',        default_value='false',        description='Whether or not to connect using SSL to the NTRIP caster'),

    # Allow the includer to specify whatever parameter file they want
    DeclareLaunchArgument('params_file', default_value=_EMPTY_PARAMS_FILE, description='Path to file that contains user defined parameters'),
  ]
  launch_description.extend(arguments)

  ntrip_node = Node(
    package    = 'ntrip_client',
    executable = 'ntrip_ros.py',
    name       = 'ntrip_client',
    namespace  = '',
    output     = 'screen',
    remappings = [
      # If using an F9P, we need to receive fix messages instead of nmea, so subscribe to the same topic as the CV7-INS
      ('fix', 'ext/llh_position'),
    ],
    parameters = [
      ParameterFile(LaunchConfiguration('params_file'), allow_substs=True),
    ]
  )
  launch_description.append(ntrip_node)

  return LaunchDescription(launch_description)
  

 
 
