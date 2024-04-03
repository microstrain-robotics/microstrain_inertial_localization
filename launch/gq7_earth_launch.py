import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions.node import ParameterFile

from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_localization'

_GQ7_PARAMS_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'config', 'gq7_earth', 'gq7.yml')
_NTRIP_CLIENT_PARAMS_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'config', 'common', 'ntrip_client.yml')

_EMPTY_PARAMS_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'config', 'common', 'gq7_empty.yml')

def generate_launch_description():

  # Append to the launch description depending on arguments
  launch_description = []

  # These arguments will be used by the params files
  arguments = [
    DeclareLaunchArgument('port',         default_value='/dev/microstrain_main', description='Serial port that the GQ7 is connected on'),
    DeclareLaunchArgument('baudrate',     default_value='115200',                description='Baudrate of the serial port that the GQ7 is connected on'),
    DeclareLaunchArgument('aux_port',     default_value='/dev/microstrain_aux',  description='Serial port that the GQ7 aux port is connected on. Only used if ntrip is true'),
    DeclareLaunchArgument('aux_baudrate', default_value='115200',                description='Baudrate of the serial port that the GQ7 aux port is connected on. Only used if ntrip is true'),

    DeclareLaunchArgument('frame_id',            default_value='gq7_link',            description='The frame ID that will represent the GQ7 in the tf tree'),
    DeclareLaunchArgument('map_frame_id',        default_value='map',                 description='The frame ID of the local tangent plane'),
    DeclareLaunchArgument('base_link_frame_id',  default_value='base_link',           description='The frame ID that you want to globally localize.'),
    DeclareLaunchArgument('gnss1_frame_id',      default_value='gnss_1_antenna_link', description='The frame ID that will represent GNSS antenna 1 in the tf tree'),
    DeclareLaunchArgument('gnss2_frame_id',      default_value='gnss_2_antenna_link', description='The frame ID that will represent GNSS antenna 2 in the tf tree'),

    DeclareLaunchArgument('gnss1_antenna_offset_source', default_value='1',                description='GNSS1 antenna offset source selector'),
    DeclareLaunchArgument('gnss1_antenna_offset',        default_value='[0.0, 0.7, 0.0]',  description='GNSS1 antenna offset. Only used if gnss1_antenna_offset_source is set to 1'),
    DeclareLaunchArgument('gnss2_antenna_offset_source', default_value='1',                description='GNSS1 antenna offset source selector'),
    DeclareLaunchArgument('gnss2_antenna_offset',        default_value='[0.0, -0.7, 0.0]', description='GNSS1 antenna offset. Only used if gnss1_antenna_offset_source is set to 1'),

    DeclareLaunchArgument('filter_auto_heading_alignment_selector', default_value='1', description='Filter initialization auto-heading alignment selector'),

    DeclareLaunchArgument('ntrip',            default_value='false',        description='Whether or not to enable the NTRIP interface on the microstrain driver and run an NTRIP client'),
    DeclareLaunchArgument('ntrip_host',       default_value='20.185.11.35', description='The host name or IP of the NTRIP caster you want to connect to'),
    DeclareLaunchArgument('ntrip_port',       default_value='2101',         description='The port of the NTRIP caster you want to connect to'),
    DeclareLaunchArgument('ntrip_mountpoint', default_value='VRS_RTCM3',    description='The mountpoint on the NTRIP caster you want to connec to'),
    DeclareLaunchArgument('ntrip_username',   default_value='user',         description='Username to use to authenticate with the NTRIP caster'),
    DeclareLaunchArgument('ntrip_password',   default_value='pass',         description='Password to use to authenticate with the NTRIP caster'),
    DeclareLaunchArgument('ntrip_ssl',        default_value='false',        description='Whether or not to connect using SSL to the NTRIP caster'),

    DeclareLaunchArgument('params_file', default_value=_EMPTY_PARAMS_FILE, description='Path to file that contains additional parameters not exposed here'),
  ]
  launch_description.extend(arguments)

  # Manually configure the node instead of using the launch file so we can use the arguments
  microstrain_node = Node(
    package    = "microstrain_inertial_driver",
    executable = "microstrain_inertial_driver_node",
    name       = "gq7_driver",
    namespace  = "/gq7",
    output     = 'screen',
    parameters = [
      # Load the parameters for our use case
      ParameterFile(_GQ7_PARAMS_FILE, allow_substs=True),

      # Load a user defined parameter file
      ParameterFile(LaunchConfiguration('params_file'), allow_substs=True),
    ]
  )
  launch_description.append(microstrain_node)

  # If ntrip was requested, launch that as well
  ntrip_node = Node(
    package    = 'ntrip_client',
    executable = 'ntrip_ros.py',
    name       = 'gq7_ntrip_client',
    namespace  = '/gq7',
    output     = 'screen',
    condition  = LaunchConfigurationEquals('ntrip', 'true'),
    parameters = [
      # Load the parameters for our use case
      ParameterFile(_NTRIP_CLIENT_PARAMS_FILE, allow_substs=True),

      # Load a user defined parameter file
      ParameterFile(LaunchConfiguration('params_file'), allow_substs=True),
    ]
  )
  launch_description.append(ntrip_node)

  # Create the LaunchDescription
  return LaunchDescription(launch_description)
  

 
 
