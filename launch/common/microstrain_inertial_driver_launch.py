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
    # These will be passed directly to the params file
    DeclareLaunchArgument('port',                                   default_value='/dev/microstrain_main', description='Location of the inertial sensor serial device'),
    DeclareLaunchArgument('baudrate',                               default_value='115200',                description='Baudrate to open the inertial sensor port at'),
    DeclareLaunchArgument('frame_id',                               default_value='imu_link',              description='Represents the inertial sensor in the tf tree'),
    DeclareLaunchArgument('map_frame_id',                           default_value='map',                   description='Represents the local tangent plane in the tf tree'),
    DeclareLaunchArgument('gnss1_frame_id',                         default_value='gnss_1_link',           description='Represents GNSS antenna 1 in the tf tree'),
    DeclareLaunchArgument('gnss2_frame_id',                         default_value='gnss_2_link',           description='Represents GNSS antenna 2 in the tf tree'),
    DeclareLaunchArgument('target_frame_id',                        default_value='base_link',             description='The frame that we will publish the transform to from map_frame_id'),
    DeclareLaunchArgument('aux_port',                               default_value='/dev/microstrain_aux',  description='Location of the inertial sensor aux serial device'),
    DeclareLaunchArgument('aux_baudrate',                           default_value='115200',                description='Baudrate to open the inertial sensor aux port at'),
    DeclareLaunchArgument('filter_relative_position_source',        default_value='2',                     description='Determines how we will initialize relative position'),
    DeclareLaunchArgument('filter_auto_heading_alignment_selector', default_value='1',                     description='Determines how we will initialize heading'),

    # Allow the includer to specify whatever parameter file they want
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
  

 
 
