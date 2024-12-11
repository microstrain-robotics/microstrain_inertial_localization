import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_localization'

_NAMESPACE = 'cv7_ins'

_MICROSTRAIN_INERTIAL_DRIVER_LAUNCH_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'launch', 'common', 'microstrain_inertial_driver_launch.py')
_NTRIP_CLIENT_LAUNCH_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'launch', 'common', 'ntrip_client_launch.py')
_UBLOX_F9P_LAUNCH_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'launch', 'common', 'ublox_f9p_launch.py')

_EMPTY_PARAMS_FILE = os.path.join(get_package_share_directory(_PACKAGE_NAME), 'config', 'common', 'empty.yml')

def generate_launch_description():

  # Append to the launch description depending on arguments
  launch_description = []

  arguments = [
    # Allow the user to enable or disable the different addons
    DeclareLaunchArgument('ntrip',       default_value='false',            description='Whether or not to enable the NTRIP interface on the microstrain driver and run an NTRIP client'),
    DeclareLaunchArgument('ublox_f9p',   default_value='false',            description='Whether or not we want to connect a ublox f9p to the CV7-INS'),

    # Allow the user to specify their own parameter file
    DeclareLaunchArgument('params_file', default_value=_EMPTY_PARAMS_FILE, description='Path to file that contains additional parameters not exposed here'),
  ]
  launch_description.extend(arguments)

  # All topics should be under the same namespace
  launch_description.append(PushRosNamespace(_NAMESPACE))

  # Add the microstrain_inertial_driver launch file
  microstrain_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(_MICROSTRAIN_INERTIAL_DRIVER_LAUNCH_FILE),
    launch_arguments={
      'params_file': LaunchConfiguration('params_file'),
      'filter_auto_heading_alignment_selector': '4',
    }.items()
  )
  launch_description.append(microstrain_launch)

  # If ntrip was requested, launch that as well
  ntrip_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(_NTRIP_CLIENT_LAUNCH_FILE),
    condition=LaunchConfigurationEquals('ntrip', 'true'),
    launch_arguments={
      'params_file': LaunchConfiguration('params_file')
    }.items()
  )
  launch_description.append(ntrip_launch)

  # If an f9p was requested, launch that as well
  ublox_f9p_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(_UBLOX_F9P_LAUNCH_FILE),
    condition=LaunchConfigurationEquals('ublox_f9p', 'true'),
    launch_arguments={
      'params_file': LaunchConfiguration('params_file')
    }.items()
  )
  launch_description.append(ublox_f9p_launch)

  # Create the LaunchDescription
  return LaunchDescription(launch_description)
