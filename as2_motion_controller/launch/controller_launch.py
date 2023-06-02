""" Launch file for the controller manager node """

import os
import sys
import logging
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from xml.etree import ElementTree

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_available_plugins(package_name: str) -> list[str]:
    """
    Parse plugins.xml file from package and return a list of plugins from a specific type
    """
    plugins_file = os.path.join(
        get_package_share_directory(package_name),
        'plugins.xml'
    )
    root = ElementTree.parse(plugins_file).getroot()

    available_plugins = []
    for class_element in root.findall('class'):
        available_plugins.append(
            class_element.attrib['type'].split('::')[0])
    return available_plugins


def get_controller_manager_node(context):
    """ Returns the controller manager node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    parameters = [{
        'plugin_name': plugin_name,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'plugin_available_modes_config_file': LaunchConfiguration(
            'plugin_available_modes_config_file')
    }]

    plugin_config_file = LaunchConfiguration(
        'plugin_config_file').perform(context)

    if not plugin_config_file:
        print("Finding default config file for plugin: " + plugin_name)
        plugin_config_file = PathJoinSubstitution([
            FindPackageShare('as2_motion_controller'),
            'plugins/' + plugin_name + '/config', 'controller_default.yaml'
        ])
        print("Found default config file: plugins/" +
              plugin_name + "/config/controller_default.yaml")

    parameters.append(plugin_config_file)

    controller_config_file = LaunchConfiguration(
        'motion_controller_config_file').perform(context)

    if not controller_config_file:
        controller_config_file = PathJoinSubstitution([
            FindPackageShare('as2_motion_controller'),
            'config', 'motion_controller_default.yaml'
        ])

    parameters.append(controller_config_file)


    node = Node(
        package='as2_motion_controller',
        executable='as2_motion_controller_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=parameters,
        output='screen',
        emulate_tty=True
    )

    return [node]


def generate_launch_description():
    """ Returns the launch description """
    launch_description = LaunchDescription([
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument(
            'motion_controller_config_file', default_value=''),
        DeclareLaunchArgument(
            'plugin_name', choices=get_available_plugins('as2_motion_controller')),
        DeclareLaunchArgument('plugin_config_file', default_value=''),
        DeclareLaunchArgument(
            'plugin_available_modes_config_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=get_controller_manager_node)
    ])

    return launch_description
