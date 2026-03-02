#!/usr/bin/env python3
"""Launch Gazebo Classic with the Booster K1 robot for simulation testing."""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

# Resolve paths relative to this file's directory (project root is one level up)
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    urdf_file = os.path.join(_PKG_DIR, 'K1_22dof.urdf')
    world_file = os.path.join(_PKG_DIR, 'worlds', 'simulation.world')
    sdf_tmp        = '/tmp/k1_robot.sdf'
    urdf_resolved  = '/tmp/k1_robot_resolved.urdf'

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Resolve relative mesh paths to absolute file:// URIs so Gazebo can find STL files.
    meshes_file_uri = f'file://{_PKG_DIR}/meshes/'
    robot_description = robot_description.replace('meshes/', meshes_file_uri)

    # Write the resolved URDF to a temp file for gz sdf conversion.
    with open(urdf_resolved, 'w') as f:
        f.write(robot_description)

    # Gazebo server.
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', world_file,
            '-s', 'libgazebo_ros_init.so',
            '--verbose',
        ],
        output='screen',
    )

    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
    )

    # Robot state publisher (broadcasts TFs from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawn the K1 robot using Gazebo's own transport layer.
    # The 6-second delay gives gzserver time to finish loading the world.
    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    f'gz sdf -p {urdf_resolved} > {sdf_tmp} && '
                    f'gz model --spawn-file {sdf_tmp} '
                    f'--model-name k1_robot '
                    f'--pose "0 0 0.9 0 0 0"'
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
    ])
