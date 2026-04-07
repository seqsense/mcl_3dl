"""launch_testing for mcl_3dl debug output test."""

import unittest
from typing import Tuple

import launch_testing
import launch_testing.markers
import pytest
from launch import LaunchDescription
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_testing.proc_info_handler import ActiveProcInfoHandler


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description() -> Tuple[LaunchDescription, dict[str, LaunchDescriptionEntity]]:
    mcl_3dl_node = Node(
        package='mcl_3dl',
        executable='mcl_3dl_exec',
        name='mcl_3dl',
        output='screen',
        parameters=[{
            'likelihood.match_dist_min': 0.3,
            'likelihood.match_dist_flat': 0.05,
            'likelihood.clip_near': 0.0,
            'beam.clip_near': 0.0,
            'lpf_step': 0.0,
            'dist_weight_z': 1.0,
            'init_var_x': 0.05,
            'init_var_y': 0.05,
            'init_var_z': 0.05,
            'init_var_roll': 0.0,
            'init_var_pitch': 0.0,
            'init_var_yaw': 0.05,
        }],
    )

    stf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf_base_link',
        arguments=['--x', '1', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'odom', '--child-frame-id', 'base_link'],
    )

    gtest_node = Node(
        package='mcl_3dl',
        executable='ros2_test_debug_output',
        name='ros2_test_debug_output',
        output='screen',
    )

    return LaunchDescription([
        mcl_3dl_node,
        stf_base_link,
        gtest_node,
        ReadyToTest(),
    ]), {'gtest_node': gtest_node}


class TestDebugOutput(unittest.TestCase):
    def test_gtest_pass(
        self, proc_info: ActiveProcInfoHandler, gtest_node: Node
    ) -> None:
        proc_info.assertWaitForShutdown(process=gtest_node, timeout=60.0)
        self.assertEqual(proc_info[gtest_node].returncode, 0)
