"""launch_testing for mcl_3dl global localization test."""

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
            'likelihood.match_dist_min': 0.5,
            'likelihood.match_dist_flat': 0.1,
            'likelihood.clip_near': 0.0,
            'beam.clip_near': 0.0,
            'lpf_step': 0.0,
            'dist_weight_z': 1.0,
            'jump_dist': 1000.0,
            'jump_ang': 1000.0,
            'odom_err_lin_lin': 0.5,
            'odom_err_integ_lin_tc': 120.0,
            'odom_err_integ_lin_sigma': 2.5,
            'odom_err_integ_ang_tc': 120.0,
            'odom_err_integ_ang_sigma': 0.1,
            'global_localization_grid_lin': 0.15,
        }],
    )

    stf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf_base_link',
        arguments=['--x', '0', '--y', '5', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'odom', '--child-frame-id', 'base_link'],
    )

    stf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf_laser',
        arguments=['--x', '0', '--y', '0', '--z', '0.5',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    gtest_node = Node(
        package='mcl_3dl',
        executable='ros2_test_global_localization',
        name='ros2_test_global_localization',
        output='screen',
    )

    return LaunchDescription([
        mcl_3dl_node,
        stf_base_link,
        stf_laser,
        gtest_node,
        ReadyToTest(),
    ]), {'gtest_node': gtest_node}


class TestGlobalLocalization(unittest.TestCase):
    def test_gtest_pass(
        self, proc_info: ActiveProcInfoHandler, gtest_node: Node
    ) -> None:
        proc_info.assertWaitForShutdown(process=gtest_node, timeout=180.0)
        self.assertEqual(proc_info[gtest_node].returncode, 0)
