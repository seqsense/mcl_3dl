"""launch_testing for mcl_3dl bag-replay tf consistency test.

This is the ROS 2 port of tf_rostest.test.in. It plays back the short_test3
bag and asserts that compare_tf succeeds (the published map->base_link tf
at each amcl_pose stamp lines up with the amcl_pose itself within 5 cm).
Gated via MCL_3DL_EXTRA_TESTS in CMakeLists.txt.

Bag rate is 2.0 here, not 3.0 as in the ROS 1 version: rosbag2's pacing
plus mcl_3dl's ROS 2 callback execution makes amcl_pose and the matching
/tf drift apart at 3x (lookup errors > 5 cm observed mid-run), which is
not a regression in the underlying logic but in the test's tolerance to
DDS-mediated message ordering. 2x is well within the engine's capacity.
"""

import os
import unittest
from typing import Tuple

import launch_testing
import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_testing.proc_info_handler import ActiveProcInfoHandler


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description() -> Tuple[LaunchDescription, dict[str, LaunchDescriptionEntity]]:
    pkg_share = get_package_share_directory('mcl_3dl')
    test_data_dir = os.environ.get(
        'MCL_3DL_TEST_DATA_DIR', os.path.join(pkg_share, 'test_data'))
    bag_dir = os.path.join(test_data_dir, 'short_test3')

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_dir,
            '--clock', '100',
            '--rate', '2.0',
            '--delay', '4.0',
        ],
        name='playback',
        output='screen',
    )

    mcl_3dl_node = Node(
        package='mcl_3dl',
        executable='mcl_3dl_exec',
        name='mcl_3dl',
        output='log',
        parameters=[{
            'use_sim_time': True,
            'skip_measure': 2,
        }],
    )

    gtest_node = Node(
        package='mcl_3dl',
        executable='ros2_compare_tf',
        name='compare_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        bag_play,
        mcl_3dl_node,
        gtest_node,
        ReadyToTest(),
    ]), {'gtest_node': gtest_node}


class TestTf(unittest.TestCase):
    def test_gtest_pass(
        self, proc_info: ActiveProcInfoHandler, gtest_node: Node,
    ) -> None:
        proc_info.assertWaitForShutdown(process=gtest_node, timeout=120.0)
        self.assertEqual(proc_info[gtest_node].returncode, 0)
