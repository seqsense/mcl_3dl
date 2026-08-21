"""
launch_testing for mcl_3dl bag-replay localization accuracy test.

This is the ROS 2 port of localization_rostest.test.in. It plays back the
short_test3 bag at 2x speed and asserts that compare_pose succeeds (each
amcl_pose is within error_limit=0.3 m and 3-sigma of the reference).
The whole thing is gated by the MCL_3DL_EXTRA_TESTS env var via the
CMakeLists.txt; if you can run this launch.py directly, it means the
gating already let it through.
"""

import os
from typing import Tuple
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.markers
from launch_testing.proc_info_handler import ActiveProcInfoHandler
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description() -> Tuple[LaunchDescription, dict[str, LaunchDescriptionEntity]]:
    pkg_share = get_package_share_directory('mcl_3dl')
    test_data_dir = os.environ.get(
        'MCL_3DL_TEST_DATA_DIR', os.path.join(pkg_share, 'test_data'))
    bag_dir = os.path.join(test_data_dir, 'short_test3')
    ref_file = os.path.join(test_data_dir, 'short_test_ref.topic')

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
        executable='ros2_compare_pose',
        name='compare_pose',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'error_limit': 0.3,
            'ref_path_file': ref_file,
        }],
    )

    return LaunchDescription([
        bag_play,
        mcl_3dl_node,
        gtest_node,
        ReadyToTest(),
    ]), {'gtest_node': gtest_node}


class TestLocalization(unittest.TestCase):
    def test_gtest_pass(
        self, proc_info: ActiveProcInfoHandler, gtest_node: Node,
    ) -> None:
        proc_info.assertWaitForShutdown(process=gtest_node, timeout=400.0)
        self.assertEqual(proc_info[gtest_node].returncode, 0)
