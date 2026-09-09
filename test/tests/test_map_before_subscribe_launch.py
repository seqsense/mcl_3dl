"""launch_testing for mcl_3dl receiving a map published before it subscribed."""

from typing import Tuple
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
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
    mcl_3dl_node = Node(
        package='mcl_3dl',
        executable='mcl_3dl_exec',
        name='mcl_3dl',
        output='screen',
        parameters=[{'map_update_interval_interval': 0.5}],
    )

    gtest_node = Node(
        package='mcl_3dl',
        executable='ros2_test_map_before_subscribe',
        name='ros2_test_map_before_subscribe',
        output='screen',
    )

    return LaunchDescription([
        # mcl_3dl starts after the gtest has published the map, so that the
        # map is always published before mcl_3dl subscribes.
        TimerAction(period=3.0, actions=[mcl_3dl_node]),
        gtest_node,
        ReadyToTest(),
    ]), {'gtest_node': gtest_node}


class TestMapPublishedBeforeSubscribe(unittest.TestCase):
    def test_gtest_pass(
        self, proc_info: ActiveProcInfoHandler, gtest_node: Node
    ) -> None:
        proc_info.assertWaitForShutdown(process=gtest_node, timeout=60.0)
        self.assertEqual(proc_info[gtest_node].returncode, 0)
