"""launch_testing for mcl_3dl beam likelihood test (pure gtest, no node needed)."""

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
    gtest_node = Node(
        package='mcl_3dl',
        executable='ros2_test_beam_likelihood',
        name='ros2_test_beam_likelihood',
        output='screen',
    )

    return LaunchDescription([
        gtest_node,
        ReadyToTest(),
    ]), {'gtest_node': gtest_node}


class TestBeamLikelihood(unittest.TestCase):
    def test_gtest_pass(
        self, proc_info: ActiveProcInfoHandler, gtest_node: Node
    ) -> None:
        proc_info.assertWaitForShutdown(process=gtest_node, timeout=60.0)
        self.assertEqual(proc_info[gtest_node].returncode, 0)
