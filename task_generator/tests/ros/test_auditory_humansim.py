from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _ros_gate():
    pytest.importorskip("rclpy")
    pytest.importorskip("std_msgs.msg")
    pytest.importorskip("std_srvs.srv")


def test_auditory_humansim_registered():
    from task_generator.constants import Constants
    from task_generator.simulators.human import HumanSimulatorRegistry

    assert Constants.HumanSimulator.AUDITORY in HumanSimulatorRegistry