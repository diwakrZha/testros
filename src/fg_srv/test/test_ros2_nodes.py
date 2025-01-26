# src/fg_srv/test/test_ros2_nodes.py

import pytest
import rclpy
from rclpy.action import ActionClient

from fg_srv.action import Mission


@pytest.fixture(scope="module")
def rclpy_init_shutdown():
    """Initialize and shutdown rclpy exactly once for all tests in this module."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def mock_action_goal():
    """An example action goal to test sending to the server."""
    return Mission.Goal(
        mission_id="123",
        mission_type="Delivery",
        latitude=12.9716,
        longitude=77.5946,
        priority=2
    )

def test_action_server_execution(rclpy_init_shutdown, mock_action_goal):
    """
    Spin up a local server node and client node, 
    send a goal, then check the result is success.
    """
    # E.g. your code is in src/fg_srv/fg_srv/rn2.py and src/fg_srv/fg_srv/rn1.py
    from fg_srv.fg_srv.rn2 import MissionExecutionActionServer
    from fg_srv.fg_srv.rn1 import MissionRetrievalActionClient

    server_node = MissionExecutionActionServer()
    client_node = MissionRetrievalActionClient()

    # Send the goal to the server
    send_goal_future = client_node._action_client.send_goal_async(mock_action_goal)
    rclpy.spin_until_future_complete(client_node, send_goal_future)
    goal_handle = send_goal_future.result()

    assert goal_handle is not None, "Action client did not return a goal handle"
    assert goal_handle.accepted, "Action goal was rejected by the server"

    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(client_node, result_future)
    result_msg = result_future.result().result

    assert result_msg.success, "Mission execution was not successful"
