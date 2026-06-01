from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from datetime import datetime
from pathlib import Path

def generate_launch_description():
    workspace_dir = Path("/home/parkjeongsu/ros2_project/minitrone_ws")
    bag_dir = workspace_dir / "bags"
    bag_dir.mkdir(parents=True, exist_ok=True)
    bag_name = str(bag_dir / f"bag_all_{datetime.now().strftime('%Y%m%d_%H%M%S')}")

    bag_record = ExecuteProcess(
        cmd=[
            "/opt/ros/humble/bin/ros2",
            "bag",
            "record",
            "-a",
            "-o", bag_name
        ],
        output="screen"
    )

    plant = Node(
        package="minitrone_plant",
        executable="minitrone_plant",
        name="minitrone_plant",
        output="screen"
    )

    wrench_controller = Node(
        package="minitrone_controller",
        executable="minitrone_wrench_controller",
        name="minitrone_wrench_controller",
        output="screen"
    )

    allocator_controller = Node(
        package="minitrone_controller",
        executable="minitrone_allocator_controller",
        name="minitrone_allocator_controller",
        output="screen"
    )

    ekf_state_estimator = Node(
        package="minitrone_controller",
        executable="minitrone_ekf_state_estimator",
        name="minitrone_ekf_state_estimator",
        output="screen"
    )

    start_controllers_after_plant = RegisterEventHandler(
        OnProcessStart(
            target_action=plant,
            # MoB/wrench_observer is intentionally not launched here.
            # Start it manually when needed:
            #   ros2 run minitrone_controller minitrone_first_wrench_observer
            # It publishes /minitrone/external_wrench_hat for monitoring only.
            on_start=[wrench_controller, allocator_controller, ekf_state_estimator]
        )
    )

    return LaunchDescription([
        bag_record,
        plant,
        start_controllers_after_plant
    ])
