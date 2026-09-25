from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from datetime import datetime
from pathlib import Path

def generate_launch_description():
    workspace_dir = Path.home() / "ros2_project" / "minitrone_ws"
    bag_dir = workspace_dir / "bags"
    bag_dir.mkdir(parents=True, exist_ok=True)
    bag_name = str(bag_dir / f"bag_all_{datetime.now().strftime('%Y%m%d_%H%M%S')}")

    bag_record = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
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
        parameters=[PathJoinSubstitution([
            FindPackageShare("minitrone_plant"), "config", "high_fidelity.yaml"
        ])],
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
    
    second_mob = Node(
        package="minitrone_controller",
        executable="minitrone_second_wrench_observer",
        name="minitrone_second_wrench_observer",
        output="screen"
    )

    start_controllers_after_plant = RegisterEventHandler(
        OnProcessStart(
            target_action=plant,
            # Normal: cmd/att_cmd -> wrench controller -> allocator.
            # Optional admittance override uses second-order MoB output and is
            # toggled from minitrone_admittance_controller in a terminal.
            on_start=[
                second_mob,
                wrench_controller,
                allocator_controller,
            ]
        )
    )

    return LaunchDescription([
        bag_record,
        plant,
        start_controllers_after_plant
    ])
