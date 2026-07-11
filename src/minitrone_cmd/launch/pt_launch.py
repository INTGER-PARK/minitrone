from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    
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

    position_cmd = Node(
        package="minitrone_cmd",
        executable="minitrone_position_cmd",
        name="minitrone_position_cmd",
        output="screen"
    )

    start_controllers_after_plant = RegisterEventHandler(
        OnProcessStart(
            target_action=plant,
            on_start=[wrench_controller, allocator_controller, ekf_state_estimator]
        )
    )

    start_cmd_after_allocator = RegisterEventHandler(
        OnProcessStart(
            target_action=allocator_controller,
            on_start=[position_cmd]
        )
    )

    return LaunchDescription([
        plant,
        start_controllers_after_plant,
        start_cmd_after_allocator
    ])
