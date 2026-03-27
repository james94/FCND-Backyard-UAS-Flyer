from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    connection_uri = LaunchConfiguration("connection_uri")
    is_px4 = LaunchConfiguration("is_px4")
    use_mock_sim = LaunchConfiguration("use_mock_sim")
    launch_session_manager = LaunchConfiguration("launch_session_manager")
    headless_mode = LaunchConfiguration("headless_mode")
    start_external_control_on_boot = LaunchConfiguration("start_external_control_on_boot")
    timeout_sec = LaunchConfiguration("timeout_sec")
    target_altitude_m = LaunchConfiguration("target_altitude_m")
    session_service_timeout_sec = LaunchConfiguration("session_service_timeout_sec")
    record_telemetry = LaunchConfiguration("record_telemetry")
    telemetry_record_mode = LaunchConfiguration("telemetry_record_mode")
    telemetry_output_dir = LaunchConfiguration("telemetry_output_dir")
    telemetry_run_id = LaunchConfiguration("telemetry_run_id")
    log_telemetry_csv = LaunchConfiguration("log_telemetry_csv")
    telemetry_log_csv_path = LaunchConfiguration("telemetry_log_csv_path")

    # If launch_session_manager is disabled, force mock mode to bypass session gating
    effective_use_mock_sim = PythonExpression(
        ["(", use_mock_sim, ") or (not", launch_session_manager, ")"]
    )

    driver_node = Node(
        package="udacidrone_driver_cpp",
        executable="uas_drive_node",
        name="uas_driver_node",
        output="screen",
        parameters=[
            {
                "connection_uri": connection_uri,
                "is_px4": is_px4,
                "use_mock_sim": effective_use_mock_sim,
                "start_external_control_on_boot": start_external_control_on_boot,
                "timeout_sec": timeout_sec,
                "target_altitude_m": target_altitude_m,
                "session_service_timeout_sec": session_service_timeout_sec,
                "log_telemetry_csv": log_telemetry_csv,
                "telemetry_log_csv_path": telemetry_log_csv_path
            }
        ]
    )

    # ROS2 equivalent to running udacidrone drone.py for manual-flight telemetry capture
    # Records key UAS topics while Unity3D is manually flown.
    rosbag_recorder = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([record_telemetry, " and '", telemetry_record_mode, "' == 'rosbag2'"])
        ),
        cmd=[
            "ros2", "bag", "record",
            "-o", PythonExpression([telemetry_output_dir, " + '/' + ", telemetry_run_id]),
            "/uas/armed",
            "/uas/local_position",
            "/uas/local_velocity",
            "/uas/global_position",
            "/uas/driver_health",
        ],
        output="screen",
    )

	# Optional placeholder for a future dedicated session helper process.
	# Keep this disabled by default until that executable exists.
    session_helper_placeholder = Node(
        package="udacidrone_driver_cpp",
        executable="session_manager_node",
        name="session_manager_node",
        output="screen",
        condition=IfCondition("false"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("connection_uri", default_value="tcp:127.0.0.1:5760"),
            DeclareLaunchArgument("is_px4", default_value="false"),
            DeclareLaunchArgument("use_mock_sim", default_value="false"),
            DeclareLaunchArgument("launch_session_manager", default_value="true"),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("start_external_control_on_boot", default_value="false"),
            DeclareLaunchArgument("timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("target_altitude_m", default_value="3.0"),
            DeclareLaunchArgument("session_service_timeout_sec", default_value="1.5"),
            DeclareLaunchArgument("record_telemetry", default_value="true"),
            DeclareLaunchArgument("telemetry_record_mode", default_value="rosbag2"),
            DeclareLaunchArgument("telemetry_output_dir", default_value="/tmp/uas_logs"),
            DeclareLaunchArgument("telemetry_run_id", default_value="manual_flight"),
            DeclareLaunchArgument("log_telemetry_csv", default_value="false"),
            DeclareLaunchArgument("telemetry_log_csv_path", default_value="/tmp/uas_telemetry.csv"),
            driver_node,
            rosbag_recorder,
            # session_helper_placeholder,
        ]
    )
