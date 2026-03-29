# ROS2 C++ and Python Launch Cheatsheet (Interview Prep)

Built from your current stack:
- ROS2 Jazzy
- C++ driver package (`udacidrone_driver_cpp`)
- Python launch orchestration
- MAVLink + libmavconn integration
- Layered UAS autonomy architecture

## 1) 60-Second Architecture Pitch

Use this interview answer:

"I split the system into transport/protocol, ROS2 driver API, mission planning, and orchestration layers. The C++ driver owns MAVLink decode/encode, state caching, command gating, and health monitoring. Missions do not depend on MAVLink internals; they depend on stable ROS2 interfaces. This improves testability, fault containment, and future extensibility."

Step-by-step explanation:
1. Transport and protocol layer connects to simulator/PX4 and handles MAVLink framing.
2. Driver API layer translates between MAVLink frames and ROS2 messages/services.
3. Mission planning layer generates waypoint or behavior sequences.
4. Orchestration layer (state machine) decides when to transition and which command to execute.
5. This separation prevents mission logic from breaking when low-level transport details change.

## 2) ROS2 C++ Syntax Quick Reference

## 2.1 Minimal Node Skeleton

```cpp
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node {
public:
	MyNode() : Node("my_node") {
		RCLCPP_INFO(get_logger(), "Node started");
	}
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MyNode>());
	rclcpp::shutdown();
	return 0;
}
```

Step-by-step explanation:
1. Include rclcpp to get ROS2 C++ APIs.
2. Inherit from rclcpp::Node so your class becomes a ROS2 node.
3. Call Node("my_node") in the constructor to set node name.
4. Use RCLCPP_INFO for startup logging and visibility.
5. In main, call rclcpp::init first to initialize ROS middleware.
6. Create the node and pass it to rclcpp::spin so callbacks run continuously.
7. Call rclcpp::shutdown for clean teardown.

## 2.2 Parameters (declare + read)

```cpp
const std::string connection_uri = declare_parameter<std::string>(
	"connection_uri", "tcp:127.0.0.1:5760"
);
const bool is_px4 = declare_parameter<bool>("is_px4", false);
const double timeout_sec = declare_parameter<double>("timeout_sec", 5.0);
```

Interview note:
- Declare parameters in the constructor so launch files can override cleanly.

Step-by-step explanation:
1. declare_parameter registers a parameter and returns its current value.
2. The template type defines the expected runtime type.
3. The second argument is the default used when launch/user does not override it.
4. Launch files can inject these values via Node parameters list.
5. This enables mode changes (sim vs PX4, timeout tuning) without recompiling.

## 2.3 Publisher

```cpp
armed_pub_ = create_publisher<std_msgs::msg::Bool>("/uas/armed", 10);

std_msgs::msg::Bool armed;
armed.data = true;
armed_pub_->publish(armed);
```

Step-by-step explanation:
1. create_publisher allocates a publisher bound to a topic and queue depth.
2. Build a message instance and populate its fields.
3. publish sends the message to all active subscribers.
4. Queue depth 10 means recent messages are buffered if subscribers are briefly slower.

## 2.4 Subscription with std::bind

```cpp
cmd_position_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
	"/uas/cmd_position",
	10,
	std::bind(&UasDriverNode::cmdPositionCallback, this, std::placeholders::_1)
);
```

Step-by-step explanation:
1. create_subscription registers interest in a topic and expected message type.
2. Queue depth defines receive buffering behavior.
3. std::bind wires member function callback to this object instance.
4. std::placeholders::_1 maps the incoming message pointer into callback argument 1.
5. Callback executes in executor context when messages arrive.

## 2.5 Service Server

```cpp
arm_srv_ = create_service<std_srvs::srv::SetBool>(
	"/uas/arm",
	std::bind(&UasDriverNode::handleArm, this, std::placeholders::_1, std::placeholders::_2)
);
```

Service callback signature pattern:

```cpp
void handleArm(
	const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
	std::shared_ptr<std_srvs::srv::SetBool::Response> res
)
```

Step-by-step explanation:
1. create_service advertises a named RPC endpoint.
2. Request type SetBool carries desired arm/disarm boolean.
3. Response object must be filled with success and message.
4. Member callback binding uses two placeholders: request and response.
5. Service is ideal for quick command/ack patterns.

## 2.6 Timers (read loop + watchdog)

```cpp
using namespace std::chrono_literals;

read_timer_ = create_wall_timer(20ms, std::bind(&UasDriverNode::readLoop, this));
watchdog_timer_ = create_wall_timer(200ms, std::bind(&UasDriverNode::watchdogLoop, this));
```

Interview note:
- Fast timer for protocol ingest, slower timer for health/timeout policy.

Step-by-step explanation:
1. using namespace std::chrono_literals enables readable time literals (20ms).
2. create_wall_timer schedules periodic callbacks based on wall-clock time.
3. readLoop runs high frequency for fresh telemetry ingestion.
4. watchdogLoop runs lower frequency to evaluate timeout and health policy.
5. Separating loops avoids mixing fast I/O path with slower safety evaluation.

## 2.7 Logging

```cpp
RCLCPP_INFO(get_logger(), "Connected to simulator");
RCLCPP_WARN(get_logger(), "Rejected command: %s", reason.c_str());
RCLCPP_ERROR(get_logger(), "Decode failed");
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Telemetry timeout detected");
```

Step-by-step explanation:
1. Use INFO for expected state transitions and startup.
2. Use WARN for recoverable safety issues (rejected command).
3. Use ERROR for failures needing attention (decode issues, missing dependencies).
4. Use WARN_THROTTLE to avoid flooding logs during repetitive timeout periods.
5. Good logging improves incident triage and interview storytelling.

## 2.8 Thread-safe state snapshot pattern

```cpp
class UasStateRepository {
public:
	void updateState(const UasStateTelemetry &msg) {
		std::lock_guard<std::mutex> lock(mutex_);
		snapshot_.state = msg;
	}

	UasStateSnapshot snapshot() const {
		std::lock_guard<std::mutex> lock(mutex_);
		return snapshot_;
	}

private:
	mutable std::mutex mutex_;
	UasStateSnapshot snapshot_{};
};
```

Step-by-step explanation:
1. A mutex guards shared state accessed by multiple callbacks.
2. updateState acquires lock, writes latest state atomically.
3. snapshot acquires lock, returns a coherent copy.
4. Returning a copy minimizes lock hold time for readers.
5. This pattern prevents race conditions between I/O, timers, and command callbacks.

## 2.9 Guarded command execution pattern

```cpp
bool executeGuardedCommand(
	UasCommandType type,
	const std::function<bool()> &fn,
	const std::string &command_name,
	std::string *failure_reason = nullptr
) {
	const auto snap = state_repo_.snapshot();
	const bool program_running = session_manager_ ? session_manager_->isProgramRunning() : true;

	if (!gatekeeper_.allow(type, snap, program_running)) {
		if (failure_reason) {
			*failure_reason = gatekeeper_.denyReason(type, snap, program_running);
		}
		health_monitor_.onCommandResult(false);
		return false;
	}

	const bool ok = fn();
	health_monitor_.onCommandResult(ok);
	return ok;
}
```

Interview note:
- This is a policy layer between ROS callback glue and actuation path.

Step-by-step explanation:
1. Read current snapshot from repository before command execution.
2. Check external run-state (program running) from session manager.
3. Ask gatekeeper if command is allowed in current safety context.
4. If denied, write human-readable reason and record failure in health monitor.
5. If allowed, execute command function object.
6. Record command success/failure for observability metrics.
7. Return final status to service callback or command caller.

## 2.10 Common CMake + package.xml dependency pattern

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(libmavconn REQUIRED)

add_executable(uas_driver_node src/main.cpp ...)

ament_target_dependencies(uas_driver_node
	rclcpp
	std_srvs
	geometry_msgs
	sensor_msgs
	mavlink
	mavros_msgs
	libmavconn
)
```

Interview note:
- Prefer `ament_target_dependencies` over hardcoded `target_link_libraries(... mavconn)` for portability.

Step-by-step explanation:
1. find_package locates each ROS2/system dependency and exports build info.
2. add_executable defines compile units for your node binary.
3. ament_target_dependencies attaches include dirs, compile defs, and link interfaces.
4. This avoids hardcoded linker assumptions that can vary by distro.
5. Keep package.xml dependencies aligned with CMake declarations.

## 3) ROS2 Python Launch Cheatsheet

## 3.1 Minimal launch file shape

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
		use_mock_sim = LaunchConfiguration("use_mock_sim")

		driver_node = Node(
				package="udacidrone_driver_cpp",
				executable="uas_driver_node",
				name="uas_driver_node",
				output="screen",
				parameters=[{"use_mock_sim": use_mock_sim}],
		)

		return LaunchDescription([
				DeclareLaunchArgument("use_mock_sim", default_value="false"),
				driver_node,
		])
```

Step-by-step explanation:
1. Import launch primitives and Node action.
2. Define generate_launch_description as launch entrypoint.
3. Create LaunchConfiguration handles to read argument values at runtime.
4. Define Node action with package, executable, name, and parameters.
5. Return LaunchDescription that contains argument declarations and actions.
6. Launch engine evaluates substitutions and starts processes.

## 3.2 Launch arguments you use in this stack

```python
DeclareLaunchArgument("connection_uri", default_value="tcp:127.0.0.1:5760"),
DeclareLaunchArgument("is_px4", default_value="false"),
DeclareLaunchArgument("use_mock_sim", default_value="false"),
DeclareLaunchArgument("timeout_sec", default_value="5.0"),
DeclareLaunchArgument("target_altitude_m", default_value="3.0"),
DeclareLaunchArgument("record_telemetry", default_value="true"),
```

Step-by-step explanation:
1. Each DeclareLaunchArgument creates a configurable runtime input.
2. connection_uri controls simulator transport endpoint.
3. is_px4 toggles platform-specific command behavior.
4. use_mock_sim bypasses real simulator dependencies for local testing.
5. timeout and altitude tune safety/performance behavior without code changes.
6. telemetry flags enable/disable recording workflows.

## 3.3 Conditional logic with IfCondition + PythonExpression

```python
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

effective_use_mock_sim = PythonExpression([
		"(", use_mock_sim, ") or (not ", launch_session_manager, ")"
])

rosbag_recorder = ExecuteProcess(
		condition=IfCondition(PythonExpression([
				record_telemetry,
				" and '", telemetry_record_mode, "' == 'rosbag2'"
		])),
		cmd=["ros2", "bag", "record", "/uas/armed", "/uas/local_position"],
		output="screen",
)
```

Step-by-step explanation:
1. PythonExpression composes string expression evaluated at launch runtime.
2. effective_use_mock_sim demonstrates derived configuration from multiple flags.
3. IfCondition gates action execution based on evaluated expression.
4. ExecuteProcess starts non-node command when condition is true.
5. This pattern is useful for optional telemetry capture and mode-specific tools.

## 3.4 Passing launch args into node parameters

```python
parameters=[
		{
				"connection_uri": connection_uri,
				"is_px4": is_px4,
				"use_mock_sim": effective_use_mock_sim,
				"timeout_sec": timeout_sec,
				"target_altitude_m": target_altitude_m,
		}
]
```

Step-by-step explanation:
1. Build parameter dictionary with keys matching C++ declare_parameter names.
2. Assign LaunchConfiguration or derived expression values.
3. ROS2 launch resolves substitutions and passes typed parameter values to node.
4. Node constructor reads values immediately through declare_parameter calls.
5. This creates a clean config pipeline from launch CLI to runtime behavior.

## 3.5 Interview pitfalls (launch)

1. Typo mismatch between built executable and launch executable name.
2. Boolean values are strings in launch arguments; treat them carefully in expressions.
3. Keep parameter names consistent between C++ `declare_parameter` and launch dict keys.
4. Avoid deep launch logic when a regular node parameter can express the mode.

Step-by-step explanation:
1. Validate executable names against built target names before debugging logic.
2. Keep boolean expression formatting explicit to avoid string-evaluation surprises.
3. Audit parameter keys whenever refactoring C++ constructor parameters.
4. Use launch logic for orchestration and keep business policy in node code.

## 4) ROS2 Design and Architecture Talking Points

## 4.1 Why this package split is strong

1. `udacidrone_driver_cpp`: transport, translation, command and telemetry APIs.
2. `uas_mission_core`: path generation and mission validation.
3. `sm_uas_missions`: event-driven mission orchestration.
4. `uas_bringup`: runtime assembly, launch profiles, and modes.

Step-by-step explanation:
1. Driver package can be tested independently with direct service/topic calls.
2. Mission core can be unit-tested without simulator transport dependencies.
3. State machine package can focus on transition correctness and safety rules.
4. Bringup package integrates all parts with profile-based launch.
5. This modularity lowers integration risk and speeds regression testing.

## 4.2 Feature decisions you can defend

1. Watchdog timeout for stale telemetry.
2. Command gatekeeper to reject unsafe requests.
3. Health monitor for observability and fail-safe transitions.
4. Runtime mode parameterization for simulator vs PX4 behavior differences.
5. Docker-first workflow for reproducibility.

Step-by-step explanation:
1. Timeout watchdog detects stale links deterministically.
2. Gatekeeper centralizes command policy and rejection reasons.
3. Health monitor exposes reliability state to operators and higher layers.
4. Runtime mode parameters isolate simulator/PX4 differences.
5. Docker pins toolchain versions and removes host-machine drift.

## 5) MAVLink Integration Talking Points (C++ Interview)

1. Use guarded include paths because header layouts vary across environments.
2. Use C-style MAVLink APIs/macros (global symbols), not `mavlink::` for those APIs.
3. Apply `#undef MAVLINK_VERSION` before `mavros_msgs/mavlink_convert.hpp` to avoid macro collisions.
4. Keep conversion boundaries explicit via `mavros_msgs::mavlink::convert`.
5. Prefer ament-exported dependency linkage for `libmavconn`.

Step-by-step explanation:
1. Check include availability across possible MAVLink header layouts.
2. Select common dialect to guarantee expected message IDs and pack/decode APIs.
3. Sanitize macro collisions before including downstream conversion headers.
4. Convert at API boundaries: ROS message in/out, MAVLink internal representation.
5. Keep linking portable by consuming exported package interfaces.

## 6) High-Value CLI Commands to Memorize

```bash
# Build
colcon build --packages-select udacidrone_driver_cpp --symlink-install

# Source workspace
source install/setup.bash

# Run node directly
ros2 run udacidrone_driver_cpp uas_driver_node

# Launch stack
ros2 launch udacidrone_driver_cpp udacidrone_driver.launch.py

# Override args at launch
ros2 launch udacidrone_driver_cpp udacidrone_driver.launch.py is_px4:=true timeout_sec:=3.0

# Call services
ros2 service call /uas/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /uas/takeoff std_srvs/srv/Trigger "{}"

# Publish cmd_position once
ros2 topic pub --once /uas/cmd_position geometry_msgs/msg/PoseStamped \
"{pose: {position: {x: 5.0, y: 5.0, z: -3.0}}}"

# Inspect interfaces
ros2 topic list
ros2 service list
ros2 param list /uas_driver_node
```

Step-by-step explanation:
1. Build package quickly with symlink-install for fast iteration.
2. Source install setup so ROS can find overlays and executables.
3. Run node directly when isolating node-level bugs.
4. Use launch when validating full runtime orchestration.
5. Use service and topic commands to validate command and telemetry paths.
6. Use inspection commands to confirm interface availability and runtime wiring.

## 7) Fast Interview Q&A Prompts

1. Topic vs Service vs Action:
Topic for streams (`/uas/local_position`), service for short command (`/uas/arm`), action for long-running mission (`/uas/fly_waypoints`).
2. Why layered architecture:
Decouples mission logic from protocol details and reduces integration risk.
3. Why watchdog + health publisher:
Transforms silent comms failure into explicit system state and safe behavior.
4. Why launch arguments:
Allow mode/profile switching without recompiling.

Step-by-step explanation:
1. Answer with one concrete example per concept from your own stack.
2. Keep responses structured as problem, design choice, and benefit.
3. Tie safety features to an actual failure mode (telemetry timeout).
4. Mention testability and maintainability outcomes for architecture answers.

## 8) Last-Minute Review Checklist

1. Can you write a node constructor with parameter declarations from memory?
Answer: Yes. I can write a constructor that inherits from `rclcpp::Node`, declares runtime parameters like `connection_uri`, `is_px4`, `timeout_sec`, and then wires publishers, subscriptions, services, and timers in one place so launch overrides apply cleanly.

2. Can you write one publisher, one service, and one timer callback on a whiteboard?
Answer: Yes. Publisher: `create_publisher<std_msgs::msg::Bool>("/uas/armed", 10)`. Service: `create_service<std_srvs::srv::SetBool>("/uas/arm", ...)`. Timer: `create_wall_timer(200ms, std::bind(&UasDriverNode::watchdogLoop, this))` to periodically enforce timeout and health policy.

3. Can you explain `ament_target_dependencies` vs raw linker flags?
Answer: `ament_target_dependencies` is ROS2-native and uses exported include/link interfaces from packages, which is portable across distros and packaging layouts. Raw linker flags like `target_link_libraries(... mavconn)` are brittle because library names and exports can differ between environments.

4. Can you describe your inbound telemetry and outbound command data paths clearly?
Answer: Inbound path: `SimulatorConnection::readMessage()` -> `MavlinkTranslator::decode()` -> `UasStateRepository` updates -> `UasDriverNode::publishTelemetry()` to ROS2 topics. Outbound path: ROS2 callback/service -> `executeGuardedCommand()` with gatekeeper checks -> `UasCommandService` encode call -> `SimulatorConnection::sendMessage()` over libmavconn.

5. Can you explain one concrete fail-safe flow when telemetry times out?
Answer: The watchdog computes telemetry age from `last_rx_time_`. If age exceeds `timeout_sec`, it marks connection false in the repository, emits throttled timeout warnings, health status degrades, and subsequent commands can be rejected by policy/gating; orchestration can then trigger abort or controlled landing behavior.

Step-by-step explanation:
1. Practice writing constructor, parameter declarations, and one callback from memory.
2. Rehearse inbound path: readMessage -> decode -> repository -> publish.
3. Rehearse outbound path: callback -> gatekeeper -> command service -> sendMessage.
4. Rehearse fail-safe path: watchdog timeout -> connected false -> health degradation -> command denial/abort transition.
5. Time your answers to fit both 30-second and 2-minute versions.
