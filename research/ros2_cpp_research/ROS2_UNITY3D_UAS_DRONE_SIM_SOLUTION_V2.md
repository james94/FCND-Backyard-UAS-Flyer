# ROS2 Unity3D UAS Drone Simulator Solution V2

## 1. Docker-First Infrastructure (Ubuntu 24.04 + ROS2 Jazzy)

This section comes first by design so your team can build a stable development runtime before writing backend code.

## 1.1 Directory Layout for Infrastructure

Create a dedicated infrastructure folder in your research workspace:

```text
research/ros2_cpp_research/infrastructure/
	Dockerfile
	docker-compose.yml
	scripts/
		build_image.sh
		run_dev_container.sh
		enter_container.sh
```

## 1.2 Dockerfile (Base Dev Image)

Use this Dockerfile as the baseline for building and manually developing the ROS2 C++ stack:

```dockerfile
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Core utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
		ca-certificates \
		curl \
		gnupg2 \
		lsb-release \
		software-properties-common \
		git \
		build-essential \
		cmake \
		gdb \
		vim \
		python3 \
		python3-pip \
		python3-venv \
		&& rm -rf /var/lib/apt/lists/*

# ROS2 Jazzy apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
		-o /usr/share/keyrings/ros-archive-keyring.gpg && \
		echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
		http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
		> /etc/apt/sources.list.d/ros2.list

# ROS2 Jazzy + tooling + dependencies for this project
RUN apt-get update && apt-get install -y --no-install-recommends \
		ros-jazzy-desktop \
		python3-colcon-common-extensions \
		python3-rosdep \
		python3-vcstool \
		python3-argcomplete \
		ros-jazzy-geometry-msgs \
		ros-jazzy-nav-msgs \
		ros-jazzy-sensor-msgs \
		ros-jazzy-std-msgs \
		ros-jazzy-std-srvs \
		ros-jazzy-rclcpp \
		ros-jazzy-rclcpp-action \
		ros-jazzy-rclcpp-components \
		ros-jazzy-launch \
		ros-jazzy-launch-ros \
		ros-jazzy-tf2 \
		ros-jazzy-tf2-ros \
		ros-jazzy-tf2-geometry-msgs \
		ros-jazzy-ament-cmake \
		ros-jazzy-ament-cmake-gtest \
		ros-jazzy-ros-testing \
		ros-jazzy-smacc2 \
		ros-jazzy-smacc2-msgs \
		ros-jazzy-moveit \
		&& rm -rf /var/lib/apt/lists/*

# Optional MAVLink development dependency (C implementation)
RUN apt-get update && apt-get install -y --no-install-recommends \
		libmavlink-dev \
		&& rm -rf /var/lib/apt/lists/*

# Workspace setup
RUN mkdir -p /opt/ws/src
WORKDIR /opt/ws

# ROS setup in shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# rosdep init (safe if already initialized)
RUN rosdep init || true
RUN rosdep update || true

CMD ["bash"]
```

## 1.3 docker-compose.yml

Use compose so you can mount your workspace and iteratively build/test:

```yaml
version: "3.8"

services:
	ros2_uas_dev:
		build:
			context: .
			dockerfile: Dockerfile
		container_name: ros2_uas_dev
		tty: true
		stdin_open: true
		network_mode: host
		environment:
			- RMW_IMPLEMENTATION=rmw_fastrtps_cpp
		volumes:
			- /home/ubuntu/src/FCND-Backyard-UAS-Flyer:/opt/ws/src/FCND-Backyard-UAS-Flyer
```

## 1.4 Helper Scripts

```bash
#!/usr/bin/env bash
set -e
docker compose build
```

```bash
#!/usr/bin/env bash
set -e
docker compose up -d
```

```bash
#!/usr/bin/env bash
set -e
docker exec -it ros2_uas_dev bash
```

## 1.5 First Container Validation

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd /opt/ws
colcon --help
ros2 --help
```

If these commands pass, infrastructure is ready for modular C++ development.

## 2. Architecture Translation from Problem Statement

The target architecture has four core code layers:

1. Transport and protocol adapter layer
2. ROS2 driver service/topic/action layer
3. Mission planning and validation layer
4. Mission orchestration layer (SMACC2)

This preserves UdaciDrone separation of concerns while enabling ROS2-native modularity.

## 3. ROS2 Workspace and Package Skeleton

Inside container:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p /opt/ws/src/uas_stack
cd /opt/ws/src/uas_stack

ros2 pkg create --build-type ament_cmake udacidrone_msgs
ros2 pkg create --build-type ament_cmake udacidrone_driver_cpp --dependencies rclcpp std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs
ros2 pkg create --build-type ament_cmake uas_mission_core --dependencies rclcpp geometry_msgs
ros2 pkg create --build-type ament_cmake sm_uas_missions --dependencies rclcpp smacc2 smacc2_msgs geometry_msgs std_msgs
ros2 pkg create --build-type ament_cmake uas_bringup --dependencies rclcpp launch launch_ros
```

## 4. Modular C++ Class-Based Design

## 4.1 udacidrone_driver_cpp Class Model

Implement the driver package with explicit classes and single responsibility.

### Class A: SimulatorConnection

Responsibility:

1. Open/close simulator transport endpoint
2. Read raw protocol frames
3. Write outbound command frames
4. Surface connection health

Suggested interface:

```cpp
class SimulatorConnection
{
public:
	bool connect(const std::string &uri);
	void disconnect();
	bool isConnected() const;
	bool readFrame(std::vector<uint8_t> &out);
	bool sendFrame(const std::vector<uint8_t> &payload);
};
```

### Class B: MavlinkTranslator

Responsibility:

1. Decode MAVLink frames to typed telemetry structs
2. Encode command structs to MAVLink frames
3. Hide MAVLink details from ROS2 application logic

Suggested interface:

```cpp
struct UasStateTelemetry;
struct UasPositionTelemetry;
struct UasVelocityTelemetry;

class MavlinkTranslator
{
public:
	bool decode(const std::vector<uint8_t> &frame);
	bool hasState() const;
	UasStateTelemetry takeState();
	bool hasPosition() const;
	UasPositionTelemetry takePosition();
	bool hasVelocity() const;
	UasVelocityTelemetry takeVelocity();

	std::vector<uint8_t> encodeArm(bool arm);
	std::vector<uint8_t> encodeTakeControl();
	std::vector<uint8_t> encodeReleaseControl();
	std::vector<uint8_t> encodeTakeoff(float altitude_m);
	std::vector<uint8_t> encodeLand();
	std::vector<uint8_t> encodeCmdPosition(float n, float e, float d, float yaw);
};
```

### Class C: UasStateRepository

Responsibility:

1. Hold latest normalized state
2. Apply frame/sign conversion rules (NED, PX4/simulator differences)
3. Provide read-only snapshots to publishers and guards

Suggested interface:

```cpp
class UasStateRepository
{
public:
	void updateState(const UasStateTelemetry &msg);
	void updatePosition(const UasPositionTelemetry &msg);
	void updateVelocity(const UasVelocityTelemetry &msg);
	UasStateSnapshot snapshot() const;
};
```

### Class D: UasCommandService

Responsibility:

1. Execute high-level operations using connection + translator
2. Ensure ordered command flow (control mode, arm, takeoff, etc.)
3. Enforce command throttling and retries where needed

### Class E: UasDriverNode

Responsibility:

1. ROS2 node wiring publishers, subscribers, and services
2. Poll/read loop timer and watchdog timer
3. Publish state and diagnostics

This class composes A, B, C, and D, and should avoid protocol details in callbacks.

## 4.2 uas_mission_core Class Model

### Interface: IMissionPlanner

```cpp
class IMissionPlanner
{
public:
	virtual ~IMissionPlanner() = default;
	virtual std::vector<Waypoint> buildPlan(const MissionRequest &request) = 0;
};
```

### Implementations

1. RectangleMissionPlanner
2. StarMissionPlanner
3. PatrolMissionPlanner

### Supporting Classes

1. GeofenceValidator
2. WaypointAcceptancePolicy
3. MissionPlanSerializer

## 4.3 sm_uas_missions Class Model

### State Machine

States:

1. StManual
2. StArming
3. StTakeoff
4. StMissionNavigate
5. StLanding
6. StDisarming
7. StCompleted
8. StAbort

### Event Definitions

1. EvArmed
2. EvTakeoffReached
3. EvWaypointReached
4. EvMissionDone
5. EvLanded
6. EvDisarmed
7. EvConnectionLost
8. EvTimeout
9. EvAbortRequested

### SMACC2 Client Behaviors

1. CbArm
2. CbTakeoff
3. CbSendWaypoint
4. CbLand
5. CbDisarm
6. CbAbort

## 5. Step-by-Step Development Workflow (Manual Coding Flow)

## Step 1: Build Infrastructure

1. Build image and run container.
2. Enter container and verify ROS2 Jazzy toolchain.
3. Confirm mounted workspace path.

## Step 2: Scaffold Driver Classes

1. Create headers and source files for the five driver classes.
2. Add minimal constructor/destructor implementations.
3. Compile with placeholders before functional logic.

## Step 3: Add Telemetry Decode Path

1. Implement SimulatorConnection read loop.
2. Implement translator decode to telemetry structs.
3. Update repository snapshots.
4. Publish `/uas/state`, `/uas/local_position`, `/uas/local_velocity`, `/uas/global_position`.

Manual test:

```bash
ros2 topic echo /uas/state
ros2 topic echo /uas/local_position
```

## Step 4: Add Command Path

1. Implement service callbacks for arm/disarm/mode/takeoff/land.
2. Implement `/uas/cmd_position` subscriber path.
3. Verify command encoding and transmit.

Manual test:

```bash
ros2 service call /uas/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /uas/takeoff std_srvs/srv/Trigger "{}"
```

## Step 5: Implement Mission Planners

1. Implement rectangle planner and test deterministic output.
2. Implement star planner with radius and point-count parameters.
3. Implement patrol planner reading YAML waypoints.
4. Apply geofence and altitude validation in a shared validator.

## Step 6: Integrate SMACC2 Mission Orchestration

1. Create state classes and transitions for FCND-equivalent flow.
2. Bind client behaviors to driver services/topics.
3. Add guards:
	 - takeoff reached by altitude tolerance
	 - waypoint reached by position and velocity tolerance
	 - landed by near-ground plus vertical rate threshold

## Step 7: Add Launch and Mission Profiles

1. Add launch files in uas_bringup.
2. Add mission YAMLs for rectangle, star, and patrol.
3. Add `mission_mode` runtime parameter.

## Step 8: System Validation in Unity3D

1. Run rectangle mission end-to-end.
2. Run star mission and verify waypoint order.
3. Run patrol mission with loop/time limits.
4. Test connection drop and verify abort to safe landing.

## 6. Suggested C++ File Layout

```text
udacidrone_driver_cpp/
	include/udacidrone_driver_cpp/
		simulator_connection.hpp
		mavlink_translator.hpp
		uas_state_repository.hpp
		uas_command_service.hpp
		uas_driver_node.hpp
	src/
		simulator_connection.cpp
		mavlink_translator.cpp
		uas_state_repository.cpp
		uas_command_service.cpp
		uas_driver_node.cpp
		main.cpp
```

```text
uas_mission_core/
	include/uas_mission_core/
		i_mission_planner.hpp
		rectangle_mission_planner.hpp
		star_mission_planner.hpp
		patrol_mission_planner.hpp
		geofence_validator.hpp
		waypoint_acceptance_policy.hpp
	src/
		rectangle_mission_planner.cpp
		star_mission_planner.cpp
		patrol_mission_planner.cpp
		geofence_validator.cpp
		waypoint_acceptance_policy.cpp
```

## 7. ROS2 Interfaces to Implement First

Prioritize this sequence:

1. Topics
	 - /uas/state
	 - /uas/local_position
	 - /uas/local_velocity

2. Services
	 - /uas/arm
	 - /uas/disarm
	 - /uas/take_control
	 - /uas/takeoff
	 - /uas/land

3. Command topic
	 - /uas/cmd_position

After this baseline works, add action-based waypoint mission execution.

## 8. Build and Test Commands

Inside container:

```bash
source /opt/ros/jazzy/setup.bash
cd /opt/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

Run stack:

```bash
ros2 launch uas_bringup uas_stack.launch.py mission_mode:=rectangle
```

## 9. Acceptance Checklist (Per Problem Statement)

1. FCND baseline mission reproduced: takeoff 3 m, rectangle path, land, disarm.
2. Star mission implemented and validated.
3. Patrol mission implemented with duration/geofence safeguards.
4. Clear separation between protocol adapter and mission logic.
5. Dockerized dev workflow is reproducible on fresh machine.
6. Connection loss triggers deterministic abort/landing behavior.

## 10. Recommended Incremental Milestones

1. M1: Driver publishes telemetry only.
2. M2: Driver executes manual command services.
3. M3: Rectangle planner integrated and executable.
4. M4: SMACC2 FCND-equivalent orchestration complete.
5. M5: Star and patrol mission modes complete.
6. M6: Fault handling, tests, and documentation complete.

## 11. Common Integration Pitfalls

1. Frame conversion mistakes (NED vs ENU).
2. Simulator/PX4 sign differences for altitude/down axis.
3. Race conditions from asynchronous callbacks.
4. Missing watchdog timeout behavior.
5. Tight coupling of mission logic to MAVLink parsing.

Mitigation: centralize conversion, isolate adapter layer, and route mission transitions through explicit events.

## 12. Final Implementation Guidance

Start with the smallest complete vertical slice:

1. Build Docker image
2. Run container
3. Implement driver node with one telemetry topic and one arm service
4. Verify in simulator
5. Expand command/telemetry coverage
6. Add planners
7. Add SMACC2 orchestration

This sequence minimizes integration risk while keeping each step testable.

