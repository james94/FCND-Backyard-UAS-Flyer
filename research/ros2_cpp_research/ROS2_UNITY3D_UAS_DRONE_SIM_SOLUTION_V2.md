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
# RUN apt-get update && apt-get install -y --no-install-recommends \
# 		libmavlink-dev \
# 		&& rm -rf /var/lib/apt/lists/*

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

### 4.1.1 UdaciDrone to ROS2 C++ Behavior and Driver Pattern Mapping

From the Python UdaciDrone implementation, we keep these behaviors in C++:

1. Connection read loop with timeout watchdog (like `dispatch_loop` and `CONNECTION_CLOSED`).
2. Internal state cache updated by decoded message types (`STATE`, `LOCAL_POSITION`, `LOCAL_VELOCITY`, `GLOBAL_POSITION`).
3. Clear separation of transport, parsing, state storage, and command wrappers.
4. Command emission APIs for arm/disarm, mode transitions, takeoff/land, and position control.
5. Optional high-rate command sending path for PX4-like behavior.

From the Universal Robots ROS2 driver patterns, we also apply these architecture rules:

1. Split responsibilities into focused components, not one giant node.
2. Treat runtime mode as first-class configuration through launch arguments.
3. Gate command channels based on simulator program run-state.
4. Wait for required dependencies before enabling command paths.
5. Keep bringup testable with mock mode and optional integration tests.

### 4.1.2 Shared Data Types (Header)

Add one shared header so all classes use the same typed model.

```cpp
// include/udacidrone_driver_cpp/telemetry_types.hpp
#pragma once

#include <cstdint>

namespace udacidrone_driver_cpp
{
struct UasStateTelemetry
{
	double stamp_sec{0.0};
	bool armed{false};
	bool guided{false};
	int32_t status{0};
};

struct UasPositionTelemetry
{
	double stamp_sec{0.0};
	double north{0.0};
	double east{0.0};
	double down{0.0};
	double longitude{0.0};
	double latitude{0.0};
	double altitude{0.0};
};

struct UasVelocityTelemetry
{
	double stamp_sec{0.0};
	double vn{0.0};
	double ve{0.0};
	double vd{0.0};
};

struct UasStateSnapshot
{
	UasStateTelemetry state;
	UasPositionTelemetry position;
	UasVelocityTelemetry velocity;
	bool connected{false};
};
}  // namespace udacidrone_driver_cpp
```

### 4.1.3 Header Interfaces

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

private:
	int sock_fd_{-1};
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

private:
	bool has_state_{false};
	bool has_position_{false};
	bool has_velocity_{false};
	UasStateTelemetry state_{};
	UasPositionTelemetry position_{};
	UasVelocityTelemetry velocity_{};
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
	void setConnected(bool connected);
	void updateState(const UasStateTelemetry &msg);
	void updatePosition(const UasPositionTelemetry &msg);
	void updateVelocity(const UasVelocityTelemetry &msg);
	UasStateSnapshot snapshot() const;

private:
	mutable std::mutex mutex_;
	UasStateSnapshot snapshot_{};
};
```

### Class D: UasCommandService

Responsibility:

1. Execute high-level operations using connection + translator
2. Ensure ordered command flow (control mode, arm, takeoff, etc.)
3. Enforce command throttling and retries where needed

Suggested interface:

```cpp
class UasCommandService
{
public:
	UasCommandService(SimulatorConnection &connection, MavlinkTranslator &translator, bool is_px4);

	bool arm();
	bool disarm();
	bool takeControl();
	bool releaseControl();
	bool takeoff(float target_altitude_m);
	bool land();
	bool cmdPosition(float n, float e, float d, float heading_rad);

private:
	SimulatorConnection &connection_;
	MavlinkTranslator &translator_;
	bool is_px4_{false};
};
```

### Class E: UasDriverNode

Responsibility:

1. ROS2 node wiring publishers, subscribers, and services
2. Poll/read loop timer and watchdog timer
3. Publish state and diagnostics

This class composes A, B, C, and D, and should avoid protocol details in callbacks.

Suggested interface:

```cpp
class UasDriverNode : public rclcpp::Node
{
public:
	UasDriverNode();

private:
	void readLoop();
	void watchdogLoop();
	void publishTelemetry();

	void handleArm(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
								 std::shared_ptr<std_srvs::srv::SetBool::Response> res);
	void handleTakeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
										 std::shared_ptr<std_srvs::srv::Trigger::Response> res);
	void handleLand(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
									std::shared_ptr<std_srvs::srv::Trigger::Response> res);
	void cmdPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	SimulatorConnection connection_;
	MavlinkTranslator translator_;
	UasStateRepository state_repo_;
	std::unique_ptr<UasCommandService> command_service_;

	rclcpp::TimerBase::SharedPtr read_timer_;
	rclcpp::TimerBase::SharedPtr watchdog_timer_;

	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_position_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr local_velocity_pub_;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_position_pub_;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_position_sub_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;

	bool connected_{false};
	rclcpp::Time last_rx_time_;
	double timeout_sec_{5.0};
	double target_altitude_m_{3.0};
};
```

### 4.1.4 Source Templates

Below are starter source files aligned with the core headers above. They are intentionally minimal, compile-friendly scaffolds you can extend incrementally.

Note: once helper classes are added, use the integrated `UasDriverNode` header/source shown in Section 4.1.7 as the canonical version.

```cpp
// src/simulator_connection.cpp
#include "udacidrone_driver_cpp/simulator_connection.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace udacidrone_driver_cpp
{
bool SimulatorConnection::connect(const std::string &uri)
{
	// Example URI expected: tcp:127.0.0.1:5760
	// Parse and open socket here; return true on success.
	(void)uri;
	return false;
}

void SimulatorConnection::disconnect()
{
	if (sock_fd_ >= 0)
	{
		::close(sock_fd_);
		sock_fd_ = -1;
	}
}

bool SimulatorConnection::isConnected() const
{
	return sock_fd_ >= 0;
}

bool SimulatorConnection::readFrame(std::vector<uint8_t> &out)
{
	out.clear();
	if (!isConnected())
	{
		return false;
	}
	uint8_t buffer[2048];
	const ssize_t n = ::recv(sock_fd_, buffer, sizeof(buffer), MSG_DONTWAIT);
	if (n <= 0)
	{
		return false;
	}
	out.insert(out.end(), buffer, buffer + n);
	return true;
}

bool SimulatorConnection::sendFrame(const std::vector<uint8_t> &payload)
{
	if (!isConnected())
	{
		return false;
	}
	const ssize_t sent = ::send(sock_fd_, payload.data(), payload.size(), 0);
	return sent == static_cast<ssize_t>(payload.size());
}
}  // namespace udacidrone_driver_cpp
```

```cpp
// src/mavlink_translator.cpp
#include "udacidrone_driver_cpp/mavlink_translator.hpp"

namespace udacidrone_driver_cpp
{
bool MavlinkTranslator::decode(const std::vector<uint8_t> &frame)
{
	// TODO: parse MAVLink bytes and populate state_/position_/velocity_.
	// Keep this aligned with UdaciDrone MsgID routing semantics.
	(void)frame;
	return false;
}

bool MavlinkTranslator::hasState() const { return has_state_; }
UasStateTelemetry MavlinkTranslator::takeState() { has_state_ = false; return state_; }

bool MavlinkTranslator::hasPosition() const { return has_position_; }
UasPositionTelemetry MavlinkTranslator::takePosition() { has_position_ = false; return position_; }

bool MavlinkTranslator::hasVelocity() const { return has_velocity_; }
UasVelocityTelemetry MavlinkTranslator::takeVelocity() { has_velocity_ = false; return velocity_; }

std::vector<uint8_t> MavlinkTranslator::encodeArm(bool arm)
{
	// TODO: encode MAV_CMD_COMPONENT_ARM_DISARM
	(void)arm;
	return {};
}

std::vector<uint8_t> MavlinkTranslator::encodeTakeControl() { return {}; }
std::vector<uint8_t> MavlinkTranslator::encodeReleaseControl() { return {}; }
std::vector<uint8_t> MavlinkTranslator::encodeTakeoff(float altitude_m) { (void)altitude_m; return {}; }
std::vector<uint8_t> MavlinkTranslator::encodeLand() { return {}; }
std::vector<uint8_t> MavlinkTranslator::encodeCmdPosition(float n, float e, float d, float yaw)
{
	(void)n;
	(void)e;
	(void)d;
	(void)yaw;
	return {};
}
}  // namespace udacidrone_driver_cpp
```

```cpp
// src/uas_state_repository.cpp
#include "udacidrone_driver_cpp/uas_state_repository.hpp"

namespace udacidrone_driver_cpp
{
void UasStateRepository::setConnected(bool connected)
{
	std::lock_guard<std::mutex> lock(mutex_);
	snapshot_.connected = connected;
}

void UasStateRepository::updateState(const UasStateTelemetry &msg)
{
	std::lock_guard<std::mutex> lock(mutex_);
	snapshot_.state = msg;
}

void UasStateRepository::updatePosition(const UasPositionTelemetry &msg)
{
	std::lock_guard<std::mutex> lock(mutex_);
	snapshot_.position = msg;
}

void UasStateRepository::updateVelocity(const UasVelocityTelemetry &msg)
{
	std::lock_guard<std::mutex> lock(mutex_);
	snapshot_.velocity = msg;
}

UasStateSnapshot UasStateRepository::snapshot() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return snapshot_;
}
}  // namespace udacidrone_driver_cpp
```

```cpp
// src/uas_command_service.cpp
#include "udacidrone_driver_cpp/uas_command_service.hpp"

namespace udacidrone_driver_cpp
{
UasCommandService::UasCommandService(
	SimulatorConnection &connection,
	MavlinkTranslator &translator,
	bool is_px4)
: connection_(connection), translator_(translator), is_px4_(is_px4)
{
}

bool UasCommandService::arm()
{
	return connection_.sendFrame(translator_.encodeArm(true));
}

bool UasCommandService::disarm()
{
	return connection_.sendFrame(translator_.encodeArm(false));
}

bool UasCommandService::takeControl()
{
	return connection_.sendFrame(translator_.encodeTakeControl());
}

bool UasCommandService::releaseControl()
{
	return connection_.sendFrame(translator_.encodeReleaseControl());
}

bool UasCommandService::takeoff(float target_altitude_m)
{
	return connection_.sendFrame(translator_.encodeTakeoff(target_altitude_m));
}

bool UasCommandService::land()
{
	return connection_.sendFrame(translator_.encodeLand());
}

bool UasCommandService::cmdPosition(float n, float e, float d, float heading_rad)
{
	// Keep this behavior aligned with UdaciDrone: simulator may need sign inversion on d.
	if (!is_px4_)
	{
		d = -d;
	}
	return connection_.sendFrame(translator_.encodeCmdPosition(n, e, d, heading_rad));
}
}  // namespace udacidrone_driver_cpp
```

The `UasDriverNode` source is intentionally not duplicated here. Use the single canonical integrated implementation in Section 4.1.7 (`include/udacidrone_driver_cpp/uas_driver_node.hpp` and `src/uas_driver_node.cpp`).

```cpp
// src/main.cpp
#include "udacidrone_driver_cpp/uas_driver_node.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<udacidrone_driver_cpp::UasDriverNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
```

### 4.1.5 UR-Inspired Helper Class Interfaces (F-H)

To apply these patterns while preserving the UdaciDrone behavior model, add concrete helper headers first, then implement source files in Section 4.1.6.

### Class F: DriverSessionManager

Responsibilities:

1. Handle simulator session lifecycle and external-control handshake.
2. Expose run-state and connection-state signals to the main driver node.
3. Wrap service-level operations such as start/stop/reset program.

Header template:

```cpp
// include/udacidrone_driver_cpp/driver_session_manager.hpp
#pragma once

#include <atomic>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace udacidrone_driver_cpp
{
class DriverSessionManager
{
public:
	explicit DriverSessionManager(rclcpp::Node &node);

	bool waitForDependencies(std::chrono::milliseconds timeout);
	bool startExternalControlSession();
	bool stopExternalControlSession();
	bool refreshProgramRunning(std::chrono::milliseconds timeout);
	bool isProgramRunning() const;

private:
	bool callTrigger(
		const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
		std::chrono::milliseconds timeout,
		bool *ok_out = nullptr,
		std::string *message_out = nullptr);

	rclcpp::Node &node_;
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_session_client_;
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_session_client_;
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr program_running_client_;
	std::atomic<bool> program_running_{false};
};
}  // namespace udacidrone_driver_cpp
```

### Class G: CommandGatekeeper

Responsibilities:

1. Decide if a command is allowed based on run-state, safety-state, and timeout-state.
2. Keep a list of always-allowed commands such as disarm and emergency-land.
3. Centralize command gating policy away from ROS callback glue.

Header template:

```cpp
// include/udacidrone_driver_cpp/command_gatekeeper.hpp
#pragma once

#include <string>

#include "udacidrone_driver_cpp/telemetry_types.hpp"

namespace udacidrone_driver_cpp
{
enum class UasCommandType
{
	ARM,
	DISARM,
	TAKE_CONTROL,
	RELEASE_CONTROL,
	TAKEOFF,
	LAND,
	CMD_POSITION
};

class CommandGatekeeper
{
public:
	bool allow(UasCommandType type, const UasStateSnapshot &snapshot, bool program_running) const;
	std::string denyReason(UasCommandType type, const UasStateSnapshot &snapshot, bool program_running) const;
};
}  // namespace udacidrone_driver_cpp
```

### Class H: DriverHealthMonitor

Responsibilities:

1. Track heartbeat age, decode errors, and command failure counters.
2. Publish health status for observability and mission-level decisions.
3. Emit transition events used by SMACC2 abort flows.

Header template:

```cpp
// include/udacidrone_driver_cpp/driver_health_monitor.hpp
#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace udacidrone_driver_cpp
{
struct DriverHealth
{
	bool healthy{true};
	double heartbeat_age_sec{0.0};
	uint64_t decode_errors{0};
	uint64_t command_failures{0};
	std::string status_text;
};

class DriverHealthMonitor
{
public:
	void onRxFrame(bool decoded_ok, rclcpp::Time stamp);
	void onCommandResult(bool ok);
	void markDependencyNotReady();
	DriverHealth evaluate(rclcpp::Time now, double timeout_sec) const;

private:
	mutable std::mutex mutex_;
	rclcpp::Time last_rx_time_{0, 0, RCL_ROS_TIME};
	std::atomic<uint64_t> decode_errors_{0};
	std::atomic<uint64_t> command_failures_{0};
	std::atomic<uint64_t> dependency_wait_events_{0};
};
}  // namespace udacidrone_driver_cpp
```

### 4.1.6 Source Templates for UR-Inspired Helper Classes (F-H)

```cpp
// src/driver_session_manager.cpp
#include "udacidrone_driver_cpp/driver_session_manager.hpp"

#include <future>
#include <memory>

namespace udacidrone_driver_cpp
{
DriverSessionManager::DriverSessionManager(rclcpp::Node &node) : node_(node)
{
	// Keep namespaced service paths so bringup can remap cleanly.
	start_session_client_ = node_.create_client<std_srvs::srv::Trigger>("/uas/session/start_external_control");
	stop_session_client_ = node_.create_client<std_srvs::srv::Trigger>("/uas/session/stop_external_control");
	program_running_client_ = node_.create_client<std_srvs::srv::Trigger>("/uas/session/program_running");
}

bool DriverSessionManager::waitForDependencies(std::chrono::milliseconds timeout)
{
	const auto t = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
	const bool start_ok = start_session_client_->wait_for_service(t);
	const bool stop_ok = stop_session_client_->wait_for_service(t);
	const bool running_ok = program_running_client_->wait_for_service(t);
	if (!start_ok || !stop_ok || !running_ok)
	{
		RCLCPP_WARN(node_.get_logger(), "Session dependencies not ready within timeout");
		return false;
	}
	RCLCPP_INFO(node_.get_logger(), "Session dependencies are ready");
	return true;
}

bool DriverSessionManager::startExternalControlSession()
{
	bool ok = false;
	std::string message;
	if (!callTrigger(start_session_client_, std::chrono::milliseconds(1000), &ok, &message))
	{
		RCLCPP_WARN(node_.get_logger(), "Failed to call start_external_control service");
		return false;
	}
	program_running_.store(ok);
	if (!message.empty())
	{
		RCLCPP_INFO(node_.get_logger(), "start_external_control response: %s", message.c_str());
	}
	return ok;
}

bool DriverSessionManager::stopExternalControlSession()
{
	bool ok = false;
	std::string message;
	if (!callTrigger(stop_session_client_, std::chrono::milliseconds(1000), &ok, &message))
	{
		RCLCPP_WARN(node_.get_logger(), "Failed to call stop_external_control service");
		return false;
	}
	program_running_.store(!ok ? program_running_.load() : false);
	if (!message.empty())
	{
		RCLCPP_INFO(node_.get_logger(), "stop_external_control response: %s", message.c_str());
	}
	return ok;
}

bool DriverSessionManager::refreshProgramRunning(std::chrono::milliseconds timeout)
{
	bool ok = false;
	if (!callTrigger(program_running_client_, timeout, &ok, nullptr))
	{
		return false;
	}
	program_running_.store(ok);
	return true;
}

bool DriverSessionManager::isProgramRunning() const
{
	return program_running_.load();
}

bool DriverSessionManager::callTrigger(
	const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
	std::chrono::milliseconds timeout,
	bool *ok_out,
	std::string *message_out)
{
	if (!client)
	{
		return false;
	}
	auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
	auto fut = client->async_send_request(req);
	if (fut.wait_for(timeout) != std::future_status::ready)
	{
		return false;
	}
	const auto res = fut.get();
	if (ok_out != nullptr)
	{
		*ok_out = res->success;
	}
	if (message_out != nullptr)
	{
		*message_out = res->message;
	}
	return true;
}
}  // namespace udacidrone_driver_cpp
```

```cpp
// src/command_gatekeeper.cpp
#include "udacidrone_driver_cpp/command_gatekeeper.hpp"

namespace udacidrone_driver_cpp
{
bool CommandGatekeeper::allow(
	UasCommandType type,
	const UasStateSnapshot &snapshot,
	bool program_running) const
{
	// Safety-style exceptions inspired by UR driver behavior:
	// disarm and land remain allowed even if run-state is degraded.
	if (type == UasCommandType::DISARM || type == UasCommandType::LAND)
	{
		return true;
	}

	if (!snapshot.connected)
	{
		return false;
	}

	if ((type == UasCommandType::TAKEOFF || type == UasCommandType::CMD_POSITION) && !snapshot.state.armed)
	{
		return false;
	}

	return program_running;
}

std::string CommandGatekeeper::denyReason(
	UasCommandType type,
	const UasStateSnapshot &snapshot,
	bool program_running) const
{
	if (type == UasCommandType::DISARM || type == UasCommandType::LAND)
	{
		return "";
	}
	if (!snapshot.connected)
	{
		return "driver not connected";
	}
	if ((type == UasCommandType::TAKEOFF || type == UasCommandType::CMD_POSITION) && !snapshot.state.armed)
	{
		return "vehicle must be armed first";
	}
	if (!program_running)
	{
		return "external control program not running";
	}
	return "";
}
}  // namespace udacidrone_driver_cpp
```

```cpp
// src/driver_health_monitor.cpp
#include "udacidrone_driver_cpp/driver_health_monitor.hpp"

namespace udacidrone_driver_cpp
{
void DriverHealthMonitor::onRxFrame(bool decoded_ok, rclcpp::Time stamp)
{
	std::lock_guard<std::mutex> lock(mutex_);
	last_rx_time_ = stamp;
	if (!decoded_ok)
	{
		++decode_errors_;
	}
}

void DriverHealthMonitor::onCommandResult(bool ok)
{
	if (!ok)
	{
		++command_failures_;
	}
}

void DriverHealthMonitor::markDependencyNotReady()
{
	++dependency_wait_events_;
}

DriverHealth DriverHealthMonitor::evaluate(rclcpp::Time now, double timeout_sec) const
{
	DriverHealth out;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		out.heartbeat_age_sec = (now - last_rx_time_).seconds();
	}
	out.decode_errors = decode_errors_;
	out.command_failures = command_failures_;
	out.healthy = (out.heartbeat_age_sec <= timeout_sec) && (dependency_wait_events_ == 0);
	out.status_text = out.healthy ? "ok" : "degraded";
	return out;
}
}  // namespace udacidrone_driver_cpp
```

### 4.1.7 Driver Node Wiring with Core + Helper Classes

Yes, migrate the primary UasDriverNode header/source into this section once helper classes are introduced. That keeps the tutorial flow linear: core classes first, helper classes second, integrated node third.

Use this integrated UasDriverNode header:

```cpp
// include/udacidrone_driver_cpp/uas_driver_node.hpp
#pragma once

#include <functional>
#include <memory>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "udacidrone_driver_cpp/command_gatekeeper.hpp"
#include "udacidrone_driver_cpp/driver_health_monitor.hpp"
#include "udacidrone_driver_cpp/driver_session_manager.hpp"
#include "udacidrone_driver_cpp/mavlink_translator.hpp"
#include "udacidrone_driver_cpp/simulator_connection.hpp"
#include "udacidrone_driver_cpp/uas_command_service.hpp"
#include "udacidrone_driver_cpp/uas_state_repository.hpp"

namespace udacidrone_driver_cpp
{
class UasDriverNode : public rclcpp::Node
{
public:
	UasDriverNode();

private:
	void readLoop();
	void watchdogLoop();
	void publishTelemetry();
	void publishHealth();

	bool executeGuardedCommand(
		UasCommandType type,
		const std::function<bool()> &fn,
		const std::string &command_name,
		std::string *failure_reason = nullptr);

	void handleArm(
		const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		std::shared_ptr<std_srvs::srv::SetBool::Response> res);
	void handleTakeoff(
		const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
		std::shared_ptr<std_srvs::srv::Trigger::Response> res);
	void handleLand(
		const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
		std::shared_ptr<std_srvs::srv::Trigger::Response> res);
	void cmdPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	SimulatorConnection connection_;
	MavlinkTranslator translator_;
	UasStateRepository state_repo_;
	std::unique_ptr<UasCommandService> command_service_;

	std::unique_ptr<DriverSessionManager> session_manager_;
	CommandGatekeeper gatekeeper_;
	DriverHealthMonitor health_monitor_;

	rclcpp::TimerBase::SharedPtr read_timer_;
	rclcpp::TimerBase::SharedPtr watchdog_timer_;

	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_position_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr local_velocity_pub_;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_position_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr driver_health_pub_;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_position_sub_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;

	bool connected_{false};
	bool use_mock_sim_{false};
	bool start_external_control_on_boot_{false};
	double session_service_timeout_sec_{1.5};
	rclcpp::Time last_rx_time_;
	double timeout_sec_{5.0};
	double target_altitude_m_{3.0};
};
}  // namespace udacidrone_driver_cpp
```

Use this integrated source template:

```cpp
// src/uas_driver_node.cpp
#include "udacidrone_driver_cpp/uas_driver_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace udacidrone_driver_cpp
{
UasDriverNode::UasDriverNode()
: Node("uas_driver_node")
{
	const std::string connection_uri = declare_parameter<std::string>("connection_uri", "tcp:127.0.0.1:5760");
	const bool is_px4 = declare_parameter<bool>("is_px4", false);
	use_mock_sim_ = declare_parameter<bool>("use_mock_sim", false);
	start_external_control_on_boot_ = declare_parameter<bool>("start_external_control_on_boot", false);
	session_service_timeout_sec_ = declare_parameter<double>("session_service_timeout_sec", 1.5);
	timeout_sec_ = declare_parameter<double>("timeout_sec", 5.0);
	target_altitude_m_ = declare_parameter<double>("target_altitude_m", 3.0);

	command_service_ = std::make_unique<UasCommandService>(connection_, translator_, is_px4);
	connected_ = connection_.connect(connection_uri);
	state_repo_.setConnected(connected_);
	last_rx_time_ = now();

	if (!use_mock_sim_)
	{
		session_manager_ = std::make_unique<DriverSessionManager>(*this);
		const auto dep_timeout = std::chrono::milliseconds(
			static_cast<int>(session_service_timeout_sec_ * 1000.0));
		const bool deps_ready = session_manager_->waitForDependencies(dep_timeout);
		if (!deps_ready)
		{
			health_monitor_.markDependencyNotReady();
		}
		if (deps_ready && start_external_control_on_boot_)
		{
			(void)session_manager_->startExternalControlSession();
		}
		(void)session_manager_->refreshProgramRunning(std::chrono::milliseconds(100));
	}

	armed_pub_ = create_publisher<std_msgs::msg::Bool>("/uas/armed", 10);
	local_position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/uas/local_position", 10);
	local_velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/uas/local_velocity", 10);
	global_position_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/uas/global_position", 10);
	driver_health_pub_ = create_publisher<std_msgs::msg::String>("/uas/driver_health", 10);

	cmd_position_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
		"/uas/cmd_position", 10,
		std::bind(&UasDriverNode::cmdPositionCallback, this, std::placeholders::_1));

	arm_srv_ = create_service<std_srvs::srv::SetBool>(
		"/uas/arm", std::bind(&UasDriverNode::handleArm, this, std::placeholders::_1, std::placeholders::_2));
	takeoff_srv_ = create_service<std_srvs::srv::Trigger>(
		"/uas/takeoff", std::bind(&UasDriverNode::handleTakeoff, this, std::placeholders::_1, std::placeholders::_2));
	land_srv_ = create_service<std_srvs::srv::Trigger>(
		"/uas/land", std::bind(&UasDriverNode::handleLand, this, std::placeholders::_1, std::placeholders::_2));

	read_timer_ = create_wall_timer(20ms, std::bind(&UasDriverNode::readLoop, this));
	watchdog_timer_ = create_wall_timer(200ms, std::bind(&UasDriverNode::watchdogLoop, this));
}

void UasDriverNode::readLoop()
{
	std::vector<uint8_t> frame;
	if (!connection_.readFrame(frame))
	{
		return;
	}

	const bool decoded_ok = translator_.decode(frame);
	health_monitor_.onRxFrame(decoded_ok, now());
	if (!decoded_ok)
	{
		return;
	}

	if (translator_.hasState())
	{
		state_repo_.updateState(translator_.takeState());
	}
	if (translator_.hasPosition())
	{
		state_repo_.updatePosition(translator_.takePosition());
	}
	if (translator_.hasVelocity())
	{
		state_repo_.updateVelocity(translator_.takeVelocity());
	}
	last_rx_time_ = now();
	state_repo_.setConnected(true);
	publishTelemetry();
}

void UasDriverNode::watchdogLoop()
{
	const double dt = (now() - last_rx_time_).seconds();
	if (dt > timeout_sec_)
	{
		state_repo_.setConnected(false);
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Telemetry timeout detected");
	}
	if (session_manager_)
	{
		const auto refresh_timeout = std::chrono::milliseconds(
			static_cast<int>(session_service_timeout_sec_ * 500.0));
		(void)session_manager_->refreshProgramRunning(refresh_timeout);
	}
	publishHealth();
}

void UasDriverNode::publishTelemetry()
{
	const auto snap = state_repo_.snapshot();

	std_msgs::msg::Bool armed;
	armed.data = snap.state.armed;
	armed_pub_->publish(armed);

	geometry_msgs::msg::PointStamped lp;
	lp.header.stamp = now();
	lp.point.x = snap.position.north;
	lp.point.y = snap.position.east;
	lp.point.z = snap.position.down;
	local_position_pub_->publish(lp);

	geometry_msgs::msg::Vector3Stamped lv;
	lv.header.stamp = now();
	lv.vector.x = snap.velocity.vn;
	lv.vector.y = snap.velocity.ve;
	lv.vector.z = snap.velocity.vd;
	local_velocity_pub_->publish(lv);

	sensor_msgs::msg::NavSatFix gp;
	gp.header.stamp = now();
	gp.latitude = snap.position.latitude;
	gp.longitude = snap.position.longitude;
	gp.altitude = snap.position.altitude;
	global_position_pub_->publish(gp);
}

void UasDriverNode::publishHealth()
{
	const auto health = health_monitor_.evaluate(now(), timeout_sec_);
	std_msgs::msg::String msg;
	msg.data = "status=" + health.status_text +
		", heartbeat_age_sec=" + std::to_string(health.heartbeat_age_sec) +
		", decode_errors=" + std::to_string(health.decode_errors) +
		", command_failures=" + std::to_string(health.command_failures);
	driver_health_pub_->publish(msg);
}

bool UasDriverNode::executeGuardedCommand(
	UasCommandType type,
	const std::function<bool()> &fn,
	const std::string &command_name,
	std::string *failure_reason)
{
	const auto snap = state_repo_.snapshot();
	const bool program_running = session_manager_ ? session_manager_->isProgramRunning() : true;
	if (!gatekeeper_.allow(type, snap, program_running))
	{
		const std::string deny = gatekeeper_.denyReason(type, snap, program_running);
		if (failure_reason)
		{
			*failure_reason = deny;
		}
		RCLCPP_WARN(get_logger(), "Rejected %s: %s", command_name.c_str(), deny.c_str());
		health_monitor_.onCommandResult(false);
		return false;
	}

	const bool ok = fn();
	health_monitor_.onCommandResult(ok);
	if (!ok && failure_reason)
	{
		*failure_reason = "transport or encode failure";
	}
	return ok;
}

void UasDriverNode::handleArm(
	const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
	std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
	std::string reason;
	const bool ok = req->data
		? executeGuardedCommand(UasCommandType::ARM, [this]() { return command_service_->arm(); }, "arm", &reason)
		: executeGuardedCommand(UasCommandType::DISARM, [this]() { return command_service_->disarm(); }, "disarm", &reason);
	res->success = ok;
	res->message = ok ? "arm/disarm command sent" : ("arm/disarm failed: " + reason);
}

void UasDriverNode::handleTakeoff(
	const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
	std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
	(void)req;
	std::string reason;
	const bool ok = executeGuardedCommand(
		UasCommandType::TAKEOFF,
		[this]() { return command_service_->takeoff(static_cast<float>(target_altitude_m_)); },
		"takeoff",
		&reason);
	res->success = ok;
	res->message = ok ? "takeoff command sent" : ("takeoff failed: " + reason);
}

void UasDriverNode::handleLand(
	const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
	std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
	(void)req;
	std::string reason;
	const bool ok = executeGuardedCommand(
		UasCommandType::LAND,
		[this]() { return command_service_->land(); },
		"land",
		&reason);
	res->success = ok;
	res->message = ok ? "land command sent" : ("land failed: " + reason);
}

void UasDriverNode::cmdPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	const float n = static_cast<float>(msg->pose.position.x);
	const float e = static_cast<float>(msg->pose.position.y);
	const float d = static_cast<float>(msg->pose.position.z);
	const float yaw = 0.0F;
	std::string reason;
	(void)executeGuardedCommand(
		UasCommandType::CMD_POSITION,
		[this, n, e, d, yaw]() { return command_service_->cmdPosition(n, e, d, yaw); },
		"cmd_position",
		&reason);
}
}  // namespace udacidrone_driver_cpp
```

Before executing any command callback, this integrated node now does the following:

1. Pull `UasStateSnapshot` from repository.
2. Ask `session_manager_.isProgramRunning()`.
3. Validate with `gatekeeper_.allow(...)`.
4. Execute command only when allowed.
5. Report result to `health_monitor_.onCommandResult(...)`.
6. Publish health summary periodically on `/uas/driver_health`.

### 4.1.8 CMakeLists.txt Additions for All Driver Sources

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(uas_driver_node
	src/main.cpp
	src/simulator_connection.cpp
	src/mavlink_translator.cpp
	src/uas_state_repository.cpp
	src/uas_command_service.cpp
	src/driver_session_manager.cpp
	src/command_gatekeeper.cpp
	src/driver_health_monitor.cpp
	src/uas_driver_node.cpp)

target_include_directories(uas_driver_node PRIVATE include)

ament_target_dependencies(uas_driver_node
	rclcpp
	std_msgs
	std_srvs
	geometry_msgs
	sensor_msgs)

install(TARGETS uas_driver_node DESTINATION lib/${PROJECT_NAME})
```

### 4.1.9 Launch Arguments and Bringup Modes (UR-Style)

Add these launch arguments to make runtime behavior explicit and reproducible.

Why this is still relevant for a drone stack:

1. UdaciDrone Python + Unity simulator already uses runtime mode choices (`threaded`, connection URI, simulator-vs-PX4 behavior), so mode-driven bringup is natural for this project.
2. UR-style launch design is not robot-arm-specific; it is a reusable ROS2 pattern for deterministic startup, dependency gating, and test modes.
3. For this drone case, these arguments control session gating and simulator bringup behavior, not manipulator controllers.

1. `use_mock_sim` default `false`
2. `headless_mode` default `false`
3. `launch_session_manager` default `true`
4. `start_external_control_on_boot` default `false`
5. `session_service_timeout_sec` default `1.5`

Meaning and drone-specific relevance:

1. `use_mock_sim`:
Use `true` for CI/unit-like runs without Unity/PX4 transport. This is directly useful for UdaciDrone-style callback and state-logic testing.
2. `headless_mode`:
Use `true` when Unity or simulator process is launched separately and no manual UI interactions are expected. This is relevant for automated test rigs.
3. `launch_session_manager`:
If `false`, bypass external-control gating and run driver in simplified mode. This is useful during early telemetry-only bringup.
4. `start_external_control_on_boot`:
If `true`, request external-control session automatically at startup. This mirrors UR auto-start behavior and reduces manual steps.
5. `session_service_timeout_sec`:
How long to wait for session services before marking bringup degraded. This replaces manipulator-oriented controller spawner timeout with a drone-relevant readiness timeout.

Node inclusion conditions:

1. If `use_mock_sim == true`, disable session-manager and command gating by external run-state.
2. If `launch_session_manager == true`, keep session-manager logic enabled (currently in-process in `UasDriverNode`; separate helper node is optional future work).
3. If `headless_mode == true`, auto-start external control when dependencies are ready.

### 4.1.10 ROS2 Python Launch Scripts for udacidrone_driver_cpp

Add launch files directly in the driver package so Unity bringup is easy to test without the full mission stack.

```python
# launch/udacidrone_driver.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

	# If launch_session_manager is disabled, force mock mode to bypass session gating.
	effective_use_mock_sim = PythonExpression(
		["(", use_mock_sim, ") or (not ", launch_session_manager, ")"]
	)

	driver_node = Node(
		package="udacidrone_driver_cpp",
		executable="uas_driver_node",
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
			}
		],
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
			driver_node,
			session_helper_placeholder,
		]
	)
```

```python
# launch/udacidrone_driver_mock.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	return LaunchDescription(
		[
			IncludeLaunchDescription(
				PythonLaunchDescriptionSource(
					PathJoinSubstitution(
						[
							FindPackageShare("udacidrone_driver_cpp"),
							"launch",
							"udacidrone_driver.launch.py",
						]
					)
				),
				launch_arguments={
					"use_mock_sim": "true",
					"launch_session_manager": "false",
					"start_external_control_on_boot": "false",
				}.items(),
			)
		]
	)
```

### 4.1.11 Recommended Code Writing Order

Use this order while implementing so each step compiles before the next one.

1. Implement shared types in Section 4.1.2.
2. Add core interfaces from Section 4.1.3.
3. Fill core templates from Section 4.1.4.
4. Add helper interfaces from Section 4.1.5.
5. Fill helper templates from Section 4.1.6.
6. Integrate node wiring rules from Section 4.1.7.
7. Register all sources in CMake from Section 4.1.8.
8. Add and validate launch scripts from Section 4.1.10.
9. Validate launch runtime modes from Section 4.1.9.

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

1. Create shared type header (`telemetry_types.hpp`) and base five class headers.
2. Add UR-inspired helper classes: `DriverSessionManager`, `CommandGatekeeper`, and `DriverHealthMonitor`.
3. Create source files shown in Sections 4.1.4 and 4.1.6, then use Section 4.1.7 for the canonical `UasDriverNode` header/source.
4. Update `CMakeLists.txt` with all new compilation units.
5. Compile with placeholder translator logic before full MAVLink decode is added.

## Step 3: Add Telemetry Decode Path

1. Implement `SimulatorConnection::connect`, `readFrame`, and `disconnect`.
2. Implement `MavlinkTranslator::decode` to map incoming frames to typed telemetry.
3. Update state snapshots in `UasStateRepository`.
4. Publish `/uas/armed`, `/uas/local_position`, `/uas/local_velocity`, `/uas/global_position`.
5. Add watchdog timeout parity with UdaciDrone dispatch timeout behavior.

Manual test:

```bash
ros2 topic echo /uas/armed
ros2 topic echo /uas/local_position
```

## Step 4: Add Command Path

1. Implement service callbacks for arm/disarm/takeoff/land first.
2. Implement `/uas/cmd_position` subscriber path.
3. Implement `UasCommandService` wrappers and sign handling for simulator `d` axis.
4. Add command gating policy using `CommandGatekeeper` and program run-state.
5. Verify command encoding and transmit.

Manual test:

```bash
ros2 service call /uas/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /uas/takeoff std_srvs/srv/Trigger "{}"
ros2 topic pub /uas/cmd_position geometry_msgs/msg/PoseStamped "{pose: {position: {x: 5.0, y: 0.0, z: 3.0}}}" -1
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

1. Add driver launch files in udacidrone_driver_cpp/launch.
2. Add mission YAMLs for rectangle, star, and patrol.
3. Add UR-style runtime mode parameters (`use_mock_sim`, `headless_mode`, `launch_session_manager`).
4. Add `mission_mode` runtime parameter.
5. Keep stack-level orchestration launch files in uas_bringup.

## Step 8: System Validation in Unity3D

1. Run rectangle mission end-to-end.
2. Run star mission and verify waypoint order.
3. Run patrol mission with loop/time limits.
4. Test connection drop and verify abort to safe landing.

## 6. Suggested Package File Layout (C++ + ROS2 Launch)

```text
udacidrone_driver_cpp/
	include/udacidrone_driver_cpp/
		telemetry_types.hpp
		simulator_connection.hpp
		mavlink_translator.hpp
		uas_state_repository.hpp
		uas_command_service.hpp
		driver_session_manager.hpp
		command_gatekeeper.hpp
		driver_health_monitor.hpp
		uas_driver_node.hpp
	src/
		simulator_connection.cpp
		mavlink_translator.cpp
		uas_state_repository.cpp
		uas_command_service.cpp
		driver_session_manager.cpp
		command_gatekeeper.cpp
		driver_health_monitor.cpp
		uas_driver_node.cpp
		main.cpp
	launch/
		udacidrone_driver.launch.py
		udacidrone_driver_mock.launch.py
	config/
		driver_params.yaml
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
	 - /uas/armed
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
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

Run only the driver first:

```bash
ros2 run udacidrone_driver_cpp uas_driver_node --ros-args -p connection_uri:=tcp:127.0.0.1:5760 -p is_px4:=false
```

Run driver using launch file (recommended):

```bash
ros2 launch udacidrone_driver_cpp udacidrone_driver.launch.py \
	connection_uri:=tcp:127.0.0.1:5760 \
	is_px4:=false \
	use_mock_sim:=false \
	launch_session_manager:=true \
	start_external_control_on_boot:=false
```

Run mock mode for fast checks (no Unity dependency):

```bash
ros2 launch udacidrone_driver_cpp udacidrone_driver_mock.launch.py
```

Quick launch smoke tests:

```bash
ros2 node list | grep uas_driver_node
ros2 topic list | grep /uas/
ros2 topic echo /uas/driver_health
ros2 service call /uas/arm std_srvs/srv/SetBool "{data: true}"
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

