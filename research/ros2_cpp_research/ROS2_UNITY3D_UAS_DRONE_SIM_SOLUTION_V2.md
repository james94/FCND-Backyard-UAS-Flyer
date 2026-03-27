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

# MAVLink and MAVROS packages (ROS2 Jazzy)
# Use ROS packages on Ubuntu 24.04 because libmavlink-dev may be unavailable.
RUN apt-get update && apt-get install -y --no-install-recommends \
		ros-jazzy-mavlink \
		ros-jazzy-libmavconn \
		ros-jazzy-mavros-msgs \
		ros-jazzy-mavros \
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

cd ~/src/FCND-Backyard-UAS-Flyer/src/infrastructure

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

For MAVLink C++ validation in the container:

```bash
apt-cache search mavlink | grep -E "mavlink|mavros|mavconn"
dpkg -l | grep -E "ros-jazzy-(mavlink|mavros|mavros-msgs|libmavconn)"
ls /opt/ros/jazzy/include/mavros_msgs/mavlink_convert.hpp
```

Note: Some Jazzy images do not expose `mavlink/v2.0/common/mavlink.h` on the default include path. The driver implementation should rely on `mavros_msgs/mavlink_convert.hpp` and `libmavconn` headers instead of directly including `mavlink.h`.
Important: avoid including `mavros/mavros_uas.hpp` in low-level driver translation units unless you explicitly need MAVROS UAS helpers. That header transitively depends on `tf2_ros/buffer.hpp`. If you do include it, ensure `tf2_ros` is installed and added to package dependencies.

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
ros2 pkg create --build-type ament_cmake uas_mission_core --dependencies rclcpp std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs tf2_ros moveit_core moveit_ros_planning moveit_ros_planning_interface
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
2. Read MAVLink packets through libmavconn
3. Write outbound MAVLink packets through libmavconn
4. Surface connection health

Suggested interface:

```cpp
class SimulatorConnection
{
public:
	bool connect(const std::string &uri);
	void disconnect();
	bool isConnected() const;
	bool readMessage(mavros_msgs::msg::Mavlink &out);
	bool sendMessage(const mavros_msgs::msg::Mavlink &msg);

private:
	mavconn::MAVConnInterface::Ptr link_;
	std::mutex rx_mutex_;
	std::deque<mavros_msgs::msg::Mavlink> rx_queue_;
	std::atomic<bool> connected_{false};
};
```

### Class B: MavlinkTranslator

Responsibility:

1. Decode `mavros_msgs::msg::Mavlink` to typed telemetry structs
2. Encode command structs to `mavros_msgs::msg::Mavlink`
3. Hide MAVLink details from ROS2 application logic while using mavros conversion utilities

Suggested interface:

```cpp
struct UasStateTelemetry;
struct UasPositionTelemetry;
struct UasVelocityTelemetry;

class MavlinkTranslator
{
public:
	bool decode(const mavros_msgs::msg::Mavlink &msg);
	bool hasState() const;
	UasStateTelemetry takeState();
	bool hasPosition() const;
	UasPositionTelemetry takePosition();
	bool hasVelocity() const;
	UasVelocityTelemetry takeVelocity();

	mavros_msgs::msg::Mavlink encodeArm(bool arm);
	mavros_msgs::msg::Mavlink encodeTakeControl();
	mavros_msgs::msg::Mavlink encodeReleaseControl();
	mavros_msgs::msg::Mavlink encodeTakeoff(float altitude_m);
	mavros_msgs::msg::Mavlink encodeLand();
	mavros_msgs::msg::Mavlink encodeCmdPosition(float n, float e, float d, float yaw);

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

#include <algorithm>
#include <deque>
#include <mutex>
#include <string>

#include <mavconn/interface.hpp>
#include <mavros_msgs/mavlink_convert.hpp>

namespace udacidrone_driver_cpp
{
namespace
{
constexpr uint8_t kCompIdOnboardComputer = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER
}

bool SimulatorConnection::connect(const std::string &uri)
{
	// Keep UdaciDrone-style URI input, adapt to libmavconn URL format.
	disconnect();

	std::string mavconn_url = uri;
	if (uri.rfind("tcp:", 0) == 0)
	{
		mavconn_url = "tcp://" + uri.substr(4);
	}

	try
	{
		link_ = mavconn::MAVConnInterface::open_url(
			mavconn_url,
			1,
			kCompIdOnboardComputer,
			[this](const mavlink::mavlink_message_t *message, const mavconn::Framing framing)
			{
				mavros_msgs::msg::Mavlink ros_msg;
				if (!mavros_msgs::mavlink::convert(*message, ros_msg, static_cast<uint8_t>(framing)))
				{
					return;
				}
				std::lock_guard<std::mutex> lock(rx_mutex_);
				rx_queue_.push_back(std::move(ros_msg));
			},
			[this]()
			{
				connected_.store(false);
			});
	}
	catch (...)
	{
		link_.reset();
		connected_.store(false);
		return false;
	}

	connected_.store(static_cast<bool>(link_) && link_->is_open());
	return connected_.load();
}

void SimulatorConnection::disconnect()
{
	if (link_)
	{
		link_->close();
		link_.reset();
	}
	connected_.store(false);
	std::lock_guard<std::mutex> lock(rx_mutex_);
	rx_queue_.clear();
}

bool SimulatorConnection::isConnected() const
{
	return connected_.load() && link_ && link_->is_open();
}

bool SimulatorConnection::readMessage(mavros_msgs::msg::Mavlink &out)
{
	if (!isConnected())
	{
		return false;
	}
	std::lock_guard<std::mutex> lock(rx_mutex_);
	if (rx_queue_.empty())
	{
		return false;
	}
	out = std::move(rx_queue_.front());
	rx_queue_.pop_front();
	return true;
}

bool SimulatorConnection::sendMessage(const mavros_msgs::msg::Mavlink &msg)
{
	if (!isConnected())
	{
		return false;
	}
	mavlink::mavlink_message_t wire_msg;
	if (!mavros_msgs::mavlink::convert(msg, wire_msg))
	{
		return false;
	}
	link_->send_message(&wire_msg);
	return true;
}
}  // namespace udacidrone_driver_cpp
```

```cpp
// src/mavlink_translator.cpp
#include "udacidrone_driver_cpp/mavlink_translator.hpp"

#include <cmath>

#if __has_include(<mavlink/v2.0/mavlink.h>)
extern "C" {
#ifndef MAVLINK_DIALECT
#define MAVLINK_DIALECT common
#endif
#include <mavlink/v2.0/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#elif __has_include(<mavlink/v2.0/common/mavlink.h>)
extern "C" {
#include <mavlink/v2.0/common/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#elif __has_include(<mavconn/mavlink_dialect.hpp>)
#include <mavconn/mavlink_dialect.hpp>
#elif __has_include(<mavlink/v2.0/minimal/mavlink.h>)
extern "C" {
#include <mavlink/v2.0/minimal/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#elif __has_include(<mavlink/common/mavlink.h>)
extern "C" {
#include <mavlink/common/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#elif __has_include(<common/mavlink.h>)
extern "C" {
#include <common/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#else
#error "No MAVLink dialect header found. Install ros-jazzy-mavlink and ros-jazzy-libmavconn."
#endif

// Prevent macro collision with MAVLink C++ headers/types in downstream includes.
#ifdef MAVLINK_VERSION
#undef MAVLINK_VERSION
#endif

#include <mavros_msgs/mavlink_convert.hpp>

namespace udacidrone_driver_cpp
{
namespace
{
constexpr uint8_t kSysId = 255;
constexpr uint8_t kCompId = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER
constexpr uint8_t kTargetSys = 1;
constexpr uint8_t kTargetComp = 1;

constexpr float kMainModeManual = 1.0F;
constexpr float kMainModeOffboard = 6.0F;

constexpr uint16_t kMaskIgnorePosition = 0x007;
constexpr uint16_t kMaskIgnoreVelocity = 0x038;
constexpr uint16_t kMaskIgnoreAcceleration = 0x1C0;
constexpr uint16_t kMaskIgnoreYaw = 0x400;
constexpr uint16_t kMaskIgnoreYawRate = 0x800;
constexpr uint16_t kMaskIsTakeoff = 0x1000;
constexpr uint16_t kMaskIsLand = 0x2000;

mavros_msgs::msg::Mavlink toRosMavlink(const mavlink::mavlink_message_t &msg)
{
	mavros_msgs::msg::Mavlink out;
	(void)mavros_msgs::mavlink::convert(msg, out);
	return out;
}

mavros_msgs::msg::Mavlink encodeCommandLong(
	uint16_t command,
	float p1,
	float p2 = 0.0F,
	float p3 = 0.0F,
	float p4 = 0.0F,
	float p5 = 0.0F,
	float p6 = 0.0F,
	float p7 = 0.0F)
{
	mavlink::mavlink_message_t msg;
	::mavlink_msg_command_long_pack(
		kSysId,
		kCompId,
		&msg,
		kTargetSys,
		kTargetComp,
		command,
		0,
		p1,
		p2,
		p3,
		p4,
		p5,
		p6,
		p7);
	return toRosMavlink(msg);
}

mavros_msgs::msg::Mavlink encodeSetPositionTarget(
	uint16_t mask,
	float n,
	float e,
	float d,
	float vn,
	float ve,
	float vd,
	float yaw,
	float yaw_rate)
{
	mavlink::mavlink_message_t msg;
	::mavlink_msg_set_position_target_local_ned_pack(
		kSysId,
		kCompId,
		&msg,
		0,
		kTargetSys,
		kTargetComp,
		MAV_FRAME_LOCAL_NED,
		mask,
		n,
		e,
		d,
		vn,
		ve,
		vd,
		0.0F,
		0.0F,
		0.0F,
		yaw,
		yaw_rate);
	return toRosMavlink(msg);
}
}  // namespace

bool MavlinkTranslator::decode(const mavros_msgs::msg::Mavlink &msg)
{
	has_state_ = false;
	has_position_ = false;
	has_velocity_ = false;

	mavlink::mavlink_message_t wire;
	if (!mavros_msgs::mavlink::convert(msg, wire))
	{
		return false;
	}

	switch (wire.msgid)
	{
		case MAVLINK_MSG_ID_HEARTBEAT:
		{
			mavlink_heartbeat_t hb;
			::mavlink_msg_heartbeat_decode(&wire, &hb);

			state_.stamp_sec = 0.0;
			state_.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
			const uint32_t main_mode = (hb.custom_mode & 0x000F0000U) >> 16U;
			state_.guided = (main_mode == static_cast<uint32_t>(kMainModeOffboard));
			state_.status = static_cast<int32_t>(hb.system_status);
			has_state_ = true;
			break;
		}
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
			mavlink_local_position_ned_t lp;
			::mavlink_msg_local_position_ned_decode(&wire, &lp);
			const double t = static_cast<double>(lp.time_boot_ms) / 1000.0;

			position_.stamp_sec = t;
			position_.north = lp.x;
			position_.east = lp.y;
			position_.down = lp.z;
			has_position_ = true;

			velocity_.stamp_sec = t;
			velocity_.vn = lp.vx;
			velocity_.ve = lp.vy;
			velocity_.vd = lp.vz;
			has_velocity_ = true;
			break;
		}
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		{
			mavlink_global_position_int_t gp;
			::mavlink_msg_global_position_int_decode(&wire, &gp);
			const double t = static_cast<double>(gp.time_boot_ms) / 1000.0;

			position_.stamp_sec = t;
			position_.latitude = static_cast<double>(gp.lat) / 1e7;
			position_.longitude = static_cast<double>(gp.lon) / 1e7;
			position_.altitude = static_cast<double>(gp.alt) / 1000.0;
			has_position_ = true;

			velocity_.stamp_sec = t;
			velocity_.vn = static_cast<double>(gp.vx) / 100.0;
			velocity_.ve = static_cast<double>(gp.vy) / 100.0;
			velocity_.vd = static_cast<double>(gp.vz) / 100.0;
			has_velocity_ = true;
			break;
		}
		default:
			break;
	}

	return true;
}

bool MavlinkTranslator::hasState() const { return has_state_; }
UasStateTelemetry MavlinkTranslator::takeState() { has_state_ = false; return state_; }

bool MavlinkTranslator::hasPosition() const { return has_position_; }
UasPositionTelemetry MavlinkTranslator::takePosition() { has_position_ = false; return position_; }

bool MavlinkTranslator::hasVelocity() const { return has_velocity_; }
UasVelocityTelemetry MavlinkTranslator::takeVelocity() { has_velocity_ = false; return velocity_; }

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeArm(bool arm)
{
	return encodeCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, arm ? 1.0F : 0.0F);
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeTakeControl()
{
	return encodeCommandLong(
		MAV_CMD_DO_SET_MODE,
		static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
		kMainModeOffboard,
		0.0F);
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeReleaseControl()
{
	return encodeCommandLong(
		MAV_CMD_DO_SET_MODE,
		static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
		kMainModeManual,
		0.0F);
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeTakeoff(float altitude_m)
{
	const float target_d = -std::fabs(altitude_m);
	const uint16_t mask = static_cast<uint16_t>(
		kMaskIsTakeoff | kMaskIgnoreYawRate | kMaskIgnoreYaw | kMaskIgnoreAcceleration | kMaskIgnoreVelocity);
	return encodeSetPositionTarget(mask, 0.0F, 0.0F, target_d, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeLand()
{
	const uint16_t mask = static_cast<uint16_t>(
		kMaskIsLand | kMaskIgnoreYawRate | kMaskIgnoreYaw | kMaskIgnoreAcceleration | kMaskIgnoreVelocity);
	return encodeSetPositionTarget(mask, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeCmdPosition(float n, float e, float d, float yaw)
{
	const uint16_t mask = static_cast<uint16_t>(
		kMaskIgnoreYawRate | kMaskIgnoreAcceleration | kMaskIgnoreVelocity);
	return encodeSetPositionTarget(mask, n, e, d, 0.0F, 0.0F, 0.0F, yaw, 0.0F);
}
}  // namespace udacidrone_driver_cpp
```

This translator implementation mirrors UdaciDrone Python behavior while using mavros conversion APIs:

MAVLink symbol note:

1. Functions such as `mavlink_msg_command_long_pack` and `mavlink_msg_set_position_target_local_ned_pack` come from the MAVLink C headers and are exposed as global C symbols/macros.
2. Do not prefix those APIs with `mavlink::`.
3. In ROS2 Jazzy, these symbols are provided through the installed MAVLink/libmavconn stack (which is generated from MAVLink C header sets like your `c_library_v1`), so your project can use them without vendoring `c_library_v1` directly.

1. Decodes `HEARTBEAT`, `LOCAL_POSITION_NED`, and `GLOBAL_POSITION_INT` into normalized state/position/velocity.
2. Uses `MAV_CMD_COMPONENT_ARM_DISARM` and `MAV_CMD_DO_SET_MODE` for arm/offboard/manual transitions.
3. Uses `SET_POSITION_TARGET_LOCAL_NED` masks for takeoff, land, and cmd_position semantics.
4. Uses `mavros_msgs::mavlink::convert()` for ROS<->MAVLink conversion boundaries.

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
	return connection_.sendMessage(translator_.encodeArm(true));
}

bool UasCommandService::disarm()
{
	return connection_.sendMessage(translator_.encodeArm(false));
}

bool UasCommandService::takeControl()
{
	return connection_.sendMessage(translator_.encodeTakeControl());
}

bool UasCommandService::releaseControl()
{
	return connection_.sendMessage(translator_.encodeReleaseControl());
}

bool UasCommandService::takeoff(float target_altitude_m)
{
	return connection_.sendMessage(translator_.encodeTakeoff(target_altitude_m));
}

bool UasCommandService::land()
{
	return connection_.sendMessage(translator_.encodeLand());
}

bool UasCommandService::cmdPosition(float n, float e, float d, float heading_rad)
{
	// Keep this behavior aligned with UdaciDrone: simulator may need sign inversion on d.
	if (!is_px4_)
	{
		d = -d;
	}
	return connection_.sendMessage(translator_.encodeCmdPosition(n, e, d, heading_rad));
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
	bool log_telemetry_csv_{false};
	std::string telemetry_log_csv_path_{};
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
	log_telemetry_csv_ = declare_parameter<bool>("log_telemetry_csv", false);
	telemetry_log_csv_path_ = declare_parameter<std::string>(
		"telemetry_log_csv_path", "/tmp/uas_telemetry.csv");
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
	mavros_msgs::msg::Mavlink mav_msg;
	if (!connection_.readMessage(mav_msg))
	{
		return;
	}

	const bool decoded_ok = translator_.decode(mav_msg);
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

	// Optional node-local CSV log for quick parity with udacidrone drone.py logging.
	// For full-fidelity ROS telemetry capture, prefer rosbag2 recording from launch.
	if (log_telemetry_csv_)
	{
		// Append one row: stamp,n,e,d,vn,ve,vd,lat,lon,alt,armed,connected
		// (Implementation detail intentionally omitted in this template.)
	}
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
find_package(mavlink REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(libmavconn REQUIRED)

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
	sensor_msgs
	mavlink
	mavros_msgs
	libmavconn)

install(TARGETS uas_driver_node DESTINATION lib/${PROJECT_NAME})
```

Linking note: do not add `target_link_libraries(uas_driver_node mavconn)` on Jazzy unless you have verified that raw library name exists on the linker path. `ament_target_dependencies(... libmavconn)` is the portable way because it uses the package-exported link interface.

If you keep the translator implementation above, install `ros-jazzy-mavlink`, `ros-jazzy-mavros-msgs`, and `ros-jazzy-libmavconn` in your dev image.

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
6. `record_telemetry` default `true`
7. `telemetry_record_mode` default `rosbag2`
8. `telemetry_output_dir` default `/tmp/uas_logs`
9. `telemetry_run_id` default `manual_flight`
10. `log_telemetry_csv` default `false`
11. `telemetry_log_csv_path` default `/tmp/uas_telemetry.csv`

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
6. `record_telemetry`:
If `true`, launch recording in the backend while you manually fly in Unity. This is the closest ROS2 equivalent to running `udacidrone` `drone.py` as a telemetry logger.
7. `telemetry_record_mode`:
Select recording backend. `rosbag2` is recommended because it preserves all typed ROS2 topics.
8. `telemetry_output_dir` and `telemetry_run_id`:
Control where logs are stored, e.g. `/tmp/uas_logs/manual_flight`.
9. `log_telemetry_csv` and `telemetry_log_csv_path`:
Optional node-level CSV output if you want a lightweight flat file similar to UdaciDrone text logs.

Node inclusion conditions:

1. If `use_mock_sim == true`, disable session-manager and command gating by external run-state.
2. If `launch_session_manager == true`, keep session-manager logic enabled (currently in-process in `UasDriverNode`; separate helper node is optional future work).
3. If `headless_mode == true`, auto-start external control when dependencies are ready.
4. If `record_telemetry == true`, record `/uas/*` topics while the Unity operator manually flies.

### 4.1.10 ROS2 Python Launch Scripts for udacidrone_driver_cpp

Add launch files directly in the driver package so Unity bringup is easy to test without the full mission stack.

```python
# launch/udacidrone_driver.launch.py
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
				"log_telemetry_csv": log_telemetry_csv,
				"telemetry_log_csv_path": telemetry_log_csv_path,
			}
		],
	)

	# ROS2 equivalent to running udacidrone drone.py for manual-flight telemetry capture.
	# Records key UAS topics while Unity is manually flown.
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
10. Validate telemetry recording behavior (rosbag2 and optional CSV).

### 4.1.12 Manual-Flight Backend Test Mode (UdaciDrone drone.py Equivalent)

Yes. You can test your ROS2 C++ driver the same way UdaciDrone uses `drone.py` during manual Unity flight:

1. Launch Unity simulator and fly manually with keyboard/joystick.
2. Launch ROS2 backend only (no mission state machine required).
3. Keep command services idle; use the driver as a telemetry receiver/logger.

Recommended command:

```bash
ros2 launch udacidrone_driver_cpp udacidrone_driver.launch.py \
	connection_uri:=tcp:127.0.0.1:5760 \
	use_mock_sim:=false \
	record_telemetry:=true \
	telemetry_record_mode:=rosbag2 \
	telemetry_output_dir:=/tmp/uas_logs \
	telemetry_run_id:=manual_flight
```

Where telemetry is recorded:

1. Default: rosbag2 output under `${telemetry_output_dir}/${telemetry_run_id}`.
2. Optional CSV: if `log_telemetry_csv:=true`, node writes rows to `telemetry_log_csv_path`.
3. Live verification: `ros2 topic echo /uas/global_position` and `ros2 topic echo /uas/local_position`.

This gives you the same practical validation loop as UdaciDrone manual-flight logging, but with ROS2-native typed logs that can be replayed using `ros2 bag play`.

## 4.2 uas_mission_core Class Model

This package should represent the C++ ROS2 MoveIt2 autonomy backend, not only mission-shape planners.

Use a layered autonomy model:

1. Sensor Layer (ingestion + time alignment)
2. Perception Layer (detection + localization + world-state updates)
3. Planning Layer (route + prediction + behavior + trajectory)
4. Control Layer (trajectory tracking + command publishing + safety fallback)
5. Mission Planner Layer (rectangle/star/patrol task generation)

This aligns with MoveIt2 tutorial patterns that emphasize:

1. `planning_scene_monitor::PlanningSceneMonitor` as the canonical world-state hub.
2. `planning_scene_monitor::CurrentStateMonitor` for robot state ingestion.
3. `planning_pipeline::PlanningPipeline` and `planning_interface::MotionPlanRequest` for motion planning.
4. `moveit_cpp::MoveItCpp` for C++-native orchestration entry points.

### 4.2.1 Sensor Layer

Goal: ingest telemetry from `udacidrone_driver_cpp` and normalize into autonomy-ready state.

Primary subscriptions:

1. `/uas/armed`
2. `/uas/local_position`
3. `/uas/local_velocity`
4. `/uas/global_position`
5. `/uas/driver_health`

Core interfaces:

```cpp
class ISensorIngestor
{
public:
	virtual ~ISensorIngestor() = default;
	virtual void onArmed(const std_msgs::msg::Bool &msg) = 0;
	virtual void onLocalPosition(const geometry_msgs::msg::PointStamped &msg) = 0;
	virtual void onLocalVelocity(const geometry_msgs::msg::Vector3Stamped &msg) = 0;
	virtual void onGlobalPosition(const sensor_msgs::msg::NavSatFix &msg) = 0;
	virtual void onDriverHealth(const std_msgs::msg::String &msg) = 0;
	virtual UasAutonomyObservation snapshot() const = 0;
};

class UasSensorFusionCache : public ISensorIngestor
{
public:
	void onArmed(const std_msgs::msg::Bool &msg) override;
	void onLocalPosition(const geometry_msgs::msg::PointStamped &msg) override;
	void onLocalVelocity(const geometry_msgs::msg::Vector3Stamped &msg) override;
	void onGlobalPosition(const sensor_msgs::msg::NavSatFix &msg) override;
	void onDriverHealth(const std_msgs::msg::String &msg) override;
	UasAutonomyObservation snapshot() const override;

private:
	mutable std::mutex mutex_;
	UasAutonomyObservation latest_{};
};
```

### 4.2.2 Perception Layer

Goal: convert fused telemetry and external observations into world entities and confidence-scored tracks.

Responsibilities:

1. Detection and tracking of obstacles, no-fly geofences, and dynamic entities.
2. Localization state estimation (NED-consistent for UAS stack).
3. Planning-scene synchronization for MoveIt2 consumers.

Core interfaces:

```cpp
class IPerceptionModule
{
public:
	virtual ~IPerceptionModule() = default;
	virtual PerceptionFrame update(const UasAutonomyObservation &obs) = 0;
};

class PerceptionSceneBridge
{
public:
	PerceptionSceneBridge(const rclcpp::Node::SharedPtr &node,
	                     const planning_scene_monitor::PlanningSceneMonitorPtr &psm);

	void apply(const PerceptionFrame &frame);
	WorldStateSnapshot snapshot() const;

private:
	planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};
```

### 4.2.3 Planning Layer

Organize planning into four explicit components:

1. Route Planning: global mission route over waypoints/geofences.
2. Prediction: short-horizon conflict and trajectory prediction for dynamic entities.
3. Behavior Planning: choose maneuver mode (proceed, hold, reroute, abort).
4. Trajectory Planning: produce feasible trajectory using MoveIt2 planning pipeline.

Core interfaces:

```cpp
class IRoutePlanner
{
public:
	virtual ~IRoutePlanner() = default;
	virtual RoutePlan computeRoute(const MissionRequest &req,
	                               const WorldStateSnapshot &world) = 0;
};

class IPredictionModule
{
public:
	virtual ~IPredictionModule() = default;
	virtual PredictionBundle predict(const WorldStateSnapshot &world,
	                                 rclcpp::Time stamp) = 0;
};

class IBehaviorPlanner
{
public:
	virtual ~IBehaviorPlanner() = default;
	virtual BehaviorDecision decide(const RoutePlan &route,
	                               const PredictionBundle &pred,
	                               const UasAutonomyObservation &obs) = 0;
};

class ITrajectoryPlanner
{
public:
	virtual ~ITrajectoryPlanner() = default;
	virtual TrajectoryPlan plan(const BehaviorDecision &decision,
	                           const UasAutonomyObservation &obs) = 0;
};

class MoveItTrajectoryPlanner : public ITrajectoryPlanner
{
public:
	MoveItTrajectoryPlanner(const rclcpp::Node::SharedPtr &node,
	                      const planning_scene_monitor::PlanningSceneMonitorPtr &psm);

	TrajectoryPlan plan(const BehaviorDecision &decision,
	                   const UasAutonomyObservation &obs) override;

private:
	planning_scene_monitor::PlanningSceneMonitorPtr psm_;
	planning_pipeline::PlanningPipelinePtr planning_pipeline_;
};
```

### 4.2.4 Control Layer

Goal: convert planned trajectory to safe UAS commands and publish through driver command interfaces.

Responsibilities:

1. Trajectory tracking and command-rate control.
2. Safety gating and fallback (hold, land, abort).
3. Publishing command goals to `/uas/cmd_position` and command services.

Core interfaces:

```cpp
class IController
{
public:
	virtual ~IController() = default;
	virtual ControlCommand compute(const TrajectoryPlan &traj,
	                              const UasAutonomyObservation &obs,
	                              rclcpp::Duration dt) = 0;
};

class UasControlExecutor
{
public:
	explicit UasControlExecutor(rclcpp::Node &node);
	bool send(const ControlCommand &cmd);
	bool emergencyLand();
};
```

### 4.2.5 Mission Planner Layer

Keep mission-shape planning but position it as an upstream producer to the autonomy stack.

```cpp
class IMissionPlanner
{
public:
	virtual ~IMissionPlanner() = default;
	virtual std::vector<Waypoint> buildPlan(const MissionRequest &request) = 0;
};
```

Implementations:

1. RectangleMissionPlanner
2. StarMissionPlanner
3. PatrolMissionPlanner

Supporting classes:

1. GeofenceValidator
2. WaypointAcceptancePolicy
3. MissionPlanSerializer

### 4.2.6 MoveIt2 Autonomy Node Composition

Suggested node composition for `uas_mission_core`:

```cpp
class UasAutonomyNode : public rclcpp::Node
{
public:
	explicit UasAutonomyNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

private:
	void autonomyTick();

	std::shared_ptr<UasSensorFusionCache> sensor_cache_;
	std::shared_ptr<IPerceptionModule> perception_;
	std::shared_ptr<PerceptionSceneBridge> scene_bridge_;
	std::shared_ptr<IRoutePlanner> route_planner_;
	std::shared_ptr<IPredictionModule> prediction_;
	std::shared_ptr<IBehaviorPlanner> behavior_planner_;
	std::shared_ptr<ITrajectoryPlanner> trajectory_planner_;
	std::shared_ptr<IController> controller_;
	std::shared_ptr<UasControlExecutor> control_executor_;
	rclcpp::TimerBase::SharedPtr autonomy_timer_;
};
```

Recommended default execution order per tick:

1. Sensor snapshot
2. Perception update
3. Route planning
4. Prediction
5. Behavior decision
6. Trajectory planning
7. Control command output

### 4.2.7 MoveIt2 Tutorial Design Anchors

While implementing this stack, mirror these patterns from your local `moveit2_tutorials` fork:

1. Planning scene ownership and synchronization from the planning scene monitor tutorials.
2. Motion plan request and planning pipeline execution from the motion planning pipeline examples.
3. C++ composition using MoveItCpp where full MoveGroup server assumptions are not required.
4. Optional hybrid-planning style split between global and local behavior when prediction complexity grows.

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

1. Implement `SimulatorConnection::connect`, `readMessage`, and `disconnect`.
2. Implement `MavlinkTranslator::decode` to map incoming MAVROS messages to typed telemetry.
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

## Step 5: Implement uas_mission_core Autonomy Layers

1. Implement Sensor Layer subscriptions and timestamp alignment from `/uas/*` telemetry topics.
2. Implement Perception Layer for detection/localization and planning-scene updates.
3. Implement Planning Layer in four modules: route, prediction, behavior, trajectory.
4. Implement Control Layer that publishes `/uas/cmd_position` and calls safety services.
5. Keep mission-shape planners (rectangle/star/patrol) as request producers into route planning.

## Step 6: Integrate SMACC2 Mission Orchestration

1. Create state classes and transitions for FCND-equivalent flow.
2. Bind SMACC2 client behaviors to autonomy node APIs (not directly to transport details).
3. Route mission requests into `uas_mission_core` autonomy pipeline and consume autonomy status events.
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
5. Run manual-flight logging mode and confirm rosbag2 capture of `/uas/global_position`.

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
	logs/
		rosbag2/
		telemetry_csv/
```

```text
uas_mission_core/
	include/uas_mission_core/
		uas_autonomy_node.hpp
		sensor/i_sensor_ingestor.hpp
		sensor/uas_sensor_fusion_cache.hpp
		perception/i_perception_module.hpp
		perception/perception_scene_bridge.hpp
		planning/i_route_planner.hpp
		planning/i_prediction_module.hpp
		planning/i_behavior_planner.hpp
		planning/i_trajectory_planner.hpp
		planning/moveit_trajectory_planner.hpp
		control/i_controller.hpp
		control/uas_control_executor.hpp
		i_mission_planner.hpp
		rectangle_mission_planner.hpp
		star_mission_planner.hpp
		patrol_mission_planner.hpp
		geofence_validator.hpp
		waypoint_acceptance_policy.hpp
	src/
		uas_autonomy_node.cpp
		sensor/uas_sensor_fusion_cache.cpp
		perception/perception_scene_bridge.cpp
		planning/route_planner.cpp
		planning/prediction_module.cpp
		planning/behavior_planner.cpp
		planning/moveit_trajectory_planner.cpp
		control/controller.cpp
		control/uas_control_executor.cpp
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
	 - /uas/global_position
	 - /uas/driver_health
	 - /autonomy/perception/world_state
	 - /autonomy/planning/behavior_decision
	 - /autonomy/planning/trajectory
	 - /autonomy/control/command_status

2. Services
	 - /uas/arm
	 - /uas/disarm
	 - /uas/take_control
	 - /uas/takeoff
	 - /uas/land
	 - /autonomy/replan
	 - /autonomy/hold
	 - /autonomy/abort

3. Command topic
	 - /uas/cmd_position

4. Optional actions (recommended for autonomy integration)
	 - /autonomy/execute_route
	 - /autonomy/execute_mission

After this baseline works, add action-based waypoint mission execution.

## 8. Build and Test Commands

Inside container:

```bash
source /opt/ros/jazzy/setup.bash
cd /opt/ws

cd /opt/ws/src/FCND-Backyard-UAS-Flyer/cpp/
rosdep install --from-paths uas_stack --ignore-src -r -y

cd /opt/ws/src/FCND-Backyard-UAS-Flyer/cpp/uas_stack
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

If build fails with `fatal error: tf2_ros/buffer.hpp: No such file or directory`, use this checklist:

1. Remove unnecessary includes of `mavros/mavros_uas.hpp` from driver source files.
2. Rebuild after `docker compose build --no-cache` so container packages match Dockerfile.
3. If `mavros_uas.hpp` is required, install and declare `tf2_ros` explicitly:
	 - apt package: `ros-jazzy-tf2-ros`
	 - CMake: `find_package(tf2_ros REQUIRED)` and add to `ament_target_dependencies(...)`.

NOTE: After running "rosdep install ....", I got the following output:

```bash
rosdep install --from-paths src --ignore-src -r -y
/usr/bin/rosdep:6: DeprecationWarning: pkg_resources is deprecated as an API. See https://setuptools.pypa.io/en/latest/pkg_resources.html
  from pkg_resources import load_entry_point
#All required rosdeps installed successfully
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
	start_external_control_on_boot:=false \
	record_telemetry:=true \
	telemetry_record_mode:=rosbag2 \
	telemetry_output_dir:=/tmp/uas_logs \
	telemetry_run_id:=manual_flight
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

Inspect recorded telemetry:

```bash
ls -lah /tmp/uas_logs/manual_flight
ros2 bag info /tmp/uas_logs/manual_flight
```

Run stack:

```bash
ros2 launch uas_bringup uas_stack.launch.py mission_mode:=rectangle
```

## 9. Acceptance Checklist (Per Problem Statement)

1. FCND baseline mission reproduced: takeoff 3 m, rectangle path, land, disarm.
2. Star mission implemented and validated.
3. Patrol mission implemented with duration/geofence safeguards.
4. Clear separation between transport adapter, sensor/perception/planning/control layers, and mission orchestration.
5. Dockerized dev workflow is reproducible on fresh machine.
6. Connection loss triggers deterministic abort/landing behavior.

## 10. Recommended Incremental Milestones

1. M1: Driver publishes telemetry only.
2. M2: Driver executes manual command services.
3. M3: Sensor + Perception layers integrated with planning scene.
4. M4: Route/Prediction/Behavior/Trajectory planning layers integrated.
5. M5: Control layer + SMACC2 FCND-equivalent orchestration complete.
6. M6: Fault handling, tests, and documentation complete.

## 11. Common Integration Pitfalls

1. Frame conversion mistakes (NED vs ENU).
2. Simulator/PX4 sign differences for altitude/down axis.
3. Race conditions from asynchronous callbacks.
4. Missing watchdog timeout behavior.
5. Tight coupling of mission logic to MAVLink parsing.
6. Stale planning scene or robot state monitor causing invalid trajectories.
7. Missing prediction stage leading to unsafe behavior in dynamic scenes.

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

