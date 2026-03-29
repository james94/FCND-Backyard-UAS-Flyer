# ROS2 Technical Concepts Glossary (Interview Prep)

This glossary is tailored to your current stack:
- ROS2 Jazzy
- C++ driver node design
- Python launch orchestration
- MAVLink and libmavconn integration
- Mission orchestration and layered autonomy architecture

## 1. Core ROS2 Concepts

1. Node: A running process that performs computation in ROS2.
2. Topic: Asynchronous pub/sub communication channel for streaming data.
3. Publisher: Writes messages to a topic.
4. Subscription: Receives messages from a topic.
5. Service: Request/response RPC pattern for short command operations.
6. Action: Long-running goal/cancel/feedback/result pattern.
7. Parameter: Runtime-configurable key/value setting on a node.
8. Message type: Strongly-typed ROS2 data schema used in topics/services/actions.
9. QoS: Quality of Service policy set controlling reliability, durability, history, and deadlines.
10. Executor: Event loop that dispatches callbacks.
11. Callback group: Execution grouping used to control callback concurrency.
12. Timer callback: Periodic callback used for loops like watchdogs and polling.
13. Composition: Running multiple ROS2 components in one process for efficiency.
14. Namespacing: Hierarchical naming for isolation and multi-robot support.
15. Remapping: Runtime remap of topic/service names without code changes.

## 2. C++ ROS2 Patterns

1. rclcpp::Node: Base C++ API class for implementing ROS2 nodes.
2. Shared ownership pattern: Use of std::shared_ptr for ROS interfaces and request/response objects.
3. Dependency injection: Passing helper classes into services or orchestrators to decouple responsibilities.
4. Single responsibility classes: Transport, translator, repository, command service, and node orchestrator are separated.
5. Watchdog loop: Periodic timeout check to detect lost telemetry and trigger degraded state.
6. Snapshot repository: Thread-safe state cache that publishes coherent telemetry snapshots.
7. Command gatekeeper: Policy layer that validates whether commands are allowed before execution.
8. Health monitor: Aggregates errors/timeouts for observability and safety decisions.
9. RAII mindset: Resource lifetime tied to object lifetime to reduce leaks and shutdown issues.
10. Thread safety: Mutex and atomic usage for shared state accessed from callbacks.

## 3. ROS2 Launch and Runtime

1. Launch file: Python-based runtime graph definition for nodes, arguments, and conditional processes.
2. DeclareLaunchArgument: Exposes runtime config from command line to launch graph.
3. LaunchConfiguration: Reads argument values at launch runtime.
4. IfCondition: Conditionally starts nodes/processes.
5. PythonExpression: Dynamic launch-time logic for derived parameters.
6. Node action: Launch action to start a ROS2 node executable.
7. ExecuteProcess: Launch action for non-node commands like ros2 bag record.
8. Bringup profile: Parameterized launch defaults representing an operating mode.
9. Headless mode: Runtime mode without simulator UI dependencies.
10. Mock mode: Runtime mode bypassing full simulator integration for quick validation.

## 4. Build and Packaging

1. colcon: ROS2 build tool for multi-package workspaces.
2. ament_cmake: CMake build system integration for ROS2 packages.
3. package.xml: Declares package metadata and dependencies.
4. find_package: CMake mechanism for discovering build dependencies.
5. ament_target_dependencies: Preferred ROS2 linkage/dependency wiring for targets.
6. Symlink install: colcon mode that symlinks artifacts for fast iteration.
7. rosdep: Dependency resolver for system packages referenced by ROS packages.
8. Overlay workspace: Layering workspace builds on top of installed ROS distributions.
9. Source setup script: Shell script that exposes package paths and environment variables.
10. Linker mismatch risk: A dependency may be exported through ament but not available under an assumed raw -l name.

## 5. MAVLink and ROS2 Integration Concepts

1. MAVLink dialect: Message set selection (for example common) used for generated headers/APIs.
2. Guarded include strategy: Multi-path header inclusion to handle environment differences.
3. MAVLINK_DIALECT macro: Compile-time switch that selects generated message set.
4. MAVLINK_VERSION macro collision: C macro conflict that can break C++ includes unless sanitized.
5. mavros_msgs::mavlink::convert: Boundary conversion between ROS message and mavlink_message_t.
6. C-style global MAVLink APIs: Generated symbols like mavlink_msg_command_long_pack and decode helpers.
7. Setpoint type mask: Bitmask controlling which setpoint fields autopilot should ignore/use.
8. MAV_FRAME_LOCAL_NED: Coordinate frame convention (North, East, Down) for local control.
9. Component ID 191: MAV_COMP_ID_ONBOARD_COMPUTER commonly used for companion computer messages.
10. PX4 vs simulator sign handling: Local down-axis sign differences can require command-side conversion.

## 6. Architecture and Design Concepts

1. Layered architecture: Transport, driver API, mission planning, mission orchestration.
2. Adapter pattern: MAVLink/transport details hidden behind simulator connection and translator interfaces.
3. Orchestration layer: State machine or supervisor that sequences mission logic.
4. Policy layer: Gatekeeper/validator that enforces safety and run-state constraints.
5. Observability: Health topic plus logs/metrics for diagnosis.
6. Determinism: Repeatable transitions and timeout behavior under async callbacks.
7. Fault containment: Errors in one component should not cascade into uncontrolled behavior.
8. Extension points: Mission planner and autonomy modules designed for future capabilities.
9. Interface contract: Stable topic/service/action schema decoupling producers and consumers.
10. Separation of concerns: Mission logic should not directly depend on transport protocol internals.

## 7. Safety and Reliability Terms

1. Fail-safe: Deterministic safe behavior on error/timeout, such as hold or land.
2. Timeout threshold: Max allowable duration since last valid telemetry frame.
3. Abort path: Explicit mechanism to cancel mission and transition to safe mode.
4. Dependency readiness gating: Delays command enablement until required services are available.
5. Command rejection reason: Structured explanation for denied commands.
6. Health degradation event: State transition when reliability metrics exceed limits.
7. Telemetry freshness: Age of last valid frame used to infer connection quality.
8. Idempotent command handling: Repeated commands should not produce unstable side effects.
9. Validation before actuation: Always check safety policy before sending control commands.
10. Recovery path: Defined behavior to resume or shut down after transient faults.

## 8. Infrastructure and DevOps Terms

1. Docker-first workflow: Reproducible environment for local and CI builds.
2. Containerized toolchain: Consistent compiler, ROS distro, and dependency versions.
3. Build reproducibility: Same source plus same image yields same build behavior.
4. CI smoke test: Fast check that package builds and key launch graphs start.
5. Artifact inspection: Checking built binaries, install tree, and launch outputs.
6. Rosbag2 logging: Topic recording for replay and debugging.
7. Launch-time telemetry recording: Starting bag recorder from launch for scenario capture.
8. Environment drift: Build issues caused by host/container version mismatch.
9. Runtime profile: Predefined launch parameter set for specific mission/testing modes.
10. Troubleshooting checklist: Ordered diagnostic sequence to reduce debugging time.

## 9. Interview-Ready Short Answers

1. Why ROS2 services and actions both exist:
Services are lightweight request/response calls; actions are for long-running goals with feedback and cancellation.
2. Why use a repository class for state:
It centralizes thread-safe state updates and provides coherent snapshots to multiple consumers.
3. Why avoid hardcoded linker flags in ROS2 packages:
ament exported dependencies are more portable across distros and packaging layouts.
4. Why keep mission logic separate from MAVLink transport:
It improves testability, maintainability, and future protocol portability.
5. Why include watchdogs in robotics drivers:
Real systems must detect stale telemetry and transition safely under communication faults.
6. How you handle simulator vs PX4 differences:
Parameterize runtime mode and isolate frame/sign differences in command translation logic.
7. Why use Docker for interview-project demos:
It reduces environment variance and makes your build/run story deterministic and reproducible.
8. How you justify architecture choices quickly:
Focus on separation of concerns, safety gating, deterministic transitions, and extension-ready interfaces.

## 10. Fast Review Checklist

1. Can you explain topic vs service vs action with one concrete example each from your stack?
Answer: A topic is async streaming, like publishing `/uas/local_position` continuously. A service is request/response, like calling `/uas/arm` once and getting success/failure. An action is long-running goal execution with feedback/cancel, like a future `/uas/fly_waypoints` mission goal.
2. Can you describe the driver data path from inbound MAVLink frame to ROS2 topic publication?
Answer: `SimulatorConnection::readMessage()` pulls a MAVROS Mavlink message, `MavlinkTranslator::decode()` parses HEARTBEAT and position messages, `UasStateRepository` stores thread-safe snapshots, and `UasDriverNode::publishTelemetry()` publishes `/uas/armed`, `/uas/local_position`, `/uas/local_velocity`, and `/uas/global_position`.
3. Can you describe the command path from ROS2 callback to outbound MAVLink packet?
Answer: A callback like `/uas/cmd_position` or `/uas/takeoff` enters `UasDriverNode`, passes through `CommandGatekeeper` and health reporting, calls `UasCommandService`, which asks `MavlinkTranslator` to encode MAVLink, and `SimulatorConnection::sendMessage()` transmits over libmavconn.
4. Can you explain why MAVLink dialect/header strategy matters in Jazzy?
Answer: Jazzy environments can expose different MAVLink header layouts and default dialects. Using guarded includes and forcing `MAVLINK_DIALECT common` ensures required symbols/messages are declared. Without this, compile errors appear even if MAVLink packages are installed.
5. Can you explain what ament_target_dependencies solves versus raw target_link_libraries?
Answer: `ament_target_dependencies` uses package-exported include/link interfaces and is portable across ROS environments. Raw `target_link_libraries` can fail when library names differ from assumed `-l` names, which happened with `mavconn`.
6. Can you explain your timeout and fail-safe behavior in under 60 seconds?
Answer: A watchdog timer checks age of last received telemetry. If timeout is exceeded, connection state is marked false and health status degrades. Command gating can reject unsafe commands, and mission orchestration can transition to abort/land behavior.
7. Can you explain how launch arguments map to runtime behavior and test modes?
Answer: Launch arguments like `connection_uri`, `is_px4`, `use_mock_sim`, and `start_external_control_on_boot` directly map to node parameters that change transport, sign conventions, and gating behavior. `record_telemetry` and related args control rosbag logging mode.
8. Can you explain your package boundaries and why they reduce technical risk?
Answer: `udacidrone_driver_cpp` handles transport and control APIs, `uas_mission_core` handles planning logic, and `sm_uas_missions` handles state transitions. This separation prevents protocol details from leaking into mission logic, improves testability, and localizes failures.

