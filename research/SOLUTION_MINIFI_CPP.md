# Backyard Flyer — MiNiFi C++ Production Pipeline (Custom Processors, Follow-Along Guide)

This document explains how to transition from the **modular C++ flight-controller core** (see `SOLUTION_MODULAR_CPP.md`) to a **MiNiFi C++ edge data pipeline** suitable for production-style deployments (factories/warehouses/connected devices), using **custom MiNiFi C++ processors**.

Scope and boundary:

- The flight controller remains the real-time actuation loop.
- MiNiFi/NiFi handle **telemetry movement and observability**, and must **not** become an actuation dependency.

Inputs analyzed for this writeup:

- `SOLUTION_DESIGN_CPP.md` (threading model + logging guidance)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md` (edge/platform layout)
- `SOLUTION_MODULAR_CPP.md` (core/controller separation and JSONL logging intent)
- `README.md` (mission + API concepts)

---

## 0) What “MiNiFi C++ production approach” means here

In production edge deployments, you generally want:

1) **Backpressure & buffering** on edge (disk-backed repositories).
2) **Routing and filtering** close to the source (reduce bandwidth).
3) **Schema/format validation** before data leaves the device.
4) A **simple handoff** to the platform (NiFi, Kafka, HTTP collectors, etc.).

MiNiFi C++ fits because it’s lightweight, resilient, and designed for edge.

---

## 1) Step 1 — Freeze your telemetry contract (controller → MiNiFi)

Before writing processors, define what your controller emits.

Recommended MVP contract: **JSON Lines** (`.jsonl`) where each line is one JSON object.

Why JSONL:

- MiNiFi can tail it safely.
- It’s append-only and works well with buffering.
- Each event is independently parseable.

### 1.1 Minimal event schema

Emit one of these event types:

- `state_transition`
- `command_waypoint`
- `position_sample` (throttle this at the source or in MiNiFi)
- `mission_summary`

Minimal fields to standardize:

```json
{
	"event_type": "state_transition",
	"t_us": 1700000000123456,
	"vehicle_id": "sim-01",
	"run_id": "2026-02-23T01-23-45Z",
	"state_from": "Takeoff",
	"state_to": "Waypoint"
}
```

For samples:

```json
{
	"event_type": "position_sample",
	"t_us": 1700000001123456,
	"vehicle_id": "sim-01",
	"position_ned": {"north": 1.2, "east": 0.3, "down": -3.0},
	"velocity_ned": {"north": 0.1, "east": 0.0, "down": 0.0}
}
```

NED reminder (from `README.md`): target altitude 3m means `down = -3.0`.

---

## 2) Step 2 — Decide the edge ingestion pattern (file vs socket)

Two common patterns:

### Option A (recommended MVP): Controller writes JSONL, MiNiFi tails

- Controller writes `Logs/telemetry.jsonl`
- MiNiFi uses built-in `TailFile` processor

Pros: simplest, durable, good offline behavior.

### Option B: Controller emits UDP/TCP, MiNiFi listens

- Controller sends JSON over local TCP/UDP
- MiNiFi uses built-in `ListenTCP` / `ListenSyslog` / etc.

Pros: lower latency, fewer disk writes.

This guide uses Option A because it’s easiest to validate.

---

## 3) Step 3 — Define the MiNiFi flow (built-in + custom)

We’ll build a flow like:

1) **TailFile** (built-in) — reads new lines from your telemetry JSONL file
2) **ParseBackyardTelemetryJson** (custom) — parse JSON, validate required fields, set attributes
3) **RateLimitBackyardTelemetry** (custom) — drop/throttle high-rate samples, always pass important events
4) **RouteOnAttribute** (built-in) — route by event_type or severity
5) **PutTCP / InvokeHTTP / Site-to-Site** (built-in) — forward to platform ingestion

Why custom processors are still useful even with many built-ins:

- Your telemetry schema is domain-specific.
- You often want domain-specific validation and minimal enrichment close to the source.
- You want domain-specific filtering (e.g., always keep state transitions; downsample position).

---

## 4) Step 4 — Implement custom MiNiFi C++ processors (follow-along)

You can implement these processors as a MiNiFi C++ extension.

Pragmatic learning approach (recommended):

- Implement them **inside** your local `forked/nifi-minifi-cpp/extensions/` so you can compile and run quickly.

Production approach (common later):

- Build an out-of-tree extension and deploy the shared library into `nifi.extension.path`.

This guide documents the in-tree approach because it’s easiest to validate.

### 4.1 Extension folder layout

Create a new extension folder in the MiNiFi C++ repo:

```text
forked/nifi-minifi-cpp/extensions/backyard_flyer/
	CMakeLists.txt
	processors/
		ParseBackyardTelemetryJson.h
		ParseBackyardTelemetryJson.cpp
		RateLimitBackyardTelemetry.h
		RateLimitBackyardTelemetry.cpp
```

Notes:

- The code below uses the same public patterns you can see in other MiNiFi processors:
	- `core::ProcessorImpl`
	- property/relationship definitions
	- `initialize()`, `onSchedule()`, `onTrigger()`
	- `REGISTER_RESOURCE(..., Processor)`

---

## 5) Custom Processor #1 — Parse + validate telemetry JSON

Goal:

- Parse each FlowFile’s content as JSON.
- Validate `event_type` and `t_us` exist.
- Copy selected fields into FlowFile attributes so the rest of the flow can route cheaply.

### 5.1 `processors/ParseBackyardTelemetryJson.h`

```cpp
#pragma once

#include <array>
#include <string>
#include <string_view>

#include "core/ProcessorImpl.h"
#include "core/PropertyDefinitionBuilder.h"
#include "minifi-cpp/core/RelationshipDefinition.h"

namespace org::apache::nifi::minifi::extensions::backyard_flyer {

class ParseBackyardTelemetryJson final : public core::ProcessorImpl {
 public:
	using ProcessorImpl::ProcessorImpl;

	EXTENSIONAPI static constexpr const char* Description =
			"Parses BackyardFlyer telemetry JSON (one JSON object per FlowFile), validates required fields, and sets routing attributes.";

	EXTENSIONAPI static constexpr auto VehicleId = core::PropertyDefinitionBuilder<>::createProperty("VehicleId")
			.withDescription("Default vehicle_id to set when missing")
			.isRequired(false)
			.build();

	EXTENSIONAPI static constexpr auto RunId = core::PropertyDefinitionBuilder<>::createProperty("RunId")
			.withDescription("Default run_id to set when missing")
			.isRequired(false)
			.build();

	EXTENSIONAPI static constexpr auto Properties = std::to_array<core::PropertyReference>({VehicleId, RunId});

	EXTENSIONAPI static constexpr auto Success = core::RelationshipDefinition{"success", "Parsed and validated telemetry"};
	EXTENSIONAPI static constexpr auto Failure = core::RelationshipDefinition{"failure", "Invalid JSON or missing required fields"};
	EXTENSIONAPI static constexpr auto Relationships = std::array{Success, Failure};

	EXTENSIONAPI static constexpr bool SupportsDynamicProperties = false;
	EXTENSIONAPI static constexpr bool SupportsDynamicRelationships = false;
	EXTENSIONAPI static constexpr core::annotation::Input InputRequirement = core::annotation::Input::INPUT_REQUIRED;
	EXTENSIONAPI static constexpr bool IsSingleThreaded = false;

	ADD_COMMON_VIRTUAL_FUNCTIONS_FOR_PROCESSORS

	void initialize() override;
	void onSchedule(core::ProcessContext& context, core::ProcessSessionFactory& session_factory) override;
	void onTrigger(core::ProcessContext& context, core::ProcessSession& session) override;

 private:
	std::string default_vehicle_id_;
	std::string default_run_id_;
};

}  // namespace org::apache::nifi::minifi::extensions::backyard_flyer
```

### 5.2 `processors/ParseBackyardTelemetryJson.cpp`

```cpp
#include "ParseBackyardTelemetryJson.h"

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "core/Resource.h"
#include "minifi-cpp/core/ProcessContext.h"
#include "core/ProcessSession.h"
#include "core/logging/LoggerFactory.h"
#include "io/InputStream.h"
#include "rapidjson/document.h"

namespace org::apache::nifi::minifi::extensions::backyard_flyer {

namespace {
class ReadFlowFileToStringCallback {
 public:
	ReadFlowFileToStringCallback(uint64_t expected_size, std::string& out)
			: expected_size_(expected_size), out_(out) {}

	int64_t operator()(const std::shared_ptr<io::InputStream>& stream) {
		out_.clear();

		std::vector<std::byte> buffer;
		buffer.resize(expected_size_);

		const auto n = stream->read(buffer);
		if (io::isError(n) || n != expected_size_) {
			return -1;
		}

		out_.assign(reinterpret_cast<const char*>(buffer.data()), buffer.size());
		return static_cast<int64_t>(out_.size());
	}

 private:
	uint64_t expected_size_{0};
	std::string& out_;
};

std::optional<std::string> getStringMember(const rapidjson::Document& d, const char* key) {
	if (!d.HasMember(key) || !d[key].IsString()) return std::nullopt;
	return std::string(d[key].GetString());
}

std::optional<uint64_t> getUint64Member(const rapidjson::Document& d, const char* key) {
	if (!d.HasMember(key) || !d[key].IsUint64()) return std::nullopt;
	return d[key].GetUint64();
}

}  // namespace

void ParseBackyardTelemetryJson::initialize() {
	setSupportedProperties(Properties);
	setSupportedRelationships(Relationships);
}

void ParseBackyardTelemetryJson::onSchedule(core::ProcessContext& context, core::ProcessSessionFactory&) {
	default_vehicle_id_ = context.getProperty(VehicleId).value_or("");
	default_run_id_ = context.getProperty(RunId).value_or("");
}

void ParseBackyardTelemetryJson::onTrigger(core::ProcessContext& context, core::ProcessSession& session) {
	auto flow_file = session.get();
	if (!flow_file) return;

	std::string content;
	ReadFlowFileToStringCallback cb(flow_file->getSize(), content);
	session.read(flow_file, std::ref(cb));

	rapidjson::Document doc;
	if (doc.Parse<rapidjson::kParseStopWhenDoneFlag>(content.c_str()).HasParseError() || !doc.IsObject()) {
		session.transfer(flow_file, Failure);
		return;
	}

	const auto event_type = getStringMember(doc, "event_type");
	const auto t_us = getUint64Member(doc, "t_us");
	if (!event_type || !t_us) {
		session.transfer(flow_file, Failure);
		return;
	}

	// Extract optional routing keys
	const auto vehicle_id = getStringMember(doc, "vehicle_id").value_or(default_vehicle_id_);
	const auto run_id = getStringMember(doc, "run_id").value_or(default_run_id_);

	session.putAttribute(*flow_file, "telemetry.event_type", *event_type);
	session.putAttribute(*flow_file, "telemetry.t_us", std::to_string(*t_us));
	if (!vehicle_id.empty()) session.putAttribute(*flow_file, "telemetry.vehicle_id", vehicle_id);
	if (!run_id.empty()) session.putAttribute(*flow_file, "telemetry.run_id", run_id);

	// Convenience: mark JSON content
	session.putAttribute(*flow_file, "mime.type", "application/json");

	session.transfer(flow_file, Success);
}

REGISTER_RESOURCE(ParseBackyardTelemetryJson, Processor);

}  // namespace org::apache::nifi::minifi::extensions::backyard_flyer
```

Implementation notes:

- This keeps validation minimal: it’s easy to extend with more required fields per `event_type`.
- We store `telemetry.event_type` and `telemetry.t_us` as attributes for cheap routing.

---

## 6) Custom Processor #2 — Rate-limit / filter high-rate samples

Goal:

- Always pass `state_transition`, `command_waypoint`, `mission_summary`.
- Downsample `position_sample` to a maximum rate (e.g., 5 Hz) to reduce bandwidth.

### 6.1 `processors/RateLimitBackyardTelemetry.h`

```cpp
#pragma once

#include <array>
#include <chrono>
#include <string>

#include "core/ProcessorImpl.h"
#include "core/PropertyDefinitionBuilder.h"
#include "minifi-cpp/core/RelationshipDefinition.h"

namespace org::apache::nifi::minifi::extensions::backyard_flyer {

class RateLimitBackyardTelemetry final : public core::ProcessorImpl {
 public:
	using ProcessorImpl::ProcessorImpl;

	EXTENSIONAPI static constexpr const char* Description =
			"Downsamples high-rate telemetry events (like position_sample) while always passing mission-critical events.";

	EXTENSIONAPI static constexpr auto MaxPositionHz = core::PropertyDefinitionBuilder<>::createProperty("MaxPositionHz")
			.withDescription("Maximum rate for position_sample events (Hz)")
			.isRequired(true)
			.withDefaultValue("5")
			.build();

	EXTENSIONAPI static constexpr auto Properties = std::to_array<core::PropertyReference>({MaxPositionHz});

	EXTENSIONAPI static constexpr auto Success = core::RelationshipDefinition{"success", "Forwarded"};
	EXTENSIONAPI static constexpr auto Dropped = core::RelationshipDefinition{"dropped", "Intentionally dropped by rate limiting"};
	EXTENSIONAPI static constexpr auto Failure = core::RelationshipDefinition{"failure", "Missing attributes or invalid config"};
	EXTENSIONAPI static constexpr auto Relationships = std::array{Success, Dropped, Failure};

	EXTENSIONAPI static constexpr bool SupportsDynamicProperties = false;
	EXTENSIONAPI static constexpr bool SupportsDynamicRelationships = false;
	EXTENSIONAPI static constexpr core::annotation::Input InputRequirement = core::annotation::Input::INPUT_REQUIRED;
	EXTENSIONAPI static constexpr bool IsSingleThreaded = true;  // uses internal state

	ADD_COMMON_VIRTUAL_FUNCTIONS_FOR_PROCESSORS

	void initialize() override;
	void onSchedule(core::ProcessContext& context, core::ProcessSessionFactory& session_factory) override;
	void onTrigger(core::ProcessContext& context, core::ProcessSession& session) override;

 private:
	std::chrono::steady_clock::time_point last_position_forward_{std::chrono::steady_clock::time_point::min()};
	std::chrono::steady_clock::duration min_position_period_{std::chrono::milliseconds(200)};
};

}  // namespace org::apache::nifi::minifi::extensions::backyard_flyer
```

### 6.2 `processors/RateLimitBackyardTelemetry.cpp`

```cpp
#include "RateLimitBackyardTelemetry.h"

#include <cmath>
#include <string>

#include "core/Resource.h"
#include "minifi-cpp/core/ProcessContext.h"
#include "core/ProcessSession.h"

namespace org::apache::nifi::minifi::extensions::backyard_flyer {

void RateLimitBackyardTelemetry::initialize() {
	setSupportedProperties(Properties);
	setSupportedRelationships(Relationships);
}

void RateLimitBackyardTelemetry::onSchedule(core::ProcessContext& context, core::ProcessSessionFactory&) {
	const auto max_hz_str = context.getProperty(MaxPositionHz).value_or("5");
	const double max_hz = std::max(0.1, std::stod(max_hz_str));
	const double seconds = 1.0 / max_hz;
	min_position_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
}

void RateLimitBackyardTelemetry::onTrigger(core::ProcessContext&, core::ProcessSession& session) {
	auto flow_file = session.get();
	if (!flow_file) return;

	std::string event_type;
	if (!flow_file->getAttribute("telemetry.event_type", event_type)) {
		session.transfer(flow_file, Failure);
		return;
	}

	// Always forward mission-critical events
	if (event_type == "state_transition" || event_type == "command_waypoint" || event_type == "mission_summary") {
		session.transfer(flow_file, Success);
		return;
	}

	// Rate-limit position samples
	if (event_type == "position_sample") {
		const auto now = std::chrono::steady_clock::now();
		if (last_position_forward_ == std::chrono::steady_clock::time_point::min() || now - last_position_forward_ >= min_position_period_) {
			last_position_forward_ = now;
			session.transfer(flow_file, Success);
		} else {
			session.transfer(flow_file, Dropped);
		}
		return;
	}

	// Default: forward everything else
	session.transfer(flow_file, Success);
}

REGISTER_RESOURCE(RateLimitBackyardTelemetry, Processor);

}  // namespace org::apache::nifi::minifi::extensions::backyard_flyer
```

Production note:

- This processor is marked `IsSingleThreaded = true` because it keeps state.
- If you need higher throughput later, you can shard by `vehicle_id` or use state storage.

---

## 7) Extension CMake (build your custom processors)

### 7.1 `extensions/backyard_flyer/CMakeLists.txt`

This follows the extension pattern used by other MiNiFi extensions.

```cmake
if (NOT (ENABLE_ALL OR ENABLE_BACKYARD_FLYER))
	return()
endif()

include(${CMAKE_SOURCE_DIR}/extensions/ExtensionHeader.txt)

file(GLOB SOURCES "processors/*.cpp")

add_minifi_library(minifi-backyard-flyer SHARED ${SOURCES})

target_include_directories(minifi-backyard-flyer PUBLIC "${CMAKE_SOURCE_DIR}/extensions/backyard_flyer")
target_link_libraries(minifi-backyard-flyer ${LIBMINIFI})

register_extension(minifi-backyard-flyer "BACKYARD FLYER EXTENSIONS" BACKYARD-FLYER-EXTENSIONS
	"Custom processors for BackyardFlyer telemetry" "")
```

Build notes:

- When you build MiNiFi C++, this will produce a shared library in the extensions output path.
- Ensure `nifi.extension.path` points to the directory containing the built `.so`.

### 7.2 Build + install MiNiFi C++ (commands)

MiNiFi’s top-level build automatically discovers extension folders under `extensions/*` that contain a `CMakeLists.txt` (see the main `CMakeLists.txt` in the MiNiFi repo).

To build quickly without adding a new CMake option, use `ENABLE_ALL=ON`:

```sh
cd /home/james/src/forked/nifi-minifi-cpp

cmake -S . -B build -DENABLE_ALL=ON
cmake --build build -j

# optional: install to a clean prefix
cmake --install build --prefix ./install
```

Where the extension library ends up:

- In an installed layout, shared libraries are typically under something like:
	- `./install/lib/nifi-minifi-cpp/extensions/`

Point MiNiFi at that directory in `minifi.properties`:

- `nifi.extension.path=<path-to-extensions-directory>`

Tip:

- If you’re running MiNiFi from a build tree rather than an install tree, you can still set `nifi.extension.path` to the directory that contains `libminifi-backyard-flyer.so`.

---

## 8) Step 5 — Wire the MiNiFi flow (config.yml)

Below is a conceptual `config.yml` that:

- tails your JSONL file
- parses and sets attributes
- rate-limits samples
- routes by event type
- forwards using `PutTCP` (as an example)

### 8.1 Example `edge/minifi-cpp/conf/config.yml`

This YAML shape can vary across MiNiFi versions, so treat it as a template to adapt.

Also: the exact `class:` string for custom processors depends on how your MiNiFi build registers the processor. If MiNiFi fails to instantiate the processor, check the agent logs/manifest for the registered name and update the `class:` accordingly.

```yaml
Flow Controller:
	name: BackyardFlyerTelemetry

Processors:
	- name: TailTelemetry
		class: org.apache.nifi.minifi.processors.TailFile
		scheduling strategy: TIMER_DRIVEN
		scheduling period: 1 sec
		Properties:
			File Name: /var/log/backyard/telemetry.jsonl
			Delimiter: "\n"

	- name: ParseTelemetry
		class: org.apache.nifi.minifi.extensions.backyard_flyer.ParseBackyardTelemetryJson
		scheduling strategy: EVENT_DRIVEN
		Properties:
			VehicleId: sim-01
			RunId: "${now():format('yyyy-MM-dd''T''HH-mm-ss''Z'')}"

	- name: RateLimit
		class: org.apache.nifi.minifi.extensions.backyard_flyer.RateLimitBackyardTelemetry
		scheduling strategy: EVENT_DRIVEN
		Properties:
			MaxPositionHz: "5"

	- name: Route
		class: org.apache.nifi.minifi.processors.RouteOnAttribute
		scheduling strategy: EVENT_DRIVEN
		Properties:
			state: "${telemetry.event_type:equals('state_transition')}"
			cmd: "${telemetry.event_type:equals('command_waypoint')}"
			sample: "${telemetry.event_type:equals('position_sample')}"

	- name: ForwardTCP
		class: org.apache.nifi.minifi.processors.PutTCP
		scheduling strategy: EVENT_DRIVEN
		Properties:
			Hostname: nifi-edge-gateway.local
			Port: "9000"
			Connection Per FlowFile: "false"

Connections:
	- name: c1
		source name: TailTelemetry
		destination name: ParseTelemetry

	- name: c2
		source name: ParseTelemetry
		source relationship name: success
		destination name: RateLimit

	- name: c3
		source name: RateLimit
		source relationship name: success
		destination name: Route

	- name: c4
		source name: Route
		source relationship name: state
		destination name: ForwardTCP

	- name: c5
		source name: Route
		source relationship name: cmd
		destination name: ForwardTCP

	- name: c6
		source name: Route
		source relationship name: sample
		destination name: ForwardTCP
```

Alternative forwarding options:

- Use `InvokeHTTP` to POST to an HTTP collector.
- Use Site-to-Site Remote Process Groups to send to NiFi (common in NiFi architectures).
- Use `PublishKafka` if Kafka is your ingestion backbone.

---

## 9) Step 6 — Validate the pipeline end-to-end

### 9.1 Local validation without a platform

1) Create a sample telemetry file:

```sh
mkdir -p /var/log/backyard
cat > /var/log/backyard/telemetry.jsonl << 'EOF'
{"event_type":"state_transition","t_us":1,"vehicle_id":"sim-01","state_from":"Manual","state_to":"Arming"}
{"event_type":"position_sample","t_us":2,"vehicle_id":"sim-01","position_ned":{"north":0,"east":0,"down":-3}}
EOF
```

2) Start MiNiFi C++ with your `config.yml` and extension loaded.

3) Confirm in logs that:

- `ParseBackyardTelemetryJson` routes to `success`.
- `RateLimitBackyardTelemetry` forwards/drops as expected.

### 9.2 Validation against your controller

Once your controller writes JSONL:

- Confirm files are appended, not rewritten.
- Confirm MiNiFi keeps state across restarts (TailFile state manager).
- Confirm you see state transitions and waypoint commands forwarded even if you downsample position.

---

## 10) Production hardening checklist (what to add next)

Keep this minimal at first; add pieces only when needed:

1) **Durability**
	 - disk-backed repositories sized for your worst offline window

2) **Security**
	 - TLS for forwarding
	 - credentials in env/secret store, not in repo

3) **Observability**
	 - processor metrics enabled
	 - drop counters (dropped samples) exported

4) **Schema evolution**
	 - version your telemetry events (e.g., `telemetry.schema_version`)

5) **Don’t block the flight loop**
	 - controller logs must be non-blocking (queue + logger thread)
	 - MiNiFi failures must not affect control

