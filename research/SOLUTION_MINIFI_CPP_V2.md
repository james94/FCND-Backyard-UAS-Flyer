
# Backyard Flyer — MiNiFi C++ Production Approach (V2)

This V2 guide extends `SOLUTION_MINIFI_CPP.md` in one specific way:

- V1 covered **telemetry movement + observability**.
- V2 keeps that, and additionally shows how MiNiFi C++ can participate in **controller operations** (start/stop/config/health) *without turning MiNiFi into the real-time flight-control loop*.

It is written as a “type-it-yourself” follow-along: you can copy the code blocks into your own files and validate them step by step.

Inputs re-analyzed:

- `README.md` (mission requirements, event-driven state machine)
- `SOLUTION_DESIGN_CPP.md` (real-time model, threading, adapter seam)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md` (staged repo layout)
- `SOLUTION_MODULAR_CPP.md` (modular controller core + app wiring)
- `SOLUTION_MINIFI_CPP.md` (V1: telemetry pipeline processors)

Mission reminder:

- Fly a **10m box** at **3m altitude** and land/disarm.
- Local frame is **NED**, so target altitude 3m means `down = -3.0`.

---

## 0) Can MiNiFi C++ “do real-time” and run the flight controller?

MiNiFi C++ is fast and written in C++, but it’s still a **dataflow runtime**, not a hard real-time control runtime.

### 0.1 What “real-time” means in this project

In `SOLUTION_DESIGN_CPP.md`, “more real-time” is really:

- fixed-rate control cadence (e.g., 100 Hz)
- minimal jitter in the control decision loop
- explicit concurrency (telemetry thread → control loop thread)
- minimal allocations in the hot path

Those are properties of a **control loop**.

### 0.2 Why MiNiFi is the wrong place for the actuation loop

MiNiFi’s scheduler can be timer-driven, but:

- execution timing is not deterministic (thread scheduling, backpressure, FlowFile repositories)
- processing latency varies with queue depth and repository I/O
- FlowFile semantics encourage “at least once” delivery and retries (great for data, risky for actuation)

So yes, you *can* embed control code in a processor, but you generally **should not** put the flight actuation loop inside MiNiFi.

### 0.3 What MiNiFi *can* do for “the remaining parts”

MiNiFi can still manage a lot of the “backend wiring” responsibilities you had in the modular C++ app:

- Start/stop and monitor the controller process (ops / supervision)
- Push high-level commands (start mission, stop mission, upload config)
- Collect and ship telemetry + health + logs with buffering/backpressure
- Downsample and route events at the edge

Think of this as:

- **Data plane (actuation)**: controller process (real-time loop)
- **Control plane (ops)**: MiNiFi flow triggers commands and reacts to health
- **Telemetry plane**: MiNiFi ships telemetry reliably

---

## 1) Target end-state (V2) architecture

We keep the modular C++ controller exactly as in `SOLUTION_MODULAR_CPP.md` (core library + app wiring), but we add a **tiny local control-plane interface**.

### 1.1 Two local “contracts”

**A) Telemetry contract (controller → MiNiFi)**

- Same as V1: JSON Lines (`Logs/telemetry.jsonl`), one JSON object per line.

**B) Control-plane contract (MiNiFi → controller)**

- A simple local command endpoint.
- Recommended MVP: controller exposes a small HTTP server on `127.0.0.1`.

Why HTTP for MVP:

- MiNiFi has a built-in `InvokeHTTP` processor.
- You can keep networking code out of your custom processors.

### 1.2 What moves where

Keep these **inside** the controller process:

- Telemetry subscriptions / transport
- Fixed-rate loop calling `BackyardFlyerController::Tick()`
- Safety-critical decision making

Move these to **MiNiFi**:

- Telemetry parsing/validation, edge routing, backpressure
- Downsampling position samples
- “Ops automation” flows (start mission at boot, stop mission on anomaly, etc.)

---

## 2) Step-by-step transition: Modular C++ → MiNiFi C++ (V2)

This is a staged path that minimizes risk.

### Step 1 — Keep the modular controller core unchanged

From `SOLUTION_MODULAR_CPP.md`, keep:

- `backyard_flyer_core/` library (planner/guards/controller/IVehicle)
- `backyard_flyer_app/` binary that wires telemetry + fixed-rate loop

Your acceptance criteria at this stage remains:

- Controller runs stable at configured rate
- Commands only happen on transitions (no spam)
- You can fly the 10m box in your environment

### Step 2 — Add structured JSONL telemetry output (if you haven’t already)

Write two append-only files:

- `Logs/telemetry.jsonl` (mission/telemetry events, as in V1)
- `Logs/controller_status.jsonl` (health/heartbeat)

Minimal **controller status** line format:

```json
{
	"event_type": "controller_status",
	"t_us": 1700000001123456,
	"vehicle_id": "sim-01",
	"run_id": "2026-02-23T01-23-45Z",
	"controller_state": "Waypoint",
	"in_mission": true,
	"loop_rate_hz": 100.0,
	"loop_jitter_ms_p95": 1.2
}
```

This status stream is your bridge between “real-time world” and “dataflow world.”

### Step 3 — Add a tiny local HTTP control-plane to the controller

Add an HTTP server (localhost only) to the controller process.

MVP endpoints:

- `POST /v1/mission/start` → starts the mission
- `POST /v1/mission/stop` → stops the mission (safe transition to Manual / Stop)
- `POST /v1/config` → updates config (optional)

You can implement this with any embedded HTTP library you prefer (civetweb, cpp-httplib, boost::beast, etc.).

MVP request body (start mission):

```json
{
	"vehicle_id": "sim-01",
	"run_id": "2026-02-23T01-23-45Z"
}
```

Important rule:

- The HTTP handler must not block the control loop thread.
- Hand off requests into a lock-free / mutex-protected queue and let the control loop apply them at a safe boundary.

### Step 4 — Add MiNiFi V2 flow: telemetry + status + control-plane

At this point, MiNiFi does three things:

1) tails and ships telemetry
2) tails and ships controller status
3) triggers control-plane calls (start/stop) using built-in `InvokeHTTP`

We’ll still use custom processors for domain parsing/validation/rate limiting (telemetry and status).

---

## 3) V2 MiNiFi flow design (built-in + custom)

### 3.1 Telemetry pipeline (same spirit as V1)

1) `TailFile` → reads `Logs/telemetry.jsonl`
2) `ParseBackyardTelemetryJson` (custom) → validate + attributes
3) `RateLimitBackyardTelemetry` (custom) → downsample `position_sample`
4) `RouteOnAttribute` → route by `telemetry.event_type`
5) forward via `PutTCP` / `InvokeHTTP` / Site-to-Site

### 3.2 Controller status pipeline (new)

1) `TailFile` → reads `Logs/controller_status.jsonl`
2) `ParseBackyardControllerStatusJson` (custom) → validate + attributes
3) `RouteOnAttribute` → route unhealthy vs healthy
4) forward the status stream to the platform (same sink choices)

### 3.3 Control-plane trigger flow (new)

There are multiple ways to trigger start/stop.

**Option A (simple): Start at boot**

- Use built-in `GenerateFlowFile` to create one FlowFile on startup.
- Custom processor `BuildBackyardMissionStartRequest` produces the JSON body and attributes.
- `InvokeHTTP` performs `POST /v1/mission/start`.

**Option B (safer): Start only when controller is healthy**

- Use status pipeline to detect `in_mission=false` and a “healthy” state.
- Trigger the start request only when healthy conditions are present.

For a follow-along, Option A is easiest.

---

## 4) Custom processors (V2)

V2 introduces two new custom processors:

1) `ParseBackyardControllerStatusJson` (status stream parser)
2) `BuildBackyardMissionStartRequest` (build request body for `InvokeHTTP`)

And it reuses the two V1 processors:

- `ParseBackyardTelemetryJson`
- `RateLimitBackyardTelemetry`

### 4.1 Extension layout

Same pattern as V1 (in-tree is easiest to validate):

```text
forked/nifi-minifi-cpp/extensions/backyard_flyer_v2/
	CMakeLists.txt
	processors/
		ParseBackyardControllerStatusJson.h
		ParseBackyardControllerStatusJson.cpp
		BuildBackyardMissionStartRequest.h
		BuildBackyardMissionStartRequest.cpp
		# (optionally copy V1 processors here too)
```

Note: you can keep V1 and V2 in the same extension folder if you prefer; V2 is a documentation split, not a technical requirement.

---

## 5) Processor — ParseBackyardControllerStatusJson (custom)

Goal:

- Parse controller status JSON objects.
- Validate required fields exist.
- Set attributes like `controller.state`, `controller.in_mission`, `controller.loop_rate_hz` for routing.

### 5.1 `processors/ParseBackyardControllerStatusJson.h`

```cpp
#pragma once

#include <array>
#include <string>

#include "core/ProcessorImpl.h"
#include "core/PropertyDefinitionBuilder.h"
#include "minifi-cpp/core/RelationshipDefinition.h"

namespace org::apache::nifi::minifi::extensions::backyard_flyer_v2 {

class ParseBackyardControllerStatusJson final : public core::ProcessorImpl {
 public:
	using ProcessorImpl::ProcessorImpl;

	EXTENSIONAPI static constexpr const char* Description =
			"Parses BackyardFlyer controller_status JSON, validates required fields, and sets routing attributes.";

	EXTENSIONAPI static constexpr auto VehicleId = core::PropertyDefinitionBuilder<>::createProperty("VehicleId")
			.withDescription("Default vehicle_id to set when missing")
			.isRequired(false)
			.build();

	EXTENSIONAPI static constexpr auto RunId = core::PropertyDefinitionBuilder<>::createProperty("RunId")
			.withDescription("Default run_id to set when missing")
			.isRequired(false)
			.build();

	EXTENSIONAPI static constexpr auto Properties = std::to_array<core::PropertyReference>({VehicleId, RunId});

	EXTENSIONAPI static constexpr auto Success = core::RelationshipDefinition{"success", "Valid controller status"};
	EXTENSIONAPI static constexpr auto Failure = core::RelationshipDefinition{"failure", "Invalid JSON or missing fields"};
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

}  // namespace org::apache::nifi::minifi::extensions::backyard_flyer_v2
```

### 5.2 `processors/ParseBackyardControllerStatusJson.cpp`

```cpp
#include "ParseBackyardControllerStatusJson.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/Resource.h"
#include "core/ProcessSession.h"
#include "core/logging/LoggerFactory.h"
#include "io/InputStream.h"
#include "minifi-cpp/core/ProcessContext.h"
#include "rapidjson/document.h"

namespace org::apache::nifi::minifi::extensions::backyard_flyer_v2 {

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

std::optional<bool> getBoolMember(const rapidjson::Document& d, const char* key) {
	if (!d.HasMember(key) || !d[key].IsBool()) return std::nullopt;
	return d[key].GetBool();
}

}  // namespace

void ParseBackyardControllerStatusJson::initialize() {
	setSupportedProperties(Properties);
	setSupportedRelationships(Relationships);
}

void ParseBackyardControllerStatusJson::onSchedule(core::ProcessContext& context, core::ProcessSessionFactory&) {
	default_vehicle_id_ = context.getProperty(VehicleId).value_or("");
	default_run_id_ = context.getProperty(RunId).value_or("");
}

void ParseBackyardControllerStatusJson::onTrigger(core::ProcessContext&, core::ProcessSession& session) {
	auto flow_file = session.get();
	if (!flow_file) return;

	std::string body;
	const auto read_ok = session.read(flow_file, ReadFlowFileToStringCallback{flow_file->getSize(), body});
	if (read_ok < 0) {
		session.transfer(flow_file, Failure);
		return;
	}

	rapidjson::Document d;
	d.Parse(body.c_str());
	if (d.HasParseError() || !d.IsObject()) {
		session.transfer(flow_file, Failure);
		return;
	}

	// Required fields for status
	const auto event_type = getStringMember(d, "event_type");
	const auto t_us = getUint64Member(d, "t_us");
	const auto controller_state = getStringMember(d, "controller_state");
	const auto in_mission = getBoolMember(d, "in_mission");

	if (!event_type || *event_type != "controller_status" || !t_us || !controller_state || !in_mission) {
		session.transfer(flow_file, Failure);
		return;
	}

	auto vehicle_id = getStringMember(d, "vehicle_id");
	auto run_id = getStringMember(d, "run_id");
	if (!vehicle_id || vehicle_id->empty()) vehicle_id = default_vehicle_id_;
	if (!run_id || run_id->empty()) run_id = default_run_id_;

	flow_file->setAttribute("controller.event_type", *event_type);
	flow_file->setAttribute("controller.t_us", std::to_string(*t_us));
	flow_file->setAttribute("controller.state", *controller_state);
	flow_file->setAttribute("controller.in_mission", *in_mission ? "true" : "false");

	if (vehicle_id && !vehicle_id->empty()) flow_file->setAttribute("controller.vehicle_id", *vehicle_id);
	if (run_id && !run_id->empty()) flow_file->setAttribute("controller.run_id", *run_id);

	session.transfer(flow_file, Success);
}

REGISTER_RESOURCE(ParseBackyardControllerStatusJson, Processor);

}  // namespace org::apache::nifi::minifi::extensions::backyard_flyer_v2
```

Follow-along note:

- This is deliberately small. You can add more fields later (`loop_rate_hz`, `jitter_ms_p95`, etc.) and route based on them.

---

## 6) Processor — BuildBackyardMissionStartRequest (custom)

Goal:

- Create a FlowFile whose **content** is a JSON request body for `POST /v1/mission/start`.
- Set attributes like `controller.vehicle_id` and `controller.run_id` so you can use Expression Language in `InvokeHTTP` if desired.

This processor does *not* perform the HTTP call itself; it just creates/validates the request payload in a domain-specific way.

### 6.1 `processors/BuildBackyardMissionStartRequest.h`

```cpp
#pragma once

#include <array>
#include <string>

#include "core/ProcessorImpl.h"
#include "core/PropertyDefinitionBuilder.h"
#include "minifi-cpp/core/RelationshipDefinition.h"

namespace org::apache::nifi::minifi::extensions::backyard_flyer_v2 {

class BuildBackyardMissionStartRequest final : public core::ProcessorImpl {
 public:
	using ProcessorImpl::ProcessorImpl;

	EXTENSIONAPI static constexpr const char* Description =
			"Builds a JSON request body for the controller start-mission endpoint.";

	EXTENSIONAPI static constexpr auto VehicleId = core::PropertyDefinitionBuilder<>::createProperty("VehicleId")
			.withDescription("Vehicle ID to put in the request body")
			.isRequired(true)
			.build();

	EXTENSIONAPI static constexpr auto RunId = core::PropertyDefinitionBuilder<>::createProperty("RunId")
			.withDescription("Run ID to put in the request body")
			.isRequired(true)
			.build();

	EXTENSIONAPI static constexpr auto Properties = std::to_array<core::PropertyReference>({VehicleId, RunId});

	EXTENSIONAPI static constexpr auto Success = core::RelationshipDefinition{"success", "Built request body"};
	EXTENSIONAPI static constexpr auto Failure = core::RelationshipDefinition{"failure", "Missing properties"};
	EXTENSIONAPI static constexpr auto Relationships = std::array{Success, Failure};

	EXTENSIONAPI static constexpr bool SupportsDynamicProperties = false;
	EXTENSIONAPI static constexpr bool SupportsDynamicRelationships = false;
	EXTENSIONAPI static constexpr core::annotation::Input InputRequirement = core::annotation::Input::INPUT_ALLOWED;
	EXTENSIONAPI static constexpr bool IsSingleThreaded = false;

	ADD_COMMON_VIRTUAL_FUNCTIONS_FOR_PROCESSORS

	void initialize() override;
	void onSchedule(core::ProcessContext& context, core::ProcessSessionFactory& session_factory) override;
	void onTrigger(core::ProcessContext& context, core::ProcessSession& session) override;

 private:
	std::string vehicle_id_;
	std::string run_id_;
};

}  // namespace org::apache::nifi::minifi::extensions::backyard_flyer_v2
```

### 6.2 `processors/BuildBackyardMissionStartRequest.cpp`

```cpp
#include "BuildBackyardMissionStartRequest.h"

#include <cstdint>
#include <string>

#include "core/Resource.h"
#include "core/ProcessSession.h"
#include "io/OutputStream.h"
#include "minifi-cpp/core/ProcessContext.h"

namespace org::apache::nifi::minifi::extensions::backyard_flyer_v2 {

void BuildBackyardMissionStartRequest::initialize() {
	setSupportedProperties(Properties);
	setSupportedRelationships(Relationships);
}

void BuildBackyardMissionStartRequest::onSchedule(core::ProcessContext& context, core::ProcessSessionFactory&) {
	vehicle_id_ = context.getProperty(VehicleId).value_or("");
	run_id_ = context.getProperty(RunId).value_or("");
}

void BuildBackyardMissionStartRequest::onTrigger(core::ProcessContext&, core::ProcessSession& session) {
	if (vehicle_id_.empty() || run_id_.empty()) {
		// If there is an incoming flow file, pass it to failure; otherwise create a new one and fail it.
		auto ff = session.get();
		if (!ff) ff = session.create();
		session.transfer(ff, Failure);
		return;
	}

	auto ff = session.get();
	if (!ff) ff = session.create();

	const std::string body = std::string{"{\"vehicle_id\":\""} + vehicle_id_ + "\",\"run_id\":\"" + run_id_ + "\"}";

	ff->setAttribute("controller.vehicle_id", vehicle_id_);
	ff->setAttribute("controller.run_id", run_id_);
	ff->setAttribute("mime.type", "application/json");

	session.write(ff, [&body](const std::shared_ptr<io::OutputStream>& out) -> int64_t {
		const auto n = out->write(reinterpret_cast<const uint8_t*>(body.data()), body.size());
		return io::isError(n) ? -1 : n;
	});

	session.transfer(ff, Success);
}

REGISTER_RESOURCE(BuildBackyardMissionStartRequest, Processor);

}  // namespace org::apache::nifi::minifi::extensions::backyard_flyer_v2
```

---

## 7) Extension CMake (V2)

```cmake
if (NOT (ENABLE_ALL OR ENABLE_BACKYARD_FLYER_V2))
	return()
endif()

include(${CMAKE_SOURCE_DIR}/extensions/ExtensionHeader.txt)

file(GLOB SOURCES "processors/*.cpp")

add_minifi_library(minifi-backyard-flyer-v2 SHARED ${SOURCES})

target_include_directories(minifi-backyard-flyer-v2 PUBLIC "${CMAKE_SOURCE_DIR}/extensions/backyard_flyer_v2")
target_link_libraries(minifi-backyard-flyer-v2 ${LIBMINIFI})

register_extension(minifi-backyard-flyer-v2 "BACKYARD FLYER V2 EXTENSIONS" BACKYARD-FLYER-V2-EXTENSIONS
	"Custom processors for BackyardFlyer telemetry + controller status + control-plane requests" "")
```

Notes:

- Building with `-DENABLE_ALL=ON` is the fastest way to get started.
- If you want an explicit option, add a MiNiFi option entry, but that’s extra plumbing.

---

## 8) Example V2 flow wiring (conceptual)

This is a conceptual wiring; exact `config.yml` shapes can vary by MiNiFi version.

### 8.1 Start mission at boot

- `GenerateFlowFile` (run once) → `BuildBackyardMissionStartRequest` → `InvokeHTTP`

`InvokeHTTP` is configured to:

- method: `POST`
- URL: `http://127.0.0.1:8088/v1/mission/start`
- content-type: `application/json`

### 8.2 Telemetry shipping

- `TailFile(Logs/telemetry.jsonl)` → `ParseBackyardTelemetryJson` → `RateLimitBackyardTelemetry` → `PutTCP` (or `InvokeHTTP`)

### 8.3 Controller health shipping

- `TailFile(Logs/controller_status.jsonl)` → `ParseBackyardControllerStatusJson` → `RouteOnAttribute(controller.in_mission == false ...)` → sink

---

## 9) Optional: process supervision (starting the controller from MiNiFi)

If you want MiNiFi to *start* the controller binary, you have two common approaches:

### Option A (simplest): OS service manager starts the controller

- systemd (Linux) starts `backyard_controllerd`
- MiNiFi is a sidecar that tails logs and triggers HTTP commands

This is usually the most production-friendly.

### Option B (MiNiFi-driven): built-in `ExecuteProcess`

MiNiFi has a built-in `ExecuteProcess` processor.

Notes:

- It may require enabling build options (for example `-DENABLE_EXECUTE_PROCESS=ON`, or just `-DENABLE_ALL=ON`).
- It is not supported on Windows.

You can wire:

- `GenerateFlowFile` → `ExecuteProcess(Command=./backyard_controllerd, Args=...)`

Then tail the controller logs/status as usual.

---

## 10) Summary: what changed from V1

V1: telemetry-only MiNiFi edge pipeline.

V2: telemetry pipeline + status pipeline + control-plane request generation, while keeping:

- the real-time actuation loop inside the dedicated controller process
- MiNiFi focused on reliability, routing, buffering, and operations workflows

