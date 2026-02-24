# Backyard Flyer — Proposed Repo Folder Structure (MVP → Modular → MiNiFi/NiFi Evolution)

This proposes a GitHub repo layout that:

- Keeps the **Udacity MVP** dead-simple and runnable (`python backyard_flyer.py`).
- Supports a **modular Python architecture** (planner/guards/config/telemetry separated).
- Provides a clean upgrade path to **edge telemetry shipping** (MiNiFi C++/Python) and **central ingestion/orchestration** (NiFi).

The key design principle: **real-time control stays local** (Python flight controller / PX4). MiNiFi/NiFi handle **data movement, observability, analytics**, not actuation.

---

## 1) Stage 0 — MVP (single-file control logic, minimum moving parts)

Goal: implement the TODOs in `backyard_flyer.py` and fly the 10m box.

Recommended MVP repo layout:

```text
FCND-Backyard-UAS-Flyer/
	README.md
	backyard_flyer.py              # MVP entrypoint (Udacity submission target)
	drone.py                       # manual logging runner (from project)
	Logs/                          # generated at runtime (gitignored)
	research/
		SOLUTION_MVP.md
		SOLUTION_DESIGN.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE.md
	.gitignore
```

Notes:

- Keep `backyard_flyer.py` as the canonical entrypoint for the assignment.
- Add `.gitignore` rules for logs and simulator artifacts (`Logs/`, `*.tlog`, etc.).

---

## 2) Stage 1 — Modular Python (package + unit-testable core)

Goal: refactor the MVP into modules without changing external behavior.

Key rule: **preserve** `backyard_flyer.py` as a thin wrapper so the simulator usage and Udacity expectations remain unchanged.

Proposed layout:

```text
FCND-Backyard-UAS-Flyer/
	README.md
	backyard_flyer.py                  # still runnable; imports package modules
	pyproject.toml                     # or requirements.txt / setup.cfg
	src/
		backyard_flyer/
			__init__.py
			app.py                         # build connection + start mission
			config.py                      # constants, tolerances, mission params
			fsm.py                         # flight-state machine orchestration
			guards.py                      # reached_altitude/reached_waypoint/etc.
			planner.py                     # calculate_box + future planners
			telemetry/
				__init__.py
				events.py                    # event models (dicts/dataclasses)
				logger.py                    # JSONL/CSV logging hooks
			adapters/
				__init__.py
				udacidrone_adapter.py        # wraps Drone calls (sim vs real friendly)
	tests/
		test_planner.py                  # pure waypoint tests
		test_guards.py                   # tolerance + landing checks
	scripts/
		run_sim.sh                       # convenience wrappers (optional)
	docs/
		architecture.md                  # optional; links to research docs
	research/
		...
```

Why this split maps well to `SOLUTION_DESIGN.md`:

- `planner.py` holds `calculate_box()` logic.
- `guards.py` makes callback conditions deterministic and unit-testable.
- `fsm.py` owns transitions and state advancement.
- `telemetry/` standardizes events (state transitions, waypoint commands, samples).
- `adapters/` is the seam that prevents UdaciDrone APIs from “infecting” all modules.

Minimal migration strategy:

1) Extract `calculate_box()` into `planner.py`.
2) Extract guard checks into `guards.py`.
3) Keep callbacks in one place (`fsm.py`), calling planner/guards.
4) Make `backyard_flyer.py` instantiate the class and start.

---

## 3) Stage 2 — Telemetry-first design (structured events + schemas)

Goal: prepare for shipping telemetry off-device without changing the controller’s safety behavior.

Add:

```text
FCND-Backyard-UAS-Flyer/
	schemas/
		telemetry_event.schema.json      # JSON Schema for emitted events
		telemetry_sample.schema.json
	data/
		samples/
			navlog_example.jsonl           # small example payloads
```

Event types (examples):

- `state_transition` (from_state, to_state, timestamp)
- `command_waypoint` (target_position, heading)
- `position_sample` (local_position, local_velocity)
- `mission_summary` (duration, success/fail, reason)

Why schemas matter:

- NiFi flows can validate and route reliably.
- You can evolve the pipeline without breaking downstream consumers.

---

## 4) Stage 3 — Edge dataflow: MiNiFi C++/Python

Goal: run the flight controller locally; ship telemetry reliably to a central collector.

Two common integration patterns:

1) **File-based**: controller writes JSONL/CSV files → MiNiFi tails + ships.
2) **Socket-based**: controller emits UDP/TCP → MiNiFi ingests stream.

Proposed repo layout:

```text
FCND-Backyard-UAS-Flyer/
	edge/
		minifi-cpp/
			README.md
			conf/
				minifi.properties
				config.yml                  # MiNiFi flow definition
			docker/
				Dockerfile                  # reproducible edge agent image (optional)
			scripts/
				run_minifi.sh
		minifi-python/
			README.md
			flows/
				flow.yml                    # if using MiNiFi Python agent
			scripts/
				run_minifi_python.sh
		telemetry/
			README.md
			example_output/
				nav_events.jsonl
```

What goes in `edge/` (and what does not):

- Include MiNiFi configs and run scripts.
- Do not include drone actuation logic here.
- Keep secrets out of repo (use env vars / secret managers).

---

## 5) Stage 4 — Platform ingestion: NiFi (central routing + storage)

Goal: ingest from many edge agents, enrich/validate, then store + alert.

Proposed repo layout:

```text
FCND-Backyard-UAS-Flyer/
	platform/
		nifi/
			README.md
			flows/
				telemetry_ingest.json       # exported NiFi flow definition (versioned)
			parameters/
				dev.params.json
				prod.params.json
			docker/
				docker-compose.yml          # local dev stack (NiFi + deps)
		nifi-registry/
			README.md                     # optional: flow versioning strategy
	infra/
		storage/
			README.md                     # S3/minio, TSDB, warehouse notes
		monitoring/
			README.md                     # metrics/logging approach
```

Suggested NiFi flow responsibilities:

- Ingest: receive edge telemetry (HTTP, Site-to-Site, Kafka, etc.).
- Validate: JSON Schema validation using `schemas/` artifacts.
- Enrich: add mission metadata (vehicle id, sim run id, build hash).
- Route: raw → object storage; metrics → time-series DB; alerts → ops channel.

---

## 6) “One repo” vs “multi repo” recommendation

For a technical challenge / portfolio repo, one repo is fine, with clear boundaries:

- `src/` = flight control software
- `edge/` = deployment configs for edge telemetry shipping
- `platform/` = NiFi flows + local dev stack

In production, a common evolution is:

- `flight-controller` repo
- `edge-data-agent` repo
- `data-platform-ingestion` repo

But start unified until the interfaces stabilize.

---

## 7) Practical conventions to keep the repo maintainable

### Keep the MVP entrypoint stable

- `backyard_flyer.py` remains runnable and thin.
- Internals move into `src/backyard_flyer/`.

### Version and provenance

- Include a `VERSION` or embed build metadata into telemetry events.
- Add `CHANGELOG.md` if you expect multiple iterations.

### Testing scope

- Only unit-test pure functions (`planner.py`, `guards.py`).
- Treat simulator runs as integration tests; optionally add a `scripts/run_mission.sh`.

### Git hygiene

- Gitignore logs, simulator outputs, and secrets.
- Check in only small sample telemetry under `data/samples/`.

---

## 8) Full “final form” tree (all stages present)

If you want a single tree that supports every stage at once:

```text
FCND-Backyard-UAS-Flyer/
	README.md
	backyard_flyer.py
	drone.py
	pyproject.toml
	src/
		backyard_flyer/
			__init__.py
			app.py
			config.py
			fsm.py
			guards.py
			planner.py
			adapters/
				__init__.py
				udacidrone_adapter.py
			telemetry/
				__init__.py
				events.py
				logger.py
	tests/
		test_planner.py
		test_guards.py
	schemas/
		telemetry_event.schema.json
		telemetry_sample.schema.json
	data/
		samples/
			navlog_example.jsonl
	edge/
		minifi-cpp/
			conf/
				minifi.properties
				config.yml
			scripts/
				run_minifi.sh
		minifi-python/
			flows/
				flow.yml
			scripts/
				run_minifi_python.sh
	platform/
		nifi/
			flows/
				telemetry_ingest.json
			parameters/
				dev.params.json
			docker/
				docker-compose.yml
	research/
		SOLUTION_MVP.md
		SOLUTION_DESIGN.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE.md
```

This layout cleanly expresses the evolution described in `SOLUTION_DESIGN.md` while keeping the original Udacity project runnable from the root.

