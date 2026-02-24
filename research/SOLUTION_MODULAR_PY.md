# Backyard Flyer — Modular, Class-Based Python Solution (Follow-Along Guide)

This document turns the MVP state machine (in `backyard_flyer.py`) into a **modular, class-based** Python design that you can type out manually in a new file/package and learn from.

It is intentionally:

- Minimal (only what the README requires: 10m box @ 3m altitude)
- Testable (planner + guards are pure functions/classes)
- Portable (UdaciDrone specifics live behind a small adapter)

> Reminder on frames: local position is **NED**; “3m altitude” means `down = -3.0`.

---

## 0) Target end state (what you’ll have)

You will end up with:

1) A thin entry script (keeps Udacity workflow) that starts the mission.
2) A small package with:
   - `MissionConfig` constants
   - `BoxPlanner` to generate waypoints
   - `FlightGuards` to decide “have we arrived / can we disarm”
   - `BackyardFlyerFSM` (the state machine)
   - `UdacidroneAdapter` (a wrapper around `Drone` methods)
3) Unit tests for planner + guards.

You can implement this either:

- **Option A (recommended):** Keep `backyard_flyer.py` as the entrypoint and have it import your modular classes.
- **Option B:** Create a new `backyard_flyer_modular.py` for your own use.

---

## 1) Suggested folder layout (Stage 1 modular)

Use the structure from `SOLUTION_DESIGN_FOLDER_STRUCTURE.md`:

```text
FCND-Backyard-UAS-Flyer/
  backyard_flyer.py
  src/
	backyard_flyer/
	  __init__.py
	  config.py
	  types.py
	  planner.py
	  guards.py
	  adapter.py
	  fsm.py
	  app.py
  tests/
	test_planner.py
	test_guards.py
```

If you don’t want packaging right now, you can still follow along by putting these “modules” into separate files in a single folder.

---

## 2) Module-by-module implementation (copyable skeletons)

### 2.1 `config.py` — mission constants

Purpose: keep mission parameters in one place.

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class MissionConfig:
	box_size_m: float = 10.0
	target_altitude_m: float = 3.0

	# tolerances
	pos_tolerance_m: float = 0.5
	vel_xy_tolerance_mps: float = 0.5
	ground_down_tolerance_m: float = 0.1

	# takeoff completion threshold
	takeoff_altitude_ratio: float = 0.95

	# heading to maintain during cmd_position
	heading_rad: float = 0.0
```

Design notes:

- `@dataclass(frozen=True)` makes config immutable (safer).
- Keep values simple for MVP; you can tune later.

---

### 2.2 `types.py` — small, explicit data types

Purpose: reduce “mystery arrays” and make intent clear.

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class NED:
	north: float
	east: float
	down: float

	def as_np(self) -> np.ndarray:
		return np.array([self.north, self.east, self.down], dtype=float)

	@staticmethod
	def from_iterable(values: Iterable[float]) -> "NED":
		v = list(values)
		return NED(float(v[0]), float(v[1]), float(v[2]))


Waypoint = NED
```

You can keep using `np.array` everywhere if you prefer; the key is consistency.

---

### 2.3 `planner.py` — waypoint planner (the 10m box)

Purpose: generate waypoints relative to the current position.

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import List

from .config import MissionConfig
from .types import Waypoint


@dataclass
class BoxPlanner:
	cfg: MissionConfig

	def build_box(self, origin_north: float, origin_east: float) -> List[Waypoint]:
		down = -self.cfg.target_altitude_m
		s = self.cfg.box_size_m

		return [
			Waypoint(origin_north + s, origin_east + 0.0, down),
			Waypoint(origin_north + s, origin_east + s, down),
			Waypoint(origin_north + 0.0, origin_east + s, down),
			Waypoint(origin_north + 0.0, origin_east + 0.0, down),
		]
```

Why it’s a class instead of a function:

- planner can later support multiple patterns (triangle, lawnmower, etc.) without changing the rest of the code.

---

### 2.4 `guards.py` — state transition “if” logic

Purpose: pure checks that are easy to unit test.

```python
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import MissionConfig
from .types import NED


@dataclass
class FlightGuards:
	cfg: MissionConfig

	def altitude_reached(self, local_position: np.ndarray) -> bool:
		# local_position is NED => altitude = -down
		altitude = -float(local_position[2])
		return altitude >= self.cfg.takeoff_altitude_ratio * self.cfg.target_altitude_m

	def waypoint_reached(self, local_position: np.ndarray, target: NED) -> bool:
		pos_ne = np.array([local_position[0], local_position[1]], dtype=float)
		tgt_ne = np.array([target.north, target.east], dtype=float)
		d = float(np.linalg.norm(pos_ne - tgt_ne))
		return d <= self.cfg.pos_tolerance_m

	def ready_to_disarm(self, local_position: np.ndarray, local_velocity: np.ndarray) -> bool:
		near_ground = abs(float(local_position[2])) <= self.cfg.ground_down_tolerance_m
		vel_xy = np.linalg.norm(np.array([local_velocity[0], local_velocity[1]], dtype=float))
		nearly_stopped = float(vel_xy) <= self.cfg.vel_xy_tolerance_mps
		return near_ground and nearly_stopped
```

---

### 2.5 `adapter.py` — isolate UdaciDrone API calls

Purpose: the FSM should not care whether it’s talking to a simulator, real drone, or a mock.

This adapter is a thin wrapper around the underlying `Drone` object.

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class DroneLike(Protocol):
	# attributes
	armed: bool
	guided: bool
	local_position: object
	local_velocity: object

	# commands
	def arm(self) -> None: ...
	def disarm(self) -> None: ...
	def take_control(self) -> None: ...
	def release_control(self) -> None: ...
	def takeoff(self, target_altitude: float) -> None: ...
	def land(self) -> None: ...
	def cmd_position(self, north: float, east: float, down: float, heading: float) -> None: ...
	def stop(self) -> None: ...


@dataclass
class UdacidroneAdapter:
	drone: DroneLike

	def take_control_and_arm(self) -> None:
		self.drone.take_control()
		self.drone.arm()

	def takeoff(self, altitude_m: float) -> None:
		self.drone.takeoff(altitude_m)

	def goto(self, north: float, east: float, down: float, heading: float) -> None:
		self.drone.cmd_position(north, east, down, heading)

	def land(self) -> None:
		self.drone.land()

	def disarm(self) -> None:
		self.drone.disarm()

	def finish(self) -> None:
		self.drone.release_control()
		self.drone.stop()
```

Design notes:

- The `Protocol` makes it easy to unit test FSM by passing a mock object.
- This adapter is where API-version differences can be handled.

---

### 2.6 `fsm.py` — the state machine class

Purpose: store state, manage waypoints, and call adapter commands.

We’ll mirror the project’s states.

```python
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

import numpy as np

from .adapter import UdacidroneAdapter
from .config import MissionConfig
from .guards import FlightGuards
from .planner import BoxPlanner
from .types import NED, Waypoint


class FlightState(Enum):
	MANUAL = 0
	ARMING = 1
	TAKEOFF = 2
	WAYPOINT = 3
	LANDING = 4
	DISARMING = 5


@dataclass
class BackyardFlyerFSM:
	cfg: MissionConfig
	adapter: UdacidroneAdapter
	planner: BoxPlanner
	guards: FlightGuards

	state: FlightState = FlightState.MANUAL
	in_mission: bool = True

	target: Optional[Waypoint] = None
	waypoints: List[Waypoint] = field(default_factory=list)

	def on_state_update(self, armed: bool, guided: bool) -> None:
		if self.state == FlightState.MANUAL and self.in_mission:
			self._arming_transition()
			return

		if self.state == FlightState.ARMING and armed:
			self._takeoff_transition()
			return

		if self.state == FlightState.DISARMING and not armed:
			self._manual_transition()
			return

	def on_local_position(self, local_position: np.ndarray) -> None:
		if self.state == FlightState.TAKEOFF and self.guards.altitude_reached(local_position):
			if not self.waypoints:
				self.waypoints = self.planner.build_box(
					origin_north=float(local_position[0]),
					origin_east=float(local_position[1]),
				)
			self._waypoint_transition()
			return

		if self.state == FlightState.WAYPOINT and self.target is not None:
			if self.guards.waypoint_reached(local_position, self.target):
				if self.waypoints:
					self._waypoint_transition()
				else:
					self._landing_transition()

	def on_local_velocity(self, local_position: np.ndarray, local_velocity: np.ndarray) -> None:
		if self.state == FlightState.LANDING:
			if self.guards.ready_to_disarm(local_position, local_velocity):
				self._disarming_transition()

	# --- transitions (actions) ---

	def _arming_transition(self) -> None:
		print("arming transition")
		self.adapter.take_control_and_arm()
		self.state = FlightState.ARMING

	def _takeoff_transition(self) -> None:
		print("takeoff transition")
		self.adapter.takeoff(self.cfg.target_altitude_m)
		self.state = FlightState.TAKEOFF

	def _waypoint_transition(self) -> None:
		print("waypoint transition")
		self.target = self.waypoints.pop(0) if self.waypoints else None
		if self.target is None:
			self._landing_transition()
			return

		self.adapter.goto(
			north=self.target.north,
			east=self.target.east,
			down=self.target.down,
			heading=self.cfg.heading_rad,
		)
		self.state = FlightState.WAYPOINT

	def _landing_transition(self) -> None:
		print("landing transition")
		self.adapter.land()
		self.state = FlightState.LANDING

	def _disarming_transition(self) -> None:
		print("disarm transition")
		self.adapter.disarm()
		self.state = FlightState.DISARMING

	def _manual_transition(self) -> None:
		print("manual transition")
		self.adapter.finish()
		self.in_mission = False
		self.state = FlightState.MANUAL
```

What to notice:

- Callback inputs (`on_local_position`, `on_state_update`, `on_local_velocity`) are pure data.
- Transitions are the only place that sends commands.
- `BackyardFlyerFSM` is now testable with a fake adapter.

---

### 2.7 `app.py` — wiring (build objects once)

Purpose: central place to construct `cfg`, planner, guards, FSM.

```python
from __future__ import annotations

from .adapter import UdacidroneAdapter
from .config import MissionConfig
from .fsm import BackyardFlyerFSM
from .guards import FlightGuards
from .planner import BoxPlanner


def build_fsm(drone) -> BackyardFlyerFSM:
	cfg = MissionConfig()
	adapter = UdacidroneAdapter(drone=drone)
	planner = BoxPlanner(cfg=cfg)
	guards = FlightGuards(cfg=cfg)
	return BackyardFlyerFSM(cfg=cfg, adapter=adapter, planner=planner, guards=guards)
```

---

## 3) Hooking this into the Udacity starter (`backyard_flyer.py`)

You need a `Drone` subclass for UdaciDrone callbacks, but it can be extremely thin:

### 3.1 Pattern: `BackyardFlyer(Drone)` delegates to `BackyardFlyerFSM`

In the existing `backyard_flyer.py`:

1) Build the FSM once in `__init__`.
2) In each callback, forward the updated attributes.

Pseudo-diff (illustrative):

```python
from backyard_flyer.app import build_fsm

class BackyardFlyer(Drone):
	def __init__(self, connection):
		super().__init__(connection)
		self.fsm = build_fsm(self)
		... register callbacks ...

	def local_position_callback(self):
		self.fsm.on_local_position(self.local_position)

	def velocity_callback(self):
		self.fsm.on_local_velocity(self.local_position, self.local_velocity)

	def state_callback(self):
		self.fsm.on_state_update(self.armed, self.guided)
```

At this point, the existing `start()` method and logging behavior can remain unchanged.

---

## 4) Unit tests you can write immediately

### 4.1 `test_planner.py`

```python
from backyard_flyer.config import MissionConfig
from backyard_flyer.planner import BoxPlanner


def test_box_planner_returns_4_waypoints():
	cfg = MissionConfig(box_size_m=10.0, target_altitude_m=3.0)
	planner = BoxPlanner(cfg)
	wps = planner.build_box(origin_north=0.0, origin_east=0.0)
	assert len(wps) == 4
	assert wps[-1].north == 0.0
	assert wps[-1].east == 0.0
	assert wps[0].down == -3.0
```

### 4.2 `test_guards.py`

```python
import numpy as np

from backyard_flyer.config import MissionConfig
from backyard_flyer.guards import FlightGuards
from backyard_flyer.types import NED


def test_altitude_reached_uses_ned_down_sign():
	cfg = MissionConfig(target_altitude_m=3.0, takeoff_altitude_ratio=0.95)
	guards = FlightGuards(cfg)
	# at altitude 3m => down = -3
	assert guards.altitude_reached(np.array([0.0, 0.0, -3.0]))


def test_waypoint_reached_xy_only():
	cfg = MissionConfig(pos_tolerance_m=0.5)
	guards = FlightGuards(cfg)
	target = NED(10.0, 0.0, -3.0)
	assert guards.waypoint_reached(np.array([10.2, 0.1, -3.0]), target)
```

---

## 5) Validation checklist (how you know your modular version works)

### 5.1 Logic validation (fast)

- Run unit tests for planner + guards.
- Sanity-check that altitude uses NED sign correctly.

### 5.2 Simulator validation (end-to-end)

- Start the Unity simulator.
- Run your script (`python backyard_flyer.py`).
- Observe state transitions printed once per transition:
  - arming → takeoff → waypoint (x4 corners) → landing → disarm → manual
- Confirm the vehicle flies a square path and lands.

---

## 6) How this supports the MiNiFi/NiFi evolution (without changing control)

Once modular, telemetry becomes a separate concern:

- Add a `telemetry/logger.py` that writes JSONL events.
- MiNiFi can tail those files and forward to a central NiFi flow.

Critically:

- The FSM remains deterministic and local.
- Dataflow (MiNiFi/NiFi) can fail without impacting flight safety.

---

## 7) If you want an even simpler modular variant

If this feels like “too many files” for learning, collapse it to 3 files:

1) `mission_config.py`
2) `fsm.py` (including planner + guards)
3) `backyard_flyer.py` (thin Drone wrapper)

Then split modules out only after your first successful sim run.

