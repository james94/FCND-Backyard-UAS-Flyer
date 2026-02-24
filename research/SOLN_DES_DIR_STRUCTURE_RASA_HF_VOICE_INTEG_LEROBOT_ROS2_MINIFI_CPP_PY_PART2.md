
# Backyard Flyer — Folder Structure (Part 2: Rasa + HF Voice + LeRobot + ROS2 + MiNiFi)

This Part 2 folder structure complements:

- `SOLN_DES_DIR_STRUCTURE_LEROBOT_ROS2_MINIFI_CPP_PY_PART2.md` (datasets/training/policy runner)
- `SOLN_DES_DIR_STRUCTURE_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY.md` (HF-only voice service)
- `SOLUTION_RASA_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY_PART2.md` (Rasa+HF integration design)

Goal:

- Add a Rasa project (NLU + dialogue) and an action server that calls the existing ROS2 HTTP gateway.
- Keep logs as JSONL so MiNiFi can tail and route them.

---

## 1) Recommended repo tree additions

```text
FCND-Backyard-UAS-Flyer/
	README.md

	# ROS2 runtime (existing)
	ros2_ws/
		src/
			backyard_flyer_http_gateway/
			backyard_flyer_nodes/
			backyard_flyer_vehicle/
			backyard_flyer_learning_nodes/
			backyard_flyer_lerobot_policy/
			backyard_flyer_policy_eval/

	# Operator stack (new)
	operator/
		voice/
			app/
				# HF ASR front-end that produces transcripts
				voice_service.py
				audio_capture.py
				hf_asr.py
				# Rasa client wrapper
				rasa_client.py
				jsonl_logger.py
			requirements.txt

		rasa/
			README.md
			requirements.txt
			config.yml
			domain.yml
			endpoints.yml
			credentials.yml
			data/
				nlu.yml
				rules.yml
				stories.yml
			actions/
				actions.py
				requirements.txt

	# Runtime logs (MiNiFi tails these; all gitignored)
	Logs/
		telemetry.jsonl
		controller_status.jsonl
		learning_events.jsonl
		policy_eval.jsonl
		operator_commands.jsonl          # shared audit stream (HF + Rasa actions)
		rasa_dialogue.jsonl              # optional: conversation events

	# Edge sidecar (existing)
	edge/
		minifi-cpp/
			conf/
				config.yml                   # add TailFile for operator_commands.jsonl (+ optional rasa_dialogue.jsonl)

	# Docs
	research/
		SOLN_DES_DIR_STRUCTURE_RASA_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY_PART2.md
		SOLUTION_RASA_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY_PART2.md
```

---

## 2) Process boundaries (important)

Recommended processes:

- `ros2_http_gateway_node` (C++) — localhost control-plane
- `operator/voice` (Python) — mic + HF ASR + send transcript to Rasa
- `rasa run` (Python) — NLU + dialogue
- `rasa run actions` (Python) — executes side effects (HTTP calls)
- MiNiFi sidecar — tails JSONL and ships

This keeps all heavy/non-deterministic work (ASR + dialogue) out of the actuation loop.

---

## 3) Logging layout

### 3.1 `operator_commands.jsonl` (recommended single source)

Keep a single shared stream for operator command attempts.

- HF voice service can log “heard transcript” + ASR metadata.
- Rasa action server can log “final executed intent” + HTTP result.

This makes it easy to correlate:

- what the operator said
- what Rasa decided
- what the gateway accepted
- what mode the system entered (from `learning_events.jsonl`)

### 3.2 Optional: `rasa_dialogue.jsonl`

If you want to ship full dialogue traces (useful for debugging confirmations):

- append Rasa conversation events here
- MiNiFi can downsample/route to low-priority storage

---

## 4) Future work placement: UI and operator gateway

Same recommended layout as earlier docs:

```text
operator/
	gateway/
		app/
			server.py            # REST + WS/SSE; bridges to ros2_http_gateway_node and tails Logs/*.jsonl
	ui/
		mobile/                # React Native (Expo)
		web/                   # React web
		desktop/               # optional wrapper
```

Clients should not depend on ROS2/DDS.

They should talk to the gateway API that:

- proxies `/v1/*` commands
- streams telemetry/status/operator-command events

