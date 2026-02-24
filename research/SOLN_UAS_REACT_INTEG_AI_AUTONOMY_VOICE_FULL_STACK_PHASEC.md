# Phase C — “Voice from UI” via HF ASR + Rasa (React Native TypeScript Walkthrough)

This Phase C guide is the “code walkthrough” version of Phase C from:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`

Goal (Phase C):

- Keep Phase A dashboard and Phase B buttons.
- Add a UI flow where an operator can record a short audio clip on phone/web, upload it, and get back:
  - transcript (HF ASR)
  - intent / confirmations (Rasa)
  - execution result (HTTP to ROS2 gateway)
- Ensure the attempt is written to and streamed from `Logs/operator_commands.jsonl`.

Safety boundary (unchanged):

- UI still does not publish actuation.
- UI sends audio to the Operator Gateway; the gateway runs HF ASR + Rasa + action server and proxies only safe `/v1/*` calls.

---

## 0) Backend requirements for Phase C

Phase C assumes your Operator Gateway provides:

- `POST /api/v1/voice/command` (multipart form-data)
  - input: `audio` file (m4a/wav)
  - output: JSON like:

```json
{
  "transcript": "enable shadow mode",
  "intent": "policy_mode_shadow",
  "requires_confirm": false,
  "ok": true,
  "http_status": 200
}
```

And it streams the final result over WS on the `operator_commands` stream.

---

## 1) Expo dependency

Add Expo AV for recording:

```bash
npx expo install expo-av
```

---

## 2) Add a Voice Panel component

### 2.1 `src/components/VoicePanel.tsx`

```tsx
import React, { useRef, useState } from "react";
import { View, Text, Pressable, Alert } from "react-native";
import { Audio } from "expo-av";
import { API_BASE_URL } from "../api/config";

type VoiceResult = {
  transcript?: string;
  intent?: string;
  requires_confirm?: boolean;
  ok?: boolean;
  http_status?: number;
  error?: string | null;
};

export function VoicePanel() {
  const recordingRef = useRef<Audio.Recording | null>(null);
  const [status, setStatus] = useState<"idle" | "recording" | "uploading">("idle");
  const [last, setLast] = useState<VoiceResult | null>(null);

  async function startRecording() {
    try {
      await Audio.requestPermissionsAsync();
      await Audio.setAudioModeAsync({ allowsRecordingIOS: true, playsInSilentModeIOS: true });
      const rec = new Audio.Recording();
      await rec.prepareToRecordAsync(Audio.RecordingOptionsPresets.HIGH_QUALITY);
      await rec.startAsync();
      recordingRef.current = rec;
      setStatus("recording");
    } catch (e: any) {
      Alert.alert("Mic error", String(e?.message ?? e));
      setStatus("idle");
    }
  }

  async function stopAndSend() {
    const rec = recordingRef.current;
    if (!rec) return;

    setStatus("uploading");
    try {
      await rec.stopAndUnloadAsync();
      const uri = rec.getURI();
      recordingRef.current = null;
      if (!uri) throw new Error("No audio file");

      const form = new FormData();
      form.append("audio", {
        // @ts-expect-error RN file type
        uri,
        name: "command.m4a",
        type: "audio/m4a",
      });

      const res = await fetch(API_BASE_URL + "/api/v1/voice/command", {
        method: "POST",
        body: form,
      });

      const json = (await res.json()) as VoiceResult;
      setLast(json);
      if (!res.ok) Alert.alert("Voice command failed", JSON.stringify(json));
    } catch (e: any) {
      Alert.alert("Upload error", String(e?.message ?? e));
    } finally {
      setStatus("idle");
    }
  }

  return (
    <View style={{ gap: 10, padding: 12, borderWidth: 1, borderRadius: 10 }}>
      <Text style={{ fontWeight: "600" }}>Voice (HF ASR + Rasa)</Text>
      <Text>Status: {status}</Text>

      <Pressable
        onPress={() => void startRecording()}
        disabled={status !== "idle"}
        style={{ padding: 10, borderWidth: 1, borderRadius: 10, opacity: status !== "idle" ? 0.5 : 1 }}
      >
        <Text>Start Recording</Text>
      </Pressable>

      <Pressable
        onPress={() => void stopAndSend()}
        disabled={status !== "recording"}
        style={{ padding: 10, borderWidth: 1, borderRadius: 10, opacity: status !== "recording" ? 0.5 : 1 }}
      >
        <Text>Stop + Send</Text>
      </Pressable>

      <Text numberOfLines={6}>Last: {last ? JSON.stringify(last) : "-"}</Text>
    </View>
  );
}
```

Push-to-talk variant (optional):

- `onPressIn` → `startRecording()`
- `onPressOut` → `stopAndSend()`

---

## 3) Add an Operator screen (optional but clean)

If you want to keep the dashboard from getting too dense:

- Create `src/screens/OperatorScreen.tsx`
- Render `VoicePanel` + `EventLog` filtered to `operator_commands`

Minimal filter idea: in `EventLog` accept an optional `stream` prop.

---

## 4) Make sure voice attempts appear in the same audit stream

Phase C is “correct” when:

- UI shows the immediate HTTP response (transcript/intent/ok)
- A matching structured event is emitted on WS (`stream=operator_commands`)
- The operator command is written to `Logs/operator_commands.jsonl`

This mirrors how your HF-only and Rasa action-server designs already audit operator commands.

---

## 5) Phase C acceptance criteria

- Voice UI can trigger `mission_start`, `mission_stop`, `policy_mode_*`, `episode_*`, and `emergency_stop` through the backend.
- Mixed/Policy mode requires confirmations (Rasa rules and/or UI confirmation UX) and still respects the backend arbiter.
- All voice attempts are observable in one place: `operator_commands` stream and `operator_commands.jsonl`.
