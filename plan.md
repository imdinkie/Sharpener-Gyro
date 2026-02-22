# Sharpener-Gyro + Android USB Implementation Plan

## Objectives

1. Keep the current sharpening vibration filtering behavior (accel-only, MPU6050 DLPF ~10 Hz) while improving responsiveness with low-risk changes.
2. Add a wired USB serial telemetry/control path to the ESP32 MicroPython firmware for Android integration.
3. Build an Android app (`/home/justin/AndroidStudioProjects/SharpeningAngleTool`) that connects over USB serial, displays live angle data, and provides core controls (mode switch + recalibrate) with a UI similar in spirit to the current web app.
4. Preserve existing web/AP functionality and avoid breaking the browser workflow.
5. Verify code quality by compiling/checking both firmware Python files and Android app builds.

## Constraints / Decisions (Locked In)

- Stay **accel-only** for now (no gyro fusion in this implementation).
- Keep MPU6050 DLPF configuration at **0x05 (~10 Hz)** to preserve vibration rejection.
- Use a **balanced tuning** profile:
  - `READ_PERIOD_MS` from `50` -> `40` ms
- Android transport: **USB serial only first** (no Wi-Fi fallback in the app yet).
- Firmware mode naming remains canonical `AXIS_X / AXIS_Y / AXIS_Z`, with legacy alias support for `PITCH/ROLL/YAW`.

## Firmware Implementation (Sharpener-Gyro repo)

### A. Sensor robustness and observability

1. Add MPU6050 I2C address auto-detection at boot (`0x68` / `0x69`) before tracker creation.
2. Expose selected address and sensor health metrics in runtime state.
3. Add `/status` HTTP endpoint returning:
   - current mode
   - sensor address
   - latest delta / age
   - read failure count
   - read success count
   - serial streaming state

### B. Timing and latency improvements (low-risk)

1. Set `READ_PERIOD_MS = 40`.
2. Separate measurement cadence from client broadcasting:
   - measurement loop updates shared latest sample at fixed cadence
   - SSE push loop broadcasts latest sample independently
3. Keep existing jitter logging and extend status counters where useful.

### C. USB serial protocol for Android

1. Add line-delimited JSON telemetry output over serial (stdout), e.g.:
   - `{"type":"telemetry","t_ms":..., "delta":..., "age_ms":..., "mode":"AXIS_Y", "ok":true}`
2. Add line-delimited JSON command input parser over serial (stdin):
   - `{"cmd":"ping"}`
   - `{"cmd":"status"}`
   - `{"cmd":"recalibrate"}`
   - `{"cmd":"set_mode","mode":"AXIS_X|AXIS_Y|AXIS_Z"}`
   - `{"cmd":"stream","enabled":true|false}`
3. Add JSON command responses / errors over serial.
4. Ensure serial tasks are non-blocking and do not stall the measurement loop.
5. Default serial streaming to **disabled** on boot (Android app will enable it).

### D. Backward compatibility

1. Keep existing HTTP endpoints unchanged:
   - `/angle`
   - `/events`
   - `/angle-mode`
   - `/recalibrate`
2. Preserve legacy mode aliases in both firmware and web UI behavior.

## Android App Implementation (SharpeningAngleTool repo)

### A. Project setup

1. Add USB serial library dependency (`usb-serial-for-android`) and any supporting AndroidX dependencies if needed.
2. Update AndroidManifest for USB host support and app strings as needed.
3. Add USB device filter resource for CH340/CP210x/common serial devices (and CDC ACM class fallback).

### B. USB serial transport layer

1. Implement a transport/service class that:
   - discovers USB serial devices with `UsbSerialProber`
   - requests USB permission
   - opens the first port
   - configures serial (start with `115200 8N1`)
   - reads/writes asynchronously
2. Parse line-delimited JSON frames from firmware telemetry.
3. Provide reconnect/disconnect status and error propagation to UI state.
4. Support sending commands (`ping`, `status`, `recalibrate`, `set_mode`, `stream`).

### C. UI (similar to web experience)

1. Compose UI with:
   - large angle display
   - deviation from target
   - target angle input
   - axis mode buttons (`Axis X/Y/Z`)
   - recalibrate button
   - connect/disconnect button
   - connection status / error text
   - optional raw telemetry info (age, update rate)
2. Basic local settings persistence (target, selected axis) using `SharedPreferences`.
3. Keep visual style close to the web UI intent (high contrast, large readout, practical controls).

### D. Command/response behavior

1. On connect:
   - request port permission
   - open serial
   - send `{"cmd":"status"}`
   - send `{"cmd":"stream","enabled":true}`
2. On disconnect/app background:
   - send `stream=false` if possible
   - close port cleanly

## Git / Branch / Commits Plan

### Sharpener-Gyro repo

1. Create feature branch for firmware work.
2. Commit `plan.md`.
3. Commit firmware latency + status changes.
4. Commit firmware serial protocol support.
5. Commit any firmware cleanup/fixes after testing.

### Android repo

1. Create feature branch for Android implementation.
2. Commit dependency + manifest + USB plumbing.
3. Commit app UI and state management.
4. Commit fixes after build/test verification.

All commit messages should be descriptive and implementation-specific.

## Verification Plan

### Firmware repo

1. `python3 -m py_compile main.py reader.py`
2. Sanity check JSON protocol code paths by static review + syntax validation.
3. (Optional hardware) deploy to ESP32 and inspect boot logs / `/status`.

### Android repo

1. `./gradlew assembleDebug`
2. `./gradlew testDebugUnitTest` (if quick / no failures from template issues)
3. Verify app compiles with USB host/serial integration classes.

## Final Deliverables

1. Updated firmware with balanced latency tuning, I2C fallback detection, `/status`, and USB serial JSON protocol.
2. Android app capable of connecting over USB serial and controlling/viewing the sharpener angle stream.
3. Verified builds/compilation results and usage instructions for both firmware and Android app.
