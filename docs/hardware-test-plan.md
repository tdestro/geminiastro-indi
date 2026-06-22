# GeminiAstro Hardware Validation Plan

This checklist validates `indi_geminiastro_focus` against a real GeminiAstro controller before production use.

Current tested-controller results are recorded in
[hardware-validation-results-2026-06-21.md](hardware-validation-results-2026-06-21.md).

## Test Rig Facts

These facts describe the current test rig, not generic driver defaults:

- Focuser/coupling: `4096` GeminiAstro steps per focus-knob turn.
- Telescope installation: Sky-Watcher Mak 180 primary-mirror focuser.
- Observed travel: about `34` focus-knob turns between endpoint regions.
- Practical clockwise endpoint region: about `33.75` turns from the counter-clockwise reference.
- Current trusted midpoint used for testing: about turn `17`.

The GeminiAstro controller is open-loop. It does not report torque, missed steps, coupler slip, or absolute physical
mirror position. If physical position trust is lost, re-establish the counter-clockwise reference manually and sync before
running motion tests.

## Preflight

1. Build the driver with INDI enabled:

   ```sh
   cmake -S . -B build -DBUILD_INDI_DRIVER=ON
   cmake --build build
   ctest --test-dir build --output-on-failure
   ```

2. Confirm the stable serial path exists:

   ```sh
   ls -l /dev/serial/by-id/
   ```

3. Run the read-only probe:

   ```sh
   ./build/geminiastro_probe /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
   ```

4. Start `indiserver` on a disposable port and connect the driver with the stable serial path.

5. Confirm on first connection:

   - handshake succeeds
   - position reads successfully
   - moving state is idle
   - temperature reads successfully if the probe is connected
   - `FOCUSER_CONTROLLER_STATUS` populates
   - `FOCUS_SAFE_LIMITS` is configured before moves larger than the temporary startup window

## Automated Bounded Tests

These tests may run unattended only inside the known midrange window.

- Absolute move inside safe limits.
- Relative move in both directions inside safe limits.
- Abort during a bounded long move.
- `indiserver` stop/start and reconnect.
- USB disconnect/reconnect while idle.
- Invalid saved `FOCUS_SAFE_LIMITS` config falls back to a temporary safe window.
- Missing saved `FOCUS_SAFE_LIMITS` config falls back to a temporary safe window.

Every test must write logs that survive process restart and must restore the focuser to the trusted midpoint if the
controller position remains trusted.

## Controller Feature Tests

Run each named INDI controller property and record the command result plus readback/status refresh.

- `GEMINIASTRO_MOTOR_SETTINGS`
- `GEMINIASTRO_MOTOR_SWITCHES`
- `GEMINIASTRO_JOG`
- `GEMINIASTRO_TEMP_COMP`
- `GEMINIASTRO_TEMP_COMP_SWITCHES`
- `GEMINIASTRO_BACKLASH`
- `GEMINIASTRO_BACKLASH_SWITCHES`
- `GEMINIASTRO_DISPLAY`
- `GEMINIASTRO_DISPLAY_SWITCHES`
- `GEMINIASTRO_PERSISTENCE`

Jog and home tests must use short runtime ceilings and immediate stop/abort behavior. They must not depend on driving
into an endpoint region.

## Pass Criteria

- Protocol tests pass.
- Full INDI driver builds.
- Read-only probe succeeds against the real controller.
- Normal INDI absolute and relative moves remain clamped to effective safe limits.
- Jog and home commands abort at the effective safe-limit edge.
- Each controller write either produces expected readback/status or marks the relevant INDI property `IPS_ALERT`.
- Disconnect/reconnect and `indiserver` restart do not expand safe movement range.
- Any loss of physical position trust is recorded and requires manual reference recovery before more movement.
