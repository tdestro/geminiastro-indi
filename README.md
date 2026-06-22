# GeminiAstro INDI Focuser Driver

[![CI](https://github.com/tdestro/geminiastro-indi/actions/workflows/ci.yml/badge.svg)](https://github.com/tdestro/geminiastro-indi/actions/workflows/ci.yml)

MIT-licensed INDI focuser driver for GeminiAstro / EAF Automatic Star Focuser Pro serial controllers.

## Status

This driver implements the tested GeminiAstro serial control surface. It has been build-tested as a full
INDI driver and bench-tested against one real GeminiAstro focuser controller. Hardware validation results are recorded in
[docs/hardware-validation-results-2026-06-21.md](docs/hardware-validation-results-2026-06-21.md).

Tested hardware and protocol characteristics:

- USB serial adapter: CH340 / QinHeng `1a86:7523`
- Serial settings: `9600 8N1`
- Command and response terminator: `#`
- Temperature probe connected and reporting through the controller
- INDI absolute and relative movement confirmed in both physical directions on the test bench

Implemented:

- Serial INDI focuser driver.
- Handshake, firmware, controller-model, position, moving-state, max-position, max-increment, and temperature reads.
- Absolute move with `:05<position>#`.
- Relative move by reading current position, clamping, then using absolute move.
- Abort with `:27#`.
- Sync current logical position with `:31<position>#`.
- Driver-enforced safe limits separate from the controller-reported software range.
- Temporary startup safe window around the current position when configured safe limits are absent.
- Assisted safe-limit calibration controls using bounded, user-commanded nudges and explicit endpoint capture.
- Home-switch and stepper-power readback for status only.
- Jog and home commands with polling-loop abort at the effective safe-limit edge.
- Read-only controller diagnostics for coil power, reverse state, temperature-compensation state, jog state, step mode/size, motor speed, delay-after-move, backlash state/steps, and display state.
- Named INDI controls for GeminiAstro motor settings, jog/home, temperature compensation, backlash, display, and persistence commands.
- Controller reset recovery using the DTR/RTS serial-line pulse required by the tested controller after `:40#`.
- Protocol formatting/parsing tests that do not require INDI.
- POSIX command-line probe utility for Linux/macOS bench tests.

## Controller Control Surface

The driver exposes GeminiAstro controller settings and actions as explicit INDI properties:

- `GEMINIASTRO_MOTOR_SETTINGS`: controller max position, motor speed, step mode, step size, delay after move.
- `GEMINIASTRO_MOTOR_SWITCHES`: coil power, reverse direction, step-size enable, speed presets.
- `GEMINIASTRO_JOG`: home, jog in, jog out, jog stop.
- `GEMINIASTRO_TEMP_COMP`: temperature precision code and temperature-compensation steps.
- `GEMINIASTRO_TEMP_COMP_SWITCHES`: temperature-compensation enable and direction.
- `GEMINIASTRO_BACKLASH`: backlash steps.
- `GEMINIASTRO_BACKLASH_SWITCHES`: backlash enable/disable.
- `GEMINIASTRO_DISPLAY`: LCD page display time and page option.
- `GEMINIASTRO_DISPLAY_SWITCHES`: LCD/display behavior commands.
- `GEMINIASTRO_PERSISTENCE`: write EEPROM, write defaults, reset controller.

These properties send the controller command after value validation and then refresh controller diagnostics. Normal INDI
absolute/relative focuser moves still use the configured safe-limit clamp. Jog and home commands are continuous controller
motion commands; the driver polls position and aborts if the focuser reaches the effective safe-limit edge. Firmware
update support is not part of this driver.

Notes from the tested controller:

- `GEMINIASTRO_DISPLAY.LCD_PAGE_DISPLAY_TIME` accepts only `2000`, `3000`, or `4000` milliseconds. The corresponding
  controller commands are `:352#`, `:353#`, and `:354#`.
- `GEMINIASTRO_PERSISTENCE.WRITE_DEFAULTS` changes controller settings immediately. On the tested controller it changed
  logical position to `5000`, enabled coil power, set speed to fast, and set step size to `50.00`.
- `GEMINIASTRO_PERSISTENCE.RESET_CONTROLLER` requires a DTR/RTS serial-line pulse after reset on the tested controller.
  The driver performs that recovery and refreshes startup values.

## Safety Model

The controller reports a broad software range, but that range is not proof of safe physical travel for the attached focuser or optical instrument.

The driver safety policy is:

- If no valid `FOCUS_SAFE_LIMITS` are configured, expose only a temporary `current +/- 50` step range.
- If configured limits are valid, expose `MIN_POSITION + GUARD_STEPS` through `MAX_POSITION - GUARD_STEPS`.
- Clamp every absolute and relative move to the effective safe range before sending `:05<position>#`.
- Abort any busy motion, including jog/home, if the polled position reaches the effective safe range edge.
- During safe-limit calibration, allow only explicit bounded nudges inside the controller-reported range.
- Apply calibrated limits only after both candidate endpoints have been captured and validated.
- Treat logical position sync/recenter as a non-moving coordinate change that clears existing configured limits.
- Treat the home-switch response as informational unless a user has independently proven that it maps to a safe hardware limit on their installation.
- Never blind-home, seek hard stops, or infer physical endpoints from step count alone.

Users must establish safe limits for their own mechanical installation before commanding larger moves.

The controller does not report load, torque, motor current, missed steps, or independent physical encoder position.
If the motor stalls against a hard stop, the controller may still report logical step movement.
For that reason, calibration is supervised and confirmatory; it is not automatic hard-stop detection.
The controller's logical `0..max` coordinate range is not assumed to match the physical mechanical range.

## Build

Protocol tests only:

```sh
cmake -S . -B build -DBUILD_INDI_DRIVER=OFF
cmake --build build
ctest --test-dir build --output-on-failure
```

Full INDI driver build requires INDI development headers and libraries:

```sh
cmake -S . -B build -DBUILD_INDI_DRIVER=ON
cmake --build build
sudo cmake --install build
```

On Debian-style systems:

```sh
sudo apt install libindi-dev
```

## Bench Probe

The probe utility sends only read commands:

```sh
./build/geminiastro_probe /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

Use the stable serial path for your own controller. On Linux, inspect `/dev/serial/by-id/` after plugging in the focuser.

## Running With INDI

After installation:

```sh
indiserver indi_geminiastro_focus
```

Set the serial port in the INDI client to the stable `/dev/serial/by-id/...` path for the GeminiAstro controller.

On first connection, inspect:

- `ABS_FOCUS_POSITION`
- `FOCUS_SAFE_LIMITS`
- `FOCUS_LIMIT_STATUS`
- `FOCUS_LIMIT_CALIBRATION_STEP`
- `FOCUS_LIMIT_CALIBRATION_STATUS`
- `FOCUS_LIMIT_CALIBRATION`
- `FOCUS_TEMPERATURE`
- `FOCUSER_CONTROLLER_INFO`
- `FOCUSER_CONTROLLER_STATUS`
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

Configure `FOCUS_SAFE_LIMITS` before commanding moves larger than the temporary startup window.

Safe-limit calibration sequence:

1. Set `FOCUS_LIMIT_CALIBRATION_STEP.STEP_SIZE` for the current phase of calibration.
2. Use coarse steps while clearly far from an endpoint, then reduce the step size near the endpoint.
3. Use `FOCUS_LIMIT_CALIBRATION.NUDGE_INWARD` and `NUDGE_OUTWARD` while watching the focuser.
4. Stop well before a mechanical hard stop, then use `CAPTURE_MIN` at the lower logical endpoint.
5. Move to the upper logical endpoint in supervised nudges, stop well before the hard stop, then use `CAPTURE_MAX`.
6. Check `FOCUS_LIMIT_CALIBRATION_STATUS.READY_TO_APPLY`.
7. Use `APPLY_LIMITS` to copy the captured endpoints into `FOCUS_SAFE_LIMITS`.
8. Save the INDI configuration after confirming normal moves remain inside the safe range.

`RESET_CANDIDATES` clears only the calibration candidate endpoints. It does not change already-applied safe limits.
The step-size maximum is derived from the controller-reported maximum increment.
`CENTER_POSITION` syncs the current physical position to the controller midpoint without moving the focuser, clears candidate
endpoints, and clears configured safe limits so calibration can restart in the new logical coordinate frame.

## Tested Controller Responses

Observed read-only responses from the test controller:

```text
:02# -> EOK#
:03# -> F324#
:04# -> FmyFP2ULN2003\r\n324#
:08# -> M10000#
:10# -> Y10000#
:00# -> P9000#
:01# -> I0#
:06# -> Z20.00#
:11# -> O0#
:13# -> R0#
:21# -> Q10#
:24# -> 10#
:25# -> A0#
:26# -> B0#
:38# -> b1#
:83# -> c0#
:87# -> k1#
:63# -> H1#
:66# -> K0#
:68# -> V0#
:89# -> 91#
:29# -> S1#
:32# -> U0#
:33# -> T5.40#
:34# -> X2000#
:37# -> D1#
:43# -> C0#
:62# -> L0#
:72# -> 30#
:74# -> 40#
:76# -> 50#
:78# -> 60#
:80# -> 70#
```

The observed physical movement test used bounded INDI moves and returned to the starting position. It confirms this driver can actuate the tested controller through INDI, but it does not establish safe travel limits for other installations.

## Hardware Validation

Validation plan: [docs/hardware-test-plan.md](docs/hardware-test-plan.md).

Real-controller results: [docs/hardware-validation-results-2026-06-21.md](docs/hardware-validation-results-2026-06-21.md).
