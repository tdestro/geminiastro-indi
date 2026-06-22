# GeminiAstro Hardware Validation Results

Date: 2026-06-21 local time.

Controller under test:

- GeminiAstro EAF Automatic Star Focuser Pro controller
- USB serial adapter: CH340 / QinHeng `1a86:7523`
- Stable test path: `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- Firmware readback: `F324#`
- Model readback: `FmyFP2ULN2003\r\n324#`
- Test host: MeLE mini PC running Debian

Mechanical installation:

- Sky-Watcher Mak 180 primary-mirror focus knob.
- Confirmed coupling: `4096` GeminiAstro steps per focus-knob turn.
- Measured OTA travel: about `34` knob turns between torque-rise endpoint regions.
- Current trusted midpoint after validation: physical turn about `17` from the counter-clockwise reference, controller logical position `9000`.
- The controller is open-loop. It does not report torque, missed steps, coupler slip, or absolute mirror position.

## Build And Protocol Tests

Passed:

- Local protocol-only CMake build.
- Local protocol unit test.
- Full INDI driver build on the MeLE.
- MeLE protocol unit test.
- Read-only `geminiastro_probe` against the real controller.

The MeLE build showed clock-skew warnings because the laptop and MeLE clocks differ. Clean builds were used, and tests passed.

## Motion And Limit Tests

Passed through INDI:

- Temporary no-config startup safe window around current position.
- Invalid saved `FOCUS_SAFE_LIMITS` rejected without expanding the effective range.
- Absolute moves inside configured safe limits.
- Relative moves in both directions inside configured safe limits.
- Abort during a bounded move.
- Idle USB disconnect/reconnect using software CH341 unbind/bind.
- USB disconnect during a bounded move using software CH341 unbind/bind.
- Jog-in and jog-out commands with explicit jog stop.
- Home command safety path: with effective limits `8825..9175`, home moved inward and the driver aborted at `8825`.

Validation note: USB disconnect testing used software unbind/bind of the CH341 interface, not a physical cable pull or external power removal.

## Controller Settings Tests

Passed through INDI with readback:

- Coil power on/off.
- Reverse on/off.
- Step-size enable/disable.
- Speed presets slow/medium/fast.
- Numeric motor speed.
- Step mode.
- Step size. `:195.40#` restores readback `T5.40#`.
- Delay after move.
- Temperature precision.
- Temperature-compensation value, enable/disable command path, and direction command path.
- Backlash step values and enable/disable command paths.
- LCD/display enable/disable.
- LCD update-while-moving enable/disable.
- Delayed display update enable/disable command path.
- LCD page display time `2000`, `3000`, and `4000`.

The installed focuser has no physical LCD. Display commands were validated by controller readback only.

`GEMINIASTRO_DISPLAY.LCD_PAGE_DISPLAY_TIME` accepts discrete values:

| Value | Command | Readback |
| ---: | --- | --- |
| `2000` | `:352#` | `X2000#` |
| `3000` | `:353#` | `X3000#` |
| `4000` | `:354#` | `X4000#` |

The earlier generic `:35<ms>#` assumption was wrong and was fixed.

## Persistence And Reset

Passed through INDI:

- `WRITE_EEPROM`
- `WRITE_DEFAULTS`
- `RESET_CONTROLLER`
- Baseline restore after defaults/reset.
- Final `WRITE_EEPROM` after baseline restore.

Observed `WRITE_DEFAULTS` effects on the tested controller:

- Logical position changed to `5000`.
- Coil power changed to enabled.
- Motor speed changed to fast (`C2#`).
- Step size changed to `T50.00#`.

The driver/test restored the trusted logical position to `9000` with `FOCUS_SYNC`, restored the baseline settings, and wrote them back to EEPROM.

Observed `RESET_CONTROLLER` behavior:

- After `:40#`, the controller required a DTR/RTS serial-line pulse before answering commands again.
- The driver now performs that pulse in the reset path and also after failed handshake attempts.
- Reset recovery passed without touching cables.

## Final Controller Baseline

Final probe after validation:

```text
:02# -> EOK#
:03# -> F324#
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
:43# -> C0#
:72# -> 30#
:74# -> 40#
:76# -> 50#
:78# -> 60#
:80# -> 70#
:37# -> D1#
:34# -> X2000#
:62# -> L0#
```

Final state is idle at logical position `9000`, with reverse disabled, coil power disabled, step-size mode disabled, speed slow, and LCD page time `2000`.

## Residual Limits

- This validation proves the tested controller and Mak 180 rig, not every GeminiAstro hardware variant.
- Temperature compensation command paths were validated, but the controller reports temperature compensation unavailable (`A0#`), so no autonomous compensation movement was validated.
- Backlash command paths were validated by readback. Practical backlash behavior was not optically characterized.
- The home switch reports `H1#` in midrange and is not trusted as a physical endpoint sensor for this Mak 180 installation.
- Physical power-loss behavior was not tested by yanking power during this run. USB disconnect was tested by software unbind/bind.
