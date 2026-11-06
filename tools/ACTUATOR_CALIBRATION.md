# Single-axis actuator calibration

`actuator_calibration` records velocity commands and encoder responses without
camera acquisition, inference, target tracking, followers, automatic homing or
end-stop calibration. It does not change `system.ini` or tuning constants.

**Default operation is a dry run.** It parses and validates a separate plan,
prints its scope and exits without constructing a hardware adapter, opening USB,
creating a recording directory, enabling power stages or addressing a drive.

## Safety boundary

This is a diagnostic tool, **not a safety-rated motion controller**. Do not arm
it until an operator has reviewed the plan and the actual mechanism:

- Clear the entire possible sweep; remove hazardous payloads. Provide an
  independent emergency stop and mechanical travel protection. Support any axis
  that can fall under gravity: both setup and shutdown **disable torque**.
- Verify absolute encoder bounds in the drive's current coordinate system.
  The harness does not zero encoders or discover limits. Bounds from a previous
  zero/calibration session are not automatically valid.
- Verify stopping distance and margin at the chosen speed, torque and load.
  The software guard includes a communication allowance, but cannot establish
  physical stopping distance or guarantee that an SDK call returns on time.
- Only one CANdle USB adapter may be connected: the pinned SDK attachment API
  selects a matching adapter rather than a user-specified serial number.
  Confirm the selected node ID and bus rate. Duplicate IDs are not supported.
- Stop other bus clients and independently disable other axes before arming.
  This tool does not inspect or stop other programs, address other motors, or
  enable a PDS. It cannot prevent another client issuing conflicting commands.
- Supply reviewed velocity PID and torque values, not guessed values. Commands
  are **drive rad/s**, without the application's velocity scale multiplier.

The hardware build requires **both** `--arm` and terminal entry of the exact
confirmation `ARM NODE <id>`. Piped confirmation is rejected. There is no retry,
automatic resume, or arming by a configuration-file flag. Output must be a new
directory. No hardware is opened until the plan, confirmation and output setup
have succeeded.

Setup disables only the selected node, checks SDK results, configures velocity
mode/PID/torque, requests zero, validates position and speed, then enables and
verifies velocity mode. The programmed PID/mode/torque are not restored or saved
to flash; the axis remains disabled afterward. Restore the normal reviewed drive
configuration before another application uses it.

## Create a reviewed plan, not an executable default

There is deliberately no ready-to-arm plan for Thor. Copy this template to a
private file and replace every placeholder with independently verified values.
It will fail validation while placeholders remain. Timing values below illustrate
the file format; they are not evidence that a particular axis is safe to operate.

```ini
[calibration]
node_id=<verified single drive ID>
datarate_mbps=<1, 2, 5 or 8>
min_position_rad=<verified absolute lower bound>
max_position_rad=<verified absolute upper bound>
max_excursion_rad=<maximum displacement from initial position>
stop_margin_rad=<validated physical stopping margin>
max_observed_velocity_rad_s=<abort threshold above every planned amplitude>
settle_velocity_rad_s=<stationary threshold consistent with encoder noise>
max_torque_nm=<reviewed torque limit>
velocity_kp=<reviewed drive velocity PID gain>
velocity_ki=<reviewed drive velocity PID gain>
velocity_kd=<reviewed drive velocity PID gain>
velocity_integral_max=<reviewed integral limit>
sample_ms=10
max_io_ms=20
max_sample_gap_ms=50
pulse_ms=100
settle_ms=500
settle_timeout_ms=3000
max_run_ms=120000
repetitions=3
amplitude_count=2
amplitude_1_rad_s=<first positive amplitude>
amplitude_2_rad_s=<second positive amplitude>
```

Every field is required and finite. There may be 1–8 amplitudes and 1–20
repetitions. Each amplitude produces repeated positive/negative pulse pairs.
Sample periods must be 2–20 ms, pulse duration at most 1 s, settling timeout at
most 10 s, and total experiment duration at most 300 s. These are software
sanity bounds, **not safe mechanical operating limits**.

Dry-run validation, with no hardware access:

```sh
build/actuator_calibration --plan /path/to/reviewed-plan.ini
```

The tool starts with a settling interval, pulses one axis, then requests zero
until measured velocity remains within the stationary threshold for the full
settling duration. Each pulse requires a new settled interval; the final pulse
does too. There is no return-to-origin motion. Drift across trials can cause the
excursion guard to abort; do not automatically recenter and resume.

On every cycle, position and velocity have separate exchanges and timestamps.
Read/write failure, nonfinite feedback, overspeed, travel-guard approach, excessive
exchange duration, sample gap, unsettled timeout, total timeout or SIGINT/SIGTERM
aborts the sequence. The tool attempts zero **and** disable even if zero fails.
The last-resort destructor repeats these attempts if explicit shutdown failed.
Neither SDK success nor process exit proves physical motion has stopped.

The guard distance is `stop_margin_rad + max_observed_velocity_rad_s ×
(max_sample_gap_ms + 2 × max_io_ms) / 1000`. Both absolute bounds and excursion
from the initial position are checked against it. Values exactly on a guard
boundary are rejected. This calculation assumes the chosen speed and I/O bounds
hold; it is not a substitute for mechanical limits. Deadlines are detected
**after blocking SDK calls return**. SIGKILL, power loss, a hung driver, or another
bus client can defeat software stopping. Verify an independent hardware watchdog
and emergency stop; this tool neither configures nor validates them.

## Records and analysis contract

A run writes:

- `plan.ini`: the exact loaded plan, unaffected by later edits to the input file.
- `run.txt`: simulation/hardware label, initial `state=incomplete`, final state,
  reason, event count and zero/disable SDK success. Read the **last** state value.
- `events.csv`: operation, begin/end host-monotonic seconds, value, SDK status,
  and zero-based pulse ID (`-1` before the first pulse).

Position uses radians, velocity/commands rad/s, and torque Nm. Command values are
the floats passed to the SDK. The pinned SDK uses **1 for success**, not 0. The
simulator uses the same status convention but is explicitly labelled simulation.
The clock is `steady_clock` time since its epoch, not wall time or the existing
tracking telemetry's process-relative epoch.

Trace storage is preallocated and bounded. Events are written only **after**
stopping/disabling to keep trace disk I/O out of the measurement loop. SDK internals
may still allocate or log. A crash or force kill can lose all buffered events;
an incomplete run is not valid calibration evidence. Capture compiler/revision,
executable hash, firmware, hardware load, power mode and temperature in operator
notes alongside the run; the harness does not infer these from source files.

For fitting, use actual command-transition timestamps and hold commands between
writes. Keep position and velocity exchange intervals separate. Bracket command
application by write start/end; do not treat a host midpoint as a drive timestamp.
Fit individual pulse responses in both directions, retain stationary/no-motion
trials as stiction evidence, and validate on held-out repetitions. Report
uncertainty, sample spacing and failed trials. Do not feed this CSV to
`telemetry_analyse.py`, which expects tracking telemetry's different schema.
This tool gathers evidence; it does not automatically tune compensation or gains.

Capture/readout latency remains a separate, motors-disabled experiment using an
independently timestamped optical event. This harness cannot calibrate camera
delay, and intentionally contains no trigger/camera implementation.

## Hardware-free verification

With `PATRONUS_REPLAY_ONLY=ON`, the same target compiles a first-order simulated
axis instead of the SDK adapter. It has **no CANdle/USB transport linked** and
cannot arm real hardware. Simulation requires `--simulate`; otherwise it too
only validates the plan. Fault injection exists only in this build.

```sh
cmake --build build/replay -j2
ctest --test-dir build/replay --output-on-failure
```

`tools/test_actuator_calibration.py` supplies synthetic test plans, checks pulse
sign/amplitude/order, settling, timestamp intervals, no-overwrite behavior,
invalid plans, fault aborts and signal shutdown. Those values are not hardware
recommendations. The production target is built by the normal Thor CMake build;
it must not be executed in CI with arming flags.
