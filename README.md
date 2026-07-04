# XCore

A fluent command-chaining API, X-drive motion algorithm, and generic subsystem classes for
VEX V5 (PROS). Released as a PROS library template.

## Dependencies

XCore requires **LemLib** to be applied to your project first - it reuses `lemlib::PID`,
`lemlib::Pose`, `lemlib::ExitCondition`, and (via dedicated tracking wheels) LemLib's own
odometry, rather than reimplementing them. Only the X-drive-specific motor mixing and the
LemLib-compatibility shim needed to use its odometry on a holonomic drivetrain are original
to XCore.

```bash
pros c add-depot LemLib https://raw.githubusercontent.com/LemLib/LemLib/depot/stable.json
pros c apply LemLib
pros c add-depot XCore <XCore depot URL, once published>
pros c apply XCore
```

Then include everything with:

```cpp
#include "xcore/api.hpp"
```

## What's in here

- `xcore::PID`, `xcore::Pose`, `xcore::ExitCondition` - aliases of the equivalent LemLib
  types (see `include/xcore/types.hpp`).
- `xcore::Subsystem` - base class for hardware-owning subsystems. Runs an internal 10ms task
  loop and exposes `isBusy()`, backed by a private `busy` flag that only clears after the
  target is reached **and** a settle delay (default 75ms) has passed.
- `xcore::XDrive` - holonomic drivetrain subsystem: `directToPose()` (async, with a `Sync`
  variant) and field-centric `arcade()` manual control. Pose tracking is delegated entirely
  to LemLib's own tracking-wheel + IMU odometry (`lemlib::setSensors()`/`init()`) - XDrive
  takes a dedicated vertical + horizontal `lemlib::TrackingWheel*` (not derived from the
  drive motors, since powered X-drive rollers slip too much to track position reliably) and
  internally builds a throwaway LemLib-compatibility shim so a holonomic robot can use
  LemLib's odometry despite it being nominally tank-oriented (see the class doc comment in
  `xdrive.hpp` for how/why).
- `xcore::MCL` - optional, toggleable (`enable()`/`disable()`) Monte Carlo Localization:
  corrects LemLib's tracked pose using 1-4 wall-facing distance sensors on a square field.
  An original particle-filter implementation (motion update from the odometry delta,
  measurement update via a square-field raycast model, low-variance resampling) - not
  derived from any specific reference codebase. Runs its own background task, independent of
  and orthogonal to `XDrive`.
- `xcore::Lift`, `xcore::Pneumatics` - generic subsystems built on `Subsystem`.
- `xcore::Command` / `xcore::Trigger` / `xcore::along()` - the chaining primitives.
- `xcore::RobotInterface` - composes subsystems and provides the `do_()`/`along()`
  orchestrator: `robot.do_(robot.driveTo(x, y, theta, timeout), {along(0.5, callback)})`
  blocks the calling task until the primary command settles (or times out), firing each
  trigger once its progress threshold is crossed.
- `xcore::config` (`include/xcore/config.hpp` + `src/xcore/config.cpp`) - the single place to
  tune the robot: drivetrain gearing, the 2 XDrive PIDs + stop-detection thresholds, the Lift
  PID, tracking wheel geometry, and MCL settings/sensor offsets. Ships with placeholder
  values (marked `TODO`) until tuned against real hardware - `DRIVE_WHEEL_RPM` is the one
  concrete confirmed figure (green cartridge x 48:24 external gear = 400 RPM). Note PROS
  itself has no concept of "external gear ratio" - `get_actual_velocity()`/`get_position()`
  only ever reflect the motor's own output shaft, so this is computed explicitly here rather
  than left as an implicit assumption anywhere else.

> **Naming note:** the original design used `robot.do(...)`, but `do` is a reserved C++
> keyword and can't be used as an identifier - it's `do_` here instead.

> **Architecture note:** MCL and Config.cpp were originally planned to live in a separate
> robot-specific repo (OverrideBot-4862A), keeping XCore fully generic/reusable. That's since
> changed by design decision - both now live here, so XCore ships with this team's actual
> tuned values rather than being a values-agnostic template. Other teams reusing XCore should
> expect to edit `config.cpp` for their own robot.

## Chaining S-curves

`DriveCommand::chain(minSpeed, proximityRadius)` marks a `driveTo()` call as a non-terminal
waypoint: instead of waiting for XDrive's full stop-settle, `isSettled()` returns true as
soon as the robot is within `proximityRadius` inches of that waypoint, so the next queued
movement can start immediately - producing a smooth S-curve rather than a stop-start motion.

## Known tuning TODOs

- All values in `config.cpp` are placeholders except `DRIVE_WHEEL_RPM` (confirmed: green
  cartridge + 48:24 external gear) - PID gains, stop-detection thresholds, tracking wheel
  offsets, and MCL sensor offsets/noise all need measuring/tuning on the real robot.
- MCL's expected-distance model assumes a clear line of sight to the field wall; it will be
  biased by goals/obstacles in a sensor's path - not solved here, flagged as future work.
- Motor/sensor ports in `src/main.cpp` are placeholders pending final wiring.

## Building/testing standalone

`src/main.cpp` represents the real Override robot's drivetrain (not a separate bench-test
chassis) - it's excluded from the packaged library template (`EXCLUDE_SRC_FROM_LIB` in the
Makefile), so you can build and upload just XCore to validate the chaining API, XDrive, and
MCL in isolation without needing the full OverrideBot-4862A repo:

```bash
pros make
pros upload
```

## Publishing a release

`deploy-depot.bat` (gitignored - machine-specific paths, run manually per dev) builds the
library, packages a versioned zip via `pros conduct create-template --zip`, and
creates/updates a GitHub Release via the `gh` CLI. It does not yet regenerate the depot's
`stable.json` - update that manually for now (or automate later with a GitHub Action, see
[`LemLib/pros-depot`](https://github.com/LemLib/pros-depot) for reference).
