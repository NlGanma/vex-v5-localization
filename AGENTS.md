# AGENTS.md

Future Codex agents: read this first. This file was written on 2026-05-02 to preserve repo context for later chats.

## Project Boundary

- The real Git/project root is `/Users/ouji/Documents/Localization Test`.
- The user may start Codex inside `/Users/ouji/Documents/Localization Test/MCL + EKF + Odom Newly improved`. That subdirectory currently looks like a copied/pruned build artifact tree: it has `bin/`, `.d/`, `.vscode/`, `compile_commands.json`, `include/`, and `src/`, but its `include/` and `src/` trees contain no source except `.DS_Store`.
- If you are in the `MCL + EKF + Odom Newly improved` folder, go up one level before doing source work. The real source is in the parent root's `src/` and `include/`.
- The parent root is a PROS V5 robot project with a customized LemLib fork, MCL + EKF + odometry localization, PTO drive/multifunction motors, and localization tuning/export tooling.
- There are sibling/legacy folders such as `MCL + Odom`, `Merge`, and `MCL + EKF + Odom Newly improved`. Do not edit them unless the user explicitly asks; they are not the active source tree.

## Git And Generated Files

- Git root: `/Users/ouji/Documents/Localization Test`.
- `.gitignore` ignores `bin/`, `.d/`, `.vscode/`, `.cache/`, `compile_commands.json`, `*.bin`, `*.elf`, `*.o`, docs build output, and `.DS_Store`.
- Current checked source of interest is under root `src/`, `include/`, `tools/`, `docs/`, plus `Makefile`, `common.mk`, `project.pros`.
- Treat `bin/`, `.d/`, `.cache/`, `compile_commands.json`, and `docs/_build` as generated. Do not hand-edit them.
- Be careful with macOS `.DS_Store` files; ignore them.
- Before editing, run `git status --short` from the root. The user may have local changes. Do not revert unrelated work.

## Build And Toolchain

- This is a PROS V5 project. `project.pros` says `project_name` is `MCL+EKF+`, target `v5`, kernel `4.2.1`, upload slot `1`, icon `robot`.
- The top-level `Makefile` configures a LemLib library template: `IS_LIBRARY:=1`, `LIBNAME:=LemLib`, `VERSION:=0.5.6`, `USE_PACKAGE:=1`.
- `README.md` is upstream-ish LemLib documentation and may mention older version badges. Trust the `Makefile` for the active library version.
- C++ standard is `gnu++23` from `common.mk`; the VS Code config says `gnu++20`, but the build uses C++23.
- Target flags include `arm-none-eabi`, Cortex-A9, `-mfpu=neon-fp16`, hard float, `-Os`, `-g`, `-mthumb`.
- Preferred build command after C++ changes: `make quick` from the root.
- Other useful make targets: `make all`, `make clean`, `make library`, `make template`.
- A dry run on 2026-05-02 reported `make: Nothing to be done for 'quick'.` and the same for `library`, so the checked build output was up to date then.
- If the ARM toolchain is missing, check `/Users/ouji/.local/arm-gnu-toolchain/bin` first. `Makefile` prepends this path when `arm-none-eabi-g++` exists.
- The generated package artifacts are normally `bin/hot.package.bin`, `bin/cold.package.bin`, `bin/hot.package.elf`, `bin/cold.package.elf`, and `bin/LemLib.a`.
- For robot terminal logs, the user has used `pros terminal`.
- For upload, use normal PROS tooling only if the user asks or confirms hardware is connected.

## Source Map

Important user/project files:

- `src/main.cpp`: PROS lifecycle callbacks. Initializes robot control, PTO, localization, tune runtime, chassis calibration, and safe stop behavior.
- `src/robot.cpp`, `include/robot.hpp`: physical hardware ports, motor groups, tracking wheels, drivetrain, controller gains, chassis singleton, PTO release roles.
- `src/robot_control.cpp`, `include/robot_control.hpp`: PTO shift logic, teleop drive output fan-out, intake/roller controls, color sort, pistons, autonomous manipulator task, 8-motor hold.
- `src/driver_control.cpp`: opcontrol loop and controller button mapping.
- `src/autonomous_control.cpp`: autonomous entrypoint, fixed-start/local-frame wrapper use, tune test selection, example/active route.
- `src/autonomous_localization.cpp`, `include/autonomous_localization.hpp`: global relocalization and `StartRelativeChassis`.
- `src/localization_config.cpp`, `include/localization_config.hpp`: active field map, MCL/EKF/fusion tuning constants, distance sensor geometry.
- `src/localization_tune.cpp`, `include/localization_tune.hpp`: brain overlay, trace capture, tune routes, SD/terminal log export, run finalization.
- `tools/localization_tune_analyzer.py`: offline analyzer for exported tune logs.
- `src/lemlib/chassis/odom.cpp`, `include/lemlib/chassis/odom.hpp`: odometry integration, sequence/delta history, telemetry capture, localization sync hooks.
- `src/lemlib/localization/localization.cpp`, `include/lemlib/localization/localization.hpp`: fusion task, EKF/MCL scheduling, correction gating, trace buffer.
- `src/lemlib/localization/mcl.cpp`, `include/lemlib/localization/mcl.hpp`: particle filter, sensor likelihood model, raycast-to-field, obstacle validity.
- `src/lemlib/localization/ekf.cpp`, `include/lemlib/localization/ekf.hpp`: 3-state EKF predict/update and NIS.
- `include/lemlib/localization/math.hpp`: tiny 3x3 matrix helpers and angle wrapping.
- `include/lemlib/api.hpp`: public LemLib include aggregator; includes localization API and aliases `AngularDirection`, `DriveSide`, `PtoRole`.
- `include/app_config.hpp`: currently only `inline constexpr bool kSmokeTestMode = false;`.

## Robot Hardware Configuration

From `src/robot.cpp`:

- Controller: `pros::Controller controller(pros::E_CONTROLLER_MASTER)`.
- Base drive motors:
  - left: ports `-4`, `-2`, blue gearset.
  - right: ports `9`, `7`, blue gearset.
- PTO/multifunction motors:
  - `leftPtoMotor1`: port `1`, blue.
  - `leftPtoMotor2`: port `3`, blue.
  - `rightPtoMotor1`: port `-10`, blue.
  - `rightPtoMotor2`: port `-8`, blue.
- IMU: port `18`.
- Distance sensors:
  - front: port `13`.
  - right: port `17`.
  - back: port `12`.
  - left: port `15`.
- PTO piston: ADI `B`, initial true.
- Tracking rotation sensors:
  - horizontal encoder: port `14`.
  - vertical encoder: port `19`.
- Tracking wheels:
  - horizontal: `lemlib::Omniwheel::NEW_2`, offset `-6.75`.
  - vertical: `lemlib::Omniwheel::NEW_2`, offset `-0.125`.
- Drivetrain:
  - track width `11.375`.
  - wheel diameter `lemlib::Omniwheel::NEW_325`.
  - rpm `450`.
  - horizontal drift `8`.
- Drive curves:
  - throttle curve `ExpoDriveCurve(3, 10, 1.019)`.
  - steer curve `ExpoDriveCurve(3, 10, 1.019)`.

## Controller Gains And PTO Profiles

From `src/robot.cpp`:

- 4-motor linear controller: `kP=13.2`, `kI=0`, `kD=129`, windup `0`, small error `1`, small timeout `100`, large error `2`, large timeout `500`, slew `0`.
- 4-motor angular controller: `kP=4.8`, `kI=0`, `kD=37`, windup `0`, small error `1`, small timeout `100`, large error `3`, large timeout `500`, slew `40`.
- 8-motor PTO linear controller: `kP=13.2`, `kI=0`, `kD=105`, windup `0`, small error `1`, small timeout `100`, large error `2`, large timeout `500`, slew `0`.
- 8-motor PTO angular controller: `kP=4.73`, `kI=0`, `kD=50`, windup `0`, small error `1`, small timeout `100`, large error `3`, large timeout `500`, slew `40`.
- PTO engaged digital value is `false` in `src/main.cpp`.
- `configureChassisPto()` maps the four PTO motor groups to left/right drive sides.
- `releasedPtoRoles()` assigns slot 0 to `MotorRole1` and slots 1-3 to `MotorRole2`.
- In project logic, `MotorRole1` acts as roller and `MotorRole2` acts as intake.

## Driver Controls

From `src/driver_control.cpp`:

- Arcade drive: left analog Y for throttle, right analog X for turn.
- Drive outputs are clamped to `[-127, 127]`.
- If PTO is engaged, `commandTeleopDriveOutputs()` mirrors base drive outputs to PTO motors so the robot drives as 8-motor.
- `X`: switch to 8-motor drive.
- `Y`: toggle loading mechanism.
- `L2`: toggle descore.
- `L1` or controller `UP`: forward intake behavior. `UP` also raises middle goal on press and lowers it on release.
- `R1`: forward intake/roller mode with right PTO motor 1 block monitoring.
- `R2`: reverse intake/roller mode.
- When intake buttons are released, code schedules return to 8-motor drive after all intake buttons are up.
- Controller display refreshes every 200 ms with current pose and controller connection status.

## Pneumatics And Manipulators

From `src/robot_control.cpp`:

- Loading mechanism: ADI `A`, initial false.
- Middle goal: ADI `C`, initial true.
- Descore: ADI `D`, initial false.
- Sorter piston: ADI `E`; `kSorterPistonExtendedValue = false`, retracted is true.
- Color sort optical sensor: port `16`, LED PWM set to `100`.
- Extra intake block sensors:
  - distance sensor port `20`, block threshold `50 mm`.
  - distance sensor port `5`, block threshold `70 mm`.
- Color sorting:
  - red ring detection uses optical hue `> 0 && < 20`.
  - proximity threshold `40`.
  - while sorting, intake power is slowed to `100` in the same sign.
  - sorter retract duration `150 ms`.
  - rearm clear duration `50 ms`.
  - color sort poll interval `2 ms`.
- PTO shift timing:
  - shift delay `45 ms`.
  - four-motor shift creep power `127`.
  - eight-motor shift creep power `35`.
  - creep delay `55 ms`.
  - shift window timeout `300 ms`.
  - four-motor shift timeout `350 ms`.
- Intake/roller constants:
  - intake forward `-127`, reverse `127`.
  - roller forward `-127`, idle `35`, reverse full `127`.
- `enableEightMotorPositionHold()` cancels motions, stops autonomous manipulator control, switches to 8-motor if needed, sets all drive/PTO motor groups to hold, commands zero, then marks hold enabled.
- `score(durationMs, direction)` switches to 4-motor drive, runs released PTO motors at `-127 * sign(direction)`, then stops and switches back to 8-motor drive.

## Initialization Flow

From `src/main.cpp`:

1. If `kSmokeTestMode` is true, do not initialize LemLib/localization. Display smoke test text, rumble, set base drive brake mode, and return.
2. Set localization tune state to startup.
3. `initializeRobotControlState()`.
4. Configure chassis PTO.
5. If `localization_tune::kEnabled`, call `lemlib::localization::configure(buildLocalizationConfig())` and `localization_tune::initializeRuntime()`.
6. Calibrate chassis with `chassis.calibrate()`. This initializes odom and starts localization if configured.
7. Stop chassis motion.
8. Switch to 4-motor drive.
9. Clear export feedback and set tune state idle/autonomous ready.

In `disabled()` and `competition_initialize()`, the code marks driver loops inactive and finalizes interrupted tune runs if needed. `disabled()` also stops autonomous manipulator control and chassis motion.

## Smoke Test Mode

- Toggle `kSmokeTestMode` in `include/app_config.hpp`.
- When true:
  - `initialize()` avoids full LemLib/localization setup.
  - `opcontrol()` runs a simple base arcade loop and displays drive values.
  - `autonomous()` runs forward 1s, stop 0.5s, backward 1s.
- This mode is useful for checking motor/controller basics without localization, PTO runtime, or tune tasks.

## Localization Field And Sensor Config

Active config is in `src/localization_config.cpp`.

- Field size: width `140.43 in`, height `140.41 in`.
- Coordinates are field-centered:
  - `minX = -70.215`, `maxX = 70.215`.
  - `minY = -70.205`, `maxY = 70.205`.
- `fieldMargin = 10.0` inches, roughly robot radius plus tolerance.
- `obstacleMargin = 1.0` inch.
- Obstacles model the 2025-2026 V5RC Push Back field geometry at the 5-7 inch sensor height band:
  - left/right loader rectangles.
  - long goal support circles.
  - center goal support rectangles.
- Distance sensor model in robot coordinates: `dx` is positive right, `dy` is positive forward, `dtheta` is clockwise from robot front, units inches/radians.
- Sensor configs:
  - front: `&distFront`, `dx=-9.312`, `dy=2.5`, `dtheta=-15 deg`, range `5..96 in`, min confidence `15`.
  - right: `&distRight`, `dx=10.0`, `dy=-0.25`, `dtheta=96 deg`, range `5..96 in`, min confidence `10`.
  - back: `&distBack`, `dx=2.375`, `dy=-6.875`, `dtheta=210 deg`, range `5..96 in`, min confidence `10`.
  - left: `&distLeft`, `dx=-3.5`, `dy=-6.75`, `dtheta=-92 deg`, range `5..96 in`, min confidence `15`.

## Localization Tuning Constants

From `src/localization_config.cpp`:

- MCL:
  - particles `450`.
  - sensor std `3.0 in`.
  - outlier threshold `7.0 in`.
  - outlier weight `0.22`.
  - min active sensors `2`.
  - motion std XY `0.20`, XY per inch `0.018`.
  - motion std theta `0.008`, theta per rad `0.018`.
  - motion lateral std per rad `0.08`, per inch-rad `0.0015`.
  - lateral bias per rad and per inch-rad both `0.0`.
  - init std XY `2.0`, init std theta `2 deg`.
  - confidence max spread `32.0`.
  - roughening XY `0.08`, theta `0.0035`, side roughening per rad `0.025`.
- EKF:
  - process std XY `0.08`, XY per inch `0.025`.
  - process std theta `0.012`, theta per rad `0.025`.
  - process lateral per rad `0.03`, per inch-rad `0.0015`.
- Fusion:
  - NIS gate `8.0`.
  - min correction sensors `3`.
  - min confidence `0.55`.
  - max var XY `81.0`.
  - max var theta `0.10`.
  - max measurement delta XY `2.5 in`.
  - max measurement delta theta `4 deg`.
  - max correction XY `0.08 in/update`.
  - max correction theta `0.5 deg/update`.
  - init std XY `2.5`, theta `6 deg`.
  - sensor stale timeout `300 ms`.

## Localization Runtime Architecture

- `chassis.calibrate()` sets odom sensors and calls `lemlib::init()`.
- `lemlib::init()` starts the odom task at 10 ms and then calls `lemlib::localization::start()` if localization was configured.
- Odom runs in `src/lemlib/chassis/odom.cpp`.
- Localization runs in a separate PROS task in `src/lemlib/localization/localization.cpp`.
- Odom maintains:
  - pose, speed, local speed.
  - latest `OdomDelta`.
  - sequence number.
  - history of recent deltas and raw telemetry for localization replay.
- Localization consumes every odom delta since its last sequence number. If it detects skipped history, it resets filters from the current odom snapshot.
- `lemlib::setPose()` syncs odom and localization.
- `lemlib::detail::setPoseSilent()` is only for the localization task to inject the fused pose into odom without recursively resetting filters.
- The localization task:
  - predicts EKF and MCL on odom deltas.
  - runs MCL at `fusion.mclPeriodMs`.
  - computes NIS against EKF.
  - accepts corrections only when sensors are live, geometry/confidence/variance/delta gates pass, candidate poses are stable, and turn-rate suppression is clear.
  - writes a trace sample every loop.
- Trace capacity is `8192` samples.
- `lemlib::localization::getLatestTraceSample()` and `getTraceSamples()` can return radians or degrees.

## Odometry Details

- Local odom delta convention: `localX` is lateral and positive left, `localY` is forward, `deltaTheta` radians.
- Odom heading priority:
  1. horizontal tracking wheel pair if both exist.
  2. vertical tracking wheel pair if both exist and both are unpowered tracking wheels.
  3. IMU delta.
  4. heading fallback/no change.
- Current robot has one vertical and one horizontal tracking wheel, so heading normally comes from IMU.
- Odom clamps impossible encoder deltas based on a speed bound derived from drivetrain speed, with fallback max speed `120 ips` and margin `3`.
- Minimum dt is `0.005 s`.
- Odom history capacity is `256` deltas/telemetry samples.
- Important sensor offsets in trace metadata:
  - vertical tracking wheel offset `-0.125`.
  - horizontal tracking wheel offset `-6.75`.
  - track width `11.375`.
  - tracking wheel diameter `2.125`.

## MCL Details

- Particle RNG is deterministic: `std::mt19937 rng_{1658u}`.
- MCL converts PROS distance readings from mm to inches.
- PROS distance confidence is only read when raw distance is at least `200 mm`; otherwise confidence remains unavailable and is accepted as true.
- Active sensor gating checks distance range and min confidence when confidence is available.
- Sensor likelihood blends Gaussian likelihood with uniform likelihood based on confidence scale.
- Outliers multiply by `outlierWeight`; no-hit predictions multiply by `minWeight`.
- Weighted mean and covariance are computed before resampling.
- Resampling is systematic when ESS falls below `resampleEssRatio * numParticles`.
- Roughening adds local side/forward and theta noise after resampling.
- `expectedDistance()` computes sensor position from pose and raycasts.
- Current `raycastToField()` only uses perimeter walls for expected sensor distances. It intentionally does not let low-profile obstacle geometry occlude runtime distance rays. Obstacles are still used to reject invalid particle positions.
- Do not casually re-enable obstacle occlusion in MCL; comments say logs showed side/back sensors return perimeter-wall distances where low-profile goal supports would otherwise block rays.

## EKF Details

- State is `x`, `y`, `theta`.
- Predict model matches LemLib odom integration:
  - global x gets `dy * sin(avgHeading) + dx * -cos(avgHeading)`.
  - global y gets `dy * cos(avgHeading) + dx * sin(avgHeading)`.
  - theta wraps.
- Observation model is identity because MCL emits a full pose estimate.
- Update uses Joseph-form covariance update for stability.
- `innovationNIS()` computes Mahalanobis distance using `P + R`.

## Autonomous Flow

From `src/autonomous_control.cpp`:

- Normal autonomous is authored in a start-relative local frame:
  - local `(0,0,0)` is robot placed at start.
  - local `+Y` is forward from starting heading.
  - local `+X` is right from starting heading.
  - local heading `0 deg` is starting heading.
- Underneath, localization remains in absolute field coordinates.
- Fixed absolute start constants:
  - `kAutonomousStartAbsX = -27.0`.
  - `kAutonomousStartAbsY = -36.0`.
  - `kAutonomousStartHeadingDeg = 0.0`.
- `kLocalizationTuneTest = 1` currently diverts autonomous into tune test 1. Values:
  - `0`: normal route.
  - `1`: turn/center test.
  - `2`: straight scale test.
  - `3`: square + cross test.
- `prepareAutonomousStart()` disables hold, stops manipulator/PTO controls, sets brake modes, stops chassis, switches to 4-motor drive, sets loading/middle/descore states, then starts fixed-start localization with a `StartRelativeChassis`.
- If using `StartRelativeChassis`, route commands are local/start-relative. Use `::chassis` only for raw absolute-field commands.
- In active route section, there are many commented examples. Preserve them unless the user asks to clean up; they are useful for route authorship.

## StartRelativeChassis

In `src/autonomous_localization.cpp` / `include/autonomous_localization.hpp`:

- Wraps global LemLib chassis commands so autonomous can be authored start-relative.
- Stores an absolute origin in radians.
- `setPose()` converts local pose to absolute and calls `::chassis.setPose(..., true)`.
- `getPose()` reads global pose and converts to local.
- `turnToHeading()`, `swingToHeading()`, and `moveToPose()` convert local headings to absolute degrees for LemLib APIs.
- `moveToPoint()`, `turnToPoint()`, and `swingToPoint()` convert local target points to absolute field coordinates.

## Global Relocalization

Implemented twice with similar code:

- `autonomous_localization::performGlobalRelocalization()` in `src/autonomous_localization.cpp`.
- `localization_tune::performGlobalRelocalization()` delegates to autonomous localization, but `src/localization_tune.cpp` also contains similar helper code for reports/routes.

Relocalization behavior:

- Captures 3 distance snapshots spaced 20 ms apart and median-combines them.
- Requires at least 2 usable distance sensors.
- First tries direct wall solve against cardinal headings:
  - `0`, `90`, `180`, `-90` degrees.
  - Needs at least one x-facing and one y-facing sensor and at least 2 sensors.
- Accepts direct wall solve if confidence/variance gates pass.
- Otherwise runs MCL heading/pose seed search:
  - timeout `3500 ms`.
  - coarse heading hypotheses `16`.
  - refine heading hypotheses `5`.
  - iterations per hypothesis `6`.
  - relocalization particle count at least `2400` for global search and at least `1600` for wall seed search.
- Strong accept:
  - valid MCL, active sensors >= 2.
  - confidence >= `0.22`.
  - variance gates <= fusion maxes.
- Weak accept:
  - active sensors >= 2.
  - confidence >= `0.06` for 3+ sensors, `0.09` for 2 sensors.
  - XY variance <= min(fusion max, `64`), theta variance <= min(fusion max, `0.05`).
- Status strings include `wall_direct`, `weak_accept`, `low_confidence`, `high_variance`, `not_enough_live_sensors`, `no_valid_pose`.

## Localization Tune Runtime

`localization_tune::kEnabled` is true.

The tune runtime starts:

- brain screen overlay task.
- overlay control task.
- terminal replay/export task.
- manual drive fallback task.
- touch callback for lower-right brain tap export.

Tune tests in `src/localization_tune.cpp`:

- Test 1: `Turn center`, objective angular PID and rotational center drift. 7 turn steps.
- Test 2: `Straight scale`, objective forward/reverse distance scale and lateral drift. 4 move steps.
- Test 3: `Square cross`, objective combined lateral PID, angular PID, tracking, and sensor fusion. 15 steps.
- Default test case in tune code is turn center.

Tune logging:

- Main SD log path: `/usd/localization_tune_latest.txt`.
- Internal fallback path: `localization_tune_latest_internal.txt`.
- Output mode is currently `DeepDive`, so reports include localization config and full trace CSV.
- If no SD card is installed or writing fails, the log is cached in RAM and persisted internally if possible.
- Terminal export wraps logs in:
  - `=== BEGIN LOCALIZATION TUNE LOG ===`
  - `=== END LOCALIZATION TUNE LOG ===`
- Lower-right brain tap can queue a terminal dump when a log is ready.
- Brain overlay pages show expected/reported/error, sensor readings, active sensors/confidence/correction, fused pose, MCL pose, NIS, ESS, variance, stale flag.
- `B` can abort tune motions in routes that allow abort.

Offline analyzer:

- Command: `python3 tools/localization_tune_analyzer.py /path/to/localization_tune_latest.txt`.
- Optional `--pose-source applied|ekf|mcl`; default is `applied`.
- Optional `--sensor-sample-limit N`; default `1500`.
- It parses the trace CSV section, estimates odom offsets/scales/turn scale/lateral bias, and performs bounded sensor dx/dy/dtheta search.
- It expects trace marker `=== LOCALIZATION TRACE CSV ===` and v2 metadata comments.

## Important Runtime Interactions

- `chassis.moveDrive()` records last commanded left/right outputs and mirrors them to PTO motor groups when PTO is engaged.
- `moveReleasedPtoDriveOutputs()` uses last drive outputs while shifting from 8-motor drive to 4-motor drive so PTO motors follow briefly before creep.
- `trySwitchToFourMotorDrive()` refuses to shift while a motion is active, unless already released. Blocking wrappers use `waitForShiftWindow()`.
- `trySwitchToEightMotorDrive()` also refuses during motion, then engages PTO, creeps, stops, delays, and sets PTO controller gains.
- Autonomous helpers often call `disableEightMotorPositionHold()` before moving mechanisms.
- Do not start long-running manipulator tasks without a corresponding stop path; the code has explicit task handles and removes the autonomous manipulator task.
- Color sort has a background task that is created once and controlled by atomics.
- `stopReleasedPtoControls()` disables color sort monitoring, resets color sort and block detection state, and commands released PTO motors to zero.

## Units And Coordinate Conventions

- Field/robot positions are in inches.
- Internal localization theta is radians.
- Most LemLib public chassis APIs take headings in degrees unless a `radians` boolean says otherwise.
- `chassis.getPose()` returns degrees by default; `chassis.getPose(true)` returns radians.
- `lemlib::localization::getDebugInfo(false)` converts pose headings to degrees.
- Distance sensors read mm from PROS and are converted to inches.
- Robot sensor coordinates:
  - `dx` positive right.
  - `dy` positive forward.
  - sensor `dtheta` positive clockwise from robot front.
- Odom local delta convention:
  - `localX` positive left.
  - `localY` positive forward.
- Start-relative autonomous convention:
  - local `+X` is right.
  - local `+Y` is forward.
- Be explicit when crossing these conventions. A sign error is very easy here.

## Coding Style And Constraints

- Prefer small, surgical changes in the active files. This repo has several tightly coupled robot-runtime tasks.
- Use C++23-compatible code; keep embedded/PROS constraints in mind.
- Existing style is 4-space indentation, braces mostly same-line for functions/blocks, file-local constants in anonymous namespaces, public constants with `k` prefix.
- Keep comments only when they document non-obvious robot behavior, coordinate conversions, task interactions, or hardware constraints.
- Prefer `std::array` where size is fixed and small.
- Avoid unbounded allocation in periodic loops. Existing `std::vector` and `std::deque` usage is deliberate in localization/tuning, but do not add more hot-loop churn casually.
- Use `pros::Mutex` around shared mutable state accessed by tasks; use `std::atomic` for simple flags/counters already following that pattern.
- When adding task state, provide a stop/finalize path. PROS tasks can persist across modes.
- Preserve deterministic behavior in localization unless changing it intentionally. The MCL RNG seed is fixed.
- Be cautious changing constants in `localization_config.cpp`; most are tuning-sensitive and should be justified by logs or a specific test.
- Be especially cautious touching:
  - `src/lemlib/chassis/odom.cpp`
  - `src/lemlib/localization/localization.cpp`
  - `src/lemlib/localization/mcl.cpp`
  - `src/robot_control.cpp`
  - `src/localization_tune.cpp`
- These files have task/timing/hardware coupling that may compile fine but fail on robot.

## Common Task Playbooks

### User Preference For Localization Tuning

- Do not ask the user to manually measure robot sensor positions, offsets, or field placement values for localization tuning.
- Infer tuning values from the logs the user provides. If a value cannot be inferred from the current log, add the needed telemetry to the robot tune log/RAM cache path, build it, and ask only for another pasted/exported log after the next run.
- Be explicit in final responses about which values are supported by the data and which values are being left unchanged because the log cannot constrain them.
- Preserve the user's real-world observation that pure odometry has been more consistent than bad distance-sensor fusion. Do not loosen sensor-fusion gates just to make corrections happen.

### If asked to change autonomous route

1. Check `src/autonomous_control.cpp`.
2. Decide whether `kLocalizationTuneTest` should remain enabled. If it is `1`, `2`, or `3`, normal route code will not run.
3. Use `StartRelativeChassis` for normal authored route points unless the user explicitly wants absolute field commands.
4. Keep startup states in `prepareAutonomousStart()` consistent with hardware.
5. Use `waitUntilDone()`/`waitUntil()` as needed; remember LemLib commands default async.
6. Build with `make quick`.

### If asked to tune localization

1. Ask for or locate a `localization_tune_latest.txt` log if one is relevant.
2. Run `python3 tools/localization_tune_analyzer.py <log>` from the root.
3. Compare suggestions against physical plausibility before editing constants.
4. For sensor geometry, edit `src/localization_config.cpp`.
5. For odom tracking wheel offsets, edit `src/robot.cpp`.
6. For motion/fusion noise, edit `src/localization_config.cpp`.
7. Build with `make quick`.

### If asked about bad pose jumps

Check, in order:

1. `src/localization_config.cpp` fusion gates: `minCorrectionSensors`, `minConfidence`, `maxMeasurementDeltaXY`, `maxMeasurementDeltaTheta`, `maxCorrectionXY`, `maxCorrectionTheta`, `sensorStaleMs`.
2. `src/lemlib/localization/localization.cpp` correction gating/stability constants and turn suppression.
3. Whether active sensors are 2 or 3+ in the trace.
4. Whether the latest correction was accepted and NIS value.
5. Whether MCL pose is aliased across symmetric field walls.
6. Whether MCL covariance/confidence is too permissive.
7. Whether odom telemetry shows impossible deltas or heading fallback.

### If asked about inaccurate distance/turn scale

- Use tune test 1 for turn center/rotational drift.
- Use tune test 2 for straight scale and lateral drift.
- Analyze trace with `tools/localization_tune_analyzer.py`.
- Tracking wheel diameter/gear ratio lives in wheel construction through `TrackingWheel` instances; offsets are in `src/robot.cpp`.
- Current tracking wheel offsets are vertical `-0.125`, horizontal `-6.75`.
- Current drivetrain track width is `11.375`; do not confuse this with tracking wheel offsets.

### If asked about PTO/intake issues

1. Read `src/robot_control.cpp`.
2. Check whether `chassis.isPtoEngaged()` should be true for 8-motor drive or false for released intake/roller.
3. Verify shift helpers are not called while a motion is active unless using blocking wrappers.
4. Check color sort monitoring because it can slow intake power.
5. Check right PTO motor 1 block logic if only one intake motor is braking.
6. Recheck motor signs before changing constants; several PTO motor commands intentionally invert left/right/roller directions.

### If asked for a quick hardware sanity mode

- Use `include/app_config.hpp` and set `kSmokeTestMode = true`.
- Rebuild and upload.
- Smoke mode avoids LemLib/localization and uses simple base drive commands.
- Remember to set it back to false for normal behavior.

## Known Current State

- `kSmokeTestMode` is false.
- `localization_tune::kEnabled` is true.
- `kLocalizationTuneTest` in `src/autonomous_control.cpp` is `1`, so autonomous currently runs the turn/center tune test instead of the normal route.
- The `MCL + EKF + Odom Newly improved` subfolder has build artifacts but no source. Do source work in the parent root.
- `make quick` and `make library` were up to date when this file was written.

## Verification Checklist

- For any C++ change: run `make quick` from `/Users/ouji/Documents/Localization Test`.
- For library/template changes: run `make library`; run `make template` only if requested because it creates template output.
- For tuning log work: run the Python analyzer on the actual exported log and include the key suggestions in your response.
- For behavior changes that cannot be tested locally because they require VEX hardware, say that clearly and identify the exact build command run.
- Do not claim robot behavior was verified unless the code was built and/or the user provided hardware logs.
