# AGENTS.md

Future Codex agents: read this first. This file was written on 2026-05-02 and updated on 2026-10-05 to preserve repo context for later chats. Source code and trace metadata override prose when tuning state changes.

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
- `README.md` is this project's own repo introduction (localization overview, validation snapshot, built-in test-route table, log-collection workflow, safety contract) — treat its claims as current and authoritative. The `Makefile` remains authoritative for the LemLib library version (0.5.6).
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
- Controller display refreshes each line every 200 ms with current pose and controller connection status. Writes are spaced one per 100 ms slot (clear lines 1 and 2 once, then alternate the pose line and the `CTL` line), because VEXos drops controller text writes sent closer together; a rejected write is retried in the next slot.

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
- `score(durationMs, direction)` disables 8-motor hold, stops autonomous manipulator control, switches to 4-motor drive, runs the released PTO motors at `-127 * (direction >= 0 ? 1 : -1)` for `durationMs`, stops them, and intentionally stays in released 4-motor mode (the air pump/PTO is unavailable for validation runs). It does not shift back to 8-motor drive; call `switchToEightMotorDrive()` explicitly if a later motion needs it.

## Initialization Flow

From `src/main.cpp`:

1. If `kSmokeTestMode` is true, do not initialize LemLib/localization. Display smoke test text, rumble, set base drive brake mode, and return.
2. Eagerly construct the LemLib logger singletons (`lemlib::bufferedStdout()`, `infoSink()`, `telemetrySink()`). Doing it on the initialize task, which PROS never deletes and which runs before any LemLib/tune task exists, means no later competition task can be killed mid-constructor and leave a single-thread libstdc++ static guard "in progress" (the next call would then throw `recursive_init_error` and terminate the program).
3. Set localization tune state to startup.
4. `initializeRobotControlState()`.
5. Configure chassis PTO.
6. If `localization_tune::kEnabled`, call `lemlib::localization::configure(buildLocalizationConfig())` and `localization_tune::initializeRuntime()`.
7. Calibrate chassis with `chassis.calibrate()`. This initializes odom and starts localization if configured.
8. Stop chassis motion.
9. Switch to 4-motor drive.
10. Clear export feedback and set tune state idle/autonomous ready.

In `disabled()` and `competition_initialize()`, the code marks driver loops inactive and finalizes interrupted tune runs if needed. `disabled()` also stops autonomous manipulator control and chassis motion, then calls `chassis.recoverInterruptedMotion()` to release a motion semaphore left by a killed autonomous task.

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
- `obstacleMargin = 0.0` inch.
- The active particle-validity field model contains one center rectangle with 3-inch half-width and half-height.
- Distance sensor model in robot coordinates: `dx` is positive right, `dy` is positive forward, `dtheta` is clockwise from robot front, units inches/radians.
- Sensor configs:
  - front: `&distFront`, `dx=-2.749`, `dy=-1.875`, `dtheta=-33.95 deg`, range `5..96 in`, min confidence `15`.
  - right: `&distRight`, `dx=9.938`, `dy=-1.25`, `dtheta=100.75 deg`, range `5..96 in`, min confidence `10`.
  - back: `&distBack`, `dx=-0.625`, `dy=-10.749`, `dtheta=202.10 deg`, range `5..96 in`, min confidence `10`.
  - left: `&distLeft`, `dx=-6.437`, `dy=-2.374`, `dtheta=-115.40 deg`, range `5..96 in`, min confidence `15`.

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
  - max correction XY `0.015 in/update`.
  - max correction theta `0.5 deg/update`.
  - boundary re-anchor enabled, XY cap `2.5 in`, theta cap `1 deg`, idle-only after a fully gated accept.
  - init std XY `2.5`, theta `6 deg`.
  - sensor stale timeout `300 ms`. Not an independent accept gate (every accept already needs live sensors in the same scan): it is the maximum age of a motion-staged correction that `Chassis::endMotion()` may still commit, and it drives the diagnostic `sensors_stale` trace flag.

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
- `lemlib::setPose()` syncs odom and localization. It bumps the motion-correction epoch (`localization::invalidateCorrectionFrame()`) inside the same odom critical section that publishes the new pose/seq, and `applyStagedBoundaryReanchor()` snapshots odom before validating the staged epoch, so a correction staged against the old frame can never land on a freshly set pose. It samples sensor baselines and builds the replacement histories before taking `odomUpdateMutex`, which narrows (but does not eliminate) the window in which a killed competition task could orphan that mutex.
- `lemlib::detail::setPoseSilentIfSeq()` injects a corrected pose into odom without recursively resetting filters. The write is always seq-checked under the odom locks (`odomUpdateMutex` -> `odomStateMutex`), so a concurrent odom integration defers it. It has two callers:
  - the localization task's continuous/idle-boundary injection, which passes `&isMotionCorrectionSuppressed` as `abortIf` so a just-started motion also defers the write;
  - `Chassis::endMotion()` -> `applyStagedBoundaryReanchor()`, running on whichever task executes the motion, which deliberately passes no `abortIf`: suppression is still true there by design (the caller owns the motion semaphore and the drivetrain has stopped). Do not make the suppression predicate mandatory or the default, or every staged boundary commit silently aborts.
- The unchecked `setPoseSilent()` overload currently has no callers.
- The localization task:
  - predicts EKF and MCL on odom deltas.
  - runs MCL at `fusion.mclPeriodMs`.
  - computes NIS against EKF.
  - accepts corrections only when sensors are live, geometry/confidence/variance/delta gates pass, candidate poses are stable, and turn-rate suppression is clear. Candidate stability needs consecutive MCL scans to agree both absolutely (`1.25 in` / `2.5 deg`) and after removing the odom-only motion between the scans (`1.25 in`); at rest the two XY tests coincide.
  - never runs the local range-grid solver or injects a pose into driven odometry while chassis motion correction suppression is active. It reads the suppression flag before taking its odom snapshot, so a snapshot taken after `endMotion()` clears suppression already contains any staged boundary commit (silent pose writes do not bump the odom seq).
  - can stage a fully gated EKF correction while moving; `Chassis::endMotion()` commits a fresh staged correction, within boundary caps, only after the drivetrain has stopped and while the motion semaphore is still owned.
  - writes a trace sample every loop.
- Trace capacity is `8192` samples.
- Trace CSV includes `boundary_reanchor_applied`, which distinguishes a legal stopped-boundary step from continuous correction steps.
- Trace CSV `meas_delta_theta_deg` (and the checkpoint report's `Gate ... dth=` value) is the raw MCL-vs-EKF heading disagreement in degrees. No correction accept gate reads it (applied candidates carry the EKF heading), but the same value above `12 deg`, with MCL confidence >= `0.45` and combined MCL XY variance >= `32`, is one trigger of the MCL recovery reseed to the EKF pose (the others are an MCL-vs-EKF pose delta above `5 in` under the same conditions, or NIS above twice the NIS gate with only the confidence condition). On a reseed row `mcl_conf`, `mcl_valid`, and `candidate_stable_scans` drop to 0 and `mcl_theta_deg` already shows the EKF anchor, so this column is the only logged record of the heading disagreement that caused it. Logs recorded before 2026-10 (all of `validation_data/` and `src/tune.txt`) always show `0.0` there. Columns and format are unchanged.
- Trace CSV heading-source flags: `odom_heading_vertical_pair=1` with `odom_heading_imu=0` means the drivetrain vertical pair carried heading that tick. On this robot that happens for the whole run when the IMU failed calibration, and also during a transient IMU fault (disconnect, recalibration, the rebaseline tick after an outage, an impossible jump). `odom_heading_fallback=1` means neither the IMU nor any wheel pair was usable, so heading was held.
- `lemlib::localization::getLatestTraceSample()` and `getTraceSamples()` can return radians or degrees.

## Odometry Details

- Local odom delta convention: `localX` is lateral and positive left, `localY` is forward, `deltaTheta` radians.
- Odom heading priority:
  1. horizontal tracking wheel pair if both exist.
  2. vertical tracking wheel pair if both exist and both are unpowered tracking wheels.
  3. IMU delta.
  4. drivetrain fallback: vertical pair including powered (motor-encoder-substituted) wheels, whenever the IMU gives no usable delta on a tick: IMU nulled after failed calibration, an invalid read (disconnect/recalibration), the forced rebaseline tick after an outage, or an impossible jump. Heading is then slip-prone: relocalization's heading prior and the boundary re-anchor inherit that drift for as long as the IMU is out (the whole run after a failed calibration, or a mid-run outage), but the wall-solve residual gates and fusion accept gates still validate every commit.
  5. hold last heading (no usable IMU delta and no usable pair either).
- A failed IMU calibration is not visibly warned on the brain. `calibrateIMU()` rumbles the controller (`---` per failed attempt, `-.-` after the final one, immediately followed by `calibrate()`'s unconditional `.` success rumble; closely spaced rumbles can be dropped over VEXnet) and logs a terminal-only `infoSink` error. Its `pros::c::lcd_print` call links to a no-op weak stub because `common.mk` filters `liblvgl.a` out of the link, and with the tune runtime enabled its `pros::screen::print` line is erased within 50 ms by the overlay. Check the trace heading flags above, or a connected terminal, to confirm the IMU state.
- Current robot has one vertical and one horizontal tracking wheel, so heading normally comes from IMU.
- Odom rejects impossible encoder deltas outright and immediately re-baselines that sensor, preventing a reset-to-zero sensor from freezing until it reaccumulates its old reading. The speed bound is derived from drivetrain speed, with fallback max speed `120 ips` and margin `3`.
- `PROS_ERR` from rotation sensors and ADI encoders, and a motor group whose motors are all invalid, come back from `TrackingWheel::getDistanceTraveled()` as NaN, and odom holds the previous reading. The wheel-reading magnitude backstop is `1e6 in` (above rotation-sensor saturation); the IMU validity gate stays `1e5 rad`.
- Non-finite or impossible IMU jumps are rejected and re-baselined instead of being clamped into a fabricated heading step. The first valid IMU read after any invalid read only rebaselines (no heading step), including after a `setPose` taken during an outage.
- A wheel feeds a differential heading pair only with a real one-tick delta (no held reading, no post-dropout catch-up, no over-bound jump). A pair-derived heading step above the per-tick bound is rejected (heading falls through to the next source), not clamped.
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
- Outliers multiply by `min(outlierWeight, blended likelihood at outlierThreshold)` -- capped so the likelihood is monotone in |error| and an outlier reading can never weight a particle better than an inlier; no-hit predictions multiply by `minWeight`. (Before 2026-07-02 the cap was missing, so the shipped `outlierWeight=0.22` beat the 0.133 Gaussian peak at `sensorStd=3` and inverted the ranking for clouds straddling the threshold.)
- Weighted mean and covariance are computed before resampling.
- Resampling is systematic when ESS falls below `resampleEssRatio * numParticles`.
- Roughening adds local side/forward and theta noise after resampling.
- `expectedDistance()` computes sensor position from pose and raycasts.
- Current `raycastToField()` raycasts perimeter walls AND the configured obstacles (shipped config: one 6"x6" rect at field center) at raw extents; `obstacleMargin` is a particle keep-out buffer only and is never applied to rays. The fusion-side `expectedDistanceFromPose()`, the reloc scorer, and both Python analyzers model the same geometry, and the 2026-06 replay validated firmware-vs-analyzer agreement on logged expected distances.
- Do not add low-profile field geometry to the obstacle list; earlier logs showed side/back sensors return perimeter-wall distances where low-profile goal supports would otherwise block rays. Only structure tall enough to reflect at sensor height belongs in `field.obstacles`.

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
- `kLocalizationTuneTest = 0` currently runs the normal route. Values:
  - `0`: normal route.
  - `1`: turn/center test.
  - `2`: straight scale test.
  - `3`: square loop test.
  - `4`: open-loop drive probe.
  - `5`: stationary sensor-angle sweep.
  - `6`: square + cross test.
- `prepareAutonomousStart()` disables hold, stops manipulator/PTO controls, sets brake modes, stops chassis, calls `chassis.recoverInterruptedMotion()` (a direct driver-to-autonomous switch bypasses `disabled()`, and PROS may have killed opcontrol mid-motion, e.g. during the 8-motor engage `drivePulse`), switches to 4-motor drive, sets loading/middle/descore states, then starts fixed-start localization with a `StartRelativeChassis`.
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

Implemented once in `autonomous_localization::performGlobalRelocalization()` in `src/autonomous_localization.cpp`. `localization_tune::performGlobalRelocalization()` delegates to it; the retired duplicate tune-harness implementation was removed.

Relocalization behavior:

- Captures 3 distance snapshots spaced 20 ms apart and median-combines them (the middle-pair mean after a dropout; the lone sample after two dropouts). The combined reading carries the strongest sample's confidence and is what the gates, solves, MCL weighting and the log use. With at least 3 usable sensors, relocalization fails closed with `low_reading_confidence` in two cases:
  - a usable sensor's median-slot reading is below its confidence gate (for a middle pair, the smaller available confidence). Samples at an identical range sort weakest first, so the veto can also fire when a sample at or above the gate reported that exact range. That costs a fallback, never a commit. Reading the strongest confidence at the median range instead would re-admit wrong commits the veto now blocks, so that is a user policy call;
  - the two samples left after a dropout differ by more than `2 in` (`kRelocalizeMaxSamplePairSpread`). Their mean is then more than 1 in from both, a range neither sample measured. Stationary consecutive readings in the logged traces (`validation_data/`, `src/tune.txt`) stayed within 1.61 in.
  It never drops that sensor and solves without it: a solve with fewer rays can commit a pose the full view would not.
- Requires at least 2 usable distance sensors to attempt a solve and at least 3 scored sensors for a normal commit.
- First tries a direct wall solve once at the trusted field-frame odometry/IMU heading. Walls solve X/Y and never steer heading.
- Wall-assignment veto. The direct solve ranks every sensor mask (mean residual + `0.05`/in of prior distance + ignored/outlier penalties) and takes the best. Walls are assigned from ray heading alone, so a ray that really ends on the adjacent wall is solved onto the corner at ~0 residual. With at least 3 usable sensors, relocalization therefore fails closed with `wall_assignment_ambiguous` (fixed start, no MCL search) when (1) a ray the pick used does not, under the full raycast (obstacle included) at the solved pose, end on its assigned wall at least `3 in` (`kWallCornerMargin`) from a corner; (2) another mask's solve whose rays fail that test still explains every scored range (3+ sensors, all within `mcl.outlierThreshold`) and lies more than `mcl.outlierThreshold` from the pick; or (3) the pick fails its `wall_direct` gates while such an alternative exists at any distance (the `mcl.outlierThreshold` separation applies only to (2)). An alternative whose own rays do end on their assigned walls never vetoes, however far away it is. With fewer than 3 usable sensors a vetoed pick (case 1 or 2) returns `not_enough_commit_sensors`, logged with mode `wall_direct` and the prior pose, not the possibly aliased pick. Otherwise the flow is unchanged: commit, or MCL search seeded at the pick. The veto never promotes another mask. An earlier version did, and committed aliases 17-78 in off where the unchecked solve did not, e.g. (-40,10,0) with the front sensor dead and the prior 1 deg low committed (29.9,10.5). `3 in` covers a `7 in` range error on the back/left rays (7*sin(22-25 deg)) and ~1 deg of prior error on the ~90 in right ray (~1.5 in per degree). Narrower per-ray margins re-admitted right-ray corner aliases.
- A direct commit requires the three-sensor guard plus confidence, variance, mean-residual, and maximum-residual checks. A robust-outlier path can use two inliers only when at least three sensors were scored and the residual gates identify the excluded ray.
- Otherwise runs MCL heading/pose seed search:
  - timeout `3500 ms`.
  - coarse heading hypotheses `16`.
  - refine heading hypotheses `5`.
  - iterations per hypothesis `6`.
  - relocalization particle count at least `2400` for global search and at least `1600` for wall seed search.
- Strong accept:
  - valid MCL, active sensors >= 3.
  - confidence >= `0.22`.
  - variance gates <= fusion maxes.
- Weak accept:
  - active sensors >= 3.
  - confidence >= `0.06`.
  - XY variance <= min(fusion max, `64`), theta variance <= min(fusion max, `0.05`).
- MCL commit contract (same as `wall_direct`: ranges solve X/Y, never heading):
  - the final candidate's heading must be within `20 deg` (`kRelocalizeHeadingTiebreakRad`) of the IMU/odom heading prior, else it is rejected with `heading_disagree`. The prior is the odom heading, i.e. the configured start heading at autonomous start. The 16-hypothesis heading sweep still runs and the prior only re-ranks its candidates, so a robot placed more than 20 deg off the configured start heading is rejected with `heading_disagree` and falls back to the fixed start even when the search finds its true heading.
  - an accepted candidate is committed AT the prior heading. Its X/Y must pass the residual (or robust-outlier) gates both at the candidate's own (unpinned) MCL estimate and at that committed pose, so the pin can only reject; `candidate_bad_residual` means the committed-pose evidence passes but the candidate's own estimate failed. The logged residuals and active/scored sensor counts are the committed-pose values. Do not drop the candidate-pose scoring (`candidateResiduals`): it is the pre-pin gate, and without it the pin would accept candidates whose own estimate fails the residual gates.
  - its inlier rays must really end on an X wall and on a Y wall (not within `kWallCornerMargin` of a corner, nor on the obstacle), else `axis_unobserved`.
  - every relocalizer MCL reset passes `clampLocalSeed=false` to `MCL::reset`. This keeps the pre-sweep placement of a particle whose local seed draws all landed outside the margin box (field-uniform scatter), and so the pre-sweep RNG stream. The 2026-10 clamp of such particles into the box applies only to the runtime filter's resets: recovery reseed, `setPose`/`resetFilters`, and the skipped-history reset. With the clamp in the relocalizer, the wall-seeded estimate changed, and the host harnesses found commits 1.4-3.1 in off where the pre-sweep code fell back or committed closer, e.g. (52.5,-55.5,270) with all sensors live and the prior 2 deg high.
- Limits shared with the pre-sweep code (every check only vetoes):
  - The three-sensor guard counts sensors, not per-axis redundancy. At the configured 0 deg start the back ray is the only Y constraint, so an occluder in it shifts Y at near-zero residual and still commits.
  - A self-consistent alias still commits when the pick itself passes, through `wall_direct` or the MCL fallback. Rays trade surfaces between two poses about 67-82 in apart (41-87 in at off-cardinal headings): a ray that ends on the obstacle or on one wall at one pose ends on another wall at a matching range at the other. Example: (56,24,0) with all sensors live commits 73 in off.
  - Heading-prior error can slide a corner alias past the margin: 2-3 deg with exact ranges, or 1 deg with about 1 in of range noise. Examples: (-46,30,0) at -3 deg commits 30 in off; (-23.75,-56.75,0) with the front sensor dead, 1 in Gaussian range noise and the prior 1 deg high commits `wall_direct` at (-42.05,-54.72), 18 in off; with 2-3 deg and the same noise, (-12.75,-56.75,0) commits 28 in off (reviewer-2 harness, main family #944259 and #1437005).
  - MCL-fallback commits are not bounded at a few inches. At the configured start (prior at the start, exact ranges, confidence 63 on every sample) they stay within about 7 in with all sensors live. They reach 23 in with the front sensor dead: (-44.75,-53.75,0) with the prior 3 deg low commits (-21.34,-54.26) (#6782). With the back sensor dead and the prior 1-3 deg high they reach about 67 in. No live ray then ends on a Y wall at the true pose, and (-32.75,-50.75,0) commits the Y alias (-27.66,16.03), where the front ray ends on the top wall within the inlier threshold (#547849). About 1 in of range noise pushes that alias to 76 in (#368693). Elsewhere on the field these commits reach about 86 in with exact ranges and 96 in with range noise. These examples come from the reviewer-2 harness's main family (static per-sensor noise, confidence 63 on every sample). The reviewer-1 harness's distance-dependent confidence model falls back at several nearby placements, so reproduce them with the matching model.
  - Catching these needs a prior-distance gate on both commit paths, a user policy choice.
- Measured effect of this sweep's relocalization changes. Source: three independent differential host harnesses (2026-10-05/06). Each extracts the shipped `performGlobalRelocalization`, links the real `mcl.cpp`, and compares it with the pre-sweep code (350a17d) on synthesized views and on every logged relocalization. Together they run about 20 M scenarios: start-band and wall-ring grids at 0.5-1 in, the rest of the field at 2-6 in, cardinal and off-cardinal headings, wall-touching placements, up to +-3 deg of prior error, single dead sensors, whole-reading range noise, the firmware (libstdc++) and host random streams, two other MCL seeds, and a per-snapshot family (independent 1 in noise per sample, 10% dropouts, half the samples below confidence 25).
  - No regression. In every harness, every commit is one the pre-sweep code makes, at the same XY; MCL commits carry the prior heading. Nothing commits where the pre-sweep code fell back, and the runtime seed clamp never fires inside the relocalizer.
  - With whole-reading noise it removes 75-82% of the pre-sweep commits more than 3 in off on the main, wall-ring, interior and off-cardinal grids (82% on the fixer's 2.6 M main grid), and 63% for wall-touching placements. Every remaining one is a pre-sweep commit covered by the limits above.
  - Cost: the share of pre-sweep commits within 3 in that fall back depends on where placements are sampled, so these are grid figures, not field-wide rates. It is about 12% in the interior, 15-20% in the start band and on 4-6 in field grids (18% on the fixer's main grid, 15% with off-cardinal headings), and 31-36% within about 10 in of the legal edge (0.5-2 in wall-ring grids, wall-touching placements).
  - At the configured start (0 deg, prior at the start, +-3 deg, all sensors live or front dead) with whole-reading noise, it removes 98% of the wrong commits, and 18-20% of commits within 3 in fall back. 82-86% of those fallbacks are placements 8-19 in behind the start, where the right or left ray ends near a bottom corner, e.g. (-27,-50,0) and the logged run1 placement (-16.6,-46.5). Apart from the reading-confidence veto below, nothing within +-5 in of the start falls back.
  - The `low_reading_confidence` veto's real-world cost is unmeasured, because logs record only the combined confidence and no per-sample spread. In the per-snapshot families, whose sample noise is several times the logged sample-to-sample noise, it dominates the cost: 53-57% of the pre-sweep commits within 3 in fall back. Its pair-spread part alone adds 5-6 points of that cost and removes another 1-1.6% of the wrong commits, about 7-11 correct commits lost per wrong commit prevented (fixer's family: 17,852 more fallbacks within 3 in, 2,630 fewer commits more than 3 in off). Reverting either part to the plain combine is the zero-cost, still reject-only alternative.
- Status strings: `wall_direct` (direct commit), `accepted` (strong MCL accept), `weak_accept` (weak-only MCL accept), `robust_outlier_accept` (only when the normal filter+residual path rejected the candidate at the committed pose), and the failures `bad_residual`, `candidate_bad_residual` (committed-pose residuals pass, MCL's own unpinned estimate failed; fixed-start fallback), `heading_disagree`, `axis_unobserved`, `wall_assignment_ambiguous`, `low_reading_confidence`, `low_confidence`, `high_variance`, `not_enough_live_sensors`, `not_enough_commit_sensors`, and `no_valid_pose`.

## Localization Tune Runtime

`localization_tune::kEnabled` is true.

The tune runtime starts:

- brain screen overlay task.
- overlay control task.
- terminal replay/export task.
- manual drive fallback task.
- touch callback for lower-right brain tap export.

Tune tests in `src/localization_tune.cpp`:

- Test 0: `Normal route`, objective full 4-motor validation autonomous with start relocalization/fallback.
- Test 1: `Turn center`, objective angular PID and rotational center drift. 7 turn steps.
- Test 2: `Straight scale`, objective forward/reverse distance scale and lateral drift. 4 move steps.
- Test 3: `Square loop`, objective translation/turn behavior and return-home drift.
- Test 4: `Drive probe`, objective open-loop drivetrain balance versus closed-loop motion.
- Test 5: `Sensor angle`, objective stationary distance-sensor mounting-angle fit.
- Test 6: `Square + cross`, objective long combined route with oblique segments and fusion opportunities.
- Default test case in tune code is turn center.

Tune logging:

- Main SD log path: `/usd/localization_tune_latest.txt`.
- Internal fallback path: `localization_tune_latest_internal.txt`.
- Output mode is currently `DeepDive`, so reports include localization config and full trace CSV.
- If no SD card is installed or writing fails, the log is cached in RAM and persisted internally if possible.
- Saving a finished run: `finalizeRun()` and `finalizeInterruptedRunIfNeeded()` only claim the run, freeze the trace (the interrupted path freezes it before the hardware stops), and run the stops synchronously. The report itself (log build, SD write to `/usd/localization_tune_latest.txt` or RAM cache plus internal persist, and the controller rumble) is built and saved afterwards by the persistent overlay control task, never on a competition task (only if the tune runtime was never initialized is it emitted inline). While it saves, the brain shows `Saving tune log...`. If a new run starts mid-build, the previous run's report is dropped as stale instead of mixing two runs.
- Terminal export wraps logs in:
  - `=== BEGIN LOCALIZATION TUNE LOG ===`
  - `=== END LOCALIZATION TUNE LOG ===`
- The terminal export holds `lemlib::stdoutWriteMutex()` from BEGIN to END, so logger lines queued during a dump print after the END marker.
- Lower-right brain tap can queue a terminal dump when a log is ready. A tap made while `Saving tune log...` is shown is held until the save completes and then exports the finished run's log (on the SD path the RAM cache is dropped, so this is a fresh rebuild of the frozen run state with a new header `time_ms`, not a read of the saved file). Each tap records the run generation it was made in, and the newest pending tap decides: a tap made before a newer run started, held or not, is dropped instead of dumping the new run's live state, and a live export that a new run overtakes mid-build is discarded rather than mixing two runs (both set `No tune data ready`, usually hidden by `Autonomous run active`). The dump cue on line 8 is `Tap lower-right to dump` on the idle page (normal route, no checkpoints), `LOG READY tap LR dump` on the checkpoint page (tests 1-6), or the persistent `Log cached in RAM` when there is no SD card or the SD write failed.
- The brain overlay shows an idle page before any checkpoint exists (mode/controller, tune status/step, sticks, start pose, live pose, distance readings) and, once checkpoints exist, only checkpoint page 0 (expected/reported/error, sensor distance/confidence, and active sensors/MCL confidence/correction accepted or the dump cue/export feedback on line 8). Line 8 is overridden by `Autonomous run active` while a run is going and by `Saving tune log...` while its report saves. `tuneDisplayPage` is never changed (no page navigation), so the page-1 branch of `printCheckpointPage()` (fused pose, MCL pose, NIS, ESS, variance, stale flag) is unreachable; those values are only in the exported log's per-checkpoint report and trace CSV.
- The checkpoint `Rep` heading (report and brain page 0) is wrapped to (-180, 180]; trace CSV `applied_theta_deg` stays continuous. Older logs in `validation_data/` show unwrapped `Rep` headings (e.g. run3 `Rep ... 714.5`); their `Err` lines were already correct.
- Routes that allow abort poll controller `B`, but tune routes run only inside competition `autonomous()`, where VEXos withholds controller input from user code, so the `B` polls are effectively inert. To stop a tune route, switch the competition switch or field to disabled: PROS ends the autonomous task, `disabled()` stops chassis motion, and the run is finalized as Interrupted. Every step is also bounded by its own timeout.

Offline analyzer:

- Command: `python3 tools/localization_tune_analyzer.py /path/to/localization_tune_latest.txt`.
- Optional `--pose-source applied|ekf|odom_only`; default is `applied`. `mcl` is not offered because the `mcl_*` columns latch at the 50 ms MCL tick, so they are not a per-row odom reference (`--sensor-pose-source` still accepts `mcl`, since the sensor columns latch on the same tick).
- Optional `--sensor-sample-limit N`; must be >= 1, default `1500`. When usable rows exceed N, the analyzer subsamples evenly across all traces instead of keeping a prefix.
- It parses the trace CSV section, estimates odom offsets/scales/turn scale/lateral bias, and performs bounded sensor dx/dy/dtheta search.
- It expects trace marker `=== LOCALIZATION TRACE CSV ===` and v2 metadata comments.

## Important Runtime Interactions

- `chassis.moveDrive()` records last commanded left/right outputs and mirrors them to PTO motor groups when PTO is engaged.
- `moveReleasedPtoDriveOutputs()` uses last drive outputs while shifting from 8-motor drive to 4-motor drive so PTO motors follow briefly before creep.
- `trySwitchToFourMotorDrive()` refuses to shift while a motion is active, unless already released, and returns false while a shift is in progress. Once the PTO is released it returns true (mechanism permission follows the physical PTO state). The 4-motor gain swap is never applied during a motion: the shift task's tail skips it while a motion runs, and the released branch applies a deferred or aborted-shift swap at the next motion-free call. Blocking wrappers use `waitForShiftWindow()`.
- `trySwitchToEightMotorDrive()` also refuses during motion, then engages PTO, creeps, stops, delays, and sets PTO controller gains.
- Neither the PTO shift task nor the autonomous manipulator task is tracked by a handle. Both are generation-keyed and cancelled cooperatively (never deleted): the shift task re-checks its generation under the lifecycle lock before every write, and stop paths wait (up to 100 ms) for the manipulator task to leave before zeroing the motors, so disabled-mode cleanup cannot leave a stale creep or mechanism command running. The manipulator also keeps a live-task count (`autonomousManipulatorTasksAlive`) for its bounded stop/re-arm wait. Lifecycle locks are never held across delays or kernel task create/delete on competition-task paths: `autonomousManipulatorLifecycleMutex` was removed, and competition tasks hold `fourMotorShiftLifecycleMutex` only around atomics (the shift task itself is a persistent task).
- Eight-motor engagement creep uses a chassis motion pulse, preserving the same no-in-motion-pose-injection contract as autonomous moves. Its `endMotion()` re-arms the boundary re-anchor one-shot, so teleop calls `lemlib::localization::clearBoundaryReanchor()` after every 8-motor engage (opcontrol entry, `X`, and the auto-return after intake buttons are released).
- Autonomous helpers often call `disableEightMotorPositionHold()` before moving mechanisms.
- Do not start long-running manipulator tasks without a corresponding stop path. The autonomous manipulator task exits cooperatively on a generation bump or when its duration expires. Every `intake(d > 0)` retires the previous task (generation bump, then a wait of at most 100 ms, normally one 10 ms tick, for the live-task count to reach 0) and starts a fresh task; `intake(0)` is the same as `stopAutonomousManipulatorControl()`.
- Color sort has a background task that is created once and controlled by atomics.
- `stopReleasedPtoControls()` cancels an in-flight 4-motor shift (state IDLE, generation bump), disables color sort monitoring, resets color sort and block detection state, and commands released PTO motors to zero. The zeroing (`hardStopReleasedPtoGroups()`) is skipped while the PTO is engaged, because those motors are then drive motors owned by `moveDrive()`/engage/disengage.
- Motion waiters are tracked per task handle (`Chassis::motionWaiters`, single-CAS slots), not by a counter. `recoverInterruptedMotion()` runs at `disabled()`/`opcontrol()`/`prepareAutonomousStart()` entry and, because PROS reuses one static TCB for every competition task, treats a motion owner or waiter handle equal to the caller as the killed predecessor. `cancelMotion()`/`cancelAllMotions()` sample `wasRunning` after suppressing corrections; `cancelAllMotions()` bumps the queue-cancel generation before the motion generation; `motionGenerationSnapshot()` returns the generation captured inside `requestMotionStart()` before `motionRunning` is published.
- LemLib logging (`bufferedStdout()`, and `infoSink()`/`telemetrySink()`, which print through it) holds no lock: a producer allocates its message node first, then publishes it with a plain load/store between `rtos_suspend_all()` and `rtos_resume_all()` (the drain task takes the whole inbox the same way), so the publish never waits on another task and a competition task killed mid-print loses only its own line. Do not restore a `compare_exchange`/`exchange` (ldrex/strex) inbox: the PROS context switch never clears the exclusive monitor, so a preempted strex can succeed on another task's reservation and silently drop a message. The only direct stdout writers (the logger drain task and the terminal tune-log export) hold `lemlib::stdoutWriteMutex()`; any new direct stdout writer must too, and that lock must never be taken on a competition-task path.

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
- PROS competition-task semantics: every competition callback (`initialize`, `disabled`, `competition_initialize`, `autonomous`, `opcontrol`) runs in a task created into one static buffer (`initialize()` itself is never deleted). On a mode change the daemon `task_delete()`s the current one without unwinding it (mutexes it holds are never released, pending waits are abandoned) and recreates the next callback in the same buffer, so it gets the same `task_t` handle. Therefore never hold a lock across a delay, long work, or kernel task create/delete on a competition-task path, and at a callback entry treat `owner == task_get_current()` as "owned by the dead predecessor". Persistent `pros::Task`s (odom, localization, tune overlay/export, logger drain, color sort, PTO shift, manipulator) are not killed on mode changes, so long work and lock-holding belong there.
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
2. Decide whether `kLocalizationTuneTest` should remain enabled. If it is `1` through `6`, normal route code will not run.
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

1. `src/localization_config.cpp` fusion gates: `minCorrectionSensors`, `minConfidence`, `maxMeasurementDeltaXY`, `maxCorrectionXY`, `maxCorrectionTheta`. `sensorStaleMs` is not an accept gate: it only limits how old a motion-staged correction may be when `endMotion()` commits it (and drives the diagnostic `sensors_stale` flag). (`maxMeasurementDeltaTheta` was removed 2026-07: heading never comes from range measurements, so the knob was provably inert.)
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
- `kLocalizationTuneTest` in `src/autonomous_control.cpp` is `0`, so autonomous currently runs the normal validation route.
- `src/tune.txt` is a May 24 pre-calibration Test 5 sweep retained as historical evidence. Its analyzer output is location-specific, contains likely unmodeled occlusions, and is not sufficient by itself for new geometry changes.
- `report/localization_report.pdf` summarizes seven validation runs and the June audit. LaTeX/report-generation sources are intentionally not published. The boundary re-anchor and latest relocalization cleanup still require the physical protocol in `validation_data/field_test_protocol.md`.
- The PDF is now partly stale and cannot be regenerated here. After the 2026-10 analysis-tool fixes (the start-relocalization `setPose` jump is no longer counted as driven path), `tools/drift_analysis.py`/`tools/audit_analysis.py` give per-run paths of 121/120/332/132/87/110/120 in for runs 1-7 (the PDF shows 135/123/335/138/172/110/130), the run-2 factor 1.8x (PDF 1.9x), aggregate/run-3 new max 2.18 in (PDF 2.16), run-2 new max 1.20 in (PDF 1.17), and a wall-solve reproduction error of 0.05 in (PDF 0.11). The headline aggregate 1.8x is unchanged. The working-tree `tools/audit_metrics.json` (and the gitignored `report/data/*` fragments) were already regenerated with the fixed tools on 2026-10-05 and hold these post-fix numbers, including a `tune` entry recomputed from the unchanged `src/tune.txt`. Do not revert the JSON to the pre-fix values, and run those two scripts again only in a scratch copy (see `TUNING_AGENT_PROMPT.md`).
- The `MCL + EKF + Odom Newly improved` subfolder has build artifacts but no source. Do source work in the parent root.
- The 2026-07-13 audited source passed a from-scratch `make quick`. The 2026-10 bug-sweep fixes (uncommitted on `main` as of 2026-10-06) build and link cleanly with `make quick` (no warnings in project files), but have not been tested on hardware. Rebuild after any later source change.

## Verification Checklist

- For any C++ change: run `make quick` from `/Users/ouji/Documents/Localization Test`.
- For library/template changes: run `make library`; run `make template` only if requested because it creates template output.
- For tuning log work: run the Python analyzer on the actual exported log and include the key suggestions in your response.
- For behavior changes that cannot be tested locally because they require VEX hardware, say that clearly and identify the exact build command run.
- Do not claim robot behavior was verified unless the code was built and/or the user provided hardware logs.
