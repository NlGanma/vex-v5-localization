# Localization Tuning Agent Prompt

Paste the prompt below into a repository-aware coding agent. Put the newest complete
robot export in `src/tune.txt` first, unless the agent
asks for a different path to preserve multiple runs.

```text
You are responsible for iteratively tuning the localization system in this repository on the real VEX robot.

Start-up procedure
1. Work from the real Git root. Read AGENTS.md completely, then inspect git status. Do not edit sibling/legacy copies or generated build trees.
2. Inspect src/autonomous_control.cpp for the actual value of kLocalizationTuneTest. Do not trust a test number stated in old prose or a previous chat.
3. Inspect the metadata inside src/tune.txt (or the newest log path I provide) to identify its route, firmware configuration, coverage, sensor activity, and whether it is newer and relevant. Preserve valuable prior logs in validation_data instead of overwriting evidence.
4. Run the checked-in analyzer before changing constants:
   python3 tools/localization_tune_analyzer.py src/tune.txt
   Use tools/trace_query.py (read-only) when its data is relevant. Do not run tools/audit_analysis.py or tools/drift_analysis.py in this working tree: they regenerate the published report evidence. audit_analysis.py always reads src/tune.txt as its "tune" entry and rewrites the tracked tools/audit_metrics.json, both scripts write report data files under report/data/, and drift_analysis.py only analyzes the fixed run1-run7 validation logs, never a new export. If their cross-checks are needed, run them only in a scratch copy of the repository.

Primary objective
Tune odometry, distance-sensor geometry, MCL, EKF, correction gating, and motion-boundary behavior until repeated hardware tests show that the shipped pose is more accurate and at least as consistent as pure odometry. Pure odometry has previously been more consistent than poorly configured range fusion, so the default failure mode must remain "stay on odometry," not "accept more corrections."

Non-negotiable rules
- Never loosen NIS, confidence, covariance, sensor-count, residual, stability, pose-delta, or motion-suppression gates, the live-sensors-in-the-same-scan requirement, or the stale timeout (sensorStaleMs) that limits how old a motion-staged correction may be when a motion boundary commits it, merely to make corrections occur.
- Compare odom_only, EKF/MCL, and applied/driven pose. Reject a change that makes repeatability worse than odom_only on comparable runs.
- Treat sensor positions and angles as uncertain, but infer them from instrumented logs. Do not ask me to manually measure sensor offsets or field placement. If a parameter is not identifiable, leave it unchanged and select another test, request more placements/repetitions, or add the minimum telemetry needed.
- Treat tracking-wheel measurements as stronger priors than distance-sensor geometry, but still validate them from turn and straight data.
- Do not fit geometry from one location-specific or occluded log. Require adequate heading span, field-position span, clean wall identities, and holdout confirmation.
- Distinguish robot motion-control error, odometry error, MCL measurement error, and applied-correction behavior. Do not tune one subsystem to hide another subsystem's fault.
- Keep changes surgical and deterministic. Preserve the fixed MCL RNG unless an intentional experiment requires otherwise.
- Do not claim the stack is fully tuned from a single run or from modeled ground truth. Require repeat hardware evidence.

Available test selector in src/autonomous_control.cpp
0 = normal validation route: end-to-end relocalization and competition-route behavior
1 = turn center: angular PID, turn scale, tracking-wheel rotational offsets, center drift
2 = straight scale: forward/reverse distance scale and lateral drift
3 = square loop: combined translation/turn behavior and return-home drift
4 = drive probe: open-loop drivetrain balance versus closed-loop motion
5 = sensor angle: stationary heading sweep for distance-sensor geometry
6 = square + cross: long combined route with oblique segments and fusion opportunities

Iteration procedure whenever I say "data ready"
1. Verify the log is complete and report its detected test, row count, duration, coverage, active-sensor distribution, accepted corrections, dominant rejection reasons, and obvious occlusions or sequence gaps.
2. State which parameters the data can constrain and which it cannot. Do not apply analyzer suggestions that are physically implausible, hit search bounds, depend on too few samples, or conflict with stronger evidence.
3. Make the smallest evidence-backed code/configuration changes. Relevant locations are usually src/robot.cpp for odometry geometry, src/localization_config.cpp for sensor/MCL/EKF/fusion constants, and localization implementation files only when traces demonstrate a logic defect.
4. Run make quick after every C++ change and run the relevant Python checks. The published report is a PDF artifact; do not recreate or commit LaTeX/report-generation files.
5. Select the next most informative test by editing kLocalizationTuneTest yourself. Tell me exactly: "Next test: N - Name."
6. Give concise physical instructions: start placement class, required clear footprint or wall views, whether to fixture the start, obstacles to remove or intentionally introduce, number of repetitions, and which comparisons must keep battery/configuration unchanged.
7. Stop and wait for me to run the robot, dump the terminal log, put it at the agreed path, and say "data ready." Then repeat this procedure.

Test-selection guidance
- Use Test 1 when turn scale, angular settling, tracking-wheel offsets, or rotational center is uncertain.
- Use Test 2 when forward scale, reverse scale, lateral drift, or straight-line asymmetry is uncertain.
- Use Test 4 to separate open-loop drivetrain imbalance from closed-loop localization/motion behavior.
- Use Test 5 for sensor geometry. Request multiple clean starts at meaningfully different field positions; one stationary sweep only constrains local geometry and occlusion.
- Use Tests 3 and 6 after component calibration to validate combined behavior and correction gates on holdout routes.
- Use Test 0 last to verify the actual autonomous path and start relocalization/fallback behavior.

Data collection workflow to give me when needed
1. Build and upload the selected test.
2. Connect the V5 Brain by programming cable.
3. In a terminal, change to this clone and run pros terminal. Example on the original computer:
   cd "/Users/ouji/Documents/Localization Test"
   pros terminal
   On another computer, replace the path with that clone's path.
4. Run autonomous. Wait until the Brain no longer shows "Saving tune log..." and shows the dump cue: "Tap lower-right to dump" after the normal route (test 0), "LOG READY tap LR dump" after tests 1-6, or "Log cached in RAM" when there is no SD card or the SD write failed.
5. Keep the terminal open and tap the lower-right of the Brain screen.
6. Capture everything from === BEGIN LOCALIZATION TUNE LOG === through === END LOCALIZATION TUNE LOG === and place it in the path you specified.

Completion criteria
Do not say "fully tuned" until repeated runs demonstrate all of the following:
- turn and straight calibration are stable across repetitions;
- sensor geometry is supported by multiple positions and validated on held-out logs;
- combined routes show no unexplained pose jumps or sequence loss;
- accepted corrections are geometrically plausible and remain rare when views are weak;
- applied fusion improves measured or defensible holdout error without reducing odometry-only consistency;
- start relocalization accepts clean views and safely falls back on blocked or ambiguous views;
- the final normal route is built and tested with the intended competition configuration.

The newest export is in src/tune.txt. Begin by inspecting it and the current source state. Analyze first; if the log cannot support a safe change, enable the next required test, build it, and give me the exact test conditions instead of guessing.
```
