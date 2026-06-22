# Localization field tests — protocol

Goal: turn the report's *modeled* numbers into *measured* ones. Every physical
measurement below is taken with the robot **stopped** (at the start, or at the
end of a route / single motion). Nothing here requires measuring the robot
mid-route.

The report's weak points these tests close:
- "It is a model, not a tape-measure" → Test 1 (real accuracy vs. ground truth)
- "Confirm the re-anchor on hardware, off vs on" → Test 2 (real before/after)
- start-pose reference / relocalization correctness → Test 3 (static, fast)
- "drift is directional (scale/slip)" is *inferred* → Test 4 (measured constants)

**If you're not sure the robot's odometry is fully tuned: run Test 4 FIRST.**
Test 4 *is* the tuning check — it measures wheel scale and degrees-per-rev as
constants. If they come back near ideal (scale ≈ 1.00, ≈ 360°/rev), you're
effectively tuned; proceed to Tests 1–2. If they're off, you now have the numbers
to tune odom — do that, then run Tests 1–2 so the accuracy figures reflect the
tuned robot. Notes on how tuning state affects each test:
- **Test 2 (before/after) is valid tuned or not** — both conditions share the
  same odom calibration, so the comparison is fair. An *under*-tuned robot drifts
  more, which makes the re-anchor's benefit *larger and clearer*, not invalid.
  Just never change tuning between the OFF and ON blocks.
- **Test 1 (accuracy) is tuning-dependent** — its absolute inches partly reflect
  odom calibration quality. Report it as "with odom as currently tuned"; re-run if
  you tune later.
- **Test 3 (start relocalization) is independent of drive tuning** — it's static
  and depends on sensor geometry/gates, not wheel scale. Safe to run anytime.

Recommended order:
1. Test 4 first if odom tuning is uncertain; tune from those constants if needed.
2. Test 3 while the robot is already on the field; it is static and quick.
3. Test 2 for the headline OFF-vs-ON repeatability result. Do not retune between
   the OFF and ON blocks.
4. Test 1 last, once the tuning state is the one you want the report to describe.

---

## Shared setup (do this once)

**Robot reference points.** Pick the point the code treats as the robot center
(the tracking center the pose is reported at). Mark it on the chassis with tape
so you can drop a plumb line / square down to the tile. Mark a **second** point
directly forward of it on the robot's centerline, a known distance `d` away
(measure `d` once, e.g. 8.00 in). Two marked points = you can recover heading
statically.

**Measuring a stationary pose (X, Y, heading).** With the robot sitting still:
1. Measure the perpendicular distance from the **center** mark to two adjacent
   perpendicular field walls. Decide once which wall is X=0 and which is Y=0.
   Those two distances are the robot's field (X, Y) at that instant.
2. Measure the **forward** mark's perpendicular distance to *one* of those same
   walls. Heading offset = `atan2( (forward_dist − center_dist), d )`. (I'll do
   the arithmetic — you just record the two distances and `d`.)

That's a tape measure and a carpenter's square. No mid-route access needed.

**Reproducible start.** For any test that repeats a route, the robot must start
in the *same physical spot* every rep, or endpoint spread is meaningless. Fixture
the start: push the chassis into a tile-seam corner with spacer blocks, or tape a
hard stop. Measure the true start pose once with the method above and reuse it.

**Routes** (from `README.txt`): `kLocalizationTuneTest = 0` wired sample,
`= 3` Square Loop, `= 6` Square + Cross. These also export the full trace/report.
The Test 4 probe below is the physical 100 in / 360° calibration check; do not
confuse it with the current firmware's `kLocalizationTuneTest = 4`, which is an
open-loop drive-balance probe.

---

## Test 1 — Endpoint accuracy vs. ground truth  *(highest value)*

**Proves:** how far the *real* robot ends from where it should be, and how far
each pose estimate (raw odom vs. filtered) is from the tape. This is the one
measurement that breaks the report's circular "truth = the filter's own estimate."

**Routes:** run 2 route (`=3`) and run 3 route (`=6`).

**Reps:** ~5 per route, same fixtured start each time.

**Per rep, physically measure (all static):**
- Start pose (once per route if the fixture is solid; the start is fixed).
- **End pose** of the robot after the route finishes: center→X-wall,
  center→Y-wall, forward→one-wall, and `d`.

**Log files needed: YES.** Export the full log each rep (same format as the
existing `run*` files). I read the final-row `odom_only_x/y/theta`,
`ekf_x/y/theta`, and `odom_x/y/theta` and compare all three against your tape.

**I deliver:** a table of real end error in inches/degrees for raw odom,
filtered, and shipped pose — replacing the "lower-bound model" caveat with
measured numbers.

---

## Test 2 — Before/after repeatability, flag off vs on  *(highest value)*

**Proves:** the headline "tighter run-to-run repeatability" and the ~1.8× claim,
on the *actual* firmware (the current traces were the old continuous-only build,
so the re-anchor curve in the report is a replay, not a real run).

**Route:** pick one — run 3 route (`=6`) is the best showcase (longest, most fixes).

**Two conditions, same fixtured start, N ≥ 8 reps each:**
- **OFF:** `loc.fusion.enableBoundaryReanchor = false` in
  `src/localization_config.cpp`, rebuild, upload, run N reps.
- **ON:** set it back to `true`, rebuild, upload, run N reps.

**Per rep, physically measure (static):** just the **end pose** (X, Y, heading)
by the shared method. That's all this test needs.

**Log files needed: YES for validating the PDF.** Endpoints alone give the
repeatability spread, but the report also claims the new boundary commit lands on
the new binary and costs no driving speed. For that, export the full logs so I can
compare route duration, accepted-fix count, and applied correction behavior. If
exporting fails, record each rep's autonomous duration and final brain-screen
pose/fix summary.

**I deliver:** endpoint spread (std-dev and max−min of X, Y, heading) for OFF vs
ON, measured before/after factor, route-duration comparison, and fix/correction
counts — the real version of the report's modeled bar chart, plus the hardware
check for the "no speed cost" and "boundary commits land on the new binary"
claims.

> Keep the *driving* identical between conditions — only the flag changes. If the
> robot physically can't repeat its own path closely with the flag OFF, that
> spread *is* the baseline you're trying to beat; don't "fix" it.

---

## Test 3 — Static start-relocalization check  *(cheap, fully static)*

**Proves:** that at `t=0` the filter actually snaps the start pose onto truth
(the report's premise that "start-pose reference" is what was wrong). Nothing
moves — you measure, place, power up, read.

**Procedure (no driving):**
1. Place the robot at a **known** field pose from the normal competition start —
   measure it with the shared method.
2. Run init / the route up to the first reported pose, then stop. Record the
   robot's *reported* start pose (`ekf_x/y/theta` and `odom_x/y/theta` from the
   first trace rows, or off the brain screen if shown).
3. Repeat for the deliberately-bad placements the report mentions: one sensor
   view blocked, and the weak/center start (the `run4`/`run5`/`run6` cases).

**Reps:** 2–3 per placement.

**Physically measure (static):** the true placed pose. That's it — the robot
never moves.

**Log files needed: YES** (short — just need the opening rows with the reported
start pose, `active_sensors`, `mcl_valid`, `last_nis`).

**I deliver:** reported-vs-true start error per placement, and confirmation the
two-sensor guard rejects the bad-view starts instead of locking onto a wrong pose.

---

## Test 4 — Controlled drift-rate probe  *(cheap, confirms "directional")*

**Proves:** the report's *inferred* claim that drift is a tank-drive scale/slip
asymmetry — by measuring the scale/heading error directly as constants, in a
single straight push and a single spin.

**Procedure:**
- **Straight:** from a fixtured start, drive straight a commanded distance
  (e.g. 100 in) on a clear lane, stop. Measure actual distance traveled with the
  tape (start mark → end mark on the floor). 3–5 reps. Repeat on the other axis.
- **Spin:** from a fixtured start, command a pure rotation (e.g. 360°), stop.
  Measure the actual heading change (shared heading method, or a protractor at
  the marks). 3–5 reps.

**Physically measure (static, single-motion endpoints):** distance for the
straight runs; heading for the spins. Both measured with the robot stopped.

**Log files needed: YES** (short — final row gives commanded vs. reported
`odom_only` distance/heading). Even the brain-screen odom readout works if
exporting is a hassle.

**I deliver:** measured X-scale, Y-scale, and degrees-per-revolution error — the
calibration constants that either confirm or replace the report's "directional
drift" hypothesis, and feed straight back into tuning.

---

## What to hand back to me

For each test, a tiny CSV is enough (I'll wire it into `drift_analysis.py`):

```
# test1_groundtruth.csv
run,rep,which,center_to_x_wall_in,center_to_y_wall_in,forward_to_wall_in,d_in
run3,1,start,12.00,12.00,12.00,8.00
run3,1,end,46.30,71.10,52.40,8.00
...
```
```
# test2_repeatability.csv
condition,rep,center_to_x_wall_in,center_to_y_wall_in,forward_to_wall_in,d_in
off,1,...
on,1,...
```

Plus the exported log files where a test says "Log files needed: YES" (drop them
next to the existing `run*` files). Tell me which wall you called X=0 / Y=0 and I
handle the rest.
