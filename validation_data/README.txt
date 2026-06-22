# Localization validation data collection files.
# Keep src/tune.txt as the latest robot export, then paste each run's full exported log into the matching file below.
# Provenance note: run1/run4 were recorded on the May 31 18:25/19:22 builds whose report header echoed a stale
# default test_case (1, Turn center). The actual route is identified by checkpoint_count and path length:
# checkpoint_count=0 with ~135 in of path = the wired normal sample route (kLocalizationTuneTest = 0).
# src/tune.txt currently holds a May 24 pre-calibration sensor-angle sweep (test 5) kept as historical
# evidence for the old two-sensor wall_direct accept and the sensor-angle calibration provenance.
# The sample autonomous route is already wired into src/autonomous_control.cpp and records a full tune trace/report.
# Set kLocalizationTuneTest = 0 to run it. Set kLocalizationTuneTest = 1..6 to run the built-in tune routes, which also record full tune traces/reports.
# Since the air pump/PTO is not available, the normal route and score() path leave the robot in 4-motor mode.

Recommended 30 minute order:
1. Set kLocalizationTuneTest = 0, run the wired sample route, and paste the exported log into run1_sample_route_moving_fusion_log.txt.
2. Set kLocalizationTuneTest = 3, run Square Loop from the normal competition start, and paste into run2_normal_start_relocalize_log.txt.
3. Set kLocalizationTuneTest = 6, run Square + Cross from the normal competition start, and paste into run3_square_cross_full_log.txt.
4. Set kLocalizationTuneTest = 0, partially block/aim away one wall sensor view at start, run the normal route again, and paste into run4_bad_view_fallback_log.txt.
5. Historical pre-guard run: run5_forced_start_fallback_log.txt exposed a false wall_direct success with only two scored sensors when the robot was placed near the middle/center object.
6. After the two-sensor wall_direct guard is built/uploaded, set kLocalizationTuneTest = 0 and repeat the forced weak-start case from the normal safe start. Paste into run6_after_two_sensor_guard_log.txt.
7. Final post-guard regression check: set kLocalizationTuneTest = 0, run the normal route from the normal safe start with no sensors covered, and paste into run7_post_guard_clean_start_log.txt.
