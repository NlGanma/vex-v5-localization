#pragma once

// Drive a signed rotation (rotation sensor degrees).
// maxPct/minPct in 0..100. kp corrects to the starting IMU heading.
void dismove_dis_maxspeed_max_min_min_kp_kp(double disDeg,
                                            double maxPct,
                                            double minPct,
                                            double kp);
