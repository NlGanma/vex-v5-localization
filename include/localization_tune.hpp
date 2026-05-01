#pragma once

#include "autonomous_localization.hpp"
#include "lemlib/api.hpp"

namespace localization_tune {
inline constexpr bool kEnabled = true;

using autonomous_localization::DistanceSnapshot;
using autonomous_localization::RelocalizationSummary;
using autonomous_localization::SensorSnapshot;

void initializeRuntime();
void setState(const char* status, const char* stepName, bool running);
void clearExportFeedback();
void finalizeRun(const char* status, const char* stepName, const char* rumble);
void finalizeInterruptedRunIfNeeded(const char* status, const char* stepName);
void prepareAutonomousRelocalizedRun(const lemlib::Pose& currentPose);
RelocalizationSummary performGlobalRelocalization();
void storeRelocalizationSummary(const RelocalizationSummary& summary);
void setStartPose(const lemlib::Pose& pose);
void runAutonomousRoute(const lemlib::Pose& start, int testNumber, bool allowAbort);
void runAutonomousRoute(const lemlib::Pose& start, bool allowAbort);
void setDriverControlLoopActive(bool active);
void setDriverDriveLoopTicking(bool active);
} // namespace localization_tune
