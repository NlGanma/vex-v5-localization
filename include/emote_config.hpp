#pragma once

#include "liblvgl/lvgl.h"

namespace emote_config {
inline constexpr bool kOverrideDisplayWithLoopVideo = false;
inline constexpr bool kForceLoopPlayback = true;

// MP4 is not decoded by gif-pros; convert your source video to GIF and place it
// at static/loop_video.gif for embedding.
inline constexpr const char* kLoopAssetFile = "static/loop_video.gif";
inline constexpr const char* kLoopName = "LoopVideo";

inline constexpr lv_coord_t kContainerWidthPx = 480;
inline constexpr lv_coord_t kContainerHeightPx = 240;
inline constexpr lv_align_t kContainerAlign = LV_ALIGN_TOP_LEFT;
inline constexpr lv_coord_t kContainerOffsetX = 0;
inline constexpr lv_coord_t kContainerOffsetY = 0;
} // namespace emote_config
