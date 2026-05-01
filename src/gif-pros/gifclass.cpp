#include "gifclass.hpp"

Gif::Gif(const char*, lv_obj_t*) {}

Gif::Gif(const uint8_t*, size_t, const char*, lv_obj_t*, bool) {}

Gif::~Gif() = default;

void Gif::pause() {}

void Gif::resume() {}

void Gif::clean() {}

bool Gif::isReady() const { return false; }

void Gif::_initFromMemory(void*, size_t, const char*, lv_obj_t*, bool, bool) {}

void Gif::_cleanup() {}

void Gif::_render() {}

void Gif::_render_task(void*) {}
