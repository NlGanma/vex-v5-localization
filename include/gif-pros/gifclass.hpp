#pragma once
#include "main.h"
#include "liblvgl/lvgl.h"
#include "gif-pros/gifdec.h"

/**
 * MIT License
 * Copyright (c) 2019 Theo Lemay
 * https://github.com/theol0403/gif-pros
 */

class Gif {

public:

  /**
   * Construct the Gif class
   * @param fname  the gif filename on the SD card (prefixed with /usd/)
   * @param parent the LVGL parent object
   */
  Gif(const char* fname, lv_obj_t* parent);

  /**
   * Construct Gif from an in-memory GIF buffer
   * @param data      pointer to gif bytes
   * @param len       buffer length in bytes
   * @param taskName  debug-friendly name for internal task
   * @param parent    the LVGL parent object
   * @param forceLoop when true, loops forever regardless of GIF loop metadata
   */
  Gif(const uint8_t* data, size_t len, const char* taskName, lv_obj_t* parent, bool forceLoop = false);

  /**
   * Destructs and cleans the Gif class
   */
  ~Gif();

  /**
   * Pauses the GIF task
   */
  void pause();

  /**
   * Resumes the GIF task
   */
  void resume();

  /**
   * Deletes GIF and frees all allocated memory
   */
  void clean();

  /**
   * Returns true when the decoder, canvas, and render task were all created.
   */
  bool isReady() const;

private:

  gd_GIF* _gif = nullptr; // gif decoder object
  void* _gifmem = nullptr; // gif file loaded from SD into memory 
  bool _ownsGifMem = false;
  bool _forceLoop = false;
  uint8_t* _buffer = nullptr; // decoder frame buffer

#if LVGL_VERSION_MAJOR >= 9
  lv_color32_t* _cbuf = nullptr; // canvas buffer
#else
  lv_color_t* _cbuf = nullptr; // canvas buffer
#endif
  lv_obj_t* _canvas = nullptr; // canvas object

  pros::task_t _task = nullptr; // render task

  /**
   * Shared constructor logic for file and memory-backed gifs.
   */
  void _initFromMemory(void* data, size_t len, const char* taskName, lv_obj_t* parent, bool ownsMemory, bool forceLoop);

  /**
   * Cleans and frees all allocated memory
   */
  void _cleanup();

  /**
   * Render cycle, blocks until loop count exceeds gif loop count flag (if any)
   */
  void _render();

  /**
   * Calls _render()
   * @param arg Gif*
   */
  static void _render_task(void* arg);

};
