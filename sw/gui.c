// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2025, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "lib/raylib.h"
#include "res/arial_14.h"
#include "res/icon.h"
#include "usb.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  ALIGN_V_TOP    = (1 << 0),
  ALIGN_V_CENTER = (1 << 1),
  ALIGN_V_BOTTOM = (1 << 2),
  ALIGN_H_LEFT   = (1 << 4),
  ALIGN_H_CENTER = (1 << 5),
  ALIGN_H_RIGHT  = (1 << 6),
  ALIGN_CENTER   = ALIGN_V_CENTER | ALIGN_H_CENTER,
};

enum
{
  Drag_None,
  Drag_HPos,
  Drag_TrigPos,
  Drag_TrigLevel,
  Drag_CursorA,
  Drag_CursorB,
};

enum
{
  TrigMode_Roll,
  TrigMode_Auto,
  TrigMode_Normal,
  TrigMode_Single,
  TrigModeCount,
};

static const char *trig_mode_str[TrigModeCount] = { "Roll", "Auto", "Normal", "Single" };

enum
{
  TrigState_Run,
  TrigState_Wait,
  TrigState_Trig,
  TrigState_Stop,
  TrigStateCount,
};

static const char *trig_state_str[TrigStateCount] = { "Run", "Wait", "Trig", "Stop" };
static const Color trig_state_color[TrigStateCount] =
{
  (Color){ 0x00, 0xaa, 0x00, 0xff },
  (Color){ 0xff, 0xa5, 0x00, 0xff },
  (Color){ 0x00, 0xff, 0x00, 0xff },
  (Color){ 0xff, 0x00, 0x00, 0xff }
};

enum
{
  TrigEdge_Rise,
  TrigEdge_Fall,
  TrigEdge_Both,
  TrigEdgeCount,
};

static const char *trig_edge_str[TrigEdgeCount] = { "/", "\\", "X" };

enum
{
  Range_Auto,
  Range_Low,
  Range_High,
  RangeCount,
};

static const char *range_str[RangeCount] = { "Range: Auto", "Range: Low", "Range: High" };

enum
{
  Filter_Off,
  Filter_Low,
  Filter_Med,
  Filter_High,
  FilterCount,
};

static const char *filter_str[FilterCount] = { "Filter: Off", "Filter: Low", "Filter: Med", "Filter: High" };

#define DRAG_SIZE            7

#define TEXT_GAP             5
#define SIDE_GAP             20
#define BUTTON_GAP           5
#define STATUS_GAP           5

#define BUTTON_HEIGHT        24
#define STATUS_HEIGHT        24

#define DIV_H                10
#define DIV_V                10

#define MIN_H_POS            -60000.0
#define MAX_H_POS            0.0

#define MIN_TRIG_LEVEL       -10.0
#define MAX_TRIG_LEVEL       100.0

#define ONE_SECOND           250000
#define TIME_PER_SAMPLE      4e-3 // ms

#define DISPLAY_BUFFER_SIZE  4096

#define CAPTURE_BLOCK_SIZE   256 // samples
#define CAPTURE_BUFFER_SIZE  65536 // blocks
#define CAPTURE_PTR_SIZE     (CAPTURE_BLOCK_SIZE * CAPTURE_BUFFER_SIZE)

#define CALIBRATION_MAGIC    0x78656c41

#define DEFAULT_MIN_VALUE    1000.0
#define DEFAULT_MAX_VALUE    -1000.0

#define HOURS_PER_YEAR       8760
#define HOURS_PER_MONTH      730
#define HOURS_PER_DAY        24

#define REFERENCE_CAPACITY   1000 // mA*h

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  bool     range;
  bool     split;
  float    min;
  float    max;
} DisplayBuffer;

typedef struct
{
  float    value[CAPTURE_BLOCK_SIZE];
  bool     range[CAPTURE_BLOCK_SIZE];
  float    min;
  float    max;
  float    sum;
  bool     block_range;
  bool     split;
} CaptureBlock;

typedef struct
{
  uint32_t magic;
  float    xa[2];
  float    ya[2];
  float    xb[2];
  float    yb[2];
} Calibration;

/*- Constants ---------------------------------------------------------------*/
static const Color c_background      = (Color){ 0x17, 0x1c, 0x1f, 0xff };
static const Color c_trace_high      = (Color){ 0xff, 0xff, 0x00, 0xff };
static const Color c_trace_low       = (Color){ 0x80, 0xf0, 0x10, 0xff };
static const Color c_trace_inactive  = (Color){ 0x80, 0x80, 0x80, 0xff };
static const Color c_trace_split     = (Color){ 0xa0, 0x10, 0x10, 0xff };
static const Color c_clipping        = (Color){ 0xff, 0x00, 0x00, 0xff };
static const Color c_grid_bg         = (Color){ 0x00, 0x00, 0x00, 0xff };
static const Color c_grid            = (Color){ 0x50, 0x50, 0x50, 0xff };
static const Color c_grid_zero       = (Color){ 0xe0, 0xe0, 0xe0, 0xff };
static const Color c_grid_text       = (Color){ 0xa9, 0xc1, 0xd1, 0xff };

static const Color c_button          = (Color){ 0x30, 0x3b, 0x41, 0xff };
static const Color c_button_border   = (Color){ 0x70, 0x70, 0x70, 0xff };
static const Color c_button_hover    = (Color){ 0x20, 0x3b, 0x51, 0xff };
static const Color c_button_text     = (Color){ 0xff, 0xff, 0xff, 0xff };
static const Color c_button_disabled = (Color){ 0xaa, 0xaa, 0xaa, 0xff };

static const Color c_label           = (Color){ 0x30, 0x3b, 0x41, 0xff };
static const Color c_label_hover     = (Color){ 0x20, 0x3b, 0x51, 0xff };
static const Color c_label_text      = (Color){ 0xff, 0xff, 0xff, 0xff };

static const Color c_map_bg          = (Color){ 0x20, 0x20, 0x20, 0xff };
static const Color c_map_win         = (Color){ 0x00, 0x00, 0x00, 0xff };

static const Color c_trig_pos        = (Color){ 0xff, 0xa0, 0x00, 0xff };
static const Color c_trig_level      = (Color){ 0x00, 0xff, 0x00, 0xff };
static const Color c_cursor_a        = (Color){ 0x20, 0xaa, 0x40, 0xff };
static const Color c_cursor_b        = (Color){ 0x20, 0xaa, 0xff, 0xff };
static const Color c_cursor_dt       = (Color){ 0xf0, 0xf0, 0x16, 0xff };
static const Color c_cursor_avg      = (Color){ 0x16, 0xf0, 0xec, 0xff };

static const Calibration c_default_cal =
{
  .magic = CALIBRATION_MAGIC,
  .xa = { 16000,  7000 },
  .ya = {   0.2,   1.0 },
  .xb = { 60000, 50000 },
  .yb = {   1.0,  80.0 },
};

/*- Variables ---------------------------------------------------------------*/
static Font g_font;

static int g_w;
static int g_h;
static int g_mx;
static int g_my;

static double g_cursor;

static s64 g_speed_test_time;
static int g_speed_test_size;

static CaptureBlock g_capture_buf[CAPTURE_BUFFER_SIZE];
static int g_capture_buf_ptr;

static DisplayBuffer g_display_buf[DISPLAY_BUFFER_SIZE];
static float g_display_min;
static float g_display_max;

static int g_drag = Drag_None;
static int g_drag_mx;
static int g_drag_my;
static double g_drag_x;
static double g_drag_y;

static double g_h_pos = 0.0;
static double g_h_win = 0.0;
static double g_v_pos = 0.0;
static int g_h_scale = 20; // ms
static int g_v_scale = 2000; // uA

static bool g_show_cursors;
static double g_cursor_a;
static double g_cursor_b;
static double g_cursor_avg;

static int g_trig_edge = TrigEdge_Rise;
static int g_trig_state = TrigState_Wait;
static int g_trig_mode = TrigMode_Auto;

static double g_trig_pos = 0.0;
static double g_trig_level = 9.0; // mA
static int g_trig_timer;
static int g_trig_blank;

static bool g_follow_trace;
static bool g_zero_trace;
static int g_trace_lock;

static int g_range = Range_High;
static int g_filter;

static int g_button_pos;
static int g_status_pos;

static int g_display_x;
static int g_display_y;
static int g_display_width;
static int g_display_height;

static double g_time_per_px;
static double g_current_per_px;

static double g_current_sum;
static int64_t g_current_samples;

static Calibration g_cal = c_default_cal;

static bool g_cal_enabled;
static int g_cal_count;
static double g_cal_raw;
static double g_cal_raw_acc;
static double g_cal_value;
static double g_cal_value_acc;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline Vector2 v2(int x, int y)
{
  return (Vector2){ x, y };
}

//-----------------------------------------------------------------------------
static inline int get_index(int index)
{
  return (CAPTURE_PTR_SIZE + index) % CAPTURE_PTR_SIZE;
}

//-----------------------------------------------------------------------------
static double clamp(double value, double min, double max)
{
  return os_max(os_min(value, max), min);
}

//-----------------------------------------------------------------------------
static void set_h_pos(double v)
{
  double win = g_h_scale * DIV_H;

  if (g_zero_trace)
    g_h_pos = 0.0;
  else if (v > MAX_H_POS)
    g_h_pos = MAX_H_POS;
  else if ((v - win) < MIN_H_POS)
    g_h_pos = MIN_H_POS + win;
  else
    g_h_pos = v;
}

//-----------------------------------------------------------------------------
static const char *time_str(double v, bool sign)
{
  static char buf[128];
  char *fmt = sign ? "%+.2f ms" : "%.2f ms";

  if (fabs(v) >= 1000.0)
  {
    fmt = sign ? "%+.2f s" : "%.2f s";
    v /= 1000.0;
  }

  snprintf(buf, sizeof(buf), fmt, v);

  return buf;
}

//-----------------------------------------------------------------------------
static const char *current_str(double v)
{
  static char buf[128];

  if (fabs(v) >= 1.0)
    snprintf(buf, sizeof(buf), "%.2f mA", v);
  else
    snprintf(buf, sizeof(buf), "%.2f uA", v * 1000.0);

  return buf;
}

//-----------------------------------------------------------------------------
void DrawTextF(const char *text, int x, int y, Color color)
{
  DrawTextEx(g_font, text, (Vector2){ x, y }, g_font.baseSize, 1, color);
}

//-----------------------------------------------------------------------------
void DrawTextA(const char *text, int x, int y, Color color, int align)
{
  double spacing = 1.0;
  Vector2 m = MeasureTextEx(g_font, text, g_font.baseSize, spacing);
  Vector2 p = { x, y };

  if (align & ALIGN_V_CENTER)
    p.y -= (int)(m.y / 2);
  else if (align & ALIGN_V_BOTTOM)
    p.y -= (int)m.y;

  if (align & ALIGN_H_CENTER)
    p.x -= (int)(m.x / 2);
  else if (align & ALIGN_H_RIGHT)
    p.x -= (int)m.x;

  DrawTextEx(g_font, text, p, g_font.baseSize, spacing, color);
}

//-----------------------------------------------------------------------------
void DrawLineW(int sx, int sy, int ex, int ey, int w, Color color)
{
  if (w == 1)
    DrawLine(sx, sy, ex, ey, color);
  else
    DrawLineEx((Vector2){sx, sy}, (Vector2){ex, ey}, w, color);
}

//-----------------------------------------------------------------------------
static void draw_grid(int x, int y, int w, int h)
{
  DrawRectangle(x, y, w, h, c_grid_bg);

  for (int i = 1; i < (DIV_H+1); i++)
  {
    int px = x + (w * i) / DIV_H;
    DrawLine(px, y, px, y+h, c_grid);
  }

  for (int i = 1; i < (DIV_V+1); i++)
  {
    int py = y + (h * i) / DIV_V;
    DrawLine(x, py, x+w, py, (i == (DIV_V-1)) ? c_grid_zero : c_grid);
  }
}

//-----------------------------------------------------------------------------
static void draw_grid_text(int x, int y, int w, int h)
{
  for (int i = 1; i < (DIV_V+1); i++)
  {
    int py = y + (h * i) / DIV_V;
    int scale = (DIV_V-1 - i) * g_v_scale;
    const char *text;

    if (abs(scale) < 1000)
      text = TextFormat("%d uA", scale);
    else
      text = TextFormat("%d mA", scale / 1000);

    DrawTextA(text, x+TEXT_GAP, py, c_grid_text, ALIGN_H_LEFT | ALIGN_V_BOTTOM);
  }

  (void)w;
}

//-----------------------------------------------------------------------------
static void draw_trig_level(int x, int y, int w, int h)
{
  int trig_level_px = y + (g_v_pos - g_trig_level) / g_current_per_px;

  DrawLine(x, trig_level_px, x + w, trig_level_px, c_trig_level);
  DrawTriangle(v2(x + w - DRAG_SIZE, trig_level_px), v2(x + w, trig_level_px + DRAG_SIZE),
      v2(x + w, trig_level_px - DRAG_SIZE), c_trig_level);

  (void)h;
}

//-----------------------------------------------------------------------------
static void draw_vert_marker(int x, int y, int w, int h, double value, Color color)
{
  int px = w + (-g_h_pos + value) / g_time_per_px;

  if (px < 0)
  {
    DrawTriangle(v2(x, y + DRAG_SIZE), v2(x + DRAG_SIZE, y + DRAG_SIZE*2), v2(x + DRAG_SIZE, y), color);
  }
  else if (px > w)
  {
    DrawTriangle(v2(x + w, y + DRAG_SIZE), v2(x + w - DRAG_SIZE, y), v2(x + w - DRAG_SIZE, y + DRAG_SIZE*2), color);
  }
  else
  {
    DrawLine(x + px, y, x + px, y + h, color);
    DrawTriangle(v2(x + px, y + DRAG_SIZE), v2(x + px + DRAG_SIZE, y), v2(x + px - DRAG_SIZE, y), color);
  }
}

//-----------------------------------------------------------------------------
static void draw_calibration(int x, int y, int w, int h)
{
  x += 10;
  y += 10;
  w = 200;
  h = g_font.baseSize * 4 + TEXT_GAP * 2;

  DrawRectangle(x, y, w, h, c_background);
  DrawRectangleLines(x, y, w, h, WHITE);

  x += TEXT_GAP;
  y += TEXT_GAP;

  if (Range_Low == g_range || Range_High == g_range)
    DrawTextA(range_str[g_range], x, y, WHITE, ALIGN_H_LEFT | ALIGN_V_TOP);
  else
    DrawTextA("Warning: select a fixed range", x, y, RED, ALIGN_H_LEFT | ALIGN_V_TOP);

  y += g_font.baseSize;

  if (g_cal_raw > 32768)
    DrawTextA("Point: High", x, y, WHITE, ALIGN_H_LEFT | ALIGN_V_TOP);
  else
    DrawTextA("Point: Low", x, y, WHITE, ALIGN_H_LEFT | ALIGN_V_TOP);

  y += g_font.baseSize;

  DrawTextA(TextFormat("Raw: %.0f", g_cal_raw), x, y, WHITE, ALIGN_H_LEFT | ALIGN_V_TOP);
  y += g_font.baseSize;

  DrawTextA(TextFormat("Value: %.3f mA", g_cal_value), x, y, WHITE, ALIGN_H_LEFT | ALIGN_V_TOP);
  y += g_font.baseSize;
}

//-----------------------------------------------------------------------------
static void draw(void)
{
  int x = g_display_x;
  int y = g_display_y;
  int w = g_display_width;
  int h = g_display_height;

  BeginScissorMode(x, y, w, h);

  draw_grid(x, y, w, h);

  draw_trig_level(x, y, w, h);

  draw_vert_marker(x, y, w, h, g_trig_pos, c_trig_pos);

  if (g_show_cursors)
  {
    draw_vert_marker(x, y, w, h, g_trig_pos + g_cursor_b, c_cursor_b);
    draw_vert_marker(x, y, w, h, g_trig_pos + g_cursor_a, c_cursor_a);
  }

  for (int i = 0; i < w; i++)
  {
    int max = y + (g_v_pos - g_display_buf[i].min) / g_current_per_px;
    int min = y + (g_v_pos - g_display_buf[i].max) / g_current_per_px;

    if (min == max)
      min = max + 1;

    if (g_display_buf[i].split)
      DrawLine(x + i, y, x + i, y + h, c_trace_split);
    else
      DrawLine(x + i, min, x + i, max, g_display_buf[i].range ? c_trace_high : c_trace_low);

    if (min < y)
      DrawLine(x + i, y, x + i, y + 5, c_clipping);

    if (max > (y + h))
      DrawLine(x + i, y + h, x + i, y + h - 5, c_clipping);
  }

  draw_grid_text(x, y, w, h);

  if (g_cal_enabled)
    draw_calibration(x, y, w, h);

  EndScissorMode();

  DrawRectangleLines(x-1, y-1, w+2, h+2, GRAY);
}

//-----------------------------------------------------------------------------
static void set_cal_value(void)
{
  if (!g_cal_enabled || (Range_Low != g_range && Range_High != g_range))
    return;

  int index = (Range_High == g_range);

  if (g_cal_raw < 32768)
  {
    g_cal.xa[index] = g_cal_raw;
    g_cal.ya[index] = g_trig_level;
  }
  else
  {
    g_cal.xb[index] = g_cal_raw;
    g_cal.yb[index] = g_trig_level;
  }
}

//-----------------------------------------------------------------------------
static void handle_map(void)
{
  int x = g_button_pos;
  int y = BUTTON_GAP;
  int w = g_w - g_button_pos - SIDE_GAP;
  int h = BUTTON_HEIGHT;
  bool hover = (x < g_mx && g_mx < (x+w)) && (y < g_my && g_my < (y+h));
  double scale = (double)w / (MAX_H_POS - MIN_H_POS);
  int win = os_max(g_h_win * scale, 10);
  int pos;

  DrawRectangle(x, y, w, h, c_map_bg);

  pos = clamp(w + (g_h_pos - g_h_win) * scale, 0, w - win);

  DrawRectangle(x + pos, y, win, h, c_map_win);

  int y1 = y;
  for (int i = 1; i < w; i++)
  {
    int y0 = y + 0.5*h + 0.2*h*sin(i * 0.5);
    bool out = (i <= pos) || (i > (pos + win));
    DrawLine(x + i, y0, x + i-1, y1, out ? c_trace_inactive : c_trace_high);
    y1 = y0;
  }

  DrawRectangleLines(x + pos, y, win, h, c_button_border);

  pos = x + w + g_trig_pos * scale;
  DrawLine(pos, y, pos, y + h, c_trig_pos);

  if (g_show_cursors)
  {
    pos = x + w + (g_trig_pos + g_cursor_a) * scale;
    DrawLine(pos, y, pos, y + h, c_cursor_a);

    pos = x + w + (g_trig_pos + g_cursor_b) * scale;
    DrawLine(pos, y, pos, y + h, c_cursor_b);
  }

  DrawRectangleLines(x, y, w, h, c_button_border);

  if (hover && IsMouseButtonDown(MOUSE_BUTTON_LEFT))
  {
    double h_pos = (double)(w - (g_mx - x)) * (MIN_H_POS - MAX_H_POS) / w;
    set_h_pos(h_pos + g_h_win/2);
  }
}

//-----------------------------------------------------------------------------
static bool handle_button(const char *text, int width, Color color)
{
  int x = g_button_pos;
  int y = BUTTON_GAP;
  int w = width;
  int h = BUTTON_HEIGHT;
  bool hover = (x < g_mx && g_mx < (x+w)) && (y < g_my && g_my < (y+h));

  DrawRectangle(x, y, w, h, hover ? c_button_hover : c_button);
  DrawTextA(text, g_button_pos + w/2, y + BUTTON_HEIGHT/2, color, ALIGN_H_CENTER | ALIGN_V_CENTER);
  DrawRectangleLines(x, y, w, h, c_button_border);

  g_button_pos += width + BUTTON_GAP;

  return hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

//-----------------------------------------------------------------------------
static void handle_buttons(void)
{
  if (handle_button(trig_state_str[g_trig_state], 50, trig_state_color[g_trig_state]))
  {
    if (TrigState_Stop == g_trig_state)
      g_trig_state = (TrigMode_Roll == g_trig_mode) ? TrigState_Run : TrigState_Wait;
    else
      g_trig_state = TrigState_Stop;
  }

  if (handle_button(trig_mode_str[g_trig_mode], 60, c_button_text))
  {
    g_trig_mode = (g_trig_mode + 1) % TrigModeCount;
    g_trig_state = (TrigMode_Roll == g_trig_mode) ? TrigState_Run : TrigState_Wait;
    g_trig_timer = 0;
    g_trig_blank = 0;
    g_trace_lock = 0;
  }

  if (handle_button(trig_edge_str[g_trig_edge], 20, c_button_text))
  {
    g_trig_edge = (g_trig_edge + 1) % TrigEdgeCount;
  }

  if (handle_button(TextFormat("T%s", time_str(g_trig_pos, true)), 80, c_trig_pos))
  {
    g_trig_pos = g_h_pos - g_h_win/2;
  }

  if (handle_button(TextFormat("%.2f mA", g_trig_level), 80, c_trig_level))
  {
    g_trig_level = (g_display_min + g_display_max) / 2.0;
  }

  if (handle_button("<>", 30, g_follow_trace ? c_button_text : c_button_disabled))
  {
    g_follow_trace = !g_follow_trace;
    g_trig_timer = 0;

    if (g_follow_trace)
    {
      g_trace_lock = (g_trace_lock < 0) ? g_trace_lock : -1;
    }
    else
    {
      g_trace_lock = 0;
      g_trig_state = (TrigMode_Roll == g_trig_mode) ? TrigState_Run : TrigState_Wait;
    }
  }

  if (handle_button(">>", 30, g_zero_trace ? c_button_text : c_button_disabled))
  {
    g_zero_trace = !g_zero_trace;

    if (g_zero_trace)
    {
      g_trace_lock = 0;
      set_h_pos(0.0);
    }
  }

  if (handle_button("Cursors", 80, g_show_cursors ? c_button_text : c_button_disabled))
  {
    g_show_cursors = !g_show_cursors;
  }

  if (handle_button(range_str[g_range], 110, c_button_text))
  {
    g_range = (g_range + 1) % RangeCount;
    usb_set_range(Range_Auto != g_range, Range_High == g_range);
  }

  if (handle_button(filter_str[g_filter], 110, c_button_text))
  {
    g_filter = (g_filter + 1) % FilterCount;
  }
}

//-----------------------------------------------------------------------------
static bool handle_label(const char *text, int width, Color color)
{
  int x = g_status_pos;
  int y = g_h - (STATUS_GAP + STATUS_HEIGHT);
  int w = width;
  int h = STATUS_HEIGHT;
  bool hover = (x < g_mx && g_mx < (x+w)) && (y < g_my && g_my < (y+h));

  DrawRectangle(x, y, w, h, hover ? c_label_hover : c_label);
  DrawTextA(text, g_status_pos + w/2, y + STATUS_HEIGHT/2, color, ALIGN_H_CENTER | ALIGN_V_CENTER);
  DrawRectangleLines(x, y, w, h, c_button_border);

  g_status_pos += width + STATUS_GAP;

  return hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

//-----------------------------------------------------------------------------
static const char *run_time_str(double avg)
{
  static char buf[128];
  double hours = (avg < 0.0) ? 0 : REFERENCE_CAPACITY / avg;

  if (hours > HOURS_PER_YEAR)
    snprintf(buf, sizeof(buf), "%.2f y", hours / HOURS_PER_YEAR);
  else if (hours > HOURS_PER_MONTH)
    snprintf(buf, sizeof(buf), "%.2f m", hours / HOURS_PER_MONTH);
  else if (hours > HOURS_PER_DAY)
    snprintf(buf, sizeof(buf), "%.2f d", hours / HOURS_PER_DAY);
  else
    snprintf(buf, sizeof(buf), "%.2f h", hours);

  return buf;
}

//-----------------------------------------------------------------------------
static void handle_status(void)
{
  const char *text;

  if (g_v_scale < 1000)
    text = TextFormat("%d uA/", g_v_scale);
  else
    text = TextFormat("%d mA/", g_v_scale / 1000);

  handle_label(text, 80, c_label_text);

  handle_label(TextFormat("%d ms/", g_h_scale), 80, c_label_text);

  handle_label(TextFormat("Min: %s", current_str(g_display_min)), 120, c_label_text);

  handle_label(TextFormat("Max: %s", current_str(g_display_max)), 120, c_label_text);

  double avg = g_current_samples ? g_current_sum / g_current_samples : 0.0;

  if (handle_label(TextFormat("Avg: %s", current_str(avg)), 120, c_label_text))
  {
    g_current_sum = 0.0;
    g_current_samples = 0;
  }

  handle_label(TextFormat("Run: %s", run_time_str(avg)), 120, c_label_text);

  if (g_show_cursors)
  {
    if (handle_label(TextFormat("A: %s", time_str(g_cursor_a, true)), 100, c_cursor_a))
    {
      g_cursor_a = 0.0;
    }

    if (handle_label(TextFormat("B: %s", time_str(g_cursor_b, true)), 100, c_cursor_b))
    {
      g_cursor_b = 0.0;
    }

    if (handle_label(TextFormat("dT: %s", time_str(g_cursor_b - g_cursor_a, false)), 100, c_cursor_dt))
    {
      g_cursor_a = 0.0;
      g_cursor_b = 0.0;
    }

    handle_label(TextFormat("Avg: %s", current_str(g_cursor_avg)), 120, c_cursor_avg);
  }
}

//-----------------------------------------------------------------------------
static void handle_shortcuts(void)
{
  if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyDown(KEY_LEFT_ALT) && IsKeyPressed(KEY_C))
  {
    g_cal_enabled = !g_cal_enabled;
  }
  else if (IsKeyPressed(KEY_UP) || IsKeyPressedRepeat(KEY_UP))
  {
    g_trig_level = clamp(g_trig_level + 0.001, MIN_TRIG_LEVEL, MAX_TRIG_LEVEL);
    set_cal_value();
  }
  else if (IsKeyPressed(KEY_DOWN) || IsKeyPressedRepeat(KEY_DOWN))
  {
    g_trig_level = clamp(g_trig_level - 0.001, MIN_TRIG_LEVEL, MAX_TRIG_LEVEL);
    set_cal_value();
  }

  if (g_cal_enabled)
  {
    if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_S))
    {
      usb_cal_write((uint8_t *)&g_cal, sizeof(Calibration));
    }
    else if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_R))
    {
      g_cal = c_default_cal;
    }
  }
}

//-----------------------------------------------------------------------------
static int get_increment(int value, int dir)
{
  if (dir > 0)
  {
    if (value > 100)
      return -100;
    else if (value > 10)
      return -10;
    else
      return -1;
  }
  else
  {
    if (value < 10)
      return 1;
    else if (value < 100)
      return 10;
    else
      return 100;
  }
}

//-----------------------------------------------------------------------------
static void handle_scale(void)
{
  int wheel = GetMouseWheelMove();

  if (0 == wheel)
    return;

  if (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL))
  {
    g_v_scale += get_increment(g_v_scale, wheel);
    g_v_scale = os_max(g_v_scale, 100);
    g_v_scale = os_min(g_v_scale, 12000);
  }
  else
  {
    double scale = g_h_scale;

    g_h_scale += get_increment(g_h_scale, wheel);
    g_h_scale = os_min(os_max(g_h_scale, 1), 6000);

    double d_px = (g_mx - (g_display_x + g_display_width)) * (scale / g_h_scale - 1.0);

    set_h_pos(g_h_pos + d_px * g_h_scale / ((double)g_display_width / DIV_H));
  }
}

//-----------------------------------------------------------------------------
static bool is_window_hover(void)
{
  return (g_display_y < g_my && g_my < (g_display_y + g_display_height)) &&
         (g_display_x < g_mx && g_mx < (g_display_x + g_display_width));
}

//-----------------------------------------------------------------------------
static bool is_trigger_hover(void)
{
  int px = g_display_y + (g_v_pos - g_trig_level) / g_current_per_px;
  return (is_window_hover() && abs(g_my - px) < DRAG_SIZE);
}

//-----------------------------------------------------------------------------
static bool is_cursor_hover(double cursor)
{
  return is_window_hover() && (fabs(g_cursor - cursor) / g_time_per_px < DRAG_SIZE);
}

//-----------------------------------------------------------------------------
static double clamp_cursor(double cursor)
{
  double abs = g_trig_pos + cursor;

  if (abs > MAX_H_POS)
    return MAX_H_POS - g_trig_pos;
  else if (abs < MIN_H_POS)
    return MIN_H_POS - g_trig_pos;
  else
    return cursor;
}

//-----------------------------------------------------------------------------
static void handle_drag(void)
{
  if (Drag_HPos == g_drag)
  {
    set_h_pos(g_drag_x - (double)(g_mx - g_drag_mx) * g_time_per_px);
  }
  else if (Drag_TrigLevel == g_drag)
  {
    g_trig_level = g_drag_y - (double)(g_my - g_drag_my) * g_current_per_px;
    g_trig_level = clamp(g_trig_level, MIN_TRIG_LEVEL, MAX_TRIG_LEVEL);
    g_trig_blank = 0;
    set_cal_value();
  }
  else if (Drag_TrigPos == g_drag)
  {
    g_trig_pos = clamp(g_cursor, MIN_H_POS, MAX_H_POS);
    g_cursor_a = clamp_cursor(g_cursor_a);
    g_cursor_b = clamp_cursor(g_cursor_b);
    g_trig_blank = 0;
  }
  else if (Drag_CursorA == g_drag)
  {
    g_cursor_a = clamp_cursor(g_cursor - g_trig_pos);
  }
  else if (Drag_CursorB == g_drag)
  {
    g_cursor_b = clamp_cursor(g_cursor - g_trig_pos);
  }

  if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
  {
    if (g_show_cursors && is_cursor_hover(g_trig_pos + g_cursor_a))
    {
      g_drag = Drag_CursorA;
      g_drag_x = g_cursor_a;
      g_drag_mx = g_mx;
    }
    else if (g_show_cursors && is_cursor_hover(g_trig_pos + g_cursor_b))
    {
      g_drag = Drag_CursorB;
      g_drag_x = g_cursor_b;
      g_drag_mx = g_mx;
    }
    else if (is_cursor_hover(g_trig_pos))
    {
      g_drag = Drag_TrigPos;
      g_drag_x = g_trig_pos;
      g_drag_mx = g_mx;
    }
    else if (is_trigger_hover())
    {
      g_drag = Drag_TrigLevel;
      g_drag_y = g_trig_level;
      g_drag_my = g_my;
    }
    else if (is_window_hover())
    {
      g_drag = Drag_HPos;
      g_drag_x = g_h_pos;
      g_drag_mx = g_mx;
    }
  }

  if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
  {
    g_drag = Drag_None;
  }
}

//-----------------------------------------------------------------------------
static void update_cursor_charge(void)
{
  int start = g_trace_lock + (g_trig_pos + os_min(g_cursor_a, g_cursor_b)) / TIME_PER_SAMPLE;
  double delta_ms = fabs(g_cursor_a - g_cursor_b);
  int size  = delta_ms / TIME_PER_SAMPLE;
  double sum = 0.0;
  int n_samples = size;

  start = get_index(g_capture_buf_ptr-1 + start);

  while (size > 0)
  {
    int block = start / CAPTURE_BLOCK_SIZE;
    int index = start % CAPTURE_BLOCK_SIZE;

    if (0 == index && size >= CAPTURE_BLOCK_SIZE)
    {
      sum += g_capture_buf[block].sum;
      size -= CAPTURE_BLOCK_SIZE;
      start = (start + CAPTURE_BLOCK_SIZE) % CAPTURE_PTR_SIZE;
    }
    else
    {
      sum += g_capture_buf[block].value[index];
      size--;
      start = (start + 1) % CAPTURE_PTR_SIZE;
    }
  }

  g_cursor_avg = n_samples ? sum / n_samples : 0.0;
}

//-----------------------------------------------------------------------------
static void fill_display_buffer(DisplayBuffer *buf, int start, int size)
{
  buf->range = false;
  buf->split = false;
  buf->min   = DEFAULT_MIN_VALUE;
  buf->max   = DEFAULT_MAX_VALUE;

  start = get_index(g_capture_buf_ptr-1 + start);

  while (size > 0)
  {
    int block = start / CAPTURE_BLOCK_SIZE;
    int index = start % CAPTURE_BLOCK_SIZE;

    if (0 == index && size >= CAPTURE_BLOCK_SIZE)
    {
      buf->range |= g_capture_buf[block].block_range;
      buf->min = os_min(buf->min, g_capture_buf[block].min);
      buf->max = os_max(buf->max, g_capture_buf[block].max);

      size -= CAPTURE_BLOCK_SIZE;
      start = (start + CAPTURE_BLOCK_SIZE) % CAPTURE_PTR_SIZE;
    }
    else
    {
      buf->range |= g_capture_buf[block].range[index];
      buf->min = os_min(buf->min, g_capture_buf[block].value[index]);
      buf->max = os_max(buf->max, g_capture_buf[block].value[index]);

      size--;
      start = (start + 1) % CAPTURE_PTR_SIZE;
    }

    buf->split |= g_capture_buf[block].split;
  }
}

//-----------------------------------------------------------------------------
static void update_display_buffer(void)
{
  os_assert(g_display_width < DISPLAY_BUFFER_SIZE);

  g_display_min = DEFAULT_MIN_VALUE;
  g_display_max = DEFAULT_MAX_VALUE;

  int size = g_time_per_px / TIME_PER_SAMPLE + 1;

  for (int i = 0; i < g_display_width; i++)
  {
    int start = g_trace_lock + (g_h_pos - g_h_win + g_time_per_px * i) / TIME_PER_SAMPLE;

    fill_display_buffer(&g_display_buf[i], start, size);

    if (i > 0)
    {
      if (g_display_buf[i-1].min > g_display_buf[i].max)
      {
        float avg = (g_display_buf[i-1].min + g_display_buf[i].max) / 2.0;
        g_display_buf[i-1].min = avg;
        g_display_buf[i].max = avg;
      }

      if (g_display_buf[i-1].max < g_display_buf[i].min)
      {
        float avg = (g_display_buf[i-1].max + g_display_buf[i].min) / 2.0;
        g_display_buf[i-1].max = avg;
        g_display_buf[i].min = avg;
      }
    }

    g_display_min = os_min(g_display_min, g_display_buf[i].min);
    g_display_max = os_max(g_display_max, g_display_buf[i].max);
  }
}

//-----------------------------------------------------------------------------
static float get_current(int v)
{
  int r = v & 1;
  float d = g_cal.xb[r] - g_cal.xa[r];
  return g_cal.ya[r] * ((g_cal.xb[r] - (float)v) / d) + g_cal.yb[r] * (((float)v - g_cal.xa[r]) / d);
}

//-----------------------------------------------------------------------------
static void process_sample(u16 sample)
{
  static const float alpha[FilterCount] = { 1.0, 0.5, 0.35, 0.2 };
  static bool was_stopped = false;
  static float value = 0.0;

  value = get_current(sample) * alpha[g_filter] + value * (1.0 - alpha[g_filter]);

  g_current_sum += value;
  g_current_samples++;

  if (TrigState_Stop == g_trig_state)
  {
    was_stopped = true;
    g_trig_blank = os_max(g_trig_blank, -g_trig_pos / TIME_PER_SAMPLE + CAPTURE_BLOCK_SIZE);
    return;
  }

  int block = g_capture_buf_ptr / CAPTURE_BLOCK_SIZE;
  int index = g_capture_buf_ptr % CAPTURE_BLOCK_SIZE;

  g_capture_buf[block].value[index] = value;
  g_capture_buf[block].range[index] = sample & 1;
  g_capture_buf_ptr = (g_capture_buf_ptr + 1) % CAPTURE_PTR_SIZE;

  if ((CAPTURE_BLOCK_SIZE-1) == index)
  {
    g_capture_buf[block].min = DEFAULT_MIN_VALUE;
    g_capture_buf[block].max = DEFAULT_MAX_VALUE;
    g_capture_buf[block].sum = 0.0;
    g_capture_buf[block].block_range = false;
    g_capture_buf[block].split = was_stopped;

    was_stopped = false;

    for (int i = 0; i < CAPTURE_BLOCK_SIZE; i++)
    {
      g_capture_buf[block].min = os_min(g_capture_buf[block].min, g_capture_buf[block].value[i]);
      g_capture_buf[block].max = os_max(g_capture_buf[block].max, g_capture_buf[block].value[i]);
      g_capture_buf[block].sum += g_capture_buf[block].value[i];
      g_capture_buf[block].block_range |= g_capture_buf[block].range[i];
    }
  }

  bool reset_lock = false;

  if (g_trace_lock < 0)
  {
    g_trace_lock--;

    if ((g_trace_lock * TIME_PER_SAMPLE + g_h_pos - g_h_win) < MIN_H_POS)
      reset_lock = true;
  }

  if (g_trig_timer > 0)
  {
    if (0 == --g_trig_timer)
    {
      if (TrigMode_Auto == g_trig_mode)
        reset_lock = true;

      g_trig_state = TrigState_Wait;
    }
  }

  if (reset_lock)
  {
    g_trace_lock = 0;
    g_follow_trace = false;
  }

  if (g_trig_blank > 0)
    g_trig_blank--;

  if (TrigMode_Roll != g_trig_mode && !g_follow_trace && 0 == g_trig_blank)
  {
    int trig_index = g_trig_pos / TIME_PER_SAMPLE;

    int a = get_index(g_capture_buf_ptr-1 + trig_index-1);
    int b = get_index(g_capture_buf_ptr-1 + trig_index);

    float va = g_capture_buf[a / CAPTURE_BLOCK_SIZE].value[a % CAPTURE_BLOCK_SIZE];
    float vb = g_capture_buf[b / CAPTURE_BLOCK_SIZE].value[b % CAPTURE_BLOCK_SIZE];

    bool rise = (va < g_trig_level && vb > g_trig_level);
    bool fall = (va > g_trig_level && vb < g_trig_level);
    bool trig = (rise && TrigEdge_Rise == g_trig_edge) || (fall && TrigEdge_Fall == g_trig_edge) ||
        ((rise || fall) && TrigEdge_Both == g_trig_edge);

    if (trig)
    {
      g_trace_lock = -1;

      if (TrigMode_Single == g_trig_mode)
      {
        g_trig_state = TrigState_Stop;
      }
      else
      {
        g_trig_state = TrigState_Trig;
        g_trig_timer = ONE_SECOND;
      }
    }
  }

  if (g_cal_enabled)
  {
    g_cal_raw_acc   += sample;
    g_cal_value_acc += value;

    if (ONE_SECOND == g_cal_count++)
    {
      g_cal_raw   = g_cal_raw_acc / g_cal_count;
      g_cal_value = g_cal_value_acc / g_cal_count;

      g_cal_count = 0;
      g_cal_raw_acc = 0;
      g_cal_value_acc = 0;
    }
  }
}

//-----------------------------------------------------------------------------
static void speed_test(void)
{
  s64 time = os_get_time();
  s64 delta = time - g_speed_test_time;

  g_speed_test_size += USB_BLOCK_SIZE * sizeof(u16);

  if (delta > 1000)
  {
    f64 speed = (f64)g_speed_test_size / ((f64)delta / 1000.0) / 1000.0;
    printf("Transfer rate: %5.2f KB/s --- %d (%d ms)\r\n", speed, g_speed_test_size, (int)delta);
    g_speed_test_size = 0;
    g_speed_test_time = time;
  }
}

//-----------------------------------------------------------------------------
static void capture_process(void)
{
  static int skip_count = 0;
  static int skip_value = 0;
  static int prev = 32768;
  u16 block[USB_BLOCK_SIZE];

  while (usb_read_buf(block))
  {
    speed_test();

    for (int i = 0; i < USB_BLOCK_SIZE; i++)
    {
      if ((prev & 1) != (block[i] & 1))
      {
        skip_count = 8;
        skip_value = prev;
      }
      else if (skip_count > 0)
      {
        skip_count--;
      }

      if (0 == skip_count)
        process_sample(block[i]);
      else
        process_sample(skip_value);

      prev = block[i];
    }
  }
}

//-----------------------------------------------------------------------------
static void load_calibration(void)
{
  Calibration cal;

  usb_cal_read((uint8_t *)&cal, sizeof(Calibration));

  if (CALIBRATION_MAGIC == cal.magic)
    g_cal = cal;
}

//-----------------------------------------------------------------------------
void gui_run(void)
{
  InitWindow(1024, 768, "Current Profiler");
  SetWindowState(FLAG_WINDOW_RESIZABLE);
  SetWindowMinSize(640, 480);
  Image icon = LoadImageFromMemory(".png", res_icon_png, res_icon_png_len);
  SetWindowIcon(icon);
  SetTargetFPS(60);
  //RestoreWindow();
  //MaximizeWindow();

  Image font = LoadImageFromMemory(".png", res_arial_14_png, res_arial_14_png_len);
  g_font = LoadFontFromImage(font, MAGENTA, ' ');

  load_calibration();

  usb_set_range(Range_Auto != g_range, Range_High == g_range);

  while (!WindowShouldClose())
  {
    g_w = GetRenderWidth();
    g_h = GetRenderHeight();

    Vector2 mouse = GetMousePosition();

    g_mx = mouse.x;
    g_my = mouse.y;

    capture_process();

    handle_shortcuts();
    handle_scale();
    handle_drag();

    g_display_x      = SIDE_GAP;
    g_display_y      = BUTTON_GAP*2 + BUTTON_HEIGHT;
    g_display_width  = g_w - SIDE_GAP*2;
    g_display_height = g_h - ((STATUS_GAP + BUTTON_GAP)*2 + STATUS_HEIGHT + BUTTON_HEIGHT);

    g_h_win          = g_h_scale * DIV_H;
    g_time_per_px    = g_h_win / (double)g_display_width;

    g_v_pos          = (g_v_scale * (DIV_V-1)) / 1000.0;
    g_current_per_px = (g_v_scale * DIV_V) / ((double)g_display_height * 1000.0);

    g_cursor         = (g_mx - (g_display_x + g_display_width)) * g_time_per_px + g_h_pos;

    if (g_show_cursors)
      update_cursor_charge();

    update_display_buffer();

    BeginDrawing();
      ClearBackground(c_background);

      g_button_pos = SIDE_GAP;
      g_status_pos = SIDE_GAP;

      handle_buttons();
      handle_map();
      handle_status();

      draw();

      DrawFPS(g_w - 100, g_h - 25);
    EndDrawing();
  }

  CloseWindow();
}


