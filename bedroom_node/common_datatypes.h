#if !defined COMMON_DATATYPES_H
#define COMMON_DATATYPES_H

#include "config.h"

/* Logical states */
template <typename T>
struct state_tracker
{
  T val;
  T old;
};

struct sync_request 
{
  uint8_t msgID;
  uint8_t msgContent;
};

struct init_struct
{
  bool hasStarted;
  bool isCompleted;
  const unsigned long timeout = INIT_COMM_TIMEOUT;
  unsigned long elapsed_time;
};

struct color_request 
{
  uint8_t msgID;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

struct RGBcolor
{
  uint8_t R;
  uint8_t G;
  uint8_t B;
};

struct lamp_status
{
  uint8_t lamp_mode;
  RGBcolor color;
  uint8_t brightness;
  bool resync;
  bool streaming;
  uint32_t effect_delay;
  uint32_t effect_speed;
  uint32_t effect_amount;
};

enum StaticMode
{
  RGB_LOOP = 0,
  FADE_IN_OUT = 1,
  STROBE = 2,
  FIRE = 3,
  HALLOWEEN_EYES = 4,
  CYCLON_BOUNCE = 5,
  TWINKLE = 6,
  TWINKLE_RANDOM = 7,
  SPARKLE = 8,
  SNOW_SPARKLE = 9,
  RUNNING_LIGHTS = 10,
  COLOR_WIPE = 11,
  RAINBOW_CYCLE = 12,
  THEATER_CHASE = 13,
  THEATER_CHASE_RAINBOW = 14,
  BOUNCING_COLORED_BALLS = 15,
  METEOR_RAIN = 16,
  FADE_TO_BLACK = 17

};

enum system_state_var
{
  STARTUP = 0,
  NORMAL = 1,
  STREAMING = 2
};

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
