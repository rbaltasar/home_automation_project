#if !defined LEDCONTROLLER_H
#define LEDCONTROLLER_H

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include "config.h"
#include <FastLED.h>
#include "common_datatypes.h"
#include "LED_Static_Effects.h"
#include "timeSync.h"

class LEDStaticEffects;

class LEDController
{

private:

  CRGB m_leds[NUM_LEDS];
  lamp_status* m_lamp_status_request;
  uint8_t m_mode;
  timeSync m_timer;
  unsigned long m_last_iteration = 0;

  uint8_t led_idx = 0;
  uint8_t print_task = 0;

  LEDStaticEffects* m_static_effects;

public: 

  LEDController(lamp_status* lamp_status_request);
  ~LEDController();
  void setup(); 

  void update_color();
  void update_brightness();
  void update_mode();

  void feed();
  void resync();

  void setRGB(RGBcolor color);
  void setRGB(uint8_t R, uint8_t G, uint8_t B);
  void setAllLeds(RGBcolor, unsigned long delay_ms);
  void setLeds(RGBcolor, unsigned long delay_ms,uint8_t num_leds, bool erase_others = true);

  void test_effect(uint32_t print_delay);

  void end_effect();

};

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
