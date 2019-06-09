
#include "LED_controller.h"


LEDController::LEDController(lamp_status* lamp_status_request):
m_lamp_status_request(lamp_status_request)
{
  m_static_effects = new LEDStaticEffects(this, m_leds, &m_timer);
}

LEDController::~LEDController()
{
  delete m_static_effects;
}

void LEDController::setRGB(RGBcolor color)
{
  setAllLeds(color,0);
}

void LEDController::resync()
{
  m_last_iteration = m_timer.getTime();
  led_idx = 0;
  print_task = 0;  
}

void LEDController::setRGB(uint8_t R, uint8_t G, uint8_t B)
{
  RGBcolor color_req;
  color_req.R = R;
  color_req.G = G;
  color_req.B = B;
  
  setAllLeds(color_req,0);
}

void LEDController::setup()
{
  FastLED.addLeds<WS2812, LED_PIN, GRB>(m_leds, NUM_LEDS);
  
  setRGB(0,0,0);
}

void LEDController::setAllLeds(RGBcolor color, unsigned long delay_ms)
{
  setLeds(color,delay_ms,NUM_LEDS);
}

void LEDController::setLeds(RGBcolor color, unsigned long delay_ms, uint8_t num_leds, bool erase_others)
{
  color.R > 255 ? color.R = 255 : color.R = color.R;
  color.G > 255 ? color.G = 255 : color.G = color.G;
  color.B > 255 ? color.B = 255 : color.B = color.B;
  for(uint8_t i = 0; i < num_leds; i++ )
  {
    m_leds[i] = CRGB(color.R, color.G, color.B);
    FastLED.show();
    delay(delay_ms); 
  }
  if(erase_others)
  {
    for(uint8_t i = num_leds; i < NUM_LEDS; i++ )
    {
      m_leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
  }
}

void LEDController::update_color()
{
  setAllLeds(m_lamp_status_request->color,0);
}
  
void LEDController::update_brightness()
{
  if(m_mode == 1)
  {
    
  }
}

void LEDController::feed()
{
  /* Music effects */
  if(m_mode > 1 && m_mode < 10)
  {
    switch(m_mode)
    {
      case 2:
        test_effect(m_lamp_status_request->effect_speed);
        break;
      case 3:
        m_static_effects->wakeup_effect(2000,1);
    }
  }

  /* Static effects */
  else if(m_mode >= 10)
  {
    switch(m_mode - 10)
    {
      case RGB_LOOP:
        m_static_effects->RGBLoop();
        break;
      case FADE_IN_OUT:
        m_static_effects->FadeInOut(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B);
        break;
      case STROBE:
        m_static_effects->Strobe(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B,50, m_lamp_status_request->effect_speed, m_lamp_status_request->effect_delay);
        break;
      case FIRE:
        m_static_effects->Fire(55, 120,m_lamp_status_request->effect_speed);
        break;
      case HALLOWEEN_EYES:
        m_static_effects->HalloweenEyes(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B, NUM_LEDS/5, 2, true, 1, m_lamp_status_request->effect_speed,  m_lamp_status_request->effect_delay);
        break;         
      case CYCLON_BOUNCE:
        m_static_effects->CylonBounce(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B, NUM_LEDS/5, m_lamp_status_request->effect_speed, m_lamp_status_request->effect_delay);
        break;
      case TWINKLE:
        m_static_effects->Twinkle(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B, 16, m_lamp_status_request->effect_speed, false);
        break;
      case TWINKLE_RANDOM:
        m_static_effects->TwinkleRandom(16, m_lamp_status_request->effect_speed, false);
        break;
      case SPARKLE:
        m_static_effects->Sparkle(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B, m_lamp_status_request->effect_speed);
        break;
      case SNOW_SPARKLE:
        m_static_effects->SnowSparkle(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B, m_lamp_status_request->effect_speed, m_lamp_status_request->effect_delay);
        break;
      case RUNNING_LIGHTS:
        m_static_effects->RunningLights(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B, m_lamp_status_request->effect_speed);
        break;
      case COLOR_WIPE:
        m_static_effects->colorWipe(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B, m_lamp_status_request->effect_speed);
         break;
      case RAINBOW_CYCLE:
         m_static_effects->rainbowCycle(m_lamp_status_request->effect_speed);
         break;
      case THEATER_CHASE:
         m_static_effects->theaterChase(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B, m_lamp_status_request->effect_speed);
         break;
      case THEATER_CHASE_RAINBOW:
         m_static_effects->theaterChaseRainbow(m_lamp_status_request->effect_speed);
         break;
      case BOUNCING_COLORED_BALLS:
      {
         byte color[10][3] = { {m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B},
                                    {m_lamp_status_request->color.G, m_lamp_status_request->color.B, m_lamp_status_request->color.R},
                                    {m_lamp_status_request->color.B, m_lamp_status_request->color.R, m_lamp_status_request->color.G},
                                    {m_lamp_status_request->color.R, m_lamp_status_request->color.R, m_lamp_status_request->color.G},
                                    {m_lamp_status_request->color.G, m_lamp_status_request->color.G, m_lamp_status_request->color.R},
                                    {m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.R},
                                    {m_lamp_status_request->color.B, m_lamp_status_request->color.B, m_lamp_status_request->color.G},
                                    {m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.R},
                                    {m_lamp_status_request->color.G, m_lamp_status_request->color.B, m_lamp_status_request->color.G},
                                    {m_lamp_status_request->color.R, m_lamp_status_request->color.B, m_lamp_status_request->color.B}};
         m_static_effects->BouncingColoredBalls(m_lamp_status_request->effect_amount, color, true, 2);
         break;
      }
      case METEOR_RAIN:
         m_static_effects->meteorRain(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B, m_lamp_status_request->effect_amount, m_lamp_status_request->effect_amount / 2, true, m_lamp_status_request->effect_speed);
         break;
      case FADE_TO_BLACK:
         m_static_effects->fadeToBlack(20/*int ledNo*/, 0x0F/*byte fadeValue*/);
         break;
      default:
        break;      
    }
  }
  
  
}
  
void LEDController::update_mode()
{
  m_mode = m_lamp_status_request->lamp_mode;

  switch(m_mode)
  {
    case 0:
      setRGB(0,0,0);
      break;
    case 1:
      setRGB(m_lamp_status_request->color.R / m_lamp_status_request->brightness, m_lamp_status_request->color.G / m_lamp_status_request->brightness, m_lamp_status_request->color.B / m_lamp_status_request->brightness);
      break;
  }
}

void LEDController::end_effect()
{
  m_static_effects->end_effect();
}

void LEDController::test_effect(uint32_t print_delay)
{
  unsigned long now = m_timer.getTime();

  if( ((now - m_last_iteration) > print_delay) && (print_task == 0) )
  {
    //Serial.println("Iteration ON due");
    m_last_iteration = now;
    if( led_idx < NUM_LEDS )
    {
      m_leds[led_idx] = CRGB(m_lamp_status_request->color.R, m_lamp_status_request->color.G, m_lamp_status_request->color.B);
      FastLED.show();
      led_idx++;
    }
    else
    {
      led_idx = 0;
      print_task = 1;
    }
  }
  
  else if( ((now - m_last_iteration) > print_delay) && (print_task == 1) )
  {
    //Serial.println("Iteration OFF due");
    m_last_iteration = now;
    if( led_idx < NUM_LEDS )
    {
      m_leds[led_idx] = CRGB(0, 0, 0);
      FastLED.show();
      led_idx++;
    }
    else
    {
      led_idx = 0;
      print_task = 0;
    }    
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
