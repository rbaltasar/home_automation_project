
#include "LED_Static_Effects.h"


LEDStaticEffects::LEDStaticEffects(LEDController* led_controller, CRGB* leds_ptr, timeSync* timer):
m_led_controller(led_controller),
leds(leds_ptr),
m_timer(timer)
{
  start_sequence = true;
  m_last_iteration = 0;
}

LEDStaticEffects::~LEDStaticEffects()
{
  end_effect();
}

void LEDStaticEffects::end_effect()
{
  start_sequence = true;
  deallocate_bouncing_mem();
}

void LEDStaticEffects::wakeup_effect(int wakeup_delay, int step)
{
  unsigned long now = m_timer->getTime();
  int r,g,b,current_delay;
  if(start_sequence)
  {
    r = 0;
    g = 0;
    b = 0;
    current_delay = wakeup_delay;
    start_sequence = false;
  }
  else
  {
    r = m_effect_state[0];
    g = m_effect_state[1];
    b = m_effect_state[2];
    current_delay = m_effect_state[3];
  }

  if( (now - m_last_iteration) > current_delay)
  {
    m_last_iteration = now;
    current_delay -= 4;

    if(r < 255)
    {
      r += (int)(2.25 * step);
      g += step;
      b = 0;

      if(r > 255) r = 255;
    }
    else
    {
      if(g < 250)
      {
        g += step;
        b += (int)(1.5 * step);
      }
    }

    setAll(r,g,b);
    showStrip();
  }

  m_effect_state[0] = r;
  m_effect_state[1] = g;
  m_effect_state[2] = b;
  m_effect_state[3] = current_delay;
  
  
}

void LEDStaticEffects::RGBLoop()
{

  unsigned long now = m_timer->getTime();
  static const uint8_t delay_ms = 3;

  if( (now - m_last_iteration) > delay_ms )
  {
    m_last_iteration = now;
    int j,k,order;
    if(start_sequence)
    {
      j = 0;
      k = 0;
      order = 0;
      start_sequence = false;
    }
    else
    {
      j = m_effect_state[0];
      k = m_effect_state[1];
      order = m_effect_state[2];
    }
    switch(j)
    {
      case 0: setAll(k,0,0); break;
      case 1: setAll(0,k,0); break;
      case 2: setAll(0,0,k); break;
    }
    showStrip();

    if(order == 0)
    {
      k++;
      if(k == 255) order = 1;
    }
    else
    {
      k--;
      if(k == 0)
      {
        order = 0;
        ++j >= 3 ? j = 0 : j = j;
      }
    }

    m_effect_state[0] = j;
    m_effect_state[1] = k;
    m_effect_state[2] = order;
  }
}

void LEDStaticEffects::FadeInOut(byte red, byte green, byte blue)
{

  int k,order;
  if(start_sequence)
  {
    k = 0;
    order = 0;
    start_sequence = false;
  }
  else
  {
    k = m_effect_state[0];
    order = m_effect_state[1];
  }
  float r, g, b;

  if(order == 0)
  {
    k++;

    if(k == 255) order = 1;
  }
  else
  {
    k--;
    if(k == 0) order = 0;
  }

  r = int((float)((k/256.0))*red);
  g = int((float)((k/256.0))*green);
  b = int((float)((k/256.0))*blue);

  setAll(r,g,b);
  showStrip();

  m_effect_state[0] = k;
  m_effect_state[1] = order;
}

void LEDStaticEffects::Strobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause)
{

  unsigned long now = m_timer->getTime();
  int j,order;
  if(start_sequence)
  {
    j = 0;
    order = 0;
    start_sequence = false;
  }
  else
  {
    j = m_effect_state[0];
    order = m_effect_state[1];
  }

  if( ((now - m_last_iteration) > FlashDelay) && ( (order == 0) || (order == 1) ) )
  {
    m_last_iteration = now;

    if(order == 0)
    {
      setAll(red,green,blue);
      showStrip();
      order = 1;
    }
    else if(order == 1)
    {
      setAll(0,0,0);
      showStrip();
      if(++j < StrobeCount) order = 0;
      else
      {
        j = 0;
        order = 2;
      }
    }
  }

  else if ( ((now - m_last_iteration) > EndPause) && (order == 2)  )
  {
    Serial.println("Running Strobe");
    Serial.println(StrobeCount);
    Serial.println(FlashDelay);
    Serial.println(EndPause);
    order = 0;
  }


  m_effect_state[0] = j;
  m_effect_state[1] = order;
}

void LEDStaticEffects::Fire(int Cooling, int Sparking, int SpeedDelay)
{

  static byte heat[NUM_LEDS];

  unsigned long now = m_timer->getTime();

  if((now - m_last_iteration) > SpeedDelay)
  {

    m_last_iteration = now;
    int cooldown;

    // Step 1.  Cool down every cell a little
    for( int i = 0; i < NUM_LEDS; i++) {
      cooldown = random(0, ((Cooling * 10) / NUM_LEDS) + 2);

      if(cooldown>heat[i]) {
        heat[i]=0;
      } else {
        heat[i]=heat[i]-cooldown;
      }
    }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= NUM_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }

    // Step 3.  Randomly ignite new 'sparks' near the bottom
    if( random(255) < Sparking ) {
      int y = random(7);
      heat[y] = heat[y] + random(160,255);
      //heat[y] = random(160,255);
    }

    // Step 4.  Convert heat to LED colors
    for( int j = 0; j < NUM_LEDS; j++) {
      setPixelHeatColor(j, heat[j] );
    }

    showStrip();
  }

}

void LEDStaticEffects::setPixelHeatColor (int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);

  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252

  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixel(Pixel, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixel(Pixel, 255, heatramp, 0);
  } else {                               // coolest
    setPixel(Pixel, heatramp, 0, 0);
  }
}

void LEDStaticEffects::HalloweenEyes(byte red, byte green, byte blue, int EyeWidth, int EyeSpace, bool Fade, int Steps, int FadeDelay, int EndPause)
{
  unsigned long now = m_timer->getTime();
  int j,order;
  if(start_sequence)
  {
    j = Steps;
    order = 0;
    start_sequence = false;
  }
  else
  {
    j = m_effect_state[0];
    order = m_effect_state[1];
  }

  randomSeed(analogRead(0));

  int StartPoint  = random( 0, NUM_LEDS - (2*EyeWidth) - EyeSpace );
  int Start2ndEye = StartPoint + EyeWidth + EyeSpace;

  if ( ((now - m_last_iteration) > EndPause) && (order == 1)  )
  {
    for(int i = 0; i < EyeWidth; i++)
    {
      setPixel(StartPoint + i, red, green, blue);
      setPixel(Start2ndEye + i, red, green, blue);
    }
  }

  if( ((now - m_last_iteration) > FadeDelay) &&  (order == 0) )
  {
    m_last_iteration = now;

    showStrip();

    if(Fade==true)
    {
      float r, g, b;

      r = j*(red/Steps);
      g = j*(green/Steps);
      b = j*(blue/Steps);

      for(int i = 0; i < EyeWidth; i++)
      {
        setPixel(StartPoint + i, r, g, b);
        setPixel(Start2ndEye + i, r, g, b);
      }

      showStrip();

      if(j-- == 0)
      {
        j = Steps;
        order = 1;
      }
    }
  }

  else if ( ((now - m_last_iteration) > EndPause) && (order == 1)  )
  {
    setAll(0,0,0); // Set all black
    order = 0;
  }

  m_effect_state[0] = j;
  m_effect_state[1] = order;
}

void LEDStaticEffects::CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay)
{

  unsigned long now = m_timer->getTime();
  int i,order;
  if(start_sequence)
  {
    i = 0;
    order = 0;
    start_sequence = false;
  }
  else
  {
    i = m_effect_state[0];
    order = m_effect_state[1];
  }

  if ( ((now - m_last_iteration) > SpeedDelay) && ((order == 0) || (order == 1))  )
  {
    m_last_iteration = now;
    if( order == 0 )
    {
      if(i < NUM_LEDS-EyeSize-2)
      {
        setAll(0,0,0);
        setPixel(i, red/10, green/10, blue/10);
        for(int j = 1; j <= EyeSize; j++)
        {
          setPixel(i+j, red, green, blue);
        }
        setPixel(i+EyeSize+1, red/10, green/10, blue/10);
        showStrip();
        i++;
      }
      else
      {
        order = 2;
      }
    }
    else if( order == 1 )
    {
      if(i > 0)
      {
        setAll(0,0,0);
      setPixel(i, red/10, green/10, blue/10);
      for(int j = 1; j <= EyeSize; j++)
      {
        setPixel(i+j, red, green, blue);
      }
      setPixel(i+EyeSize+1, red/10, green/10, blue/10);
      showStrip();
      i--;
      }
      else
      {
        order = 3;
      }
    }
  }
  else if ( ((now - m_last_iteration) > ReturnDelay) && ((order == 2) || (order == 3))  )
  {
    if(order == 2) order = 1;
    else if(order == 3) order = 0;
  }

  m_effect_state[0] = i;
  m_effect_state[1] = order;

}

void LEDStaticEffects::Twinkle(byte red, byte green, byte blue, int Count, int SpeedDelay, bool OnlyOne)
{

  unsigned long now = m_timer->getTime();
  int i;
  if(start_sequence)
  {
    i = 0;
    start_sequence = false;
  }
  else
  {
    i = m_effect_state[0];
  }

  if ( ((now - m_last_iteration) > SpeedDelay))
  {
    m_last_iteration = now;
    setAll(0,0,0);

    if(i < Count)
    {
      setPixel(random(NUM_LEDS),red,green,blue);
      showStrip();
      i++;
    }
    else
    {
      i = 0;
    }
  }

  m_effect_state[0] = i;

}

void LEDStaticEffects::TwinkleRandom(int Count, int SpeedDelay, bool OnlyOne)
{
  unsigned long now = m_timer->getTime();
  int i;
  if(start_sequence)
  {
    i = 0;
    start_sequence = false;
  }
  else
  {
    i = m_effect_state[0];
  }

  if ( ((now - m_last_iteration) > SpeedDelay))
  {
    m_last_iteration = now;
    setAll(0,0,0);

    if(i < Count)
    {
      setPixel(random(NUM_LEDS),random(0,255),random(0,255),random(0,255));
      showStrip();
      i++;
    }
    else
    {
      i = 0;
    }
  }
  m_effect_state[0] = i;
}

void LEDStaticEffects::Sparkle(byte red, byte green, byte blue, int SpeedDelay)
{
  unsigned long now = m_timer->getTime();
  int Pixel = random(NUM_LEDS);
  int order;
  if(start_sequence)
  {
    order = 0;
    start_sequence = false;
  }
  else
  {
    order = m_effect_state[0];
  }

  if(order == 0)
  {
    setPixel(Pixel,red,green,blue);
    showStrip();

    m_last_iteration = now;
    order = 1;
  }
  else if ( ((now - m_last_iteration) > SpeedDelay) && (order == 1) )
  {
    setPixel(Pixel,0,0,0);

    m_last_iteration = now;
    order = 0;
  }
  
  m_effect_state[0] = order;  
}

void LEDStaticEffects::SnowSparkle(byte red, byte green, byte blue, int SparkleDelay, int SpeedDelay)
{

  unsigned long now = m_timer->getTime();
  int order;
  int Pixel = random(NUM_LEDS);
  
  if(start_sequence)
  {
    order = 0;
    start_sequence = false;
  }
  else
  {
    order = m_effect_state[0];
  }

  if (order == 0)
  {
    setAll(red,green,blue);
    setPixel(Pixel,0xff,0xff,0xff);
    showStrip();

    order = 1;
  }
  else if ( ((now - m_last_iteration) > SparkleDelay) && (order == 1) )
  {
    m_last_iteration = now;
    order = 2;
    setPixel(Pixel,red,green,blue);
    showStrip();
  }
  else if ( ((now - m_last_iteration) > SpeedDelay) && (order == 2) )
  {
    order = 0;
  }

  m_effect_state[0] = order;
}

void LEDStaticEffects::RunningLights(byte red, byte green, byte blue, int WaveDelay)
{
  unsigned long now = m_timer->getTime();
  int position;
  if(start_sequence)
  {
    position = 0;
    start_sequence = false;
  }
  else
  {
    position = m_effect_state[0];
  }

  if ( ((now - m_last_iteration) > WaveDelay) )
  {
    m_last_iteration = now;

    if(position < NUM_LEDS*2)
    {
      position++; // = 0; //Position + Rate;
      for(int i=0; i<NUM_LEDS; i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixel(i,level,0,0);
        //float level = sin(i+Position) * 127 + 128;
        setPixel(i,((sin(i+position) * 127 + 128)/255)*red,
                   ((sin(i+position) * 127 + 128)/255)*green,
                   ((sin(i+position) * 127 + 128)/255)*blue);
      }

      showStrip();
    }
    else
    {
      position = 0;
    }
  }
  m_effect_state[0] = position;
}

void LEDStaticEffects::colorWipe(byte red, byte green, byte blue, int SpeedDelay)
{
  unsigned long now = m_timer->getTime();
  int i;
  if(start_sequence)
  {
    i = 0;
    start_sequence = false;
  }
  else
  {
    i = m_effect_state[0];
  }

  if ( ((now - m_last_iteration) > SpeedDelay) )
  {
    m_last_iteration = now;

    if(i < NUM_LEDS)
    {
      i++;
      setPixel(i, red, green, blue);
      showStrip();
    }
    else
    {
      i = 0;
    }
  }
  m_effect_state[0] = i;
}

void LEDStaticEffects::rainbowCycle(int SpeedDelay)
{
  unsigned long now = m_timer->getTime();
  byte *c;
  int j;
  if(start_sequence)
  {
    j = 0;
    start_sequence = false;
  }
  else
  {
    j = m_effect_state[0];
  }

  if ( ((now - m_last_iteration) > SpeedDelay) )
  {
    m_last_iteration = now;

    if(j < 256*5)
    {
      j++;
      for(int i=0; i< NUM_LEDS; i++) {
        c=Wheel(((i * 256 / NUM_LEDS) + j) & 255);
        setPixel(i, *c, *(c+1), *(c+2));
      }
      showStrip();
    }
    else
    {
      j = 0;
    }
  }
  m_effect_state[0] = j;
}

// used by rainbowCycle and theaterChaseRainbow
byte * LEDStaticEffects::Wheel(byte WheelPos)
{
  static byte c[3];

  if(WheelPos < 85)
  {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  }
  else if(WheelPos < 170)
  {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  }
  else
  {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }

  return c;
}

void LEDStaticEffects::theaterChase(byte red, byte green, byte blue, int SpeedDelay)
{

  unsigned long now = m_timer->getTime();
  int q,order;
  if(start_sequence)
  {
    q = 0;
    order = 0;
    start_sequence = false;
  }
  else
  {
    q = m_effect_state[0];
    order = m_effect_state[1];
  }

  if (q < 3)
  {
    q++;

    if(order == 0)
    {
      m_last_iteration = now;
      order = 1;

      for (int i=0; i < NUM_LEDS; i=i+3)
      {
        setPixel(i+q, red, green, blue);    //turn every third pixel on
      }
      showStrip();
    }
    else if ( ((now - m_last_iteration) > SpeedDelay) && (order == 1) )
    {
      m_last_iteration = now;
      order = 0;

      for (int i=0; i < NUM_LEDS; i=i+3)
      {
        setPixel(i+q, 0,0,0);        //turn every third pixel off
      }
    }
  }
  else
  {
    q = 0;
  }

  m_effect_state[0] = q;
  m_effect_state[1] = order;
}

void LEDStaticEffects::theaterChaseRainbow(int SpeedDelay)
{
  byte *c;
  unsigned long now = m_timer->getTime();
  int j,q,order;
  if(start_sequence)
  {
    j = 0;
    q = 0;
    order = 0;
    start_sequence = false;
  }
  else
  {
    j = m_effect_state[0];
    q = m_effect_state[1];
    order = m_effect_state[2];
  }

  if(j < 255)
  {
    j++;

    if (q < 3)
    {
      q++;

      if(order == 0)
      {
        m_last_iteration = now;
        order = 1;

        for (int i=0; i < NUM_LEDS; i=i+3) {
          c = Wheel( (i+j) % 255);
          setPixel(i+q, *c, *(c+1), *(c+2));    //turn every third pixel on
        }
        showStrip();
      }
      else if ( ((now - m_last_iteration) > SpeedDelay) && (order == 1) )
      {
        m_last_iteration = now;
        order = 0;

        for (int i=0; i < NUM_LEDS; i=i+3)
        {
          setPixel(i+q, 0,0,0);        //turn every third pixel off
        }
      }
    }
    else
    {
      q = 0;
    }
  }
  else
  {
    j = 0;
  }

  m_effect_state[0] = j;
  m_effect_state[1] = q;
  m_effect_state[2] = order;
}

bouncing_mem_struct* LEDStaticEffects::allocate_bouncing_mem(int BallCount)
{
  if(m_bouncing_mem.ImpactVelocity == NULL &&
     m_bouncing_mem.TimeSinceLastBounce == NULL &&
     m_bouncing_mem.Position == NULL &&
     m_bouncing_mem.ClockTimeSinceLastBounce == NULL &&
     m_bouncing_mem.Dampening == NULL &&
     m_bouncing_mem.ballBouncing == NULL &&
     m_bouncing_mem.Height == NULL
  )
  {
    m_bouncing_mem.ImpactVelocity = new float[BallCount];
    m_bouncing_mem.TimeSinceLastBounce = new float[BallCount];
    m_bouncing_mem.Position = new int[BallCount];
    m_bouncing_mem.ClockTimeSinceLastBounce = new long[BallCount];
    m_bouncing_mem.Dampening = new float[BallCount];
    m_bouncing_mem.ballBouncing = new bool[BallCount];
    m_bouncing_mem.Height = new float[BallCount];
  }

  return &m_bouncing_mem;
}

void LEDStaticEffects::deallocate_bouncing_mem()
{
  if(m_bouncing_mem.ImpactVelocity != NULL &&
     m_bouncing_mem.TimeSinceLastBounce != NULL &&
     m_bouncing_mem.Position != NULL &&
     m_bouncing_mem.ClockTimeSinceLastBounce != NULL &&
     m_bouncing_mem.Dampening != NULL &&
     m_bouncing_mem.ballBouncing != NULL &&
     m_bouncing_mem.Height != NULL
  )
  {
    delete [] m_bouncing_mem.ImpactVelocity;
    delete [] m_bouncing_mem.TimeSinceLastBounce;
    delete [] m_bouncing_mem.Position;
    delete [] m_bouncing_mem.ClockTimeSinceLastBounce;
    delete [] m_bouncing_mem.Dampening;
    delete [] m_bouncing_mem.ballBouncing;
    delete [] m_bouncing_mem.Height;
  }
  m_bouncing_mem.ImpactVelocity = NULL;
  m_bouncing_mem.TimeSinceLastBounce = NULL;
  m_bouncing_mem.Position = NULL;
  m_bouncing_mem.ClockTimeSinceLastBounce = NULL;
  m_bouncing_mem.Dampening = NULL;
  m_bouncing_mem.ballBouncing = NULL;
  m_bouncing_mem.Height = NULL;
}

bouncing_mem_struct* LEDStaticEffects::get_bouncing_mem()
{
  return &m_bouncing_mem;
}

void LEDStaticEffects::BouncingColoredBalls(int BallCount, byte colors[][3], bool continuous, const uint8_t background)
{
  float Gravity = -9.81;
  int StartHeight = 1;
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );

  int ballsStillBouncing;
  bouncing_mem_struct* bouncing_mem;
  if(start_sequence)
  {
    //Serial.println(BallCount);
    ballsStillBouncing = 1;
    /* Allocate memory */
    bouncing_mem = allocate_bouncing_mem(BallCount);
    start_sequence = false;
    /* Initialize memory */
    for (int i = 0 ; i < BallCount ; i++)
    {
      bouncing_mem->ClockTimeSinceLastBounce[i] = millis();
      bouncing_mem->Height[i] = StartHeight;
      //Serial.println(StartHeight);
      //Serial.println(bouncing_mem->Height[i]);
      bouncing_mem->Position[i] = 0;
      bouncing_mem->ImpactVelocity[i] = ImpactVelocityStart;
      bouncing_mem->TimeSinceLastBounce[i] = 0;
      bouncing_mem->Dampening[i] = 0.90 - float(i)/pow(BallCount,2);
      bouncing_mem->ballBouncing[i]=true;
    }
  }
  else
  {
    ballsStillBouncing = m_effect_state[0];
    bouncing_mem = get_bouncing_mem();
  }

  if(ballsStillBouncing == 0)
  {
    /* Initialize memory */
    for (int i = 0 ; i < BallCount ; i++)
    {
      bouncing_mem->ClockTimeSinceLastBounce[i] = millis();
      bouncing_mem->Height[i] = StartHeight;
      bouncing_mem->Position[i] = 0;
      bouncing_mem->ImpactVelocity[i] = ImpactVelocityStart;
      bouncing_mem->TimeSinceLastBounce[i] = 0;
      bouncing_mem->Dampening[i] = 0.90 - float(i)/pow(BallCount,2);
      bouncing_mem->ballBouncing[i]=true;
    }
    ballsStillBouncing = 1;
  }

  for (int i = 0 ; i < BallCount ; i++) {
    bouncing_mem->TimeSinceLastBounce[i] =  millis() - bouncing_mem->ClockTimeSinceLastBounce[i];
    bouncing_mem->Height[i] = 0.5 * Gravity * pow( bouncing_mem->TimeSinceLastBounce[i]/1000 , 2.0 ) + bouncing_mem->ImpactVelocity[i] * bouncing_mem->TimeSinceLastBounce[i]/1000;
    
    if ( bouncing_mem->Height[i] < 0 ) {
      bouncing_mem->Height[i] = 0;
      bouncing_mem->ImpactVelocity[i] = bouncing_mem->Dampening[i] * bouncing_mem->ImpactVelocity[i];
      bouncing_mem->ClockTimeSinceLastBounce[i] = millis();

      if ( bouncing_mem->ImpactVelocity[i] < 0.01 ) {
        if (continuous) {
          bouncing_mem->ImpactVelocity[i] = ImpactVelocityStart;
        } else {
          bouncing_mem->ballBouncing[i]=false;
        }
      }
    }
    bouncing_mem->Position[i] = round( bouncing_mem->Height[i] * (NUM_LEDS - 1) / StartHeight);
  }

  ballsStillBouncing = 0; // assume no balls bouncing
  for (int i = 0 ; i < BallCount ; i++) {
    setPixel(bouncing_mem->Position[i],colors[i][0],colors[i][1],colors[i][2]);
    if ( bouncing_mem->ballBouncing[i] ) {
      ballsStillBouncing = 1;
    }
  }

  showStrip();
  setAll(background,background,background);

  m_effect_state[0] = ballsStillBouncing;
}

void LEDStaticEffects::meteorRain(byte red, byte green, byte blue, byte meteorSize, byte meteorTrailDecay, bool meteorRandomDecay, int SpeedDelay, bool dir)
{

  unsigned long now = m_timer->getTime();
  int i;
  if(start_sequence)
  {
    if(dir) i = 0;
    else i = NUM_LEDS;
    start_sequence = false;
  }
  else
  {
    i = m_effect_state[0];
  }

  if(dir)
  {
    if ( ((now - m_last_iteration) > SpeedDelay) )
    {
      m_last_iteration = now;

      if( i < NUM_LEDS + NUM_LEDS/3 )
      {
        i++;
        // fade brightness all LEDs one step
        for(int j=0; j<NUM_LEDS; j++) {
          if( (!meteorRandomDecay) || (random(10)>5) ) {
            fadeToBlack(j, meteorTrailDecay );
          }
        }

        // draw meteor
        for(int j = 0; j < meteorSize; j++) {
          if( ( i-j <NUM_LEDS) && (i-j>=0) ) {
            setPixel(i-j, red, green, blue);
          }
        }
        showStrip();
      }
    }
    else
    {
      i = 0;
    }
  }
  else
  {
    if ( ((now - m_last_iteration) > SpeedDelay) )
    {
      m_last_iteration = now;

      if( i > -NUM_LEDS/3 )
      {
        i--;
        // fade brightness all LEDs one step
        for(int j=0; j<NUM_LEDS; j++) {
          if( (!meteorRandomDecay) || (random(10)>5) ) {
            fadeToBlack(j, meteorTrailDecay );
          }
        }

        // draw meteor
        for(int j = 0; j < meteorSize; j++) {
          if( ( i-j <NUM_LEDS) && (i-j>=0) ) {
            setPixel(i-j, red, green, blue);
          }
        }
        showStrip();
      }
    }
    else
    {
      i = NUM_LEDS;
    }
  }
  m_effect_state[0] = i;
}

// used by meteorrain
void LEDStaticEffects::fadeToBlack(int ledNo, byte fadeValue) {
 #ifdef ADAFRUIT_NEOPIXEL_H
    // NeoPixel
    uint32_t oldColor;
    uint8_t r, g, b;
    int value;

    oldColor = strip.getPixelColor(ledNo);
    r = (oldColor & 0x00ff0000UL) >> 16;
    g = (oldColor & 0x0000ff00UL) >> 8;
    b = (oldColor & 0x000000ffUL);

    r=(r<=10)? 0 : (int) r-(r*fadeValue/256);
    g=(g<=10)? 0 : (int) g-(g*fadeValue/256);
    b=(b<=10)? 0 : (int) b-(b*fadeValue/256);

    strip.setPixelColor(ledNo, r,g,b);
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   leds[ledNo].fadeToBlackBy( fadeValue );
 #endif
}

// Apply LED color changes
void LEDStaticEffects::showStrip()
{
#ifdef ADAFRUIT_NEOPIXEL_H
   // NeoPixel
   strip.show();
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
#endif
}

// Set a LED color (not yet visible)
void LEDStaticEffects::setPixel(int Pixel, byte red, byte green, byte blue)
{
  if(Pixel >= NUM_LEDS)
  {
    Serial.println("ERROR: Led memory size exceeded");
    Serial.print("Idx ");
    Serial.println(Pixel);
    Pixel = NUM_LEDS - 1;
  }


 #ifdef ADAFRUIT_NEOPIXEL_H
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

// Set all LEDs to a given color and apply it (visible)
void LEDStaticEffects::setAll(byte red, byte green, byte blue)
{
  for(int i = 0; i < NUM_LEDS; i++ )
  {
    setPixel(i, red, green, blue);
  }
  showStrip();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
