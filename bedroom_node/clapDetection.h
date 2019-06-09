#if !defined CLAPDETECTION_H
#define CLAPDETECTION_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "config.h"
#include "frequency_utilities.h"

enum clapDetectionType
{
  NONE = 0,
  SINGLE_CLAP = 1, //Not used
  DOUBLE_CLAP = 2,
  TRIPLE_CLAP = 3
};


class clapDetection
{

private:
  
  /* Clap detection variables */
  unsigned long last_clap = 0;
  uint8_t clap_number = 0;
  unsigned long time_since_clap = 0;

  FrequencyUtilities* m_FreqUtilities;

  clapDetectionType m_detectionType;
  
public:

  clapDetection(FrequencyUtilities* FreqUtilities);

  bool feed();
  clapDetectionType getDetectionType();
  void resetDetectionType();
  
};

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
