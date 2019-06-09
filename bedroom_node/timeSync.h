#if !defined TIMESYNC_H
#define TIMESYNC_H

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include "config.h"

class timeSync
{

private:

  unsigned long m_delay_shift;

public: 

  timeSync();

  unsigned long getTime() const;
  void setDelay( unsigned long delay_shift );
};

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
