
#include "timeSync.h"


timeSync::timeSync():
m_delay_shift(0)
{
  
}

unsigned long timeSync::getTime() const
{
  return millis() + m_delay_shift;
}

void timeSync::setDelay(unsigned long delay_shift)
{
  m_delay_shift = delay_shift;
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
