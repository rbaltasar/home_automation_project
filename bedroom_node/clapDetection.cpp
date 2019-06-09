
#include "clapDetection.h"

clapDetection::clapDetection(FrequencyUtilities* FreqUtilities):
m_FreqUtilities(FreqUtilities),
m_detectionType(NONE)
{
  
}

clapDetectionType clapDetection::getDetectionType()
{
  return m_detectionType;
}

void clapDetection::resetDetectionType()
{
  m_detectionType = NONE;
}


bool clapDetection::feed()
{
  /* State machine to handle the clap detection asynchronously */
  unsigned long current_time = millis();
  //Time since last clap detected
  time_since_clap = current_time - last_clap;

  //Detection of a clap in the current iteration
  //This includes signal sampling, FFT and spectrum analysis
  bool clap = m_FreqUtilities->detect_single_clap();

  /* In case a clap is detected */
  if(clap)
  {
#if (DEBUG_TRACES_CLAP == 1)
    Serial.print("Time since last clap: ");
    Serial.println(time_since_clap);
#endif

    //Last clap happened long time ago, the detected clap is the first of the sequence
    if( (time_since_clap > 2000) )
    {
      //First clap detected!
      clap_number = 1;
#if (DEBUG_TRACES_CLAP == 1)
      Serial.println("First clap detected");
#endif
    }
    //A clap happened "recently"
    else
    {
      //A second clap has already been detected before
      if(clap_number == 2)
      {
        //If the timing is right, this is the second clap of the sequence
        if( (time_since_clap > 100) && (time_since_clap < 800) )
        {
          clap_number = 3;
#if (DEBUG_TRACES_CLAP == 1)
          Serial.println("Third clap detected");
#endif
        }
      }
      
      //A first clap has already been detected before
      else if(clap_number == 1)
      {
        //If the timing is right, this is the second clap of the sequence
        if( (time_since_clap > 100) && (time_since_clap < 800) )
        {
          clap_number = 2;
#if (DEBUG_TRACES_CLAP == 1)
          Serial.println("Second clap detected");
#endif
        }
      }
      
      //More than one clap have been detected before
      else if(clap_number > 2)
      {
        //If the timing is right, this is just another clap in a large sequence of claps
        if( (time_since_clap > 150) && (time_since_clap < 400) )
        {
          clap_number++;
#if (DEBUG_TRACES_CLAP == 1)
          Serial.println("Further claps detected");
#endif
        }
      }
    }
    //Save the timestamp of the clap
    last_clap = millis();
  }

  /* No clap detected */
  else
  {
    //If we already counted two claps and no further claps are detected in one second, sequence complete
    if( (clap_number == 2) && (time_since_clap > 1500) )
    {
//#if (DEBUG_TRACES_CLAP == 1)
      Serial.println("ON/OFF order detected!!");
//#endif
      clap_number = 0;
      m_detectionType = DOUBLE_CLAP;
      //Force to enter the node_specific_loop in the next loop() iteration for fastest responsibity
      return true;
    }
    //If we already counted two claps and no further claps are detected in one second, sequence complete
    else if( (clap_number == 3) && (time_since_clap > 1500) )
    {
#if (DEBUG_TRACES_CLAP == 1)
      Serial.println("Color change order detected!!");
#endif
      clap_number = 0;
      m_detectionType = TRIPLE_CLAP;
      //Force to enter the node_specific_loop in the next loop() iteration for fastest responsibity
      return true;
    }
  }

  return false;
}
