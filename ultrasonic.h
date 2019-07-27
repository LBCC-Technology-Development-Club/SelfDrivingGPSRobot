#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <NewPing.h>

namespace ultrasonic
{
  NewPing us1(24,24,50); //1 meter maximal distance
  int distInt = 0.0;


  void keepDistance(int distSet, float &velLSetPoint, float &velRSetPoint)
  {//@brief: calculates velocity setpoint in such a way that the distance to an obstacle is constant
    int valUs1 = us1.ping_cm(); //read ultrasonic sensor
    if(valUs1!=0) 
    {
      int deltaDist = distSet - valUs1;
      int uDist = 30 * deltaDist + 1 * distInt;
      if(uDist<=1000 && uDist>=-1000 || uDist>1000 && deltaDist<0 || uDist<-1000 && deltaDist>0)
      { distInt += deltaDist; }
      if(deltaDist == 0){distInt = 0;}   
      velLSetPoint = velRSetPoint = -0.001 * uDist;
    }
  }

}


#endif
