#ifndef COMPASS_H
#define COMPASS_H

#include <Wire.h>
#include <HMC5883L_Simple.h>

namespace compass
{
  HMC5883L_Simple Compass;

void setupCompass()
{
	Wire.begin();
	Compass.SetDeclination(2, 31, 'E');  // http://www.magnetic-declination.com/
	Compass.SetSamplingMode(COMPASS_CONTINUOUS); //COMPASS_CONTINUOUS COMPASS_SINGLE
	Compass.SetScale(COMPASS_SCALE_130); //Sensitivity
	Compass.SetOrientation(COMPASS_HORIZONTAL_Y_NORTH); //this depends on how its mounted on the robot
	Serial1.println("Compass initialized.");
}

float getHeading()
{//@brief: returns the current heading and substracts 180 Deg because the compass is mounted with zero pointing to the back
	 return Compass.GetHeadingDegrees()-180.0;
}

}


#endif
