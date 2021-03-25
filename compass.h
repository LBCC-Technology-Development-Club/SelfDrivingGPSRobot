//JWL: Define the new compass library and make sure the name has not been used elsewhere in the code.
#ifndef COMPASS_H
#define COMPASS_H

//JWL: I2C Communciation Added To New Library
#include <Wire.h> 
//JWL: Original Compass Library: https://github.com/sleemanj/HMC5883L_Simple (I believe this is it.) / Added to GitHub
#include <HMC5883L_Simple.h> 

namespace compass
{
  	HMC5883L_Simple Compass;

	//JWL: Each Compass.something comes directly from the HMC5883L_Simple.cpp (the original library).
	void setupCompass()
	{
		Wire.begin();
		/*Setting Declination
		Found LBCC http://www.magnetic-declination.com/
		Edited by Jonathan Landers
		*/
		Compass.SetDeclination(14, 55, 'E'); 
		//JWL: The below information depends on how the compass is oriented on the Robot
		Compass.SetSamplingMode(COMPASS_CONTINUOUS); //COMPASS_CONTINUOUS COMPASS_SINGLE
		Compass.SetScale(COMPASS_SCALE_130); //SensitivityCOMPASS_HORIZONTAL_Y_NORT
		Compass.SetOrientation(H); //this depends on how its mounted on the robot
		Serial1.println("Compass initialized.");
	}

	float getHeading()
	{
		//@brief: returns the current heading and substracts 180 Deg because the compass is mounted with zero pointing to the back
		//JWL: May need to change depending on the compasses orientation within the robot.
		return Compass.GetHeadingDegrees()-180.0;
	}

}


#endif
