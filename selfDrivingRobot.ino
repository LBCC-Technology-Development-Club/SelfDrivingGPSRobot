#include "gps.h"
#include "motor.h"
#include "sdCard.h"
#include "ultrasonic.h"
#include "compass.h"

/* Loop timing */
long timeOld = 0;
long dt = 0;

/* Bluetooth */
char blContent[10];
int blIndex = 0;
int angleSetPoint = 0;
int powerSetPoint = 0;
int storeLocation = 0;
int returnToHome = 0;
int navigateToWaypoint = 0;

/* Gps */
gps::geoLocFloat myPos;           //struct to store current geolocation in float
gps::geoLocFloat goalPos;         //struct to store goal geolocation in float
const int numTrajPoints = 10;     //max number of positions that can be stored in the goal array
const float maxGoalDist = 0.5;    //distance to goal that is needed before setpoint switch or return to home finished
long posLat[numTrajPoints] = {0}; //storage array for latitude
long posLon[numTrajPoints] = {0}; //storage array for longitude
int storageIndex = 0;             //this index is used for storing the geolocations
int averagingIndex = 0;           //this index in used to avarage goalocations before storing
int setPointIndex = 0;            //this index is used to release new setpoints
bool firstTimeWaypointNav = true;
bool homePosUpdated = false;
float distanceToGoal = 0.0;
float bearing = 0.0;
float bearingOld = 0.0; //Jump detection
float heading = 0.0;
float headingOld = 0.0; //Jump detection
int overflowCounter = 0;
float bearingSP = 0;
float deltaHeading = 0.0;
float deltaHeadingInt = 0.0; 

///////////////////////////////////////////////////////////////////

void setup() {
  delay(5000); //erstmal warten, damit Bluetooth verbinden kann
  
  Serial1.begin(9600);  //bluetooth
  Serial2.begin(9600);  //gps
  
  motor::setupMotor();

  gps::setupGPS();

  compass::setupCompass();

  //sdCard::setupSdCard(); 
 
}

///////////////////////////////////////////////////////////////////

void loop() {
  
  /* Fix Looptime */
  dt = micros() - timeOld;
  while(dt<loopTimeMicros){dt = micros() - timeOld;}
  timeOld = micros();

  getBlCommand();   //read bluetooth commands
  
  if(gps::processGPS())   //new Gps data available (10Hz update rate)
  {  
    if(storeLocation == 1)
    {
      storeCurrentLocation();
    }

    myPos.lat = goalPos.lat = gps::gpsData.lat*1e-7;    //set myPos and goalPos equal
    myPos.lon = goalPos.lon = gps::gpsData.lon*1e-7;

    homePosUpdated = false;
    if( returnToHome == 1       //return to home activated
        && storeLocation == 2   //the lat, lon arrays contain data
        && navigateToWaypoint == 0 )
    {   
        homePosUpdated = true;
        goalPos.lat = posLat[0]*1e-7;
        goalPos.lon = posLon[0]*1e-7;
    }  
    
    if(navigateToWaypoint == 1  //waypoint navigation activated
       && storeLocation == 2    //the lat, lon arrays contain data
       && returnToHome == 0 )
    {   
        if(distanceToGoal < maxGoalDist && firstTimeWaypointNav == false)  //switch goal when closer than 1m
        {
          Serial1.print("Goalpos["); Serial1.print(setPointIndex); Serial1.println("] reached. Switching to next.");
          setPointIndex ++;
          
          int endSetPointIndex = storageIndex;
          if(storageIndex == 0){endSetPointIndex = numTrajPoints; }
          if(setPointIndex == endSetPointIndex) //begin again when storageIndex is reached.
          {
            setPointIndex = 0;
          }
        }
        firstTimeWaypointNav = false;
        goalPos.lat = posLat[setPointIndex]*1e-7;    //use current setpoint as goal
        goalPos.lon = posLon[setPointIndex]*1e-7;
        
    }   

    distanceToGoal = gps::getDistance(myPos, goalPos);
    bearing = gps::getBearing(myPos, goalPos);   
  }    
  
  if(returnToHome == 1 || navigateToWaypoint == 1 )  //autonomous driving activated 
  {    
    powerSetPoint = 100;   
    if(distanceToGoal < maxGoalDist && returnToHome == 1 && homePosUpdated == true)
    {
      //home position reached
      powerSetPoint = 0; 
      returnToHome = 0; 
      Serial1.println("Homepos reached."); Serial1.println("Return to home deactivated.");
    }
    if(distanceToGoal < 1.5 * maxGoalDist && navigateToWaypoint == 1)
    {
      powerSetPoint = 50; //slow nearby the goal
    }
    
    /* PI controller for the angle setpoint */
    calcAngleSetPoint();
  }

  /* second cascade of direction controller and pwm generation */
  motor::drive(powerSetPoint,angleSetPoint); //needs values between [0..100] and [0..359]

}


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

void calcAngleSetPoint()
{ //@brief: this function uses heading and bearing to calculate 
  //the anglesetPoint with an PI-Controller. This is the outer cascade of the direction controller 

  headingOld = heading;
  heading = compass::getHeading();

  /* This is to handle the case when bearing and heading jump between -180 and 180 degree. 
     Its necessery othewise the controller won't work when driving in southern direction */
  if(headingOld < -90 && heading > 90){overflowCounter += 1;}  //jump from -180 to +180 
  if(headingOld > 90 && heading < -90){overflowCounter -= 1;}  //jump from +180 to -180
  if(bearingOld < -90 && bearing > 90){overflowCounter -= 1;}  //jump from -180 to +180 
  if(bearingOld > 90 && bearing < -90){overflowCounter += 1;}  //jump from +180 to -180 
  bearingSP = overflowCounter*360 + bearing;
  
  deltaHeading = bearingSP - heading ;
  if(deltaHeading < -89){deltaHeading = -89;} 
  if(deltaHeading > 89){deltaHeading = 89;}
  
  float uHeading = 1.0*deltaHeading + 0.05*deltaHeadingInt;
  deltaHeadingInt += deltaHeading;
  
  if(deltaHeading <= 0.05 && deltaHeading >= -0.05 || //anti-windup
     deltaHeading < 0.0 && deltaHeadingInt > 0.0 || 
     deltaHeading > 0.0 && deltaHeadingInt < 0.0)
  {deltaHeadingInt = 0.0;} 

  if(deltaHeadingInt>1000){ deltaHeadingInt = 1000; powerSetPoint = 50;}
  if(deltaHeadingInt<-1000){ deltaHeadingInt = -1000; powerSetPoint = 50;}
  
  if(uHeading<-89){uHeading = -89;} //only allow forward driving
  if(uHeading>89){uHeading = 89;}
  angleSetPoint = static_cast<int>(90.0 - uHeading + 0.5);   //1...179 and 90 means straight, 0.5 for rounding since always positive
}


void storeCurrentLocation(void)
{ //@brief: this function stores an average of the current 
  //location in the position array and increments the storage index
    posLat[storageIndex] += gps::gpsData.lat;  
    posLon[storageIndex] += gps::gpsData.lon;
    averagingIndex ++;
    if(averagingIndex == 4)   //careful, more than 4 won't fit in long
    {
      posLat[storageIndex] = posLat[storageIndex]/4; 
      posLon[storageIndex] = posLon[storageIndex]/4;
      storeLocation = 2;
      Serial1.print("Goalpos[");
      Serial1.print(storageIndex);
      Serial1.print("] set to: ");
      Serial1.print(posLat[storageIndex]*1e-7, 7);  Serial1.print(" ");
      Serial1.println(posLon[storageIndex]*1e-7,7);
      storageIndex ++;
      if(storageIndex == numTrajPoints)
      {
        storageIndex = 0;
        Serial1.println("End of setpoint array reached.");
        Serial1.println("Next one will overwrite home position.");
      }

    }
}


void getBlCommand(void)
{ //@brief: this function reads bluetooth commands from serial1
  while(Serial1.available()) {      
      blContent[blIndex] = Serial1.read();
      blIndex++;

    if(blContent[blIndex-1] == ' ')  //endcharacter reached
    {    
      if(blContent[0] == 'a')  //angleSetPoint
      { 
        angleSetPoint = atoi(&blContent[1]);  //start with second char and parse for an int
      }
      if(blContent[0] == 'p') //powerSetPoint
      {
        powerSetPoint = atoi(&blContent[1]);  //start with second char and parse for an int
      }      
      if(blContent[0] == 'g') //store trajectory point
      {
        storeLocation = 1;
        posLat[storageIndex] = 0;
        posLon[storageIndex] = 0; 
        averagingIndex = 0; 
      }  
      if(blContent[0] == 'r') //activate return to home
      {
        if(returnToHome == 0){ 
          returnToHome = 1; 
          Serial1.println("Return to home activated."); 
          } 
        else{
          returnToHome = 0; 
          powerSetPoint = 0;
          Serial1.println("Return to home deactivated."); 
        } 
      }
      if(blContent[0] == 's') //activate waypoint navigation
      {        
        if(navigateToWaypoint == 0){ 
          navigateToWaypoint = 1; 
          firstTimeWaypointNav = true;
          setPointIndex = 0; //always begin at home
          Serial1.println("Waypoint navigation activated.");  
        } 
        else{
          navigateToWaypoint = 0; 
          powerSetPoint = 0;
          Serial1.println("Waypoint navigation deactivated."); 
        } 
      }   
      if(blContent[0] == 'd') //print debug info
      { 
        if(blContent[1] == '0')
        {
          Serial1.print("dt: "); Serial1.println(dt);
          Serial1.print("overflowCounter: "); Serial1.println(overflowCounter);
          Serial1.print("deltaHeading: "); Serial1.println(deltaHeading);
          Serial1.print("distGoal: "); Serial1.println(distanceToGoal);
        }
        if(blContent[1] == '1')
        {
          Serial1.print("myPos: "); Serial1.print(myPos.lat, 7); Serial1.print(" "); Serial1.println(myPos.lon,7);
          Serial1.print("goalPos: "); Serial1.print(goalPos.lat, 7); Serial1.print(" "); Serial1.println(goalPos.lon,7);
        }
      
      }
      if(blContent[0] == 'q') //delete goal positions in array and reset
      { 
        navigateToWaypoint = 0; 
        returnToHome = 0;
        powerSetPoint = 0;
        setPointIndex = 0;
        storageIndex = 0;
        averagingIndex = 0;
        storeLocation = 0;
        for (int i = 0; i<numTrajPoints; i++)
        {
          posLat[i] = 0;
          posLon[i] = 0; 
        }
        Serial1.println("Goal array reset succesful.");
      }
      blIndex = 0;
    }
  }
  
}
