#ifndef MOTOR_H
#define MOTOR_H

#include <Encoder.h>
#include "constants.h"

namespace motor
{
  Encoder encL(2, 4);
  Encoder encR(3, 5);
  
  const int MR_DIRPIN1 = 8; //turning dir
  const int MR_DIRPIN2 = 7; //turning dir
  const int MR_MOTPIN = 6;  //PWM 
  
  const int ML_DIRPIN1 = 9;  //turning dir
  const int ML_DIRPIN2 = 10; //turning dir
  const int ML_MOTPIN = 11;  //PWM
  
  float posROld = 0.0;
  float posLOld = 0.0;
  float velROld = 0.0;
  float velLOld = 0.0;
  float velRInt = 0.0;
  float velLInt = 0.0;
  
  void setupMotor()
  {
    pinMode(MR_DIRPIN1, OUTPUT);    
    pinMode(MR_DIRPIN2, OUTPUT);    
    pinMode(MR_MOTPIN, OUTPUT); 
    pinMode(ML_DIRPIN1, OUTPUT);    
    pinMode(ML_DIRPIN2, OUTPUT);    
    pinMode(ML_MOTPIN, OUTPUT);     
  }
    
  void motorR(int u){
    //@brief: set pwm registers for right motor
    if(u>=0){
      if(u>255){u=255;}      
      digitalWrite(MR_DIRPIN1,LOW);  
      digitalWrite(MR_DIRPIN2,HIGH);
      analogWrite(MR_MOTPIN,u);
    }
    else{
        if(u<-255){u=-255;}   
        digitalWrite(MR_DIRPIN2,LOW);
        digitalWrite(MR_DIRPIN1,HIGH);
        analogWrite(MR_MOTPIN,-u);     
    }
    
  }
  
  void motorL(int u){
    //@brief: set pwm registers for left motor
    if(u>=0){
      if(u>255){u=255;}    
      digitalWrite(ML_DIRPIN1,HIGH);  
      digitalWrite(ML_DIRPIN2,LOW);
      analogWrite(ML_MOTPIN,u);
    }
    else{
        if(u<-255){u=-255;} 
        digitalWrite(ML_DIRPIN2,HIGH);
        digitalWrite(ML_DIRPIN1,LOW);
        analogWrite(ML_MOTPIN,-u);     
    }
    
  }
  
  void drive(float powerSetPoint, float angleSetPoint )
  { //@brief: function needs power SP [0..100] and angle SP [0..359], 
    //reads the encoder and does the PI Control for the motor
    if(powerSetPoint>100)powerSetPoint = 100; if(powerSetPoint<0)powerSetPoint = 0;
    if(angleSetPoint>359)angleSetPoint = 359; if(angleSetPoint<0)angleSetPoint = 0;
    float velSetR, velSetL;
    
    /* calc setpoint for right and left motor depending on the sector of the anglesetpoint */
    if(angleSetPoint<90)
    {
      velSetL = 0.01*powerSetPoint;
      velSetR = 0.01*powerSetPoint * sin(toRad*angleSetPoint);
    }
    else
    if(angleSetPoint<180)
    {
      velSetL = 0.01*powerSetPoint * sin(toRad*angleSetPoint);
      velSetR = 0.01*powerSetPoint;
    }
    else
    if(angleSetPoint<270)
    {
      velSetL = 0.01*powerSetPoint * sin(toRad*angleSetPoint);
      velSetR = -0.01*powerSetPoint;
    }
    else
    {
      velSetL = -0.01*powerSetPoint;
      velSetR = 0.01*powerSetPoint * sin(toRad*angleSetPoint);
    }

    /* here we have setpoints for the indivisual wheels in the range [0..1] and scale it up to vmax */  
    velSetR = velMax * velSetR;
    velSetL = velMax * velSetL;
  
    if(velSetR<0.03 && velSetR>-0.03)
    {
      velSetR = 0.0;
    }
    if(velSetL<0.03 && velSetL>-0.03)
    {
      velSetL = 0.0;
    }
  
    /* read encoder values in m */
    float posR = convFact * encR.read();
    float posL = convFact * encL.read();
    
    /* finit differences and lowpass filter for right motor in m/s */
    float velR = (posR - posROld) * loopFrequency; 
    posROld = posR;
    velR = 0.9*velR + 0.1*velROld; 
    velROld = velR;
    
    /* finit differences and lowpass filter for left motor in m/s */
    float velL = (posL - posLOld) * loopFrequency;
    posLOld = posL;
    velL = 0.9*velL + 0.1*velLOld; 
    velLOld = velL;
    
    /* velocitycontroller right motor */
    float deltaVelR = velSetR - velR;
    int uR = static_cast<int>( 900.0 * deltaVelR + 25 * velRInt);
    if(uR<=255 && uR>=-255 || uR>255 && deltaVelR<0 || uR<-255 && deltaVelR>0)
    { velRInt += deltaVelR; }
    if(deltaVelR == 0.0){velRInt = 0;} 
    motorR(uR); 
    
    /* velocitycontroller left motor */
    float deltaVelL = velSetL - velL;
    int uL = static_cast<int>( 900.0 * deltaVelL  + 25 * velLInt);
    if(uL<=255 && uL>=-255 || uL>255 && deltaVelL<0 || uL<-255 && deltaVelL>0)
    { velLInt += deltaVelL; } 
    if(deltaVelL == 0.0){velLInt = 0;}             
    motorL(uL); 
    
  }

}


#endif
