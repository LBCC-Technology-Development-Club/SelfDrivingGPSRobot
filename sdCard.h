#ifndef SD_CARD_H
#define SD_CARD_H

#include <SPI.h>
#include <SD.h>

namespace sdCard
{
  File myFile;
  const int pinCS = 53; 
  
  void setupSdCard()
  {
    if(SD.begin(pinCS)){
      Serial1.println("SD initialisiert.");
    }
    else{
      Serial1.println("SD Fail: Karte eingelegt?");
    }
    myFile = SD.open("gps.txt", FILE_WRITE);
  }

}
#endif
