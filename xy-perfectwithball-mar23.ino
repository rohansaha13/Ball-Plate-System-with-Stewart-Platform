// Touch screen library with X Y and Z (pressure) readings as well as oversampling to avoid 'bouncing'
// This demo code returns raw readings, public domain

#include <stdint.h>
#include "TouchScreen.h"

/*#define YP A2  // must be an analog pin, use "An" notation!
#define XM A5  // must be an analog pin, use "An" notation!
#define YM A4  // can be a digital pin
#define XP A3// can be a digital pin*/

#define YP A2  // must be an analog pin, use "An" notation!
#define XP A3  // must be an analog pin, use "An" notation!
#define YM A4  // can be a digital pin
#define XM A5  // can be a digital pin*/

// For better pressure precision, we need to know the resistance between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate

TouchScreen ts = TouchScreen(XP, YP, XM, YM,1);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // a point object holds x y and z coordinates
  TSPoint p = ts.getPoint();
  
  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  //if (p.z > ts.pressureThreshhold) {
     /*Serial.print("X = "); Serial.print(p.x);
     Serial.print("\tY = "); Serial.print(p.y);
     Serial.print("\tXCAL = "); Serial.print(p.x/(1000/26));
     Serial.print("\tYCAL = "); Serial.print(p.y/(900/20));
     Serial.print("\tPressure = "); Serial.println(p.z);*/
    
    double XX = 11-(p.y/(900.000/20.000));
    double YY = 13-(p.x/(1000.000/26.000));

    if ((XX<-4.0) && (XX>-6.5) && (YY>12.5) && (YY<13.5)) {
      Serial.print("\tNo Ball Detected");
      Serial.print('\n');
    }
    
    else {
     Serial.print("\tX = "); Serial.print(XX);
     Serial.print("\tY = "); Serial.print(YY);
     Serial.print('\n');
    }

  //}

  delay(100);
}