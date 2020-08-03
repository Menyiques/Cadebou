#include <VL6180X.h>

VL6180X sensor;

#define TCAADDR 0x70
 
void tca_select(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setupLaser(){

  tca_select(0);//right sensor 
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
  sensor.setScaling(3);

  tca_select(1);//front right sensor  
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500); 
  sensor.setScaling(3);

  tca_select(2);//front left sensor  
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
  sensor.setScaling(3);
  
  tca_select(4);//left sensor  
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
  sensor.setScaling(3);
}

uint8_t dLmm(){
   tca_select(4);
   return sensor.readRangeSingleMillimeters();
}
uint8_t dRmm(){
   tca_select(0);
   return sensor.readRangeSingleMillimeters();
}
uint8_t dFRmm(){
   tca_select(1);
   return sensor.readRangeSingleMillimeters();
}
uint8_t dFLmm(){
   tca_select(2);
   return sensor.readRangeSingleMillimeters();
}



  
