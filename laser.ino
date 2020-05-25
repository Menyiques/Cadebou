#include <VL53L0X.h>
#include <VL6180X.h>

VL53L0X sensor_c;
VL6180X sensor_i;
VL6180X sensor_d;

  #define TCA_SENSOR_DERECHA 0
  #define TCA_SENSOR_IZQUIERDA 2
  #define TCA_SENSOR_CENTRO 1
  #define TCAADDR 0x70

  boolean error_dc=false;
 
void tca_select(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup_laser(){

  tca_select(TCA_SENSOR_IZQUIERDA);
  sensor_i.init();
  sensor_i.configureDefault();
  
  tca_select(TCA_SENSOR_DERECHA);
  sensor_d.init();
  sensor_d.configureDefault();

  tca_select(TCA_SENSOR_CENTRO);
  sensor_c.setTimeout(20);
    if (!sensor_c.init()){Serial.println("Error DC...................");}
  sensor_c.setMeasurementTimingBudget(20000);
  sensor_c.startContinuous();
  }

uint8_t dl_mm(){
   tca_select(TCA_SENSOR_IZQUIERDA);
   return sensor_i.readRangeSingleMillimeters();
}
uint8_t dr_mm(){
   tca_select(TCA_SENSOR_DERECHA);
   return sensor_d.readRangeSingleMillimeters();
}

uint16_t dc_mm(){
  tca_select(TCA_SENSOR_CENTRO);
  return sensor_c.readRangeContinuousMillimeters();
  
}


  
