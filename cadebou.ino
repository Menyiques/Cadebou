#include <Wire.h>
#include <TFT_eSPI.h>
//#include "BluetoothSerial.h"
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#define BLYNK_PRINT Serial

double SetpointL,SetpointR, InputL, OutputL,InputR, OutputR;
char auth[] = "63a8db7bb424472aaf7e1361fe90e708";
char ssid[] = "MI";
char pass[] = "moralesmartinezwifi";

double consKp=0.07, consKi=0.5, consKd=0.003;
PID myPIDL(&InputL, &OutputL, &SetpointL, consKp, consKi, consKd, DIRECT);
PID myPIDR(&InputR, &OutputR, &SetpointR, consKp, consKi, consKd, DIRECT);


//interrupts
volatile int interrupts;
int totalInterrupts;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;



uint32_t oldpl=0;
uint32_t oldpr=0;
float spR=0;
float spL=0;
float maxSpL=0;
float maxSpR=0;
uint32_t loops=0;
uint32_t oldMicros=0;
uint32_t newMicros=0;
uint32_t thisMicros=0;
uint32_t oldStepsL=0;
uint32_t oldStepsR=0;

uint8_t dl,dr;
uint16_t dc,yaw;

//BluetoothSerial bt;

String modo="stop";
uint8_t defaultPower=150;
uint8_t actualPowerLeft=defaultPower;
uint8_t actualPowerRight=defaultPower;




TFT_eSPI tft = TFT_eSPI(135, 240);

BLYNK_WRITE(V0)
{
   consKp = param.asDouble(); // assigning incoming value from pin V1 to a variable
   
    myPIDL.SetTunings(consKp, consKi, consKd);
    myPIDR.SetTunings(consKp, consKi, consKd);
    actualPowerLeft=0;actualPowerRight=0;
    m_power(0,0);
    delay(1000);

}
BLYNK_WRITE(V4)
{

    SetpointL=param.asInt();
    SetpointR=SetpointL;
    myPIDL.SetTunings(consKp, consKi, consKd);
    myPIDR.SetTunings(consKp, consKi, consKd);
    actualPowerLeft=0;actualPowerRight=0;
    m_power(0,0);
    delay(1000);

}
BLYNK_WRITE(V1)
{
   consKi = param.asDouble(); // assigning incoming value from pin V1 to a variable
   
    myPIDL.SetTunings(consKp, consKi, consKd);
    myPIDR.SetTunings(consKp, consKi, consKd);
    actualPowerLeft=0;actualPowerRight=0;
    m_power(0,0);
    delay(1000);

}
BLYNK_WRITE(V2)
{
   consKd = param.asDouble(); // assigning incoming value from pin V1 to a variable

    myPIDL.SetTunings(consKp, consKi, consKd);
    myPIDR.SetTunings(consKp, consKi, consKd);
    actualPowerLeft=0;actualPowerRight=0;
    m_power(0,0);
    delay(1000);
}

BLYNK_WRITE(V3){
  if (param.asInt()==0){
    actualPowerLeft=0;actualPowerRight=0;
    m_power(0,0);
    delay(1000);
    }

    if (param.asInt()==1){
    actualPowerLeft=defaultPower;actualPowerRight=defaultPower;
    m_power(defaultPower,0);
    delay(1000);
    }
  }


void setup() {
Serial.begin(115200);
Wire.begin(21,22);
Blynk.begin(auth, ssid, pass);
setup_enc();
setup_laser();
setup_interrupts();
setup_mpu();
setup_motores();
setup_tft();
//setup_bt();
pinMode(35, INPUT);
pinMode(0, INPUT);
myPIDL.SetMode(AUTOMATIC);
myPIDR.SetMode(AUTOMATIC);
SetpointL=800;SetpointR=SetpointL;
}


void loop() {
   Blynk.run();

  modo="forward";
  loops++;


if (interrupts > 0) {
  portENTER_CRITICAL(&timerMux);
  interrupts--;
  portEXIT_CRITICAL(&timerMux);
  totalInterrupts++;
  newMicros=micros();      
  thisMicros=newMicros-oldMicros;
  oldMicros=newMicros;
  int pasosL=encL()-oldpl;oldpl=encL();
  int pasosR=encR()-oldpr;oldpr=encR();
  //spL=(pasosL*1000*66.254/140/thisMicros);if (spL>maxSpL){maxSpL=spL;} //m/s
  //spR=(pasosR*1000*66.254/140/thisMicros);if (spR>maxSpR){maxSpR=spR;} //m/s
  
  spL=(pasosL*1000000*66.254/140/thisMicros);if (spL>maxSpL){maxSpL=spL;} //mm/s
  spR=(pasosR*1000000*66.254/140/thisMicros);if (spR>maxSpR){maxSpR=spR;} //mm/s
 
 dl=dl_mm();
 dr=dr_mm();
 dc=dc_mm();
 yaw=get_mpu();

if (modo=="forward"){/*m_power(actualPowerLeft,actualPowerRight);*/}    
if (dc<100){modo="stop";m_power(0,0);}
if (modo=="stop"){m_power(0,0);}

InputL = spL;
myPIDL.Compute();
actualPowerLeft=OutputL;

InputR = spR-(encL()-encR());
myPIDR.Compute();
actualPowerRight=OutputR;

uint16_t v = analogRead(34);
float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (1.1);

Serial.printf("encL: %d, encR: %d, spL: %f, spR: %f, error: %d",encL(),encR(),spL,spR,(encL()-encR()));
Serial.println();

  tft.fillScreen(TFT_BLACK);
  print3AtG(String(dl),String(dc),String(dr),0,TFT_YELLOW);
  print2AtG(String(spL),String(spR),1,TFT_WHITE);
  print1AtG("CADEBOU",2,TFT_GREEN);
  print1AtG(String(encL()-encR()),3,TFT_CYAN);
  print2AtG(String(battery_voltage)+"v",String("MPU"),4,TFT_WHITE);
  }
}

 
