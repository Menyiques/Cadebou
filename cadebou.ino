#include <Wire.h>
#include <TFT_eSPI.h>
#include "BluetoothSerial.h"
#include <PID_v1.h>



double SetpointX, InputX, OutputX;
double KpX=0.06, KiX=0.03, KdX=0.005;

double SetpointW, InputW, OutputW;
double KpW=25, KiW=0.05, KdW=0.005;

uint8_t PWM_L[51]={0,83,84,85,86,87,88,89,90,91,92,93,94,96,97,99,100,101,103,104,105,108,110,113,116,119,123,174,178,181,183,185,187,189,191,192,194,195,197,198,199,200,201,203,204,206,207,208,210,211,213};
//desde 835 hasta 1815 de 20 en 20 para el motor L

uint8_t PWM_R[51]={0,77,78,80,80,81,82,82,83,84,85,86,87,88,89,90,91 ,92 ,93 ,94 ,96 ,97 ,98 ,100,101,102,104,105,107,110,112,114,117,119,133,134,140,138,145,189,190,192,193,195,196,197,198,199,201,202,203};
//desde 835 hasta 1815 de 20 en 20 para el motor R

PID myPIDX(&InputX, &OutputX, &SetpointX,KpX,KiX,KdX,P_ON_M, DIRECT); 

PID myPIDW(&InputW, &OutputW, &SetpointW,KpW,KiW,KdW,P_ON_M, DIRECT); 

//interrupts
volatile int interrupts;
int totalInterrupts;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//sensor
uint8_t dL, dR, dFL, dFR;
int16_t yaw; //signed integer 

BluetoothSerial bt;
String modo = "stop";
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
float linear_error; 
int pwml;
int pwmr;


float objetive_linear_speed=0.0;  //mm/s
float objetive_angular_speed=0.0; //rad/s


TFT_eSPI tft = TFT_eSPI(135, 240);


int n=-1;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  setupEnc();
  setupLaser();
  setupInterrupts();
  setupMPU();
  setupMotors();
  setupTFT();
  //setupBT();
  pinMode(35, INPUT);
  pinMode(0, INPUT);
  objetive_linear_speed=1000;//mm/s
  modo = "stop";
  bt.begin("CaDeBou"); //Bluetooth device name
  myPIDX.SetMode(AUTOMATIC);
  myPIDX.SetOutputLimits(0.0, 50.0); 
  myPIDX.SetSampleTime(25); 
  myPIDW.SetMode(AUTOMATIC);
  myPIDW.SetOutputLimits(-50.0, 50.0); 
  myPIDW.SetSampleTime(25); 
  
}

void loop(){
  if (digitalRead(35) == 0) {delay(2000);encReset();delay(3000); modo="forward";}
  if (digitalRead(0) == 0) {modo="stop";}
  
  if (interrupts > 0) {

    portENTER_CRITICAL(&timerMux);
    interrupts--;
    portEXIT_CRITICAL(&timerMux);
    totalInterrupts++;

    newMicros=micros();      
    thisMicros=newMicros-oldMicros;
    oldMicros=newMicros;

    int32_t pasosL=encL()-oldpl;oldpl=encL();
    int32_t pasosR=encR()-oldpr;oldpr=encR();

    spL=(pasosL*1000000*66.254/140/thisMicros);if (spL>maxSpL){maxSpL=spL;} //mm/s
    spR=(pasosR*1000000*66.254/140/thisMicros);if (spR>maxSpR){maxSpR=spR;} //mm/s

    float measured_linear_speed=(spL+spR)/2.0;
    float measured_angular_speed=(spR-spL)/75.0;
    
    SetpointX = 800;

    InputX=measured_linear_speed;
    InputW=measured_angular_speed;
    myPIDX.Compute();
    myPIDW.Compute();
    
    if (modo=="forward"){
       pwml=(int)OutputX-(int)OutputW; if(pwml>50)pwml=50; if(pwml<0)pwml=0;
       pwmr=(int)OutputX+(int)OutputW; if(pwmr>50)pwmr=50; 

      mPower(PWM_L[pwml],PWM_R[pwmr]);
      }
    else 
    {mPower(0,0);}

  String text=(String)modo+"|"+
              (String)spL+"|"+
              (String)spR+"|"+
              (String)encL()+"|"+
              (String)encR()+"|"+              
              (String)OutputX+"|"+
              (String)OutputW+"|"+
              (String)pwml+"|"+
              (String)pwmr+"|"+
              (String)measured_linear_speed+"|"+
              (String)measured_angular_speed+"|"+
              (String)interrupts+"|"+
              (String)thisMicros;

              text.replace(".",",");
              
              bt.println(text);
    
  }
  }







void loop_victor() {
  leeSerial();
  if (digitalRead(35) == 0) {delay(3000); modo="forward";}
  if (digitalRead(0) == 0) {modo="stop";}
  if (interrupts > 0) {
    portENTER_CRITICAL(&timerMux);
    interrupts--;
    portEXIT_CRITICAL(&timerMux);
    totalInterrupts++;
    uint16_t v = analogRead(34);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (1.1);
    newMicros=micros();      
    thisMicros=newMicros-oldMicros;
    oldMicros=newMicros;
    int32_t pasosL=encL()-oldpl;oldpl=encL();
    int32_t pasosR=encR()-oldpr;oldpr=encR();
    //spL=(pasosL*1000*66.254/140/thisMicros);if (spL>maxSpL){maxSpL=spL;} //m/s
    //spR=(pasosR*1000*66.254/140/thisMicros);if (spR>maxSpR){maxSpR=spR;} //m/s
    spL=(pasosL*1000000*66.254/140/thisMicros);if (spL>maxSpL){maxSpL=spL;} //mm/s
    spR=(pasosR*1000000*66.254/140/thisMicros);if (spR>maxSpR){maxSpR=spR;} //mm/s

    float measured_linear_speed=(spL+spR)/2;
    float measured_angular_speed=(spR-spL)/75;

    float linear_speed_error=objetive_linear_speed-measured_linear_speed; //error positivo es que vamos más lentos de lo que debemos

    float angular_speed_error=measured_angular_speed-objetive_angular_speed; //Es positivo si giramos a la izqda, es decir dcha corre más que izqda

    uint32_t power_left=objetive_linear_speed+KpX*linear_speed_error+angular_speed_error*KpW; 
    uint8_t pwm_left=map(power_left,835,1815, 0,50);
    
    uint32_t power_right=objetive_linear_speed+KpX*linear_speed_error-angular_speed_error*KpW; //(entre -50 y 50)
    uint8_t pwm_right=map(power_right,835,1815, 0,50);

    
    if (modo=="forward"){mPower(PWM_L[pwm_left],PWM_R[pwm_right]);}
    else 
    {mPower(0,0);}
  
    
    //bt.println("ErrorW:"+(String)angular_speed_error+",PWMLeft:"+(String)pwm_left+",PWMRight:"+(String)pwm_right+",spLeft:"+(String)(spL/18)+",spRight:"+(String)(spR/18));

    bt.println( (String)angular_speed_error+","+
                (String)linear_speed_error+","+
                (String)encL()+","+
                (String)encR()+","+
                (String)power_left+","+
                (String)power_right+","+
                (String)pwm_left+","+
                (String)pwm_right+","+
                (String)spL+","+
                (String)spR
                );
                
                
                
                
      dL=dLmm();
      dR=dRmm();
      dFR=dFRmm();
      dFL=dFLmm();
      
      float yawtemp=get_mpu();
      if ((yawtemp>=-180.0)&&(yawtemp<180.0)){
        yaw=(int16_t)yawtemp;
        }
        
      
 

      tft.fillScreen(TFT_BLACK);
      print2AtG(String(yawtemp),String(yaw),4,TFT_WHITE);
      print2AtG(String(spL).substring(0,6),String(spR).substring(0,6),1,TFT_WHITE);
      print1AtG("CADEBOU",2,TFT_GREEN);
      //print1AtG(String(encL()-encR()),3,TFT_CYAN);
      //print2AtG(String(SetpointXL),String(SetpointXL),3,TFT_WHITE);

  }
}
