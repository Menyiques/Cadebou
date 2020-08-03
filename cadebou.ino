#include <Wire.h>
#include <TFT_eSPI.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiUDP.h>

/*
 * (stty speed 9600 >/dev/null && cat) </dev/cu.CaDeBou-ESP32SPP > /Users/sergio/logscadebou.txt
 * 
 */
 
int status = WL_IDLE_STATUS;     // the Wifi radio's status
int port = 8089;
const char* ssid     = "MI";
const char* password = "moralesmartinezwifi";

TaskHandle_t Task2;


double SetpointL, InputL, OutputL;
double SetpointR, InputR, OutputR;
double SetpointW, InputW, OutputW;
double Kp=0.05, Ki=0.025, Kd=0.005;
double KpW=10, KiW=0.025, KdW=0.005;


PID myPIDL(&InputL, &OutputL, &SetpointL,Kp,Ki,Kd,P_ON_M, DIRECT); 
PID myPIDR(&InputR, &OutputR, &SetpointR,Kp,Ki,Kd,P_ON_M, DIRECT); 

PID myPIDW(&InputW, &OutputW, &SetpointW,KpW,KiW,KdW,P_ON_M, DIRECT); 


//interrupts
volatile int interrupts;
int totalInterrupts;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//sensor
uint8_t dL, dR, dFL, dFR;
float yaw; //signed integer 

//BluetoothSerial bt;
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
float pwml;
float pwmr;
float x=0;float y=0;
float accSpeed=0;
float desiredSpeed=0;
String url;
const char host[]="192.168.1.46";
String payload;
int32_t pasosL;
int32_t pasosR;
float measured_linear_speed;
float measured_angular_speed;

TFT_eSPI tft = TFT_eSPI(135, 240);
WiFiUDP udp;
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
  modo = "stop";
  //bt.begin("CaDeBou"); //Bluetooth device name

  myPIDL.SetMode(AUTOMATIC);
  myPIDL.SetOutputLimits(100,230); 
  myPIDL.SetSampleTime(25); 
  myPIDR.SetMode(AUTOMATIC);
  myPIDR.SetOutputLimits(100,230); 
  myPIDR.SetSampleTime(25); 
  
  myPIDW.SetMode(AUTOMATIC);
  myPIDW.SetOutputLimits(-10.0, 10.0); 
  myPIDW.SetSampleTime(25); 
  desiredSpeed=1000;
  
  SetpointW = 0;

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    url = "/write";
    url += "?db=cadebou";
/* Task function,name of task,Stack size of task,parameter of the task,priority of the task,Task handle to keep track of created task,pin task to core 0 */
 xTaskCreatePinnedToCore(Task2code,"Task2",10000,NULL,1,&Task2,0);          
}
void loop(){
  if (digitalRead(35) == 0) {
        delay(3000);
        encReset();
        modo="forward";
        oldMicros=micros()-25000;
        oldpl=0;
        oldpr=0;
        y=0;
        }

  
  if (interrupts > 0) {
    portENTER_CRITICAL(&timerMux);
    interrupts--;
    portEXIT_CRITICAL(&timerMux);
    totalInterrupts++;

    newMicros=micros();      
    thisMicros=newMicros-oldMicros;
    oldMicros=newMicros;

    pasosL=encL()-oldpl;oldpl=encL();
    pasosR=encR()-oldpr;oldpr=encR();

    spL=(pasosL*1000000*66.254/140/thisMicros);if (spL>maxSpL){maxSpL=spL;} //mm/s
    spR=(pasosR*1000000*66.254/140/thisMicros);if (spR>maxSpR){maxSpR=spR;} //mm/s

    measured_linear_speed=(spL+spR)/2.0;
    measured_angular_speed=(spR-spL)/75.0; //negativo es que se corre más L que R por lo que se va hacia R... Negativo tiende a R, Positivo tiende a L

   

    if (modo=="forward"){

        SetpointL=desiredSpeed-OutputW*100;//desiredSpeed*acc_profile[(int)acc_step/10];
        SetpointR=desiredSpeed+OutputW*100;//desiredSpeed*acc_profile[(int)acc_step/10];

        if (y>180*16){modo="stop";}
        InputL=spL;
        InputR=spR;
        InputW=measured_angular_speed;
        
        myPIDL.Compute();
        myPIDR.Compute();
        myPIDW.Compute();
        
        pwml=OutputL;
        pwmr=OutputR;

       if (pwml<=101){pwml=0;}
       if (pwmr<=101){pwmr=0;}
        
        mPower(pwml,pwmr);
    }else{
        mPower(0,0);
    }

    y=y+measured_linear_speed*thisMicros/1000000.0;//mm recorridos

  String text=(String)modo+"|"+
              (String)desiredSpeed+"|"+
              (String)SetpointL+"|"+
              (String)spL+"|"+
              (String)spR+"|"+
              (String)encL()+"|"+
              (String)encR()+"|"+              
              (String)OutputL+"|"+
              (String)OutputR+"|"+
              (String)OutputW+"|"+
              (String)pwml+"|"+
              (String)pwmr+"|"+
              (String)measured_linear_speed+"|"+
              (String)measured_angular_speed+"|"+
              (String)interrupts+"|"+
              (String)y+"|"+
              (String)yaw+"|"+
              (String)dR+"|"+
              (String)dL+"|"+
              (String)thisMicros;
              text.replace(".",",");
               //bt.println(text);

sendInflux(
    "micros value="+(String)thisMicros
+"\n yaw value="+(String)yaw
+"\n spL value="+(String)spL
+"\n spR value="+(String)spR
+"\n encL value="+(String)encL()
+"\n encR value="+(String)encR()
+"\n LSp value="+(String)measured_linear_speed
+"\n ASp value="+(String)measured_angular_speed
+"\n dL value="+(String)dL
+"\n dR value="+(String)dR
+"\n dFL value="+(String)dFR
+"\n dFR value="+(String)dFL  
+"\n OutputW value="+(String)OutputW
+"\n OutputL value="+(String)OutputL
+"\n OutputR value="+(String)OutputR 
+"\n SetpointL value="+(String)SetpointL
+"\n SetpointR value="+(String)SetpointR
);         
            

     
      
     tft.fillScreen(TFT_BLACK);
     print1AtG(String(yaw),0,TFT_WHITE);
     print1AtG("CADEBOU",4,TFT_WHITE);
     print2AtG(String(dL),String(dR),3,TFT_CYAN);
     print2AtG(String(dFL),String(dFR),2,TFT_CYAN);

  }
  }


  void sendInflux(String line){
  udp.beginPacket(host, port);
  udp.print(line);
  udp.endPacket();
     
    }


void Task2code( void * pvParameters ){

   for(;;){
      long m=micros();
      float yawtemp=get_mpu();
      if ((yawtemp>=-180.0)&&(yawtemp<180.0)){
        yaw=(int16_t)yawtemp;
        }
      dL=dLmm();
      dR=dRmm();
      dFR=dFRmm();
      dFL=dFLmm();
   }
  }
