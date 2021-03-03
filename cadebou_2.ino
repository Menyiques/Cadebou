#include <Wire.h>
#include <TFT_eSPI.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiUDP.h>


/*Influxdb restart
 * brew services restart influxdb
 * 
 * Grafana restart
 * brew services restart grafana
 * 
 * Grafana URL: localhost:3000
 * Sergio/1234
 * 
 * Logs bluetooth(obsoleto)
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
double VBAT=0;


PID myPIDL(&InputL, &OutputL, &SetpointL,Kp,Ki,Kd,P_ON_E, DIRECT); 
PID myPIDR(&InputR, &OutputR, &SetpointR,Kp,Ki,Kd,P_ON_E, DIRECT); 
PID myPIDW(&InputW, &OutputW, &SetpointW,KpW,KiW,KdW,P_ON_E, DIRECT); 

//interrupts
volatile int interrupts;
int totalInterrupts;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//sensor
uint8_t dL, dR, dFL, dFR;
float yaw; //signed integer 

//BluetoothSerial bt;
String modo = "0";//stop
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
float pwmL, pwmR;
float x=0;float y=0;
float accSpeed=0;
float desiredSpeed=0;
String url;
const char host[]="192.168.1.8";
String payload;
int32_t pasosL;
int32_t pasosR;
float measured_linear_speed;
float measured_angular_speed;
float delta=0.0;

int ss=1;

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
  modo = "0";//stop
  //bt.begin("CaDeBou"); //Bluetooth device name

  myPIDL.SetMode(AUTOMATIC);
  myPIDL.SetOutputLimits(0,10);
  myPIDL.SetSampleTime(25); 
  myPIDR.SetMode(AUTOMATIC);
  myPIDR.SetOutputLimits(0,10);
  myPIDR.SetSampleTime(25);  
  desiredSpeed=400;
 
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
  
  if (digitalRead(35) == 0 && modo=="0") {
        delay(1000);
        encReset();
        modo="1";//forward
        oldMicros=micros()-25000;
        oldpl=0;
        oldpr=0;
        y=0;
  }

if (digitalRead(35) == 0 && modo=="1") {
        modo="0";//stop
        mPower(0,0);
        delay(1000);
        
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

    spL=(pasosL*1000000*66.254/140/thisMicros);
    spR=(pasosR*1000000*66.254/140/thisMicros);

    measured_linear_speed=(spL+spR)/2.0;
    measured_angular_speed=(spR-spL)/75.0; //negativo es que se corre más L que R por lo que se va hacia R... Negativo tiende a R, Positivo tiende a L



    if (modo=="1"){
        SetpointL=desiredSpeed;
        SetpointR=desiredSpeed;
        
        InputL=spL;
        InputR=spR;
        InputW=measured_angular_speed;
        
        myPIDL.Compute();
        myPIDR.Compute();
  
        pwmL=round(OutputL)-delta;
        pwmR=round(OutputR)+delta;
        
        mPower((totalInterrupts%10<=(pwmL)?100:0),(totalInterrupts%10<=(pwmR)?105:0));
        
        
    }else{
        mPower(0,0);
    }
    delta=round((45.0-dR)/30.0);     
    x=x+(measured_linear_speed*thisMicros/1000000.0*sin(yaw));//mm recorridos
    y=y+(measured_linear_speed*thisMicros/1000000.0*cos(yaw));//mm recorridos



    

sendInflux(
    "micros value="+(String)thisMicros
+"\n mode value="+modo
+"\n yaw value="+(String)yaw
+"\n spL value="+(String)spL
+"\n spR value="+(String)spR
+"\n encL value="+(String)encL()
+"\n encR value="+(String)encR()
+"\n LSp value="+(String)measured_linear_speed
+"\n ASp value="+(String)measured_angular_speed
+"\n dL value="+(String)dL
+"\n dR value="+(String)dR
+"\n dFL value="+(String)dFL
+"\n dFR value="+(String)dFR  
+"\n pwmL value="+(String)pwmL
+"\n pwmR value="+(String)pwmR
+"\n OutputR value="+(String)OutputR
+"\n OutputL value="+(String)OutputL 
+"\n SetpointL value="+(String)SetpointL
+"\n SetpointR value="+(String)SetpointR
+"\n Battery value="+(String)VBAT
+"\n PosX value="+(String)x
+"\n PosY value="+(String)y
+"\n Delta value="+(String)delta
);         

    tft.fillScreen(TFT_BLACK);          

    if ((dFL<180)&&(dFR<180))
    { 
     
      if (dR>200) print1AtG("-->",4,TFT_WHITE);
      if (dL>200) print1AtG("<--",4,TFT_WHITE);
      }

    if (VBAT<3.4){print1AtG("BatteryLow",0,TFT_RED);}
      
    // tft.fillScreen(TFT_BLACK);
     print1AtG("Yaw="+String(yaw),0,TFT_WHITE);
     print2AtG(String(dFL),String(dFR),1,TFT_WHITE);
     print2AtG(String(dL),String(dR),2,TFT_WHITE);
     print1AtG("Delta="+String(delta),3,TFT_YELLOW);
     print2AtG(String(x),String(y),4,TFT_CYAN); 
     
    // print1AtG("CADEBOU",4,TFT_WHITE);
    // print2AtG(String(dL),String(dR),3,TFT_CYAN);
    // print2AtG(String(dFL),String(dFR),2,TFT_CYAN);

  }
  }


  void sendInflux(String line){
  udp.beginPacket(host, port);
  udp.print(line);
  udp.endPacket();
    }


void Task2code( void * pvParameters ){
for(;;){
      //long m=micros();
      float yawtemp=get_mpu();
      if (yawtemp<990){yaw=yawtemp;}//valid value
      dL=dLmm();
      dR=dRmm();
      dFR=dFRmm();
      dFL=dFLmm();
      VBAT = (float)(analogRead(34)) / 4095*2*3.3*1.1;
   }
  }
