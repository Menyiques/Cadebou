const int freq = 5000;
const int ledChannel0 = 0;
const int ledChannel1 = 1;
const int ledChannel3 = 4;
const int ledChannel4 = 3;
const int resolution = 8;



#define LEFT_MOTOR_REV_PIN 33// PIN 26 35 y 33 NO FUNCIONA COMO SALIDA
#define LEFT_MOTOR_FWD_PIN 32
#define RIGHT_MOTOR_REV_PIN 27
#define RIGHT_MOTOR_FWD_PIN 13


//140 pasos por vuelta
//66.254 mm por vuelta
//180 mm por celda


uint32_t pasos_a_mm(uint32_t pasos){
  return (pasos*66.254/140);
  }

void setupMotors(){

  pinMode(32,OUTPUT);pinMode(33,OUTPUT);pinMode(27,OUTPUT);pinMode(13,OUTPUT);
//----------------------------------------------  
  ledcSetup(ledChannel0, freq, resolution);
  ledcSetup(ledChannel1, freq, resolution);
  ledcAttachPin(LEFT_MOTOR_FWD_PIN, ledChannel0);
  ledcAttachPin(LEFT_MOTOR_REV_PIN, ledChannel1);
  
  ledcSetup(ledChannel3, freq, resolution);
  ledcSetup(ledChannel4, freq, resolution);
  ledcAttachPin(RIGHT_MOTOR_FWD_PIN, ledChannel3);
  ledcAttachPin(RIGHT_MOTOR_REV_PIN, ledChannel4);
//----------------------------------------------
  
}


void mPower(float pwml,float pwmr){

uint16_t ui_pwml=abs((int)pwml);
uint16_t ui_pwmr=abs((int)pwmr);
  
 //LEFT
if (pwml<0){
        ledcWrite(ledChannel0, 0);//hacia delante
        ledcWrite(ledChannel1, ui_pwml);//hacia atrás
    }
if (pwml>=0){
        ledcWrite(ledChannel0, ui_pwml);//hacia delante
        ledcWrite(ledChannel1, 0);//hacia atrás

    }
 //RIGHT
if (pwmr<0){
        ledcWrite(ledChannel3, 0);//hacia delante
        ledcWrite(ledChannel4, ui_pwmr);//hacia atrás

    }
if (pwmr>=0){
        ledcWrite(ledChannel3, ui_pwmr);//hacia delante
        ledcWrite(ledChannel4, 0);//hacia atrás

    }



  }

  
