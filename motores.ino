const int freq = 5000;
const int ledChannel0 = 0;
const int ledChannel1 = 1;
const int ledChannel3 = 3;
const int ledChannel4 = 4;
const int resolution = 8;


uint8_t powerWeightedTable[]={101,101,101,102,102,102,103,104,103,104,104,104,104,104,105,105,105,105,105,105,105,105,105,106,105,105,105,105,106,105,105,105,105,105,105,105,105,105,105,105,104,104,104,104,104,103,103,103,103,103,103,103,103,103,103,103,104,104,104,104,105,105,105,107,108,108,111,114,114,118,120,166,128,153,179,181,183,184,185,186,188,189,190,191,193,194,195,196,196,197,198,199,200,201,201,202,203,204,205,206,207,208,209,210,211,211,213,212,213,214,215,215,216,217,218,220,220,221,221,223,224,224,224,226,226,226,226,226,226,227,228,228,228,227,229,230,229,230,232};


#define LEFT_MOTOR_REV_PIN 32// PIN 26 35 y 33 NO FUNCIONA COMO SALIDA
#define LEFT_MOTOR_FWD_PIN 33
#define RIGHT_MOTOR_REV_PIN 27
#define RIGHT_MOTOR_FWD_PIN 13


uint8_t powerRightCorrected(uint8_t left){ //117 to 255
  /*if (left>=117&&left<256){
    return(powerWeightedTable[left-117]);
  }else{
    return 0;
    }//
*/
    return left;
  }

void setup_motores(){
//----------------------------------------------  
  ledcSetup(ledChannel0, freq, resolution);
  ledcSetup(ledChannel1, freq, resolution);
  ledcAttachPin(LEFT_MOTOR_REV_PIN, ledChannel0);
  ledcAttachPin(LEFT_MOTOR_FWD_PIN, ledChannel1);
  
  ledcSetup(ledChannel3, freq, resolution);
  ledcSetup(ledChannel4, freq, resolution);
  ledcAttachPin(RIGHT_MOTOR_REV_PIN, ledChannel3);
  ledcAttachPin(RIGHT_MOTOR_FWD_PIN, ledChannel4);


//----------------------------------------------
  
}


void m_power(int16_t pwml,int16_t pwmr){

if (pwml<0){
        ledcWrite(ledChannel0, 0);
        ledcWrite(ledChannel1, (uint8_t)-pwml);
    }
if (pwml>=0){
        ledcWrite(ledChannel0, (uint8_t)pwml);
        ledcWrite(ledChannel1, 0);
        actualPowerLeft=pwml;
    }

if (pwmr<0){
        ledcWrite(ledChannel3, (uint8_t)-powerRightCorrected(pwmr));
        ledcWrite(ledChannel4, 0);
    }
if (pwmr>=0){
        ledcWrite(ledChannel3, 0);
        ledcWrite(ledChannel4, (uint8_t)powerRightCorrected(pwmr));
        actualPowerRight=pwmr;
    }
 
  
  }

void m_forward(uint8_t pwml,uint8_t pwmr){
  actualPowerLeft=pwml;
  actualPowerRight=pwmr;
  
  ledcWrite(ledChannel0, pwml);
  ledcWrite(ledChannel1, 0);
  ledcWrite(ledChannel3, 0);
  ledcWrite(ledChannel4, pwmr);
  }

void m_stop(){
  actualPowerLeft=0;
  actualPowerRight=0;
  ledcWrite(ledChannel0, 0);
  ledcWrite(ledChannel1, 0);
  ledcWrite(ledChannel3, 0);
  ledcWrite(ledChannel4, 0);
  
  }
void m_backwards(uint8_t pwml,uint8_t pwmr){
  actualPowerLeft=-pwml;
  actualPowerRight=-pwmr;
  
  ledcWrite(ledChannel0, 0);
  ledcWrite(ledChannel1, pwml);
  ledcWrite(ledChannel3, pwmr);
  ledcWrite(ledChannel4, 0);
  }
  
