#include <ESP32Encoder.h>

#define RIGHT_ENCODER_A_PIN 26  //este es el que genera la interrupción
#define RIGHT_ENCODER_B_PIN 25

#define LEFT_ENCODER_A_PIN 36 //este es el que genera la interrupción 26,25
#define LEFT_ENCODER_B_PIN 37

ESP32Encoder encoderR,encoderL;



void setupEnc(){

   pinMode(RIGHT_ENCODER_A_PIN, INPUT);
   pinMode(RIGHT_ENCODER_B_PIN, INPUT);  
   pinMode(LEFT_ENCODER_A_PIN, INPUT);
   pinMode(LEFT_ENCODER_B_PIN, INPUT);    
    
	// Enable the weak pull down resistors
  ESP32Encoder::useInternalWeakPullResistors=UP;

  encoderL.clearCount();
  encoderL.setCount(0);
  
  encoderR.clearCount();
  encoderR.setCount(0);

	// Attache pins for use as encoder pins
encoderR.attachHalfQuad(RIGHT_ENCODER_A_PIN,RIGHT_ENCODER_B_PIN);
encoderL.attachHalfQuad(LEFT_ENCODER_B_PIN,LEFT_ENCODER_A_PIN);

}

int32_t encL(){
  return encoderL.getCount();
  }

int32_t encR(){
  return encoderR.getCount();
  }


void encReset(){
  encoderL.clearCount();
  encoderL.setCount(0);
  encoderR.clearCount();
  encoderR.setCount(0);
  }
