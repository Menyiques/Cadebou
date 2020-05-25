#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

 
// 0x68
MPU6050 mpu; 
 
#define LED_PIN 13
bool blinkState = false;
 
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
Quaternion q;           // [w, x, y, z]
VectorInt16 aa;         // [x, y, z]
VectorInt16 aaReal;     // [x, y, z]
VectorInt16 aaWorld;    // [x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]
 
void setup_mpu() {
   tca_select(3);
    mpu.initialize();
    // Comprobar  conexion
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
 
    // Valores de calibracion
    // Mio
    mpu.setXAccelOffset(905);
    mpu.setYAccelOffset(368);
    mpu.setZAccelOffset(563);
    mpu.setXGyroOffset(54);
    mpu.setYGyroOffset(-57);
    mpu.setZGyroOffset(29);
    
/*  //Victor  
    mpu.setXAccelOffset(-4947);
    mpu.setYAccelOffset(-1170);
    mpu.setZAccelOffset(1428);
    mpu.setXGyroOffset(123);
    mpu.setYGyroOffset(4);
    mpu.setZGyroOffset(31);
 */   
    // Activar DMP
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        mpuIntStatus = mpu.getIntStatus();
 
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
 

 
float get_mpu() {
     tca_select(3);

    // Si fallo al iniciar, parar programa    
    if (!dmpReady) return 0.0;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } 
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
 
   // MMostrar Yaw, Pitch, Roll
   mpu.dmpGetQuaternion(&q, fifoBuffer);
   mpu.dmpGetGravity(&gravity, &q);
   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   
/*
   Serial.print("ypr\t");
   Serial.print(ypr[0] * 180/M_PI+180);
   Serial.print("\t");
   Serial.print(ypr[1] * 180/M_PI);
   Serial.print("\t");
   Serial.println(ypr[2] * 180/M_PI);
*/
   return ypr[0]* 180/M_PI+180;
   
   // Mostrar aceleracion
/*   mpu.dmpGetQuaternion(&q, fifoBuffer);
   mpu.dmpGetAccel(&aa, fifoBuffer);
   mpu.dmpGetGravity(&gravity, &q);
   mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
   Serial.print("areal\t");
   Serial.print(aaReal.x);
   Serial.print("\t");
   Serial.print(aaReal.y);
   Serial.print("\t");
   Serial.println(aaReal.z);*/
    }
}
