
// 4 motors 


#include "I2Cdev.h"                          // including the 12c devices library that allows the comunication via i2c comunication protocol

#include "MPU6050_6Axis_MotionApps20.h"     // including the library file that contains the calibration and fusion between the gyroscope and accelorometer 
#include <Servo.h>                          // including the servo library 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

int j=0 ;
float correct ;                           // variable used in removing the first 600 readings of the yaw as it takes several iterations to stablize 

                                        // flex sensors variables // 
const int pinflex0 = 0;
const int pinflex1 = 1;

int servopos3;
int servopos4;

int flexpos0;
int flexpos1;

                                       //   servos decleration   //

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;



#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2 

                                          // MPU control//

 //DMP stands for Digital Motion Processing. The MPU 6050 has a built-in motion processor. It processes the values from the accelerometer , gyroscope and magnetometer to give us accurate 3D values.


bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

                               //  INTERRUPT DETECTION //   

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();                                   // starting the i2c comunication 
        Wire.setClock(400000);                          // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

  
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) {
        
        mpu.setDMPEnabled(true);
      
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

                                          // Define the pins to which the 4 servo motors are connected//
  servo1.attach(8);                       //base
  servo2.attach(9);                       //elbow
  servo3.attach(10);                      //wrist
  servo4.attach(11);                      //gripper 
}


void loop() {

    if (!dmpReady) return;                                 // if programming failed, don't try to do anything
    while (!mpuInterrupt && fifoCount < packetSize) {      // wait for MPU interrupt or extra packet(s) available
        if (mpuInterrupt && fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();                  // try to get out of the infinite loop , it will loop until the fifo is filled
        }  
        
    }

    mpuInterrupt = false;                                  // reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();                       // check for overflow (this should never happen unless our code is too inefficient)

    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;

       


        #ifdef OUTPUT_READABLE_YAWPITCHROLL               // Get Yaw, Pitch and Roll values
        
            mpu.dmpGetQuaternion(&q, fifoBuffer);                   
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                                                          // Yaw, Pitch, Roll values - Radians to degrees
             ypr[0] = ypr[0] * 180 / M_PI;
             ypr[1] = ypr[1] * 180 / M_PI;
             ypr[2] = ypr[2] * 180 / M_PI;

            flexpos0 = analogRead(pinflex0);
            flexpos1 = analogRead(pinflex1);
                                                          
           
                                                          // Skip 600 readings (self-calibration process)
            if (j <= 1000) {
            correct = ypr[0]; 
            j++;
          }
          else {
            ypr[0] = ypr[0] - correct;
            int servo1Value = map( ypr[0], 0,-180,180, 0);
            int servo2Value = map(ypr[2],80, -100,180, 0);     //60, -120,180, 0

            servopos3 = map(flexpos0 ,500, 700, 80,170);               // new flex   
            servopos4 = map(flexpos1, 40, 50, 87,160);

            
            servo1Value = constrain(servo1Value, 0, 180);
            servo2Value = constrain(servo2Value, 0, 180);
            servopos3 = constrain(servopos3, 80,170);
            servopos4 = constrain(servopos4, 150,87);


            servo1.write(servo1Value);
            servo2.write(servo2Value);
            servo3.write(servopos3);
            servo4.write(servopos4);

                        
            Serial.print("ypr\t");
            Serial.print(ypr[0] );
            Serial.print("  ");
            //Serial.print(ypr[1]);
           //Serial.print("\t");
            Serial.print(ypr[2] );
            
            Serial.print("\t");
           // Serial.print("base:");
            Serial.print(servo1Value);
            Serial.print("  ");
           // Serial.print("elbow:");
            Serial.print(servo2Value);
            Serial.print("  ");
            
            Serial.print("\t");
           // Serial.print("Flex1:");
            Serial.print(flexpos0);
            Serial.print("  ");
            //Serial.print("Flex2:");
            Serial.print(flexpos1);
              Serial.print("\t");
              
           // Serial.print("wrist:");
            Serial.print(servopos3);
            Serial.print("  ");
           // Serial.print("gripper:");
            Serial.print(servopos4);
            Serial.print("\n");


    }
        #endif

       

    
      
    }
    
}
