#include <Servo.h>

Servo servo1;   // right rear upper
Servo servo2;   // right rear lower
Servo servo3;   // right front upper
Servo servo4;   // right front lower
Servo servo5;   // right wheel upper
Servo servo6;   // right wheel lower
Servo servo7;   // body back

Servo servo8;     // left rear upper
Servo servo9;     // left rear lower
Servo servo10;    // left front upper
Servo servo11;    // left front lower
Servo servo12;    // left wheel upper
Servo servo13;    // left wheel lower
Servo servo14;    // body front

int offset1;
int offset2;
int offset3;
int offset4;
int offset5;
int offset6;
int offset7;
int offset8;
int offset9;
int offset10;
int offset11;
int offset12;
int offset13;
int offset14;

int steering;
int frontServo;
int backServo;
int frontLift;

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

int RLR = 0;
int RFB = 0;
int RT = 0;
int LLR = 0;
int LFB = 0;
int LT = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer

long previousWalkMillis = 0;    // set up timers
int walkFlag = 0;

void setup() {

   
    // initialize serial communication
    Serial.begin(115200);
    
    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);

    radio.startListening();

    servo1.attach(43);     // right rear upper
    servo2.attach(41);     // right rear lower
    servo3.attach(39);     // right front upper
    servo4.attach(37);     // right front lower
    servo5.attach(35);     // right wheel upper
    servo6.attach(33);     // right wheel lower
    servo7.attach(31);     // body back

    servo8.attach(42);     // left rear upper
    servo9.attach(40);     // left rear lower
    servo10.attach(38);    // left front upper
    servo11.attach(36);    // left front lower
    servo12.attach(34);    // left wheel upper
    servo13.attach(32);    // left wheel lower
    servo14.attach(30);    // body front

    offset1 = 1600;
    offset2 = 1600;
    offset3 = 1500;
    offset4 = 1560;
    offset5 = 2600;
    offset6 = 1480;
    offset7 = 1500;
    offset8 = 1650;
    offset9 = 1550;
    offset10 = 1560;
    offset11 = 1500;
    offset12 = 2300;
    offset13 = 1580;
    offset14 = 1600;

    servo1.writeMicroseconds(offset1);   // right rear upper
    servo2.writeMicroseconds(offset2);   // right rear lower
    servo3.writeMicroseconds(offset3);   // right front upper
    servo4.writeMicroseconds(offset4);   // right front lower
    servo5.writeMicroseconds(offset5);   // right wheel upper
    servo6.writeMicroseconds(offset6);   // right wheel lower
    servo7.writeMicroseconds(offset7);   // body back

    servo8.writeMicroseconds(offset8);   // left rear upper
    servo9.writeMicroseconds(offset9);   // left rear lower
    servo10.writeMicroseconds(offset10);  // left front upper
    servo11.writeMicroseconds(offset11);  // left front lower
    servo12.writeMicroseconds(offset12);  // left wheel upper
    servo13.writeMicroseconds(offset13);  // left wheel lower
    servo14.writeMicroseconds(offset14);  // body front
    
}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {  
      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
            previousMillis = currentMillis;


            // check for radio data
            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));   
            }  

            // threshold remote data
            // some are reversed based on stick wiring in remote
            RFB = (thresholdStick(mydata_remote.RFB))*-1;   
            RLR = thresholdStick(mydata_remote.RLR);
            RT = thresholdStick(mydata_remote.RT);   
            LFB = (thresholdStick(mydata_remote.LFB))*-1;   
            LLR = thresholdStick(mydata_remote.LLR);
            LT = thresholdStick(mydata_remote.LT); 

            // front wheel steering
            
            steering = map(RT,-460,460,-500,500);     // arbirtary map for tuning
            servo6.writeMicroseconds(offset6-steering);   // right wheel lower
            servo13.writeMicroseconds(offset13-steering);   // left wheel lower

            // body bend - front + wheel lift
            LFB = constrain(LFB,0,460);
            frontServo = map(LFB,0,460,0,-2500);  // map for scale
            frontLift = map(LFB,0,460,0,-1600);  // map for scale
            servo14.writeMicroseconds(offset14+frontServo);  // body front
            servo5.writeMicroseconds(offset5+frontLift);   // right wheel upper
            servo12.writeMicroseconds(offset12+frontLift);  // left wheel upper

            //body bend - rear
            LT = constrain(LT,-100,460);
            backServo = map(LT,-460,460,-1800,1800);  // map for scale
            servo7.writeMicroseconds(offset7+backServo);  // body front
            
            

            if (mydata_remote.toggle1 == 0) {                                                    // mode - two legs
                servo4.writeMicroseconds(offset4+600);   // right front lower
                servo3.writeMicroseconds(offset3-500);   // right front upper
                servo11.writeMicroseconds(offset11+600);  // left front lower
                servo10.writeMicroseconds(offset10-500);  // left front upper

                if (RFB > 50) {                     // two-legged walking
                    
                    if (walkFlag == 0 && currentMillis - previousWalkMillis >= 0) {
                        servo8.writeMicroseconds(offset8-50);   // left rear upper   // 1
                        servo9.writeMicroseconds(offset9+300);   // left rear lower
                        //***********************
                        servo1.writeMicroseconds(offset1+200);   // right rear upper    // 3
                        servo2.writeMicroseconds(offset2-50);   // right rear lower
                        previousWalkMillis = currentMillis;
                        walkFlag = 1;
                    }         
                    else if (walkFlag == 1 && currentMillis - previousWalkMillis >= 120) {
                        servo8.writeMicroseconds(offset8-200);   // left rear upper   // 2
                        servo9.writeMicroseconds(offset9+200);   // left rear lower 
                        //***********************   
                        servo1.writeMicroseconds(offset1+200);   // right rear upper    // 4
                        servo2.writeMicroseconds(offset2-50);   // right rear lower 
                        previousWalkMillis = currentMillis;
                        walkFlag = 2;                  
                    }
                    else if (walkFlag == 2 && currentMillis - previousWalkMillis >= 120) {
                        servo8.writeMicroseconds(offset8+200);   // left rear upper   // 3
                        servo9.writeMicroseconds(offset9-50);   // left rear lower
                        //***********************      
                        servo1.writeMicroseconds(offset1-50);   // right rear upper  //  1
                        servo2.writeMicroseconds(offset2+300);   // right rear lower
                        previousWalkMillis = currentMillis;
                        walkFlag = 3; 
                    }
                    else if (walkFlag == 3 && currentMillis - previousWalkMillis >= 120) {
                        servo8.writeMicroseconds(offset8+200);   // left rear upper   // 4
                        servo9.writeMicroseconds(offset9-50);   // left rear lower
                        //***********************      
                        servo1.writeMicroseconds(offset1-200);   // right rear upper    // 2
                        servo2.writeMicroseconds(offset2+200);   // right rear lower
                        previousWalkMillis = currentMillis;
                        walkFlag = 4;                  
                    }
                    else if (walkFlag == 4 && currentMillis - previousWalkMillis >= 120) {
                        // go to the start again                                
                        previousWalkMillis = currentMillis;
                        walkFlag = 0;                  
                    }
                }   // end of two-legged walking

               else {     // put legs back to normal
                    servo1.writeMicroseconds(offset1);   // right rear upper
                    servo2.writeMicroseconds(offset2);   // right rear lower
                    servo8.writeMicroseconds(offset8);   // left rear upper
                    servo9.writeMicroseconds(offset9);   // left rear lower
               }
            }  // end of two legs
            

            else if (mydata_remote.toggle1 == 1) {                                // mode - four legs
                if (RFB <= 50) {
                    servo1.writeMicroseconds(offset1);   // right rear upper
                    servo2.writeMicroseconds(offset2);   // right rear lower
                    servo8.writeMicroseconds(offset8);   // left rear upper
                    servo9.writeMicroseconds(offset9);   // left rear lower
                    servo4.writeMicroseconds(offset4-200);   // right front lower
                    servo3.writeMicroseconds(offset3-140);   // right front upper
                    servo11.writeMicroseconds(offset11-200);  // left front lower
                    servo10.writeMicroseconds(offset10-140);  // left front upper
                }
                else if (RFB > 50) {                     // four-legged walking
                    
                    if (walkFlag == 0 && currentMillis - previousWalkMillis >= 0) {
                        servo8.writeMicroseconds(offset8-300);   // left rear upper   // 1
                        servo9.writeMicroseconds(offset9+800);   // left rear lower
                        //***********************
                        servo1.writeMicroseconds(offset1+200);   // right rear upper    // 3
                        servo2.writeMicroseconds(offset2-50);   // right rear lower
                        //***********************
                        //***********************
                        servo10.writeMicroseconds(offset10+200);   // left rear upper   // 3
                        servo11.writeMicroseconds(offset11-50);   // left rear lower
                        //***********************      
                        servo3.writeMicroseconds(offset3-300);   // right rear upper  //  1
                        servo4.writeMicroseconds(offset4+800);   // right rear lower

                        previousWalkMillis = currentMillis;
                        walkFlag = 1;
                    }         
                    else if (walkFlag == 1 && currentMillis - previousWalkMillis >= 250) {
                        servo8.writeMicroseconds(offset8-200);   // left rear upper   // 2
                        servo9.writeMicroseconds(offset9+200);   // left rear lower 
                        //***********************   
                        servo1.writeMicroseconds(offset1+200);   // right rear upper    // 4
                        servo2.writeMicroseconds(offset2-50);   // right rear lower 
                        //***********************
                        //***********************
                        servo10.writeMicroseconds(offset10+200);   // left rear upper   // 4
                        servo11.writeMicroseconds(offset11-50);   // left rear lower
                        //***********************      
                        servo3.writeMicroseconds(offset3-200);   // right rear upper    // 2
                        servo4.writeMicroseconds(offset4+200);   // right rear lower

                        previousWalkMillis = currentMillis;
                        walkFlag = 2;                  
                    }
                    else if (walkFlag == 2 && currentMillis - previousWalkMillis >= 250) {
                        servo8.writeMicroseconds(offset8+200);   // left rear upper   // 3
                        servo9.writeMicroseconds(offset9-50);   // left rear lower
                        //***********************      
                        servo1.writeMicroseconds(offset1-300);   // right rear upper  //  1
                        servo2.writeMicroseconds(offset2+800);   // right rear lower
                        previousWalkMillis = currentMillis;
                        //***********************
                        //***********************
                        servo10.writeMicroseconds(offset10-300);   // left rear upper   // 1
                        servo11.writeMicroseconds(offset11+800);   // left rear lower
                        //***********************
                        servo3.writeMicroseconds(offset3+200);   // right rear upper    // 3
                        servo4.writeMicroseconds(offset4-50);   // right rear lower

                        walkFlag = 3; 
                    }
                    else if (walkFlag == 3 && currentMillis - previousWalkMillis >= 250) {
                        servo8.writeMicroseconds(offset8+200);   // left rear upper   // 4
                        servo9.writeMicroseconds(offset9-50);   // left rear lower
                        //***********************      
                        servo1.writeMicroseconds(offset1-200);   // right rear upper    // 2
                        servo2.writeMicroseconds(offset2+200);   // right rear lower
                        previousWalkMillis = currentMillis;
                        //***********************
                        //***********************
                        servo10.writeMicroseconds(offset10-200);   // left rear upper   // 2
                        servo11.writeMicroseconds(offset11+200);   // left rear lower 
                        //***********************   
                        servo3.writeMicroseconds(offset3+200);   // right rear upper    // 4
                        servo4.writeMicroseconds(offset4-50);   // right rear lower                        

                        walkFlag = 4;                  
                    }
                    else if (walkFlag == 4 && currentMillis - previousWalkMillis >= 250) {
                        // go to the start again                                
                        previousWalkMillis = currentMillis;
                        walkFlag = 0;                  
                    }                
                }   // end of four-legged walking
            }

            

 
      
        }     // end of timed loop         
   
}       // end  of main loop
