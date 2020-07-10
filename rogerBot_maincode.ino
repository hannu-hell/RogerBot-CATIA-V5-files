/*aux1, aux2 and aux3 will send 2byte number 9200, 9300, 9400 respectively which can be used for additional programs within the main code.
written by: Hanoon Malik
Special shoutout to youtuber Robojax, How to Mechatronics, Mert Arduino for this code is not purely mine and i have tweaked sections of various codes from the above mentioned authors to suit my project.
It is in no way perfect and i would be happy to accept any modifications.

ROGER BOT 

  Pin No      Description                                                           Logic Type
    
  TX          Transmit Serial Communication to Bluetooth module                     Digital signal
  RX          Receive Serial Communication to Bluetooth module                      Digital signal
  2           Signal to Servo 4                                                     PWM
  3           Signal to Servo 5                                                     PWM
  4           Signal to Servo 3                                                     PWM
  5           Signal to Servo 1                                                     PWM
  6           Roger Head Right LED (Red)                                            PWM
  7           Roger Head Right LED (Yellow)                                         PWM
  8           Roger Head Right LED (Green)                                          PWM
  9           Roger Head Left LED (Green)                                           PWM
  10          Roger Head Left LED (Yellow)                                          PWM
  11          Roger Head Left LED (Red)                                             PWM
  12          RGB and Blue LED                                                      PWM
  13          Red LED set                                                           PWM
  A1          LM 35 Temperature Signal                                              Analog signal
  A2          Photocell Signal                                                      Analog signal
  A15         Driver Cooling Fan                                                    Analog signal
  22          N/A                                                                   N/A
  23          PIR Signal                                                            Digital signal
  24          N/A                                                                   N/A
  25          N/A                                                                   N/A
  26          N/A                                                                   N/A
  27          IN 1 Gripper Motor                                                    Digital signal
  28          Signal to Servo 6 (Base Rotation Servo)                               Digital signal
  29          IN 2 Gripper Motor                                                    Digital signal
  30          Signal to Servo 7 (Steering Servo)                                    Digital signal
  31          IN 3 Gripper Motor                                                    Digital signal
  32          Signal to Servo 2                                                     Digital signal
  33          IN 4 Gripper Motor                                                    Digital signal
  34          Left Side Drive Motor LED                                             Digital signal
  35          TRIG Sonic Sensor                                                     Digital signal
  36          Drive Motor 1 DIR Pin                                                 Digital signal
  37          Drive Motor 1 STEP Pin                                                Digital signal
  38          Drive Motor 2 DIR Pin                                                 Digital signal
  39          Drive Motor 2 STEP PIN                                                Digital signal
  40          Drive Motor 1 Arm Pin                                                 Digital signal
  41          Drive Motor 2 Arm Pin                                                 Digital signal
  42          MP3 - Previous Signal                                                 Digital signal
  43          Gripper End Stop Switch                                               Digital signal
  44          MP3 - Next Signal                                                     Digital signal
  45          DHT11 - Humidity Signal                                               Digital signal
  46          MP3 Play/Pause Signal                                                 Digital signal
  47          MP3 Repeat Signal                                                     Digital signal
  48          ISD1820 Voice Record                                                  Digital signal
  49          ISD1820 Voice Playback                                                Digital signal
  50          Right Side Drive Motor LED                                            Digital signal
  51          Signal to Servo 8 (Head Servo)                                        Digital signal
  52          ECHO Sonic Sensor                                                     Digital signal
  53          MP3 Power Signal                                                      Digital signal
  SDA         LCD Display via IIC                                                   Digital signal
  SCL         LCD Display via IIC                                                   Digital signal

*/

#include <Servo.h> 
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <dht.h>
dht DHT;
#define DHT11_PIN 45

Servo myservo1, myservo2, myservo3, myservo4,myservo5,myservo6,myservo7,myservo8; 
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7,3, POSITIVE);


        int _step = 0; 
        const int Pin1 = 27; 
        const int Pin2 = 29; 
        const int Pin3 = 31; 
        const int Pin4 = 33; 
        const int sleep = 40;
        const int sleep2 = 41;
        const int stepPin = 37;
        const int dirPin = 36;
        const int stepPin2 = 39;
        const int dirPin2 = 38;
        const int redLed = 13;
        const int rgbLed = 12;
        const int rightsideLed = 50;
        const int leftsideLed = 34;
        const int rec = 48;
        const int playback = 49;
        const int head1redLed = 11;
        const int head1yellowLed = 10;
        const int head1greenLed = 9;
        const int head2redLed = 6;
        const int head2yellowLed = 7;
        const int head2greenLed = 8;
        const int trigPin = 35;
        const int echoPin = 52;
        const int humidity = 45;
        const int tempPin = A1;
        const int fan = A15;
        const int mp3Power = 53;
        const int mp3Play = 46;
        const int mp3Next = 44;
        const int mp3Previous = 42;
        const int mp3Repeat = 47;
        long duration;
        int distanceInch, distanceCm, avedist90, avedist60, avedist120;
        boolean dir;// false=clockwise, true=anticlockwise


void setup()
{ 
  lcd.begin(16, 2);
  

  pinMode(Pin1, OUTPUT);  
  pinMode(Pin2, OUTPUT);  
  pinMode(Pin3, OUTPUT);  
  pinMode(Pin4, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(A15, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(tempPin, INPUT);
  pinMode(53, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(44, OUTPUT);
  
  for (int i = 36; i<42; i++){
    pinMode(i, OUTPUT);
  }
  for (int j = 6; j<12; j++){
    pinMode(j, OUTPUT);
    delay(5);
    digitalWrite(j, LOW);
  }

  
  
//ON START-UP THREAD---------------------------

  lcdmain();
  delay(3000);
  lcd.clear();
  digitalWrite(redLed, LOW);
  digitalWrite(rgbLed, LOW);
  digitalWrite(rightsideLed, LOW);
  digitalWrite(leftsideLed, LOW);

  for (int k = 0; k<16; k++){
    lcd.setCursor(k, 0);
    lcd.print("Arming..");
    delay(500);
    lcd.clear();
  }
  
  myservo1.attach(5); 
  myservo2.attach(32);
  myservo3.attach(4);
  myservo4.attach(2);
  myservo5.attach(3);
  myservo6.attach(28);
  myservo7.attach(30);
  myservo8.attach(51);
  
  delay(1000);
  
// armCheck();
  
  lcd.setCursor(5, 0);
  lcd.print("Ready for");
  lcd.setCursor(5, 1);
  lcd.print("Bluetooth");
  delay(1000);
  lcd.clear();
  lcdmain();
  delay(100);
  Serial.begin(9600);

}



void loop()
{
  
  if(Serial.available()>= 2 )
  {
     
        unsigned int servopos = Serial.read();
        unsigned int servopos1 = Serial.read();
        unsigned int realservo = (servopos1 *256) + servopos;
        Serial.println(realservo);

          if (realservo >= 1000 && realservo < 1180) {
          int servo1 = realservo;
          int servo2 = realservo;
          servo1 = map(servo1, 1000, 1176, 0, 176);
          servo2 = map(servo2, 1000, 1180, 180, 0);
          myservo1.write(servo1);
          myservo2.write(servo2+5.5);
          delay(25);
          }
          
          if (realservo >= 2000 && realservo < 2136) {
          int servo3 = realservo;
          servo3 = map(servo3, 2000, 2136, 20, 145);
          myservo3.write(servo3);
          delay(15);
          }
          if (realservo >= 3000 && realservo < 3180) {
          int servo4 = realservo;
          servo4 = map(servo4, 3000, 3180, 0, 180);
          myservo4.write(servo4);
          delay(15);
          }
          if (realservo >= 4000 && realservo < 4180) {
          int servo5 = realservo;
          servo5 = map(servo5, 4000, 4180, 0, 180);
          myservo5.write(servo5);
          delay(20);
          }
          if (realservo >= 5000 && realservo < 5180) {
          int servo6 = realservo;
          servo6 = map(servo6, 5000, 5180, 0, 180);
          myservo6.write(servo6);

          delay(15);
          }
          if (realservo >= 6000 && realservo < 6180) {
          int servo7 = realservo;
          servo7 = map(servo7, 6000, 6180, 0, 180);
          myservo7.write(servo7);
          delay(2);
          }
          
          if (realservo == 7000) {
            grippertightMove(3000, 1.2);
          }
          if (realservo == 7100) {
            gripperlooseMove(3000, 1.2);
          }
          if (realservo == 7200){
            forwardMove(60, 15);
          }
          if (realservo == 7300){
            reverseMove(60, 15);
          }
          if (realservo == 7400){
            clockwiseMove(50, 15);
          }
          if (realservo == 7500){
            anticlockwiseMove(50, 15);
          }
          if (realservo == 7600){
            forwardMove(30, 15);
          }
          if (realservo == 7700){
            reverseMove(30, 15);
          }
          if (realservo == 7800){
            digitalWrite(redLed, HIGH);
            digitalWrite(rgbLed, HIGH);
            digitalWrite(rightsideLed, HIGH);
            digitalWrite(leftsideLed, HIGH);
          }
          if (realservo == 7900){
            digitalWrite(redLed, LOW);
            digitalWrite(rgbLed, LOW);
            digitalWrite(rightsideLed, LOW);
            digitalWrite(leftsideLed, LOW);
          }
          if (realservo == 8000){
            record(8000);
          }
          if (realservo == 8100){
            playBack(8000);
          }
          if (realservo == 8200){
            powerSave();
          }
          if (realservo == 8300){
            analogWrite(fan, 1023);
          }  
          if (realservo == 8400){
            SURVEILLANCE(100, 2000);
          }
          if (realservo == 8500){
            SENSORDATA();
          }
          //if (realservo == 8600){
            //WATCHDOG();
          //}
          //if (realservo == 8700){
            //AUX button1
          //}
          //if (realservo == 8800){
           //AUX button2
          //}
          //if (realservo == 8900){
           //AUX button3
          //}
          if (realservo == 9000){
            digitalWrite(mp3Play, HIGH);
            delay(250);
            digitalWrite(mp3Play, LOW);
            delay(10);
          }
          if (realservo == 9100){
            digitalWrite(mp3Next, HIGH);
            delay(250);
            digitalWrite(mp3Next, LOW);         
          }
          if (realservo == 9200){
            digitalWrite(mp3Previous, HIGH);
            delay(250);
            digitalWrite(mp3Previous, LOW);          
          }
          if (realservo == 9300){
            digitalWrite(mp3Repeat, HIGH);
            delay(250);
            digitalWrite(mp3Repeat, LOW);       
          }
          if (realservo == 9400){
            digitalWrite(mp3Power, HIGH);
          }
            
}
}

// DRIVE MOTORS FUNCTIONS DEFINED BELOW--------------------------
void forwardMove(int fwdPace, int fwdpaceDelay){
            digitalWrite(sleep, HIGH);
            digitalWrite(sleep2, HIGH);
            delay(5);
            digitalWrite(dirPin, HIGH);
            digitalWrite(dirPin2, LOW);
              for(int fwd=0; fwd<fwdPace; fwd++){
              digitalWrite(stepPin, HIGH);
              digitalWrite(stepPin2, HIGH);
              delay(fwdpaceDelay);
              digitalWrite(stepPin, LOW);
              digitalWrite(stepPin2, LOW);
              delay(fwdpaceDelay);
              }
            digitalWrite(sleep, LOW);
            digitalWrite(sleep2, LOW);
            delay(5);
}

void reverseMove(int rvsPace, int rvspaceDelay){
            digitalWrite(sleep, HIGH);
            digitalWrite(sleep2, HIGH);
            delay(5);
            digitalWrite(dirPin, LOW);
            digitalWrite(dirPin2, HIGH);
              for(int rvs=0; rvs<rvsPace; rvs++){
              digitalWrite(stepPin, HIGH);
              digitalWrite(stepPin2, HIGH);
              delay(rvspaceDelay);
              digitalWrite(stepPin, LOW);
              digitalWrite(stepPin2, LOW);
              delay(rvspaceDelay);
              }
            digitalWrite(sleep, LOW);
            digitalWrite(sleep2, LOW);
            delay(5);
}

void clockwiseMove(int clkPace, int clkpaceDelay){
            digitalWrite(sleep, HIGH);
            digitalWrite(sleep2, HIGH);
            delay(5);
            digitalWrite(dirPin, HIGH);
            digitalWrite(dirPin2, HIGH);
              for(int clk=0; clk<clkPace; clk++){
              digitalWrite(stepPin, HIGH);
              digitalWrite(stepPin2, HIGH);
              delay(clkpaceDelay);
              digitalWrite(stepPin, LOW);
              digitalWrite(stepPin2, LOW);
              delay(clkpaceDelay);
              }
            digitalWrite(sleep, LOW);
            digitalWrite(sleep2, LOW);
            delay(5);
}

void anticlockwiseMove(int aclkPace, int aclkpaceDelay){
            digitalWrite(sleep, HIGH);
            digitalWrite(sleep2, HIGH);
            delay(5);
            digitalWrite(dirPin, LOW);
            digitalWrite(dirPin2, LOW);
              for(int aclk=0; aclk<aclkPace; aclk++){
              digitalWrite(stepPin, HIGH);
              digitalWrite(stepPin2, HIGH);
              delay(aclkpaceDelay);
              digitalWrite(stepPin, LOW);
              digitalWrite(stepPin2, LOW);
              delay(aclkpaceDelay);
              }
            digitalWrite(sleep, LOW);
            digitalWrite(sleep2, LOW);
            delay(5);
}


// GRIPPER MOTOR FUNCTIONS DEFINED-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void grippertightMove(int grptightPace, int grptightpaceDelay){
          for (int grptight=0; grptight<grptightPace; grptight++){     
          switch(_step){ 
              case 0: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, HIGH); 
              break;  
              case 1: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, HIGH); 
              digitalWrite(Pin4, HIGH); 
              break;  
              case 2: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, HIGH); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 3: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, HIGH); 
              digitalWrite(Pin3, HIGH); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 4: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, HIGH); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 5: 
              digitalWrite(Pin1, HIGH);  
              digitalWrite(Pin2, HIGH); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 6: 
              digitalWrite(Pin1, HIGH);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 7: 
              digitalWrite(Pin1, HIGH);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, HIGH); 
              break;  
              default: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, LOW); 
              break;  
              } 
          _step--; 
       
          if(_step<0){ 
          _step=7; 
          } 
             if(_step>7){ 
          _step=0; 
          } 
          delay(grptightpaceDelay);
          
  }
}

void gripperlooseMove(int grploosePace, int grploosepaceDelay){
           for(int grploose=0; grploose<grploosePace; grploose++){ 
              switch(_step){ 
              case 0: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, HIGH); 
              break;  
              case 1: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, HIGH); 
              digitalWrite(Pin4, HIGH); 
              break;  
              case 2: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, HIGH); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 3: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, HIGH); 
              digitalWrite(Pin3, HIGH); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 4: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, HIGH); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 5: 
              digitalWrite(Pin1, HIGH);  
              digitalWrite(Pin2, HIGH); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 6: 
              digitalWrite(Pin1, HIGH);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, LOW); 
              break;  
              case 7: 
              digitalWrite(Pin1, HIGH);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, HIGH); 
              break;  
              default: 
              digitalWrite(Pin1, LOW);  
              digitalWrite(Pin2, LOW); 
              digitalWrite(Pin3, LOW); 
              digitalWrite(Pin4, LOW); 
              break;  
              } 
          _step++; 
          if(_step>7){ 
          _step=0; 
          } 
         if(_step<0){ 
          _step=7; 
          }
          delay(grploosepaceDelay); 
         
        } 
}


// SERVO MOTORS' FUNCTIONS DEFINED BELOW--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void dualservoSetposition(int servoAngle){
  int servo1 = servoAngle;
  int servo2 = servoAngle;
  servo1 = map(servo1, 0, 176, 0, 176);
  servo2 = map(servo2, 0, 180, 180, 0);
  myservo1.write(servo1);
  myservo2.write((servo2)+5.5);
}

void servo3Setposition(int servo3Angle){
  int servo3 = servo3Angle;
  servo3 = map(servo3, 0, 136, 0, 136);
  myservo3.write(servo3);
}

void servo4Setposition(int servo4Angle){
  int servo4 = servo4Angle;
  servo4 = map(servo4, 0, 180, 0, 180);
  myservo4.write(servo4);
}

void servo5Setposition(int servo5Angle){
  int servo5 = servo5Angle;
  servo5 = map(servo5, 0, 180, 0, 180);
  myservo5.write(servo5);
}

void servo6Setposition(int servo6Angle){
  int servo6 = servo6Angle;
  servo6 = map(servo6, 0, 180, 0, 180);
  myservo6.write(servo6);
}

void servo7Setposition(int servo7Angle){
  int servo7 = servo7Angle;
  servo7 = map(servo7, 0, 180, 0, 180);
  myservo7.write(servo7);
}

void servo8Setposition(int servo8Angle){
  int servo8 = servo8Angle;
  servo8 = map(servo8, 0, 180, 0, 180);
  myservo8.write(servo8);
}

void dualservoMove(int dualservoAngle, int dualservodesiredAngle, int dualservostepDelay){
  if (dualservodesiredAngle > dualservoAngle){
    for (dualservoAngle; dualservoAngle < dualservodesiredAngle; dualservoAngle++){
      int servo1 = dualservoAngle;
      int servo2 = dualservoAngle;
      servo1 = map(servo1, 0, 176, 0, 176);
      servo2 = map(servo2, 0, 180, 180, 0);
      myservo1.write(servo1);
      myservo2.write((servo2)+5.5);
      delay(dualservostepDelay);
      }
  }
  if (dualservodesiredAngle < dualservoAngle){
    for (dualservoAngle; dualservoAngle > dualservodesiredAngle; dualservoAngle--){
      int servo1 = dualservoAngle;
      int servo2 = dualservoAngle;
      servo1 = map(servo1, 0, 176, 0, 176);
      servo2 = map(servo2, 0, 180, 180, 0);
      myservo1.write(servo1);
      myservo2.write((servo2)+5.5);
      delay(dualservostepDelay);
      }
  }
}

void servo3Move(int servo3Angle, int servo3desiredAngle, int servo3stepDelay){
  if (servo3desiredAngle > servo3Angle){
    for (servo3Angle; servo3Angle < servo3desiredAngle; servo3Angle++){
      int servo3 = servo3Angle;
      servo3 = map(servo3, 0, 136, 0, 136);
      myservo3.write(servo3);
      delay(servo3stepDelay);
      }
   }
   if (servo3desiredAngle < servo3Angle){
    for (servo3Angle; servo3Angle > servo3desiredAngle; servo3Angle--){
      int servo3 = servo3Angle;
      servo3 = map(servo3, 0, 136, 0, 136);
      myservo3.write(servo3);
      delay(servo3stepDelay);
      }
    }
}

void servo4Move(int servo4Angle, int servo4desiredAngle, int servo4stepDelay){
  if (servo4desiredAngle > servo4Angle){
    for (servo4Angle; servo4Angle < servo4desiredAngle; servo4Angle++){
      int servo4 = servo4Angle;
      servo4 = map(servo4, 0, 136, 0, 136);
      myservo4.write(servo4);
      delay(servo4stepDelay);
      }
   }
   if (servo4desiredAngle < servo4Angle){
    for (servo4Angle; servo4Angle > servo4desiredAngle; servo4Angle--){
      int servo4 = servo4Angle;
      servo4 = map(servo4, 0, 136, 0, 136);
      myservo4.write(servo4);
      delay(servo4stepDelay);
      }
    }
}

void servo5Move(int servo5Angle, int servo5desiredAngle, int servo5stepDelay){
  if (servo5desiredAngle > servo5Angle){
    for (servo5Angle; servo5Angle < servo5desiredAngle; servo5Angle++){
      int servo5 = servo5Angle;
      servo5 = map(servo5, 0, 136, 0, 136);
      myservo5.write(servo5);
      delay(servo5stepDelay);
      }
   }
   if (servo5desiredAngle < servo5Angle){
    for (servo5Angle; servo5Angle > servo5desiredAngle; servo5Angle--){
      int servo5 = servo5Angle;
      servo5 = map(servo5, 0, 136, 0, 136);
      myservo5.write(servo5);
      delay(servo5stepDelay);
      
      }
    }
}

void servo6Move(int servo6Angle, int servo6desiredAngle, int servo6stepDelay){
  if (servo6desiredAngle > servo6Angle){
    for (servo6Angle; servo6Angle < servo6desiredAngle; servo6Angle++){
      int servo6 = servo6Angle;
      servo6 = map(servo6, 0, 136, 0, 136);
      myservo6.write(servo6);
      delay(servo6stepDelay);
      }
   }
   if (servo6desiredAngle < servo6Angle){
    for (servo6Angle; servo6Angle > servo6desiredAngle; servo6Angle--){
      int servo6 = servo6Angle;
      servo6 = map(servo6, 0, 136, 0, 136);
      myservo6.write(servo6);
      delay(servo6stepDelay);
      
      }
    }
}

void servo7Move(int servo7Angle, int servo7desiredAngle, int servo7stepDelay){
  if (servo7desiredAngle > servo7Angle){
    for (servo7Angle; servo7Angle < servo7desiredAngle; servo7Angle++){
      int servo7 = servo7Angle;
      servo7 = map(servo7, 0, 136, 0, 136);
      myservo7.write(servo7);
      delay(servo7stepDelay);
      }
   }
   if (servo7desiredAngle < servo7Angle){
    for (servo7Angle; servo7Angle > servo7desiredAngle; servo7Angle--){
      int servo7 = servo7Angle;
      servo7 = map(servo7, 0, 136, 0, 136);
      myservo7.write(servo7);
      delay(servo7stepDelay);
      
      }
    }
}

void servo8Move(int servo8Angle, int servo8desiredAngle, int servo8stepDelay){
  if (servo8desiredAngle > servo8Angle){
    for (servo8Angle; servo8Angle < servo8desiredAngle; servo8Angle++){
      int servo8 = servo8Angle;
      servo8 = map(servo8, 0, 136, 0, 136);
      myservo8.write(servo8);
      delay(servo8stepDelay);
      }
   }
   if (servo8desiredAngle < servo8Angle){
    for (servo8Angle; servo8Angle > servo8desiredAngle; servo8Angle--){
      int servo8 = servo8Angle;
      servo8 = map(servo8, 0, 136, 0, 136);
      myservo8.write(servo8);
      delay(servo8stepDelay);
      
      }
    }
}

// ISD VOICE RECORD FUNCTIONS DEFINED--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void record(int recordTime){
  digitalWrite(rec, HIGH);
  delay(recordTime);
  digitalWrite(rec, LOW);
  delay(10);
}

void playBack(int playbackTime){
  digitalWrite(playback, HIGH);
  delay(playbackTime);
  digitalWrite(playback, LOW);
  delay(10);
}

// MISC FUNCTIONS------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void powerSave(){
  digitalWrite(mp3Power, LOW);
  digitalWrite(redLed, LOW);
  digitalWrite(rgbLed, LOW);
  digitalWrite(rightsideLed, LOW);
  digitalWrite(leftsideLed, LOW);
  analogWrite(fan, 0);
  digitalWrite(head1redLed, LOW);
  digitalWrite(head1yellowLed, LOW);
  digitalWrite(head1greenLed, LOW);
  digitalWrite(head2redLed, LOW);
  digitalWrite(head2yellowLed, LOW);
  digitalWrite(head2greenLed, LOW);
}

void lcdmain(){
  lcd.setCursor(5, 0);
  lcd.print("RogerbOt");
  lcd.setCursor(5, 1);
  lcd.print("inc.");
}

void armCheck(){
  lcd.setCursor(5, 0);
  lcd.print("Roger Arm");
  lcd.setCursor(5, 1);
  lcd.print("Check..");
  delay(1000);
 
  
  dualservoSetposition(95);
  delay(500);
  servo3Setposition(90);
  delay(500);
  servo4Setposition(90);
  delay(500);
  servo5Setposition(90);
  delay(500);
  servo6Setposition(90);
  delay(500);
  servo6Move(90, 150, 50);
  delay(500);
  servo6Move(150, 90, 50);
  delay(1000);
  servo6Move(90, 60, 50);
  delay(500);
  servo6Move(60, 90, 50);
  delay(1000);
  dualservoMove(95, 120, 50);
  delayMicroseconds(30);
  servo3Move(90, 110, 35);   
  delayMicroseconds(30);  
  servo4Move(90, 70, 35);  
  delayMicroseconds(30);  
  servo5Move(90, 170, 5);
  delayMicroseconds(3000);  
  servo5Move(170, 90, 5);
  delayMicroseconds(30);  
  servo4Move(70, 90, 35);
  delayMicroseconds(30);  
  servo3Move(110, 90, 35);
  delayMicroseconds(30);    
  dualservoMove(120, 95, 50);
  
  delay(1000);
  
  dualservoMove(95, 75, 60);
  delay(500);  
  servo3Move(90, 70, 50);   
  delay(500);  
  servo4Move(90, 110, 50);  
  delay(500);
  servo5Move(90, 10, 5);
  delay(500);
  servo5Move(10, 90, 5);
  delay(500);
  servo4Move(110, 90, 50);
  delay(500);
  servo3Move(70, 90, 50);
  delay(500);  
  dualservoMove(75, 95, 60);

  delay(1000);

  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Arm Check");
  lcd.setCursor(5, 1);
  lcd.print("Completed");
  delay(1500);
  lcd.clear();
}

int surveillance(){

digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distanceCm = duration*0.034/2;
distanceInch = duration*0.0133/2;
lcd.setCursor(0,0); 
lcd.print("Distance: "); 
lcd.print(distanceCm); 
lcd.print(" cm");
delay(10);
lcd.setCursor(0,1);
lcd.print("Distance: ");
lcd.print(distanceInch);
lcd.print(" inch");
return distanceCm;
}


int accuracy90(){
  lcd.clear();
  delay(10);
  servo8Setposition(90);
  delay(1000);
  surveillance();
  int dist1 = distanceCm;
  delay(1000);
  lcd.clear();
  surveillance();
  int dist2 = distanceCm;
  delay(1000);
  lcd.clear();
  surveillance();
  int dist3 = distanceCm;
  delay(15);
  avedist90 = ((dist1 + dist2 + dist3) / 3);
  return avedist90;
}

int accuracy60(){
  lcd.clear();
  delay(10);
  servo8Move(90, 60, 10);
  delay(1000);
  surveillance();
  int dist1 = distanceCm;
  delay(1000);
  lcd.clear();
  surveillance();
  int dist2 = distanceCm;
  delay(1000);
  lcd.clear();
  surveillance();
  int dist3 = distanceCm;
  delay(15);
  avedist60 = ((dist1 + dist2 + dist3) / 3);
  return avedist60;
}

int accuracy120(){
  lcd.clear();
  delay(10);
  servo8Move(50, 120, 10);
  delay(1000);
  surveillance();
  int dist1 = distanceCm;
  delay(1000);
  lcd.clear();
  surveillance();
  int dist2 = distanceCm;
  delay(1000);
  lcd.clear();
  surveillance();
  int dist3 = distanceCm;
  delay(15);
  avedist120 = ((dist1 + dist2 + dist3) / 3);
  return avedist120;
}

void SURVEILLANCE(int limitDist, int driveDelay){ 
while(Serial.available() < 2){
 
            accuracy90();
            int AD90 = avedist90;
            delay(1000);
            accuracy60();
            int AD60 = avedist60;
            delay(1000);
            accuracy120();
            int AD120 = avedist120;
            delay(1000);
            servo8Move(120, 90, 10);
            delay(1000);
            lcd.clear();
            delay(20);
            lcdmain();
    
            if ((AD90 < limitDist) && (AD60 < limitDist) && (AD120 < limitDist)){
            reverseMove(50, 15); 
                if(AD120 > AD60){
                  anticlockwiseMove(90, 15);
                  delay(driveDelay);
                }
                if (AD120 < AD60){
                  clockwiseMove(90, 15);
                  delay(driveDelay);
                }
            }
           
            if ((AD90 > limitDist) && (AD60 > limitDist) && (AD120 > limitDist)){
            forwardMove(80, 15);
            delay(driveDelay);
                if(AD120 > AD60){
                  anticlockwiseMove(90, 15);
                  delay(driveDelay);
                }
                if (AD120 < AD60){
                  clockwiseMove(90, 15);
                  delay(driveDelay);
                }
            }
            
            if ((AD90 < limitDist) && (AD120 < limitDist) && (AD60 > limitDist)){
            clockwiseMove(90, 15);
            delay(driveDelay);
            forwardMove(50, 15);
            delay(driveDelay);

            }

            if ((AD60 < limitDist) && (AD120 < limitDist) && (AD90 > limitDist)){
            forwardMove(50, 15);
              if (AD120 < AD60){
                  clockwiseMove(90, 15);
                  delay(driveDelay);
                }
               if (AD120 > AD60){
                  anticlockwiseMove(90, 15);
                  delay(driveDelay);
                }
          
            }

            if ((AD60 < limitDist) && (AD90 < limitDist) && (AD120 > limitDist)){
            anticlockwiseMove(90, 15);
            delay(driveDelay);
            forwardMove(50, 15);
            delay(driveDelay);
            }

            if ((AD90 > limitDist) && (AD120 > limitDist) && (AD60 < limitDist)){
              if(AD120 > AD90){
                anticlockwiseMove(90, 15);
                delay(driveDelay);
                forwardMove(50, 15);
                delay(driveDelay);
              }
              if (AD120 < AD90){
                forwardMove(50, 15);
                delay(driveDelay);
              }
              
            }
            if ((AD90 > limitDist) && (AD60 > limitDist) && (AD120 < limitDist)){
              if(AD60 > AD90){
                clockwiseMove(90, 15);
                delay(driveDelay);
                forwardMove(50, 15);
                delay(driveDelay);
              }
              if (AD60 < AD90){
                forwardMove(50, 15);
                delay(driveDelay);
              }
            }
            if ((AD60 > limitDist) && (AD120 > limitDist) && (AD90 < limitDist)){
              if(AD120 > AD60){
                anticlockwiseMove(90, 15);
                delay(driveDelay);
                forwardMove(50, 15);
                delay(driveDelay);
              }
              if (AD120 < AD60){
                clockwiseMove(90, 15);
                delay(driveDelay);
                forwardMove(50, 15);
                delay(driveDelay);
              }
            }
            if ((AD90 > 2000) || (AD120 > 2000) || (AD60 > 2000)){
            accuracy90();
            delay(1000);
            accuracy60();
            delay(1000);
            accuracy120();
            delay(1000);
            servo8Move(120, 90, 10);
            delay(1000);
            lcd.clear();
            delay(20);
            lcdmain();
            }
}
}

void SENSORDATA(){
  int chk = DHT.read11(DHT11_PIN);
  float temp = analogRead(tempPin);
  temp = (temp * 0.48828125);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print("C");
  delay(10);
  lcd.setCursor(0, 1); 
  lcd.print("Humidity: ");
  lcd.print((DHT.humidity) - 121);
  lcd.print("%");
}

void WATCHDOG(){
  /*I have left this function blank.  For watchdog mode several alarms could be triggered via the PIR sensor. 
  For example the power to mp3 module could be supplied and the rover could also move in a programmed manner if the alarm is triggered.*/
}



