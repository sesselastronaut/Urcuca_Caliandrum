//****************************************************************
/*Code for uruca caliandrum - Bürstenspinner Robot
by olsen sesselastronaut@hasa-labs.org
------------------------------------------------
Arduino CPP Code HMC6343
from http://www.sparkfun.com/commerce/product_info.php?products_id=8656
------------------------------------------------
Watchdog/Sleep codesnippets from Martin Nawraths nawrath@khm.de example:
http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
------------------------------------------------
*/
//****************************************************************
#include <Servo.h> 
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif
#include <Wire.h>
#include "HMC6343.h"

#include <avr/sleep.h>
#include <avr/wdt.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//watchdog/sleep_______________________________________________
int sleepCycle;
int pinLed=13;
volatile boolean f_wdt=1;

byte del;
int cnt;
byte state=0; //state 1 to 0 day; state 0 to 1 night
int light=0;


//servos_______________________________________________________
Servo myservo1;   // 1.Servo for Travelling wave 
Servo myservo2;   // 2.Servo for Travelling wave 
Servo myservo3;	  // 3.Servo for Travelling wave
int A_0 = 105;      //define middle position of servo
int A_delta = 25;       //servo deflection around A_0
//float Pi = 3.1415; // sine wave parameters
int p1 = 0;
int p2 = 0;
int p3 = 0;
int pos = 0;     
float omega = 4;
float t;         //time
unsigned long startOfLoop;
unsigned long endOfLoop;
int delayTime = 15;

char keyinput = 0; //keyboard input
enum robotstate { on, off, wait }; //robot states
robotstate roboState;

int lightThresholdNight = 175; //579;  //light threshold for Night & Day
int lightThresholdDay = 200; //650;

int averageCompassangle;  //compass parameters
int compassWindow[3] ;
int eastValue = 155;  //value where east is located

//****************************************************************
void setup() { 
  Serial.begin(38400);
  pinMode(pinLed,OUTPUT);
  Serial.println("start navigation");

  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode

  setup_watchdog(7);  
  
  myservo1.attach(9);   //Servo on pwm pins()
  myservo2.attach(6);
  myservo3.attach(11);
  Wire.begin();  
  t = 0;  
  state = on;
  pos = A_0 + A_delta; //Set Servo to default pos
  //Serial.println(pos);
  myservo1.write(pos);
  myservo2.write(pos);
  myservo3.write(pos);
  
  //set compassWindow to 0 
  compassWindow[0] = 0;
  compassWindow[1] = 0;
  compassWindow[2] = 0;
} 

//****************************************************************
void RemoteControl(){ //read keyboard input
  if (Serial.available()){
    keyinput = Serial.read(); 
    switch (keyinput) {
      case 115: //s key input, pause the robot
        state = wait; 
        Serial.println("s-key");
        Serial.println(state);
        break;
      case 97: //a key input, move the robot
        state = on;
        Serial.println("a-key");
        break;
      case 50: //2 key input, change omega
        omega = 2;
        Serial.println("2-key");
        break;
      case 51: //3 key input, change omega
        omega = 3;
        Serial.println("3-key");
        break;
      case 52: //4 key input, change omega
        omega = 4;
        Serial.println("4-key");
        break;
      case 53: //5 key input, change omega
        omega = 5;
        Serial.println("5-key");
        break;
    }
  }
  else
    keyinput = 0; 
}

//****************************************************************
//read Angle from Compass Module HMC6343
int ReadAngleHMC6343::getAngle() {
  byte headingData[6];
  Wire.beginTransmission(ReadAngleHMC6343::i2cAddress);
  Wire.write(0x50);
  Wire.endTransmission();
  delay(2);
  Wire.requestFrom(ReadAngleHMC6343::i2cAddress, 6);
  int i = 0;
  while(Wire.available() && i < 6)
  {
    headingData[i] = Wire.read();
    i++;
  }
  return (headingData[0]*256 + headingData[1])/10;
}

//****************************************************************  
// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() {
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
  sleep_disable();                   // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                  // switch Analog to Digitalconverter ON
}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  Serial.print("ww= ");
  Serial.println(ww);

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}

//****************************************************************  
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}

//**************************************************************** 
//move the brush towards east
void moveTowardsSun() { 
    do{
      //start measuring time
      startOfLoop = millis(); //measure time at start of loop
        //calculate servos position
        p1 = A_0 + A_delta * sin(omega * t);
        //p2 = A_0 + A_delta * sin(omega * t + 0.33 * 2 * Pi);
        //reverse 2.servo because it's turning the other way round
        p2 = A_0 + (A_delta * sin(omega * t + 0.33 * 2 * PI))*-1;
        p3 = A_0 + A_delta * sin(omega * t + 0.66 * 2 * PI);
         
        //Remote control, read from keyboard input
        RemoteControl();    
    
        //read sensor values_____________________________________________________________
        //Read Compass position & apply moving window filter to it from the last readings
        compassWindow[0] = compassWindow[1];
        compassWindow[1] = compassWindow[2];
        compassWindow[2] = ReadAngleHMC6343::getAngle();
        averageCompassangle = (compassWindow[0] + compassWindow[1] + compassWindow[2] )/3;
        Serial.print( "averageCompassangle: " );
        Serial.println( averageCompassangle );
        roboState = on; //set robot state
        
        //action - move brush;
        switch (state){
          case on:
            myservo1.write(p1);    
            myservo2.write(p2);
            myservo3.write(p3);
            break;
          case off:
            break;
        }
      //check running time and delay for a bit
      endOfLoop = millis(); //measure time at end of loop
      if ((endOfLoop-startOfLoop) > 10) { //as millis() resets after 50days running make shure the reset causes no problems
          delay (delayTime); 
      }
      else {
          delay(delayTime -(endOfLoop-startOfLoop)); // waits 15ms for the servo to reach the position 
          //Serial.println("endOfLoop-startOfLoop:");
          //Serial.println(endOfLoop-startOfLoop);
      }
     t = t+0.01; 
     }
     //when brush/averageCompassangle is in range of sunrise exit loop & return to main 
   while(averageCompassangle > eastValue+4 || averageCompassangle < eastValue-4); //east = 155
     roboState = off;
     Serial.println( "here comes the sun" );
     myservo1.write(A_0 + A_delta);    
     myservo2.write(A_0 - A_delta);
     myservo3.write(A_0 + A_delta);
}   

//****************************************************************
//****************************************************************
//****************************************************************
void loop() {
  if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt=0;       // reset flag

    light=analogRead(1);  // reading solarcell
   
    Serial.print("light: " );
    Serial.print(light); 
    Serial.print("\t");
    Serial.print("State: " );
    Serial.print((int)state);
    Serial.print("\t");
    switch (state){
    case 0:     // waiting for night/dusk
      if (light < lightThresholdNight) {  // light threshold for night
        state=1; //night is coming put state to night
      }
      break;
    case 1:  // waiting for day/dawn
      if (light > lightThresholdDay) { // light threshold
        state=0; //day is coming put state to day & move towards sun
        moveTowardsSun();
      }
      break;
    }
    sleepCycle++;
    digitalWrite(pinLed,1);  // let led blink
    Serial.print("Sleepcycle: " );
    Serial.println(sleepCycle );
    delay(2);               // wait until the last serial character is send
    digitalWrite(pinLed,0);
    system_sleep();
  }

}

