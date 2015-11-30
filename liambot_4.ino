/* This example shows how to smoothly control a single servo on a
Maestro servo controller using the PololuMaestro library. It
assumes you have an RC hobby servo connected on channel 0 of your
Maestro, and that you have already used the Maestro Control
Center software to verify that the servo is powered correctly and
moves when you command it to from the Maestro Control Center
software.

Before using this example, you should go to the Serial Settings
tab in the Maestro Control Center and apply these settings:

* Serial mode: UART, fixed baud rate
* Baud rate: 9600
* CRC disabled

Be sure to click "Apply Settings" after making any changes.

This example also assumes you have connected your Arduino to your
Maestro appropriately. If you have not done so, please see
https://github.com/pololu/maestro-arduino for more details on how
to make the connection between your Arduino and your Maestro. */

#include <PololuMaestro.h>

/* On boards with a hardware serial port available for use, use
that port to communicate with the Maestro. For other boards,
create a SoftwareSerial object using pin 10 to receive (RX) and
pin 11 to transmit (TX). */
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(10, 11);
#endif

/* Next, create a Maestro object using the serial port.

Uncomment one of MicroMaestro or MiniMaestro below depending
on which one you have. */
MicroMaestro maestro(maestroSerial);
//MiniMaestro maestro(maestroSerial);


// leg adjustments variables
int thighSpeed=0;
int thighAccel=160;
int calfSpeed=0;
int calfAccel=120;

int rightCalfOffset=1500;
int leftCalfOffset=0;

// range=4000-7800
int thighMin=4000;
int thighMax=7800;
int thighStand=800;
int calfMin=4000;
int calfMax=7800;
int calfStand=1000;

bool leftOn=true;
bool rightOn=true;
int lastLeftCalf, lastRightCalf, lastLeftThigh, lastRightThigh, standRightCalf, standLeftCalf, standRightThigh, standLeftThigh=0;


// nunchuck init
#include <Wire.h>
#include <ArduinoNunchuk.h>
ArduinoNunchuk nunchuk = ArduinoNunchuk();
int thisX, thisY, thisZ=0;
int zeroX=519;
int zeroZ=380;
float drift=0.04;
int moveSpeed=20;

bool walking=false;
char lastStep='l';
unsigned long nextStepTime=0;
int stepMillis=1000;

void setup()
{
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  
  // nunchuck setup
  nunchuk.init();
  
  Serial.begin(9600);

  // start with legs in standing position
  if (rightOn){
    maestro.setSpeed(0, thighSpeed);
    maestro.setAcceleration(0,thighAccel);
    maestro.setSpeed(2, calfSpeed);
    maestro.setAcceleration(2,calfAccel);
    lastRightThigh=4000+thighStand;
    maestro.setTarget(0, lastRightThigh);
    lastRightCalf=4000+calfStand+rightCalfOffset;
    maestro.setTarget(2, lastRightCalf);
    standRightThigh=lastRightThigh;
    standRightCalf=lastRightCalf;
  }
  
  if (leftOn){
    maestro.setSpeed(1, thighSpeed);
    maestro.setAcceleration(1,thighAccel);
    maestro.setSpeed(3, calfSpeed);
    maestro.setAcceleration(3,calfAccel);
    lastLeftThigh=7800-thighStand;
    maestro.setTarget(1, lastLeftThigh);
    lastLeftCalf=4000+calfStand+leftCalfOffset;
    maestro.setTarget(3, lastLeftCalf);
    standLeftThigh=lastLeftThigh;
    standLeftCalf=lastLeftCalf;
  }
  
  delay(2000);
}

void loop()
{
  nunchuk.update();

  // joystick X and Y:
  // nunchuk.analogX
  // nunchuk.analogY
  // buttons
  // nunchuk.zButton
  // nunchuk.cButton  
  thisX=nunchuk.accelX;
  thisY=nunchuk.accelY;
  thisZ=nunchuk.accelZ;
  
  if (nunchuk.cButton==1){
    // turn on/off walking
    if (walking){
      walking=false;
    } else {
      walking=true;
      nextStepTime=stepMillis+millis();
    }
  }
 
  if (walking){   
    // take a quick step every two seconds
    if (millis()>nextStepTime){
      // time to take another step
      maestro.setSpeed(1, 0);
      maestro.setAcceleration(1, 0);
      maestro.setSpeed(3, 0);
      maestro.setAcceleration(3, 0);
      maestro.setSpeed(0, 0);
      maestro.setAcceleration(0, 0);
      maestro.setSpeed(2, 0);
      maestro.setAcceleration(2, 0);
  
      if (lastStep=='l'){
        lastStep='r';
        //maestro.setTarget(0, lastRightThigh-200);
        maestro.setTarget(2, lastRightCalf-300);
        delay(300);
        maestro.setAcceleration(2, 100);
        //maestro.setTarget(0, lastRightThigh);   
        maestro.setTarget(2, lastRightCalf);
        
      } else {
        lastStep='l';
        //maestro.setTarget(1, lastLeftThigh+200);
        maestro.setTarget(3, lastLeftCalf+300);
        delay(300);
        maestro.setAcceleration(3, 100);
        //maestro.setTarget(1, lastLeftThigh);
        maestro.setTarget(3, lastLeftCalf);
      }

      //maestro.setTarget(3, calfMax);
      //delay(300);
      //maestro.setTarget(3, calfMax);
      
      
      // return left thigh and calf to previous position


      
      nextStepTime=stepMillis+millis();
    }
  }
    
  if (true){
    Serial.print("x=");
    Serial.print(thisX);
    Serial.print("  y=");
    Serial.print(thisY);
    Serial.print("  z=");
    Serial.print(thisZ);
    Serial.println("");
  }
  

  
  /* setSpeed takes channel number you want to limit and the
  speed limit in units of (1/4 microseconds)/(10 milliseconds).

  A speed of 0 makes the speed unlimited, so that setting the
  target will immediately affect the position. Note that the
  actual speed at which your servo moves is also limited by the
  design of the servo itself, the supply voltage, and mechanical
  loads. */
  //maestro.setSpeed(0, 0);

  /* setAcceleration takes channel number you want to limit and
  the acceleration limit value from 0 to 255 in units of (1/4
  microseconds)/(10 milliseconds) / (80 milliseconds).

  A value of 0 corresponds to no acceleration limit. An
  acceleration limit causes the speed of a servo to slowly ramp
  up until it reaches the maximum speed, then to ramp down again
  as the position approaches the target, resulting in relatively
  smooth motion from one point to another. */
  //maestro.setAcceleration(0, 0);
  
  int mode=3;
  
  if (mode==0){
    // move thighs
    // Configure channel 0 to move slowly and smoothly.
    maestro.setSpeed(0, 50);
    maestro.setAcceleration(0,127);
    maestro.setSpeed(1, 50);
    maestro.setAcceleration(1,127);
  
    // Set the target of channel 0 to 1500 us, and wait 2 seconds.
    maestro.setTarget(0, 4000);
    maestro.setTarget(1, 4000);
    delay(1000);
  
    maestro.setTarget(0, 8000);
    maestro.setTarget(1, 8000);
    delay(1000);
  
    //maestro.setTarget(0, 6000);
    //maestro.setTarget(1, 6000);
    //delay(2000);
  }

  if (mode==1){
    // move calfs
    // Configure channel 0 to move slowly and smoothly.
    maestro.setSpeed(2, 50);
    maestro.setAcceleration(0,127);
    maestro.setSpeed(3, 50);
    maestro.setAcceleration(1,127);
  
    // Set the target of channel 0 to 1500 us, and wait 2 seconds.
    maestro.setTarget(2, 4000);
    maestro.setTarget(3, 4000);
    delay(1000);
  
    maestro.setTarget(2, 8000);
    maestro.setTarget(3, 8000);
    delay(1000);
  
    //maestro.setTarget(0, 6000);
    //maestro.setTarget(1, 6000);
    //delay(2000);
  }  

  if (mode==2){
    // move thighs and calfs
    

    
    if (rightOn){
      maestro.setSpeed(0, thighSpeed);
      maestro.setAcceleration(0,thighAccel);
      maestro.setSpeed(2, calfSpeed);
      maestro.setAcceleration(2,calfAccel);
      maestro.setTarget(2, calfMin);
    }
    
    if (leftOn){
      maestro.setSpeed(1, thighSpeed);
      maestro.setAcceleration(1,thighAccel);
      maestro.setSpeed(3, calfSpeed);
      maestro.setAcceleration(3,calfAccel);
      maestro.setTarget(3, calfMin);
    }
    
    delay(300);
    
    if (rightOn){
      maestro.setTarget(0, thighMin);
    }
    if (leftOn){
      maestro.setTarget(1, thighMin);
    }
    
    delay(500);
    
    if (rightOn){
      maestro.setTarget(2, calfMax+rightCalfOffset);
    }
    if (leftOn){
      maestro.setTarget(3, calfMax+leftCalfOffset);
    }

    //maestro.setTarget(1, 4000);
    //maestro.setTarget(3, 4000);
    delay(500);
  
    if (rightOn){
      maestro.setTarget(0, thighMax);
      maestro.setTarget(2, calfMax+rightCalfOffset);
    }
    if (leftOn){
      maestro.setTarget(1, thighMax);
      maestro.setTarget(3, calfMax+leftCalfOffset);
    }
    
    //maestro.setTarget(1, 8000);
    //maestro.setTarget(3, 8000);
    delay(500);
  
    //maestro.setTarget(0, 6000);
    //maestro.setTarget(1, 6000);
    //delay(2000);
  }  
  
  if (mode==3){
    // stand still
    if (false && thisX<(zeroX*(1.0-drift))){
      // lower left leg
      lastLeftThigh=lastLeftThigh+50;
      lastLeftCalf=lastLeftCalf+50;
      if (lastLeftThigh>thighMax) lastLeftThigh=thighMax;
      if (lastLeftCalf>calfMax) lastLeftCalf=calfMax;
      maestro.setTarget(1, lastLeftThigh);
      maestro.setTarget(3, lastLeftCalf);
      
      // raise right leg
      lastRightThigh=lastRightThigh+50;
      lastRightCalf=lastRightCalf+50;
      if (lastRightThigh>thighMax) lastRightThigh=thighMax;
      if (lastRightCalf>calfMax) lastRightCalf=calfMax;
      maestro.setTarget(0, lastRightThigh);
      maestro.setTarget(2, lastRightCalf);
      
    } else if(false && thisX>zeroX*(1.0+drift)){
      // raise left leg
      lastLeftThigh=lastLeftThigh-50;
      lastLeftCalf=lastLeftCalf-50;
      if (lastLeftThigh<thighMin) lastLeftThigh=thighMin;
      if (lastLeftCalf<calfMin) lastLeftCalf=calfMin;
      maestro.setTarget(1, lastLeftThigh);
      maestro.setTarget(3, lastLeftCalf);

      // lower right leg
      lastRightThigh=lastRightThigh-50;
      lastRightCalf=lastRightCalf-50;
      if (lastRightThigh<thighMin) lastRightThigh=thighMin;
      if (lastRightCalf<calfMin) lastRightCalf=calfMin;
      maestro.setTarget(0, lastRightThigh);
      maestro.setTarget(2, lastRightCalf);
      
    } else if(thisZ>zeroZ*(1.0+drift)){
      // leaning backward
      
      int thisMoveSpeed=moveSpeed;
      
      if (thisZ>zeroZ*(1.0+(drift*3))){
        // REALLY leaning backward
        //thisMoveSpeed=moveSpeed*3;
      }
      
      maestro.setSpeed(0, thighSpeed);
      maestro.setSpeed(1, thighSpeed);
      maestro.setAcceleration(0, thighAccel);
      maestro.setAcceleration(1, thighAccel);
      maestro.setSpeed(2, calfSpeed);
      maestro.setSpeed(3, calfSpeed);
      maestro.setAcceleration(2, calfAccel);
      maestro.setAcceleration(3, calfAccel);
      
      // lower right thigh
      lastRightThigh=lastRightThigh+thisMoveSpeed;
      if (lastRightThigh>thighMax) lastRightThigh=thighMax;
      
      // lower left thigh
      lastLeftThigh=lastLeftThigh-thisMoveSpeed;
      if (lastLeftThigh<thighMin) lastLeftThigh=thighMin;      

      // raise right calf
      lastRightCalf=lastRightCalf-thisMoveSpeed;
      if (lastRightCalf<calfMin) lastRightCalf=calfMin;
      
      // raise left calf
      lastLeftCalf=lastLeftCalf+thisMoveSpeed;
      if (lastLeftCalf>calfMax) lastLeftCalf=calfMax;
      
      maestro.setTarget(0, lastRightThigh);   
      maestro.setTarget(1, lastLeftThigh); 
      maestro.setTarget(2, lastRightCalf);   
      maestro.setTarget(3, lastLeftCalf);

    } else if (thisZ<(zeroZ*(1.0-drift))){      
      // leaning forward
      
      int thisMoveSpeed=moveSpeed;
      
      if (thisZ<zeroZ*(1.0+(drift*3))){
        // REALLY leaning
        //thisMoveSpeed=moveSpeed*3;
      }
      
      maestro.setSpeed(0, thighSpeed);
      maestro.setSpeed(1, thighSpeed);
      maestro.setAcceleration(0, thighAccel);
      maestro.setAcceleration(1, thighAccel);
      maestro.setSpeed(2, calfSpeed);
      maestro.setSpeed(3, calfSpeed);
      maestro.setAcceleration(2, calfAccel);
      maestro.setAcceleration(3, calfAccel);

      // raise right thigh
      lastRightThigh=lastRightThigh-thisMoveSpeed;
      if (lastRightThigh<thighMin) lastRightThigh=thighMin;
      
      // raise left thigh
      lastLeftThigh=lastLeftThigh+thisMoveSpeed;
      if (lastLeftThigh>thighMax) lastLeftThigh=thighMax;
      
      // lower right calf
      lastRightCalf=lastRightCalf+thisMoveSpeed;
      if (lastRightCalf>calfMax) lastRightCalf=calfMax;
      
      // lower left calf
      lastLeftCalf=lastLeftCalf-thisMoveSpeed;
      if (lastLeftCalf<calfMin) lastLeftCalf=calfMin;
      
      maestro.setTarget(0, lastRightThigh);   
      maestro.setTarget(1, lastLeftThigh); 
      maestro.setTarget(2, lastRightCalf);   
      maestro.setTarget(3, lastLeftCalf);
      
    } else {
      if (false) {
        // level, try to return to stand position
        if (lastRightThigh>standRightThigh) lastRightThigh=lastRightThigh-25;
        if (lastLeftThigh>standLeftThigh) lastLeftThigh=lastLeftThigh-25;
        if (lastRightCalf>standRightCalf) lastRightCalf=lastRightCalf-25;
        if (lastLeftCalf>standLeftCalf) lastLeftCalf=lastLeftCalf-25;
  
        maestro.setTarget(0, lastRightThigh);
        maestro.setTarget(1, lastLeftThigh);
        maestro.setTarget(2, lastRightCalf);
        maestro.setTarget(3, lastRightCalf);
      }
    }
  }
}
