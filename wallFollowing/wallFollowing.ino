#include <Zumo32U4.h>
#include <Zumo32U4Motors.h>
#include <Zumo32U4Encoders.h>
#include <Wire.h>
#include "button.h"       //include your button class from last week
#include "eventTimer.h"  //include your shiny, new event timer class
#include "Segment.h"

#include "params.h"
#include "serial_comm.h"

Button buttonA(17); //button A is pin 14 on the Zumo
eventTimer timer;   //assumes you named your class EventTimer

//use the Pololu libraries for motors and encoders
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

//US stuff
volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;

//define the states for the echo capture
enum PULSE_STATE {PLS_IDLE, PLS_WAITING_LOW, PLS_WAITING_HIGH, PLS_CAPTURED};

//and initialize to IDLE
volatile PULSE_STATE pulseState = PLS_IDLE;
//this may be any appropriate pin, connect the pin to Trig on the sensor
const uint8_t trigPin = 14;

//for scheduling pings
uint32_t lastPing = 0;
uint32_t PING_INTERVAL = 17; //ms


//declare the robot states here
enum MOTOR_STATE  {IDLE, DRIVING, SHOWING_OFF };
MOTOR_STATE state = IDLE;

eventTimer myTimer;//Declare timer

//PID Stuff
volatile uint8_t readyToPID = 0;

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

static int16_t sumLeft = 0;
static int16_t sumRight = 0;

//linesensorStuff
#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];

//bool useEmitters = true;

uint8_t selectedSensorIndex = 0;


void setup()
{
  lineSensors.initFiveSensors();
  Serial.begin(115200);
  Serial.println("Hello.");

  iSeg = 0;
  countLimit += segments[iSeg].dist * 65;

  noInterrupts(); //disable interupts while we mess with the Timer4 registers

  //sets up timer 4
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at one-half the timer frequency
  TCCR4D = 0x00; //normal mode

  OCR4C = 132;   //TOP goes in OCR4C
  TIMSK4 = 0x04; //enable overflow interrupt

  interrupts(); //re-enable interrupts

  noInterrupts(); //disable interupts while we mess with the control registers

  //sets timer 3 to normal mode (16-bit, fast counter)
  TCCR3A = 0;

  interrupts(); //re-enable interrupts

  pinMode(trigPin, OUTPUT);
  pinMode(13, INPUT); //explicitly make 13 an input, since it defaults to OUTPUT in Arduino World (LED)

  lastPing = millis();

  buttonA.Init(); //don't forget to call Init()!
}

void loop()
{
  switch (state) {
    case IDLE:
      // delay(100);
      motors.setSpeeds(0, 0);

      if (buttonA.CheckButtonPress()) {
        Serial.println("here");
        myTimer.start(1000);
        while (!(myTimer.checkExpired())) {}
        state = DRIVING;
      }


      break;

    case DRIVING:
      lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
      currentDist = distanceFromWall();
      offset = calcTargets(currentDist);

      leftTarget = baseLeft + offset;
      rightTarget = baseRight - offset;

       if (lineSensorValues[0] < 1200 && lineSensorValues[4] < 1200) {
      PID();
     } else state = SHOWING_OFF;
    break;

    case SHOWING_OFF:
      myTimer.start(1000);
      while (!(myTimer.checkExpired())) {
        rightTarget = 30;
        leftTarget = -30;
        PID();
      }
      state = IDLE;
      break;
  }
}

float distanceFromWall() {
  //schedule pings roughly every PING_INTERVAL milliseconds
  if (millis() - lastPing > PING_INTERVAL)
  {
    lastPing = millis();
    CommandPing(trigPin); //command a ping
  }

  if (pulseState == PLS_CAPTURED) //we got an echo
  {
    //update the state to IDLE
    pulseState = PLS_IDLE;

    /*
       Calculate the length of the pulse (in timer counts!). Note that we turn off
       interrupts for a VERY short period so that there is no risk of the ISR changing
       pulseEnd or pulseStart. As noted in class, the way the state machine works, this
       isn't a problem, but best practice is to ensure that no side effects can occur.
    */
    noInterrupts();
    uint16_t pulseLengthTimerCounts = pulseEnd - pulseStart;
    interrupts();

    //EDIT THIS LINE: convert pulseLengthTimerCounts, which is in timer counts, to time, in us
    //You'll need the clock frequency and the pre-scaler to convert timer counts to time
    //prescalers/sysclock * ticks
    uint32_t pulseLengthUS = (64 / 16) * pulseLengthTimerCounts; //pulse length in us


    //EDIT THIS LINE AFTER YOU CALIBRATE THE SENSOR
    float distancePulse = (pulseLengthUS + 5.5107) / 58.215;  //distance in cm

    return distancePulse;

  }

}

void CommandPing(int trigPin)
{
  cli(); //disable interrupts

  TIFR3 = 0x20; //clear any interrupt flag that might be there

  TIMSK3 |= 0x20; //enable the input capture interrupt
  TCCR3B |= 0xC0; //set to capture the rising edge on pin 13; enable noise cancel

  sei(); //re-enable interrupts

  //update the state and command a ping
  pulseState = PLS_WAITING_LOW;

  digitalWrite(trigPin, HIGH); //command a ping by bringing TRIG HIGH
  delayMicroseconds(10);      //we'll allow a delay here for convenience; it's only 10 us
  digitalWrite(trigPin, LOW);  //must bring the TRIG pin back LOW to get it to send a ping
}

ISR(TIMER3_CAPT_vect)
{
  if (pulseState == PLS_WAITING_LOW) //we're waiting for a rising edge
  {
    pulseStart = ICR3; //copy the input capture register (timer count)
    TCCR3B &= 0xBF;    //now set to capture falling edge on pin 13
    pulseState = PLS_WAITING_HIGH;
  }

  else if (pulseState == PLS_WAITING_HIGH) //waiting for the falling edge
  {
    pulseEnd = ICR3;
    pulseState = PLS_CAPTURED; //raise a flag to indicate that we have data
  }
}

int calcTargets(float currDist) {
  if (readyToPID) //timer flag set
  {
    //clear the timer flag
    readyToPID = 0;
    float distError = targetDist - currDist;

    distSum -= errorCount[errorIndex];
    distSum += distError;
    errorCount[errorIndex] = distError;
    if (errorIndex == 99) errorIndex = 0;
    else errorIndex++;
    
    return distKp * distError + distKi * distSum;

  }
}

void PID() {
  Serial.println("got to PID");
  if (readyToPID) //timer flag set
  {
    Serial.println("got IN");
    //clear the timer flag
    readyToPID = 0;

    //for tracking previous counts
    static int16_t prevLeft = 0;
    static int16_t prevRight = 0;

    //error sum
    sumLeft = 0;
    sumRight = 0;

    /*
       Do PID stuffs here. Note that we turn off interupts while we read countsLeft/Right
       so that it won't get accidentally updated (in the ISR) while we're reading it.
    */
    noInterrupts();
    int16_t speedLeft = countsLeft - prevLeft;
    int16_t speedRight = countsRight - prevRight;

    prevLeft = countsLeft;
    prevRight = countsRight;
    interrupts();

    int16_t errorLeft = leftTarget - speedLeft;
    int16_t errorRight = rightTarget - speedRight;



    if (bufferIndexRight < 99) {
      bufferCountRight[bufferIndexRight] = errorLeft;
      bufferIndexRight++;
    } else bufferIndexRight = 0;

    for (int i = 0; i < 99; i++) {
      sumRight += bufferCountRight[i];
    }

    if (sumRight > 170) {
      sumRight = 170;
    }
    else if (sumRight < -135) {
      sumRight = -135;
    }


    if (bufferIndexLeft < 99) {
      bufferCountLeft[bufferIndexLeft] = errorLeft;
      bufferIndexLeft++;
    } else bufferIndexLeft = 0;


    for (int i = 0; i < 99; i++) {
      sumLeft += bufferCountLeft[i];
    }

    if (sumLeft > 170) {
      sumLeft = 170;
    }
    else if (sumLeft < -135) {
      sumLeft = -135;
    }


    float effortLeft = Kp * errorLeft + Ki * sumLeft;
    float effortRight = Kp * errorRight + Ki * sumRight;


    motors.setSpeeds(effortLeft, effortRight);

    Serial.print(0);
    Serial.print('\t');
    Serial.print(effortLeft);
    Serial.print('\t');
    Serial.print(effortRight);
    Serial.print('\n');

  }
}


//add you handlers here:
//handle the button press
bool handleButtonPress(Button button) {
  return button.CheckButtonPress();
}

//handle the timer expiring
bool handleTimerExp(eventTimer timer) {
  return timer.checkExpired();
}

bool handleTimerTransition(eventTimer timer) {
  Serial.println(timer.getRunning());
  return timer.getRunning();
}

ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = 1;
}
