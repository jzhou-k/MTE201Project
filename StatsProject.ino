#include <Stepper.h>

// Rotary Encoder Inputs
#define CLK_PIN 3 //pin number reading from encoder clk : signals when rotations occur
#define DT_PIN 4 // pin number determining direction 
#define ENCODER_CLICKS_PER_REV 20
#define BUTTON_PIN 12
#define DIRECTION_CW 0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction

// chang
// Rotary encoder constants
const int STEPS_PER_REVOLUTION = 2048;
const int START_DIST = 300; 
const int TOTAL_RACK_LENGTH = 1500; 
const int PITCH_MM = 2; 
const int TEETH_NUM = 18; 
const int MAX_MEASURMENT = 12; 
const int PULSES_PER_REV = 128; 

//Encoder variables
Stepper myStepper = Stepper(STEPS_PER_REVOLUTION, 8, 10, 9, 11);
int click_counter = 0;
int currentStateCLK;
int lastStateCLK;
int currentDir = DIRECTION_CW;
volatile int pulses = 0; 

/*
encoder count 1 / 128 rev * 18 * 2mm = distance traveld 
displacement = distance traveled * 100 - 3 
maxMeasurement = totalRackLength - startingCm 
objectLength = maxMeasurement - displacement  */

//Touch sensor
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9;  // Analog output pin that the LED is attached to

int sensorValue = 0;  // value read from the pot
int outputValue = 0;  // value output to the PWM (analog out)
int ini_sensorValue = 0;

// Calibration
unsigned long lastButtonPress = 0;
// TODO: initial dist?

void count2() {
  /*Reads encoder pulses, 
    Assumes clockwise is the positive direction when counting pulses
  */
  currentStateCLK = digitalRead(CLK_PIN);

  //If the state has changed, it indicates that it has finished a revolution
  if ( currentStateCLK != lastStateCLK &&  currentStateCLK == HIGH) {
    if (digitalRead(DT_PIN) == HIGH) {
      click_counter--;
      currentDir = DIRECTION_CCW;
    } else {
      // the encoder is rotating in clockwise direction => increase the counter
      click_counter++;
      currentDir = DIRECTION_CW;
    }

    Serial.print("DIRECTION: ");
    if (currentDir == DIRECTION_CW)
      Serial.print("Clockwise");
      Serial.println(click_counter);
    else
      Serial.print("Counter-clockwise");
      Serial.println(click_counter);
  }
  // save last CLK state
  lastStateCLK = currentStateCLK;
}



void count_encoder_by_interrupt() { 
  if (digitalRead(CLK)) { 
    pulses++; 
  } else { 
    pulses--; 
  }
}

// returns distance rack traveled by mm 
unsigned long calculateRackDistance(int pulse) { 
  return ( (float)pulse / (float)pulseperturn ) * 59.69 ; 
}

int calculateDisplacement(int pulse){ 
  return totalRackLength - calculateRackDistance(pulse); 
}

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CLK_PIN, INPUT); 
  pinMode(DT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(0, count_encoder_by_interrupt, RISING); 
  Serial.begin(9600);

  lastStateCLK = digitalRead(CLK_PIN);
  myStepper.setSpeed(10);
}


void measure() { 
    myStepper.step(8);
    Serial.print("pulses = "); 
    Serial.println(pulses); 
    Serial.print("rack distance traveled = "); 
    Serial.println(calculateRackDistance(pulses)); 

    delay(1000);
}


bool pressureDetected() { 
  sensorValue = analogRead(analogInPin);
  return (sensorValue > 5); 
}


// the loop function runs over and over again forever
void loop() {
  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)

  digitalWrite(LED_BUILTIN, HIGH); // LED on, motor rotating backwards 
  while (digitalRead(BUTTON_PIN)) {
    myStepper.step(-12);
    delay(10);
  }
 
  digitalWrite(LED_BUILTIN, LOW); //LED off, motor rotating forwards 

  pulses = 0;  
  while (!pressureDetected()) {
    measure(); 
    count2();
  }

  Serial.println("------------------------------------")  
  Serial.println("MEASURING FINISHED") 
  Serial.println("LENGTH OF THE OBJECT IS: ")
  Serial.print(objectLength); 

  Serial.println("PRESS BUTTON AGAIN TO REVERT THE RACK AND MEASURE AGAIN") 
  while (digitalRead(BUTTON_PIN)) { 
      //waiting for user button press 
  }
}




