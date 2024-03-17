#include <Stepper.h>

#define CLK_PIN 3  //pin number reading from encoder clk : signals when rotations occur
//#define DT_PIN 4   // pin number determining direction
//#define BUTTON_PIN 4
const int PULSES_PER_REV = 128;
volatile int pulses = 0;

const int fsrPin = 0;  // the FSR and 10K pulldown are connected to a0
int fsrReading;        // the analog reading from the FSR resistor divider
int fsrReadingAna;

const int stepsPerRevolution = 64;  // change this to fit the number of steps per revolution
// for your motor
volatile long temp, counter = 0;  //This variable will increase or decrease depending on the rotation of encoder

int a, b;
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void count_encoder_by_interrupt() {
  if (digitalRead(CLK_PIN)) {
    pulses++;
  }
}

float calculateRackDistance(int pulse) {
  //return ((float)pulse / (float)23.2) * 10;
  return ( (float)pulse / (float)PULSES_PER_REV ) * 51 ;
}

float calculateDimension(int pulse) {
  float traveled = calculateRackDistance(pulse);
  return  100 - traveled;
}

int min_value_ana = 0;
int max_value_ana = 0;
int max_value;
int tolerance_val = 1;

// constants won't change. They're used here to set pin numbers:
const int BUTTON_PIN = 7; // the number of the pushbutton pin

// Variables will change:
int lastState = HIGH; // the previous state from the input pin
int currentState;    // the current reading from the input pin



void setup() {

  pinMode(2, INPUT_PULLUP);  // internal pullup input pin 2
  pinMode(3, INPUT_PULLUP);  // internalเป็น pullup input pin 3
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt((0), ai0, RISING);

  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt((1), ai1, RISING);
  // set the speed at 60 rpm:
  myStepper.setSpeed(30);
  // initialize the serial port:
  Serial.begin(9600);
}

int min_value;


void loop() {
  // step one revolution  in one direction:

  max_value_ana = analogRead(fsrPin);

  // Serial.println(max_value_ana);
  // Serial.println("adjust motor");
  // delay(5000);
  //currentState = digitalRead(BUTTON_PIN);

  counter = 0;
  //min_value = fsrReading = map(min_value_ana, 0, 1023, 0, 255);
  currentState = digitalRead(BUTTON_PIN);

  if(lastState == LOW && currentState == HIGH)
    Serial.println("The state changed from LOW to HIGH");
  fsrReadingAna = 0;
  while (digitalRead(BUTTON_PIN)) {
    //Serial.println("clockwise");
    //myStepper.step(12);

    // fsrReadingAna = analogRead(fsrPin);
    // //fsrReading = map(fsrReadingAna, 0, 1023, 0, 255);
    // Serial.print("Analog reading = ");
    // Serial.print(fsrReadingAna);     // print the raw analog reading

    // if (fsrReadingAna <= max_value_ana + tolerance_val) {
    //   Serial.println(" - No pressure");
    //   Serial.println("max value = ");
    //   Serial.print(max_value_ana);
    // } else {
    //   Serial.println(" - Big squeeze");
    // }
    if (counter != temp) {
      Serial.println(counter);
      temp = counter;
    }
    Serial.print("Distance: ");
    Serial.print(calculateDimension(counter));
    Serial.print("   Encoder: ");
    Serial.println(counter);
    delay(50);
  }

  Serial.println("------------------------------------");
  Serial.println("MEASURING FINISHED");
  Serial.print("LENGTH OF THE OBJECT IS: ");
  Serial.print(calculateDimension(counter));
  Serial.print("mm");
  delay(5000);
  lastState = currentState;

  // delay(10);
}



void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  // (a == HIGH && a == b) || (a == LOW && a == b)
  //
  a = digitalRead(3);
  b = digitalRead(2);
  if ((a == HIGH && a == b) || (a == LOW && a == b)) {
    counter++;
  } else {
    //counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  a = digitalRead(3);
  b = digitalRead(2);
  if (a != b) {
    //counter--;
  } else {
    counter++;
  }
}
