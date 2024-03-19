#define CLK_PIN 3  //pin number reading from encoder clk : signals when rotations occur
#define BUTTON_PIN 7; // the number of the pushbutton pin
const int PULSES_PER_REV = 128;
volatile int pulses = 0;
volatile long temp, counter = 0;  //This variable will increase or decrease depending on the rotation of encoder
int a, b;

float calculateRackDistance(int pulse) {
  return ( (float)pulse / (float)PULSES_PER_REV ) * 51 ;
}

float calculateDimension(int pulse) {
  float traveled = calculateRackDistance(pulse);
  return  100 - traveled;
}

// Variables will change:
int lastState = HIGH; // the previous state from the input pin
int currentState;    // the current reading from the input pin

void setup() {
  pinMode(2, INPUT_PULLUP);  // internal pullup input pin 2
  pinMode(3, INPUT_PULLUP);  // internalเป็น pullup input pin 3
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt((0), ai0, RISING);
  attachInterrupt((1), ai1, RISING);
  
  Serial.begin(9600);
}

void loop() {
  counter = 0;
  currentState = digitalRead(BUTTON_PIN);

  while (digitalRead(BUTTON_PIN)) {
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
}


void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
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
