#include <Arduino.h>

// Rotary encoder pins
#define PIN_A 20
#define PIN_B 19
#define PIN_DRIVE_LEFT 8
#define PIN_DRIVE_RIGHT 9
#define PIN_SPEED 10
#define SPEED 50

// A turn counter for the rotary encoder (negative = anti-clockwise)
int rotationCounter = 0;
int previousCount = 0;
int encoderTicksPerRevolution = 500;

// Flag from interrupt routine (moved=true)
volatile bool rotaryEncoder = false;

long interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
long RPM = 0;



// Interrupt routine just sets a flag when rotation is detected
void rotary()
{
    rotaryEncoder = true;
}


int8_t checkRotaryEncoder()
{
    // Reset the flag that brought us here (from ISR)
    rotaryEncoder = false;

    static uint8_t lrmem = 3;
    static int lrsum = 0;
    static int8_t TRANS[] = {0, -1, 1, 14, 1, 0, 14, -1, -1, 14, 0, 1, 14, 1, -1, 0};

    // Read BOTH pin states to deterimine validity of rotation (ie not just switch bounce)
    int8_t l = digitalRead(PIN_A);
    int8_t r = digitalRead(PIN_B);

    // Move previous value 2 bits to the left and add in our new values
    lrmem = ((lrmem & 0x03) << 2) + 2 * l + r;

    // Convert the bit pattern to a movement indicator (14 = impossible, ie switch bounce)
    lrsum += TRANS[lrmem];

    /* encoder not in the neutral (detent) state */
    if (lrsum % 4 != 0)
    {
        return 0;
    }

    /* encoder in the neutral state - clockwise rotation*/
    if (lrsum == 4)
    {
        lrsum = 0;
        return 1;
    }

    /* encoder in the neutral state - anti-clockwise rotation*/
    if (lrsum == -4)
    {
        lrsum = 0;
        return -1;
    }

    // An impossible rotation has been detected - ignore the movement
    lrsum = 0;
    return 0;
}

void setup()
{
    Serial.begin(115200);


    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_A), rotary, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), rotary, CHANGE);

    Serial.println("Setup completed");
}

void loop()
{

  digitalWrite(PIN_DRIVE_LEFT, HIGH);
  digitalWrite(PIN_DRIVE_RIGHT, LOW);
  analogWrite(PIN_SPEED, SPEED);
  delay(5000);

  currentMillis = millis();
    if(currentMillis - previousMillis > interval){
      long encoderTicksPerSecond = rotationCounter - previousCount;
      //Serial.print("RotationCounter:");
      //Serial.println(rotationCounter);
      //Serial.print("PreviousCount");
      //Serial.println(previousCount);
      RPM = (encoderTicksPerSecond*60)/(encoderTicksPerRevolution);
      //Serial.print("RPM:");
      Serial.println(RPM);
      previousCount = rotationCounter;
      previousMillis = currentMillis;
    }

  // Stop the motor for 2 second and send RPM
  digitalWrite(PIN_DRIVE_LEFT, LOW);
  digitalWrite(PIN_DRIVE_RIGHT, LOW);
  analogWrite(PIN_SPEED, 0);
  delay(2000);
  currentMillis = millis();
    if(currentMillis - previousMillis > interval){
      long encoderTicksPerSecond = rotationCounter - previousCount;
      //Serial.print("RotationCounter:");
      //Serial.println(rotationCounter);
      //Serial.print("PreviousCount");
      //Serial.println(previousCount);
      RPM = (encoderTicksPerSecond*60)/(encoderTicksPerRevolution);
      //Serial.print("RPM:");
      Serial.println(RPM);
      previousCount = rotationCounter;
      previousMillis = currentMillis;
    }

  // Run the motor in reverse direction for 5 seconds
  digitalWrite(PIN_DRIVE_LEFT, LOW);
  digitalWrite(PIN_DRIVE_RIGHT, HIGH);
  analogWrite(PIN_SPEED, SPEED);
  delay(5000);
  currentMillis = millis();
    if(currentMillis - previousMillis > interval){
      long encoderTicksPerSecond = rotationCounter - previousCount;
      //Serial.print("RotationCounter:");
      //Serial.println(rotationCounter);
      //Serial.print("PreviousCount");
      //Serial.println(previousCount);
      RPM = (encoderTicksPerSecond*60)/(encoderTicksPerRevolution);
      //Serial.print("RPM:");
      Serial.println(RPM);
      previousCount = rotationCounter;
      previousMillis = currentMillis;
    }

  // Stop the motor for 2 second
  digitalWrite(PIN_DRIVE_LEFT, LOW);
  digitalWrite(PIN_DRIVE_RIGHT, LOW);
  analogWrite(PIN_SPEED, 0);
  delay(2000);
    currentMillis = millis();
    if(currentMillis - previousMillis > interval){
      long encoderTicksPerSecond = rotationCounter - previousCount;
      //Serial.print("RotationCounter:");
      //Serial.println(rotationCounter);
      //Serial.print("PreviousCount");
      //Serial.println(previousCount);
      RPM = (encoderTicksPerSecond*60)/(encoderTicksPerRevolution);
      //Serial.print("RPM:");
      Serial.println(RPM);
      previousCount = rotationCounter;
      previousMillis = currentMillis;
    }
   

    // Has rotary encoder moved?
    if (rotaryEncoder)
    {
        // Get the movement (if valid)
        long rotationValue = checkRotaryEncoder();
        // If valid movement, do something
        if (rotationValue != 0)
          rotationCounter += rotationValue;
          //Serial.print(rotationValue < 1 ? 'L' : 'R');
          //Serial.println(rotationCounter);
    }
}

