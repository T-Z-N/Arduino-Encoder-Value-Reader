#include <Arduino.h>

// Rotary encoder pins
#define PIN_A 20
#define PIN_B 19

// A turn counter for the rotary encoder (negative = anti-clockwise)
int rotationCounter = 0;

// Flag from interrupt routine (moved=true)
volatile bool rotaryEncoder = false;

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


    if (Serial.available()) {
        byte nr = Serial.read();
        Serial.print("The following char was received: ");
        Serial.println(nr, DEC);
    }
    
    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_A), rotary, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), rotary, CHANGE);

    Serial.println("Setup completed");
}

void loop()
{
    // Has rotary encoder moved?
    if (rotaryEncoder)
    {
        // Get the movement (if valid)
        int8_t rotationValue = checkRotaryEncoder();

        // If valid movement, do something
        if (rotationValue != 0)
        {
            rotationCounter += rotationValue;
            Serial.print(rotationValue < 1 ? "L" :  "R");
            Serial.println(rotationCounter);
        }
    }

    
}
