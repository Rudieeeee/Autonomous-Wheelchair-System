#include <Arduino.h>

int ledPin = 13;                // choose the pin for the LED
int inputPin = 2;               // choose the input pin
int val = 0;                    // variable for reading the pin status
void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);      // declare LED as output
  pinMode(inputPin, INPUT);     // declare pushbutton as input
}
void loop(){
  val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
    digitalWrite(LED_BUILTIN, LOW);  // turn LED OFF
  } else {
    digitalWrite(LED_BUILTIN, HIGH); // turn LED ON
  }
}