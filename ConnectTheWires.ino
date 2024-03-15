#include <LiquidCrystal_I2C.h>

#define DEBUG

//Const

// Gesamtanzahl der Sockets
const byte numSockets = 4;

//array mit Pins der Sockets         (0   1   2   3) <- connections
const byte signalPins[numSockets] = {8, 9, 10, 11 };

//array mit Pins der zugehörigen grünen LED
const byte greenLedPins[numSockets] = {40, 41, 42, 43};

//array mit Pins der zugehörigen roten LED
const byte redLedPins[numSockets] = {30, 31, 32, 33};

//Gesamtanzahl der Verbindungen / Kabel
const byte numConnections = 2;

//Socketpaare Lösung
const byte connections[numConnections][2] = {{0,1}, {2,3}};

//Globale Variablen
// Richtig verbundene Pins
bool lastState[numConnections] = {false, false};

//Rätselfortschritt
enum PuzzleState {Initialising, Running, Solved};
PuzzleState puzzleState = Initialising;



LiquidCrystal_I2C lcd(0x3F, 16, 2);  // I2C address 0x27, 16 column and 2 rows

void setup() {
  lcd.init();
  lcd.backlight();
  
  // Initialise the pins. When wires are disconnected, the input pins would be // floating and digitalRead would be unpredictable, so we'll initialise them 
  // as INPUT_PULLUP and use the wires to connect them to ground.
  // The default state of all pins will be INPUT_PULLUP, though they will be reassigned throughout the duration of the program in order
  // We also initialise all the LED pins as ouput  
  for (int i=0; i<numSockets; i++) {
    pinMode (signalPins[i], INPUT_PULLUP);
    pinMode (greenLedPins[i], OUTPUT);
    pinMode (redLedPins[i], OUTPUT); 
  } 
  // Serial connection used only for monitoring / debugging 
  #ifdef DEBUG 
  Serial.begin(9600); 
  Serial.println(F("Serial communication started"));
  Serial.println(lastState[0] + "ohh yeah");
  #endif
  puzzleState = Running;
}



void loop() {

  // Assume that all wires are correct 
  bool AllWiresCorrect = true;
 
  // Assume that the puzzle state has not changed since last reading 
  bool stateChanged = false;
 
  // Check each connection in turn 
  for (int i=0; i<numConnections; i++) {
 
    // Get the pin numbers that should be connected by this wire 
    byte pin1 = signalPins [connections [i][1]];
    byte pin2 = signalPins [connections [i][0]]; 
 
    // Test whether the appropriate pins are correctly connected 
    bool currentState = isConnected(pin1, pin2);
 
    // Has the connection state changed since last reading? 
    if (currentState != lastState[i]) {
 
      // Light the LEDs at both ends of this connection 
      digitalWrite(greenLedPins[connections[i][0]], currentState); 
      digitalWrite(greenLedPins[connections[i][1]], currentState);

      // Set the flag to show the state of the puzzle has changed
      stateChanged = true;

      // Update the stored connection state
      lastState[i] = currentState;
    }

    // If any connection is incorrect, puzzle is not solved
    if(currentState == false) {
      AllWiresCorrect = false;
    }
  }
  checkForIncorrectConnections();

  // If a connection has been made/broken since last time we checked
  if(stateChanged) {
    #ifdef DEBUG
      //Dump to serial the current state of all connections
      for(uint8_t i=0; i<numConnections; i++) {
        Serial.print(F(" Pin#"));
        Serial.print(signalPins[connections[i][0]]);
        Serial.print(F(" - Pin#"));
        Serial.print(signalPins[connections[i][1]]);
        Serial.print(F(" : "));
        Serial.println(lastState[i] ? "CONNECTED" : "NOT CONNECTED");
      }
      Serial.println(F("---"));
    #endif
  }
  // If the state of the puzzle has changed
  if(AllWiresCorrect && puzzleState == Running) {
    onSolve();
  }
  // If the state of the puzzle has changed
  else if(!AllWiresCorrect && puzzleState == Solved) {
    onUnsolve();
  }
  delay(0);
} // loop end













// Called when the puzzle is solved
void onSolve(){
  #ifdef DEBUG
    Serial.println(F("Das Rätsel wurde gelöst!"));
  #endif  
  puzzleState = Solved;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" - 4157~~~~~~");
}

// Called when the previously solved puzzle becomes unsolved
void onUnsolve(){
  #ifdef DEBUG
    Serial.println(F("The puzzle is no longer solved!"));
    lcd.clear();
  #endif
  puzzleState = Running;
}



// Tests wheter an output pin is connected to a given INPUT_PULLUP pin
bool isConnected (byte OutputPin, byte InputPin) {
 
  // To test whether the pins are connected, set the first as output and the second as input 
  pinMode (OutputPin, OUTPUT);
 
  pinMode (InputPin, INPUT_PULLUP);
 
  // Set the output pin LOW 
  digitalWrite (OutputPin, LOW);
 
  // If connected, the LOW signal should be detected on the input pin
 
  // (Remember, we're using LOW not HIGH, because an INPUT PULLUP will read HIGH by default) 
  bool isConnected = !digitalRead(InputPin);
 
  // Set the output pin back to its default state 
  pinMode (OutputPin, INPUT_PULLUP);

  return isConnected;
}


void checkForIncorrectConnections() {
  for (byte i = 0; i < numSockets; i++) {
    pinMode(signalPins[i], OUTPUT);
    digitalWrite(signalPins[i], LOW); // Set the current pin LOW

    // Check all other pins to see if any are incorrectly connected to this pin
    for (byte j = 0; j < numSockets; j++) {
      if ((i == 1 && j == 0) || (i == 0 && j == 1) || (i == 2 && j == 3) || (i == 3 && j == 2) || j == i) {
        // Skip checking the pin against itself and the corresponding correct pin
      } else {
        pinMode(signalPins[j], INPUT_PULLUP);
        bool connectionState = (digitalRead(signalPins[j]) == LOW);
        if (connectionState) {
          // Incorrect connection detected, turn on the red LED
          digitalWrite(redLedPins[i], HIGH);
          digitalWrite(redLedPins[j], HIGH);

        } else {
          // No incorrect connection detected, turn off the red LED
          digitalWrite(redLedPins[i], LOW);
          digitalWrite(redLedPins[j], LOW);
        }

        #ifdef DEBUG
        if (connectionState) {
          Serial.print(F("Incorrect connection detected between pins "));
          Serial.print(i);
          Serial.print(F(" and "));
          Serial.println(j);
        }
        #endif
      }
    }

    // Reset the pin to INPUT_PULLUP after checking
    pinMode(signalPins[i], INPUT_PULLUP);
  }
}