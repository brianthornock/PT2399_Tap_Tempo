#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Assign pins

//DigitalPin Assignments
const unsigned int tapSwitch = 1;
const unsigned int ledOut = 2;
const unsigned int vOut = 4;
const unsigned int encA = 3;
const unsigned int encB = 0;

//AnalogPin Assignment
const unsigned int encSW = 0;
const unsigned int encSWDig = 5;


// Set up delay specific parameters
unsigned int defaultDelayTime = 60; //duration in ms of default tempo
unsigned int delayTime = 60; //set this to the default
unsigned int prevDelayTime = 60; //set this to the default
unsigned int currentMillis; // Used for debouncing tap tempo switch

//Set up bounds for how fast/slow we allow things to go before divisions
uint8_t minTime = 60; // shortest allowable delay time, in ms
unsigned int maxTime = 600; // longest allowable delay time, in ms
uint8_t timeStep = 5; // The amount of time between divisions (in ms) over the range of [minTime:maxTime]


// Initiate encoder stuff
volatile int lastEnc = 0;
uint8_t encSWCount = 0;
uint8_t prevSWState = LOW;

// Since the encoder switch governs the multiplier, set up those parameters here
uint8_t multiplier = 1; // How much to divide the tapped tempo
uint8_t prevMultiplier = 1; // Last multiplier for checking to see what we need to do
uint8_t updateMult = 0;
uint8_t updateLEDInterval = 1;


// Initiate the tap tempo button stuff
uint8_t buttonState;
uint8_t lastButtonState = LOW; // Previous state of tap tempo switch
uint8_t tapStatus = LOW; // Current status of the tap tempo switch

unsigned long lastDebounceTime = 0; // initiate the debounce time counter
uint8_t debounceDelay = 50; // How long we enforce a debounce, ms

uint8_t prevTaps = 0; // How many taps we have in our history
unsigned int tapTimeout = 1000; // 1s tap timeout
uint8_t maxTaps = 10; // Max number of taps to keep in history
unsigned int prevTimes [10]; // Array for storing the tap periods
unsigned int tapTime = 60; // Averaged value of times in tap history, give it a default value
unsigned int prevTapTime = 60; // Previous averaged tap tempo time, give it a default value
uint8_t useTap = 0; // This is used to determine if we have enough taps to calculate the tap period
unsigned int prevTapDelay = 0; // This is used for debouncing the tap tempo switch


// Set up LED blinking parameters
unsigned int prevMillis; // Used for keeping track of LED blinking
unsigned int maxLEDTime = 300; //Maximum LED blink on duration in ms
unsigned int currLEDState = LOW; // LED starts off off
unsigned int currLEDOffInterval; // How long the LED has been off
unsigned int currLEDOnInterval; // How long the LED has been on
unsigned int blinkDuration; // How long should the LED be on
unsigned int currVoltInterval; // How long have we held the current voltage level

uint8_t prevDutyCycle = 230;

// Table to convert delay times into 8-bit PWM duty cycles. 652 entries supports delay times up to 650ms.
// Table copied from https://www.electrosmash.com/forum/time-manipulator/429-tap-delay-effect-pedal?lang=en
// Since values do not exceed 255, use uint8_t to reduce memory. Stick it in PROGMEM so it lives in flash and doesn't use any SRAM 
const uint8_t delay_conv[] PROGMEM ={ 230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,
 230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,
 230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,
 230,230,230,230,230,230,230,230,230,230,230,
 230,229,228,227,226,224,223,222,221,220,219,218,217,216,214,213,212,211,210,209,208,207,206,205,204,203,202,201,200,198,196,194,192,190,189,188,187,186,184,183,182,181,180,179,178,176,175,174,
 173,171,170,169,168,168,167,166,165,164,163,163,162,161,160,159,158,157,156,155,155,154,153,152,151,150,149,148,148,147,146,145,144,143,143,142,141,140,139,139,138,137,137,136,135,135,134,133,133,132,131,131,130,
 129,129,128,128,127,127,126,126,125,124,124,123,123,122,122,121,121,120,120,119,119,118,118,117,117,116,116,115,115,114,114,113,113,112,112,111,111,110,110,109,109,108,108,108,107,107,106,106,105,105,105,104,104,
 103,103,103,102,102,101,101,100,100,100,99,99,99,98,98,98,97,97,97,96,96,96,95,95,94,94,94,93,93,93,92,92,92,91,91,91,90,90,90,89,89,89,89,88,88,88,88,87,87,87,86,86,86,86,85,85,85,84,84,84,84,83,83,83,83,82,82,
 82,81,81,81,81,80,80,80,80,79,79,79,79,79,78,78,78,78,78,77,77,77,77,76,76,76,76,76,75,75,75,75,75,74,74,74,74,74,73,73,73,73,73,72,72,72,72,71,71,71,71,71,70,70,70,70,70,70,69,69,69,69,69,69,68,68,68,68,68,68,68,
 67,67,67,67,67,67,66,66,66,66,66,66,66,65,65,65,65,65,65,64,64,64,64,64,64,64,63,63,63,63,63,63,62,62,62,62,62,62,62,61,61,61,61,61,61,60,60,60,60,60,60,60,60,59,59,59,59,59,59,59,59,58,58,58,58,58,58,58,58,58,57,
 57,57,57,57,57,57,57,57,56,56,56,56,56,56,56,56,55,55,55,55,55,55,55,55,55,54,54,54,54,54,54,54,54,53,53,53,53,53,53,53,53,53,52,52,52,52,52,52,52,52,52,51,51,51,51,51,51,51,51,50,50,50,50,50,50,50,50,50,50,
 50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};


void setup() {
  //Define what each pin is
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(encSWDig, INPUT_PULLUP);
  pinMode(tapSwitch, INPUT_PULLUP);
  pinMode(vOut, OUTPUT);
  pinMode(ledOut, OUTPUT);

  //Set up the initial state of the pins
  digitalWrite(encA, HIGH);
  digitalWrite(encB, HIGH);
  digitalWrite(tapSwitch, LOW);
  digitalWrite(ledOut, LOW);
  digitalWrite(vOut, LOW);

  prevMillis = millis();

  //Update the output PWM duty cycle to start at the default
  analogWrite(vOut, delay_conv[defaultDelayTime]);

  // Set up Timer1 so that it will output the PWM
  TCCR1 = (TCCR1 & 0b11110000) | 0b0001; 
  
  //Set up an ISR for the encoder switch pin so that it reacts instantly
  //and to reduce loop execution time with determining multiplier
  GIMSK = 0b00100000; //enable pin change interrupts
  PCMSK = 0b00001001; //enable PB3 and PB4 as pin change interruptible
  sei(); //start interrupt service
}



void loop() {
  //Update the multiplier by polling the encoder switch
  updateMultiplier();

  //Check to see if tap tempo is used
  checkTapTempo();

  //Update the delay time based on above, if needed
  updateDelayTime();

  // What time is it now?
  currentMillis = millis();

  //Update the LED blink rate
  //Blink LED for 1/2 period with a max illumination of maxLEDTime ms per period
  updateLED();

  // If we have changed our delay time, update the PWM output
  if updateDTime {
    updatePWM();
  }
}


//Interrupt handling
ISR (PCINT0_vect) {
  int  readA1 = digitalRead(encA);
  int  readB1 = digitalRead(encB);

  delayMicroseconds(2000); //Brute force debouncing
  int  readB2 = digitalRead(encB);
  int  readA2 = digitalRead(encA);

  if (readA1 == readA2 && readB1 == readB2) {

    int MSB = readA1;
    int LSB = readB1;

    int encoded = (MSB << 1) | LSB; // shift MSB by one bit and append LSB
    int sum = (lastEnc << 2) | encoded; // shift last ENC value by two bits and append the latest two

    //Clockwise turn, make it go slower, longer delay time
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
      tapTime = (tapTime / multiplier) + timeStep;

      // Don't let it exceed maxTime ms
      if (tapTime > maxTime) {
        tapTime = maxTime;
      }
    }
    //Counterclockwise turn, make it go faster, shorter delay time
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
      tapTime = (tapTime / multiplier) - timeStep;

      // Don't let it go below minTime ms
      if (tapTime < minTime) {
        tapTime = minTime;
      }

    }
    lastEnc = encoded;
  }

}



void updateMultiplier() {
  
  // Poll the switch, but let the encoders use the interrupts
  int currSW = analogRead(encSW) > 925;
  int isDebounced = 0;

  // Only do a debounce check if it goes high
  if (currSW == HIGH) {
    delay(100);

    int tempSW = analogRead(encSW) > 925;

    if (tempSW == currSW || prevSWState == LOW) {
      encSWCount++;
      updateMult = 1;
      isDebounced = 1;
      prevSWState = HIGH;
    }
    else {
      prevSWState = LOW;
    }
  }

  //Check to see what the multiplier is and if we are using the pattern
  if (updateMult && isDebounced) {
    switch (encSWCount) {
      case 0:
        multiplier = 1;
        break;
      case 1:
        multiplier = 2;
        break;
      case 2:
        multiplier = 1;
        encSWCount = 0;
        break;
    }
    isDebounced = 0;
  }
}


void updateDelayTime() {
  int tempDelayTime = tapTime;


  //If we are more than 5 ms off and not using pattern, update the delayTime
  if ((abs(tapTime - prevTapTime) >= 5) | updateMult) {
    if (updateMult) {
      tempDelayTime = round(tapTime/multiplier);
    }
    else {
      tempDelayTime = tapTime;
    }
    
    prevTapTime = tapTime;
    encSWCount = 0; // If we change the tempo, reset encoder count and go to a multiplier of 1
    updateLEDInterval = 1;
    updateDTime = 1;
  }


  delayTime = tempDelayTime;
}



//Code for debouncing tap tempo switch
void switchDebounce() {
  int reading = digitalRead(tapSwitch);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading != buttonState) {

      buttonState = reading;

      if (buttonState == HIGH) {
        tapStatus = HIGH;
      }
    }
  }

  lastButtonState = reading;
}


void checkTapTempo() {

  //Check to see if the tap tempo switch has been pressed
  switchDebounce();

  if (tapStatus == HIGH) {

    tapStatus = LOW;
    //Check to see if we already have a tap tempo history. If so, add this to
    //the history. If not, start a new count.
    if (prevTaps > 0) {
      int currTime = millis();
      int currDelay = currTime - prevTapDelay;
      // Check to make sure we didn't time out
      if (currDelay < tapTimeout) {
        //Set the flag for using tap tempo
        useTap = 1;

        // Create the temp array for storing times in
        unsigned int newPrevTimes [maxTaps];

        if (prevTaps < maxTaps) {

          //Fill up the new array with all the old values first
          for (int k = 0; k < prevTaps - 1; k++) {
            newPrevTimes[k] = prevTimes[k];
          }

          //Then add in the new value at the end
          newPrevTimes[prevTaps - 1] = currDelay;
          prevTaps++;

        } // End if prevTaps < maxTaps
        
        for (int nTime = 0; nTime < maxTaps; nTime++) {
          prevTimes[nTime] = newPrevTimes[nTime];
        }

      } // End if currDelay < tapTimeout
      else {
        //If we timeout, reset the counter and zero out the tempo array
        prevTaps = 1;

        for (int i = 0; i < maxTaps; i++) {
          prevTimes[i] = 0;
        }

        useTap = 0;
      } // End if tap has timed out
    } // End if prevTaps > 0
    // If we do not have any previous taps (first tap after timeout)
    else {
      prevTaps = 1;

      for (int i = 0; i < maxTaps; i++) {
        prevTimes[i] = 0;
      }

      useTap = 0;
    }

    if (useTap == 1 && prevTaps > 2) {
      //Calculate the average polling time, including the multiplier and the random switch
      int sum, loop, numVals;
      float avg;

      sum = avg = 0;
      numVals = 0;

      for (loop = 0; loop < prevTaps - 1; loop++) {
        if (prevTimes[loop] != 0) {
          sum += prevTimes[loop];
          numVals++;
        }
      }
      avg = (float)sum / numVals;
      tapTime = round(avg);
      multiplier = 1;
      encSWCount = 0;
    }
    else {
      //If we don't have the information to produce a tap tempo, stick with what we have
    }
    prevTapDelay = millis();
  }

}


//Code for LED flashing update
void updateLED() {

  if (updateLEDInterval) {
    updateLEDInterval = 0;

    if (delayTime / 2 >= maxLEDTime) {
      currLEDOnInterval = maxLEDTime;
    }
    else {
      currLEDOnInterval = round(delayTime / 2);
    }
    currLEDOffInterval = round(delayTime - currLEDOnInterval);
  }
  

  //Check to see if we have completed the LED on or off interval and change if we have
  if (currLEDState == LOW) {
    if (currentMillis - prevMillis >= currLEDOffInterval) {
      currLEDState = HIGH;
      prevMillis += currLEDOffInterval;
      digitalWrite(ledOut, HIGH);
    }
  }

  if (currLEDState == HIGH) {
    if (currentMillis - prevMillis >= currLEDOnInterval) {
      currLEDState = LOW;
      prevMillis += currLEDOnInterval;
      digitalWrite(ledOut, LOW);
    }
  }

}


void updatePWM() {
   //Update the output PWM duty cycle
   uint8_t newDutyCycle = delay_conv[tapTime];
   analogWrite(vOut, newDutyCycle);
   prevDutyCycle = newDutyCycle;
}

// Some helpful debug functions
/*
  void blinkLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(ledOut, HIGH);
    delay(duration);
    digitalWrite(ledOut, LOW);
    delay(duration);
  }
  }

  void blinkDebugLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(encSW, HIGH);
    delay(duration);
    digitalWrite(encSW, LOW);
    delay(duration);
  }
  }
*/
