#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Assign pins

//DigitalPin Assignments
const unsigned int tapSwitch = 1; // Tap tempo switch on PB1/ Pin 6
const unsigned int ledOut = 2; // LED indicator out on PB2/Pin 7
const unsigned int vOut = 4; // PWM out on PB4/Pin 3
const unsigned int encA = 3; // Encoder A terminal on PB3/Pin 2
const unsigned int encB = 0; // Encoder B terminal on PB0/Pin 5

//AnalogPin Assignment
const unsigned int encSW = 0; // Encoder switch on A0/Pin 1
const unsigned int encSWDig = 5; //Encoder switch on PB5/Pin 1


// Set up delay specific parameters
unsigned int defaultDelayTime = 250; //duration in ms of default tempo
unsigned int delayTime; //set this to the default in setup()
unsigned int baseDelayTime; // Used for tracking delay time without multiplier applied
unsigned int prevDelayTime; //set this to the default in setup()
unsigned int currentMillis; // Used for debouncing tap tempo switch

//Set up bounds for how fast/slow we allow things to go before divisions
uint8_t minTime = 50; // shortest allowable delay time, in ms
unsigned int maxTime = 650; // longest allowable delay time, in ms
uint8_t timeStep = 5; // The amount of time between divisions (in ms) over the range of [minTime:maxTime]


// Initiate encoder stuff
volatile int lastEnc = 0;
uint8_t encSWCount = 0;
uint8_t prevSWState = LOW;

// Since the encoder switch governs the multiplier, set up those parameters here
float multiplier = 1; // How much to divide the tapped tempo
uint8_t updateMult = 0;
uint8_t updateLEDInterval = 1;
uint8_t updateDTime = 0;


// Initiate the tap tempo button stuff
uint8_t buttonState;
uint8_t lastButtonState = LOW; // Previous state of tap tempo switch
uint8_t tapStatus = LOW; // Current status of the tap tempo switch
uint8_t updateTapTempo = 0;

unsigned long lastDebounceTime = 0; // initiate the debounce time counter
uint8_t debounceDelay = 50; // How long we enforce a debounce, ms

uint8_t prevTaps = 0; // How many taps we have in our history
unsigned int tapTimeout = 1000; // 1s tap timeout
uint8_t maxTaps = 10; // Max number of taps to keep in history
unsigned int prevTimes [10]; // Array for storing the tap periods
uint8_t useTap = 0; // This is used to determine if we have enough taps to calculate the tap period
unsigned int prevTapDelay = 0; // This is used for debouncing the tap tempo switch


// Set up LED blinking parameters
unsigned int prevMillis; // Used for keeping track of LED blinking
unsigned int maxLEDTime = 200; //Maximum LED blink on duration in ms
unsigned int currLEDState = LOW; // LED starts off off
unsigned int currLEDOffInterval; // How long the LED has been off
unsigned int currLEDOnInterval; // How long the LED has been on
unsigned int blinkDuration; // How long should the LED be on
unsigned int currVoltInterval; // How long have we held the current voltage level

uint8_t prevDutyCycle = 220;

// Table to convert delay times into 8-bit PWM duty cycles. 651 entries supports delay times up to 650ms.
//Table created with ATTiny85, R=1.5k, C=1uF
const uint8_t delay_conv[] PROGMEM ={220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,
210,200,187,175,169,162,157,152,148,144,140,136,133,131,128,126,123,121,118,115,113,111,109,106,104,102,100,98,96,94,92,90,88,86,84,83,81,79,78,76,74,73,71,70,69,68,67,66,66,65,
64,63,63,62,61,60,60,59,58,57,57,56,56,55,54,53,53,52,51,50,50,49,49,48,48,47,47,46,46,45,45,44,44,43,43,42,42,41,41,40,40,40,39,39,39,39,38,38,37,37,
37,37,37,36,36,36,35,35,35,34,34,34,34,33,33,33,33,32,32,32,32,32,31,31,30,30,30,30,29,29,29,29,29,29,28,28,28,28,28,28,28,28,27,27,27,27,27,27,26,26,
26,26,26,26,26,26,25,25,25,25,25,25,24,24,24,24,24,24,24,24,23,23,23,23,23,23,23,23,23,23,22,22,22,22,22,22,22,22,21,21,21,21,21,21,21,21,21,21,20,20,
20,20,20,20,20,20,20,20,20,20,20,19,19,19,19,19,19,19,19,19,19,19,19,18,18,18,18,18,18,18,18,18,18,18,18,17,17,17,17,17,17,17,17,17,17,17,17,17,17,16,
16,16,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
14,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
12,12,12,12,12,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,10,10,10,10,10,10,10,10,
10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,7,7,7,7,7,7,7,7,7,
7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7};

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
  analogWrite(vOut,128);

  prevMillis = millis();


  // Set up timer/PWM items
  PLLCSR |= (1 << PLLE); //Enable PLL

  //Wait for PLL to lock
  while ((PLLCSR & (1<<PLOCK)) == 0x00)
    {
        // Do nothing until plock bit is set
    }

  // Enable clock source bit
  PLLCSR |= (1 << PCKE);

  // Set prescaler to PCK/8, turn off PWM1A, and set COM1A bits to match the COM1B bits due to attiny bug
  TCCR1 = 0;
  TCCR1 |= (1<< CTC1 | 1 << COM1A1) | (1 << COM1A0 | 1 << CS10);
  
  // Enable OCRB output on PB4, configure compare mode and enable PWM B
  DDRB |= (1 << PB4);
  GTCCR |= (1 << PWM1B) | (1 << COM1B1);

  // Set OCR1B compare value and OCR1C TOP value
  OCR1C = 255; 

  //Update the output PWM duty cycle to start at the default
  OCR1B = pgm_read_word_near(delay_conv + defaultDelayTime);
  delayTime = defaultDelayTime;
  prevDelayTime = defaultDelayTime;
  baseDelayTime = defaultDelayTime;
  
  //Set up an ISR for the encoder switch pin so that it reacts instantly
  //and to reduce loop execution time with determining multiplier
  GIMSK = 0b00100000; //enable pin change interrupts
  PCMSK = 0b00001011; //enable PB0, PB1, and PB3 as pin change interruptible
  sei(); //start interrupt service
}



void loop() {
  //Update the multiplier by polling the encoder switch
  updateMultiplier();

  //Check to see if tap tempo is used only if the PCInt says we need it
  if(updateTapTempo){
    checkTapTempo();
  }

  //Update the delay time based on above, if needed
  updateDelayTime();

  // What time is it now?
  currentMillis = millis();

  //Update the LED blink rate
  //Blink LED for 1/2 period with a max illumination of maxLEDTime ms per period
  updateLED();

  // If we have changed our delay time, update the PWM output
  if (updateDTime) {
    updatePWM();
  }
}


//Interrupt handling
ISR (PCINT0_vect) {

  if(digitalRead(tapSwitch)){//Tap switch was pressed, set flag to update tap tempo
    updateTapTempo = 1;
  }
  else {//Not tap switch, must be the encoder
    
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
  
      
      //Counterclockwise turn, shorter delay time
      if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {

        baseDelayTime += timeStep;
  
        // Don't let it exceed maxTime ms
        if (baseDelayTime > maxTime) {
          baseDelayTime = maxTime;
        }

      }
      //Clockwise turn, longer delay time
      else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {

        baseDelayTime -= timeStep;
  
        // Don't let it go below minTime ms
        if (baseDelayTime < minTime) {
          baseDelayTime = minTime;
        }
      }
      lastEnc = encoded;
      updateMult = 0;
    }
  }

}



void updateMultiplier() {
  
  // Poll the switch, but let the encoders use the interrupts
  int currSW = analogRead(encSW) > 925;

  // Only do a debounce check if it goes high
  if (currSW == HIGH) {
    delay(60); // Wait before next read as software debouncing

    int tempSW = analogRead(encSW) > 925;

    if (tempSW == currSW && prevSWState == LOW) {// Button was really pressed and was previously low; switch state
      encSWCount++; // Go to the next multiplier state
      updateMult = 1; // Set flag
      prevSWState = HIGH; // Update switch state
    }
    else {// Button wasn't really pressed
      prevSWState = LOW; // Set state to low, since button wasn't really pressed
      updateMult = 0; // Don't update multiplier
    }
  }
  else {// Button wasn't pressed this cycle
    prevSWState = LOW; // Switch state is low since nothing was pressed
    updateMult = 0; // Don't update multiplier
  }

  //Update multiplier based on encoder switch count
  if (updateMult) {
    
    switch (encSWCount) {
      case 0:
        multiplier = 1; // Normal delay time
        break;
      case 1:
        multiplier = 1.5; // Dotted eighth note
        break;
      case 2:
        multiplier = 2; // Eighth note
        break;
      case 3:
        multiplier = 1; // Go back to initial delay time and reset encoder switch count
        encSWCount = 0;
        break;
    }
  }
}


void updateDelayTime() {
  //If we are more than timeStep ms off or changing multiplier, update the delayTime
  if ((abs(baseDelayTime - prevDelayTime) >= timeStep) || updateMult) {
    if (updateMult) {
      float tempTime = baseDelayTime/multiplier;
      int tempDelayTime = round(tempTime);

      //Don't update if multiplier would make us go outside of the time ranges
      if ((tempDelayTime >= minTime) && (tempDelayTime <= maxTime)){
        delayTime = tempDelayTime;

        updateLEDInterval = 1; // Set flag to change LED flashing time
        updateDTime = 1; // Set flag to actually change the delay time
        updateMult = 0; // We have updated the multiplier, so clear the flag
      }
      
    }
    else {
      delayTime = baseDelayTime; // We didn't update multiplier, so the base delay time was changed
      encSWCount = 0; // If we change the tempo, reset encoder count and go to a multiplier of 1

      prevDelayTime = baseDelayTime; // Update prevDelayTime to the new value
      updateLEDInterval = 1; // Set flag to change LED flashing time
      updateDTime = 1; // Set flag to actually change delay time
      updateMult = 0; // Clear the update multiplier flag just in case
    }
    
    
  }
  else {// No update this cycle
    updateDTime = 0; // Clear flag to update delay time
  }
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
  
  //Check to see if the tap tempo switch has actually been pressed
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
      delayTime = round(avg);

      //Don't let it go longer than our max delay time
      if (delayTime > maxTime) {
        delayTime = maxTime;
      }
      baseDelayTime = delayTime;
      multiplier = 1;
      updateMult = 0;
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

  // Figure out what the on and off times are for LED flashing
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
   uint8_t newDutyCycle = pgm_read_word_near(delay_conv + delayTime);

   // Only change the duty cycle if it's different from before
   if (newDutyCycle != prevDutyCycle){
    OCR1B = newDutyCycle;
    prevDutyCycle = newDutyCycle;
   }
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
