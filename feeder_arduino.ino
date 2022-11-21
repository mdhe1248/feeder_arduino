// Originally by Nickolas Belenko at Rockefeller University.
#include <CapacitiveSensor.h>
//#include <toneAC.h>
//#include <Volume.h>

//Volume vol;

#define LEVER_PIN 3   // the pin that the pushbutton is attached to
#define LED_PIN 8   // the pin that the LED is attached to
#define GatePin 5 
#define TEST_PIN 2     
#define CAP_SEND 6
#define CAP_REC 7
#define BUZZ 9
#define FREQ 1000  // FREQ works with tone(), but it cannot change the volume.

// Feeder activation
int GatePWM = 0; // initially closed state
int feederState = 0;
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

// Capacitive sensor
int bytes = 30;
int caplimit = 6000;
long licks; 

// Feeder and lickometer modulator
unsigned long initialMillis;
unsigned long dt; // delta time. `millis() - initialMillis`
int cycleDuration = 50; // The length of each cycle in `loop` in millisecond: This is also lickometer sampling rate.
int timeoutDelay = cycleDuration-1; // lickometer time out max in millisecond.
int valveOpenDelay = 3000; // Delayed feeding after button press in millisecond
int valveOpenDuration = 2000; // in millisecond.
int toneDuration = 1000; // in millisecond.
int toneVol = 30; // volume intensity less than 256. This does not work with `tone()` function; and tone frequency cannot be changed.
int _toneVol;
unsigned long timeCounter = 0;

// Wash
int minWashTime = 1000; // Multipled by cycleDuration
int buttonPushDuration = 0;  // To measure the duration of button press
int pressTimeThresh = 200;   // Open the valve when lever pressed longer than `thresh`  (multipled by cycleDuration)

CapacitiveSensor   lickSensor = CapacitiveSensor(CAP_SEND,CAP_REC);

void setup() {
  //vol.begin();
  pinMode(LEVER_PIN, INPUT);
  pinMode(GatePin, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(TEST_PIN, INPUT);

  //lickSensor.set_CS_AutocaL_Millis(2000);
  lickSensor.set_CS_AutocaL_Millis(0xFFFFFFFF);
  lickSensor.set_CS_Timeout_Millis(timeoutDelay); // Timeout
  analogWrite(GatePin, GatePWM); // Initially feeder is closed
  
  Serial.begin(9600);
}

void loop() {
  //test = digitalRead(TEST_PIN);
  // read the pushbutton input pin:
  initialMillis = millis();  // initial time.
  buttonState = digitalRead(LEVER_PIN);
  licks = lickCount(bytes, caplimit); // Obtain capacitance value.
  
  //// Feeder activator
  // Turn on `feederState` and initialize `timeCounter`
  if (buttonState != lastButtonState && buttonState == HIGH){ 
    timeCounter = 0;
    feederState = 1;
  }
  // When feederState is On, turn on `GatePWM`. `timeCounter` is for delayed feeder activation.
  if (GatePWM == 0 && timeCounter >= valveOpenDelay && feederState == 1){
      _toneVol = toneVol;
      GatePWM = 255;
      analogWrite(GatePin, GatePWM);
      analogWrite(BUZZ, _toneVol);
      //tone(BUZZ, FREQ, toneDuration); // For now `toneDuration` cannot be longer than valveOpenDuration.
      //toneAC(FREQ, 2, 1000);
      //vol.tone(440, 255);
      timeCounter = 0;
  }
  // Turn off `feederState` and `GatePWM`
  if (GatePWM == 255 && timeCounter > valveOpenDuration && feederState == 1){ 
    GatePWM = 0;
    feederState = 0;
    analogWrite(GatePin, GatePWM);
  }
  // Turn off the buzzer.
  if (_toneVol > 0 && timeCounter > toneDuration){
    _toneVol = 0;
    noTone(BUZZ);
    analogWrite(BUZZ, _toneVol);
  }
  lastButtonState = buttonState;
  ////
  
  buttonPushDuration = valveOpenSignal(minWashTime, buttonState, buttonPushDuration, pressTimeThresh); // Wash function
  dt = millis()-initialMillis; //Measure time difference between the beginning and the end.
  while(dt < cycleDuration){
    dt = millis()-initialMillis;
  }
  timeCounter += dt;
  Serial.print(buttonState);
  Serial.print(",");
  Serial.print(GatePWM);
  Serial.print(",");
  Serial.println(licks);
}


long lickCount (int samples, int maximum)
{
  long licksRaw = lickSensor.capacitiveSensor(samples);
  long licks = min(licksRaw, maximum);
  return licks;
}


int valveOpenSignal(int minWashTime, int buttonState, int buttonPushDuration, int pressTimeThresh){
  int var = 0;
  if (buttonState == HIGH)
  {
    buttonPushDuration += 1;
    if (buttonPushDuration >= pressTimeThresh)
    {
      tone(BUZZ, FREQ, 1000);
      analogWrite(GatePin, 255);  //Leave the valve open
      while (var <= minWashTime || buttonState == LOW){
        var++;
        buttonState = digitalRead(LEVER_PIN);
      }
      buttonPushDuration = 0;
    }
    return buttonPushDuration;
  }
  else {
    return buttonPushDuration = 0;
  }
}
