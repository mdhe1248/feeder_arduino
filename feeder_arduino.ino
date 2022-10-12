#include <CapacitiveSensor.h>
 
#define LEVER_PIN 3   // the pin that the pushbutton is attached to
#define LED_PIN 8   // the pin that the LED is attached to
#define GatePin 5 
#define TEST_PIN 2     
#define CAP_SEND 6
#define CAP_REC 7
#define BUZZ 9
#define FREQ 400
//#define BUTTCOUNTMAX 100
//#define WASHMAX 1000000
//#define BYTES 30
//#define MAX_CAP 60000

// Feeder activation
int GatePWM = 0; // initially closed state
int feederState = 0;
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

// Capacitive sensor
int bytes = 20;
int caplimit = 6000;
long licks; 

// Feeder and lickometer modulator
unsigned long initialMillis;
unsigned long dt; // delta time. `millis() - initialMillis`
int cycleDuration = 25; // The length of each cycle in `loop` in millisecond: This is also lickometer sampling rate.
int timeoutDelay = cycleDuration-1; // lickometer time out max in millisecond.
int valveOpenDuration = 1000; // in millisecond.
int valveOpenDelay = 500; // Delayed feeding after button press in millisecond
int toneDuration = 1000; // in millisecond
unsigned long timeCounter = 0;

// Wash
int buttonPushDuration = 0;  // To measure the duration of button press
int pressTimeThresh = 200;      // Open the valve when lever pressed longer than `thresh` 

CapacitiveSensor   lickSensor = CapacitiveSensor(CAP_SEND,CAP_REC);

void setup() {
  pinMode(LEVER_PIN, INPUT);
  pinMode(GatePin, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(TEST_PIN, INPUT);

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
      GatePWM = 255;
      analogWrite(GatePin, GatePWM);
      tone(BUZZ, FREQ, toneDuration); // For now `toneDuration` cannot be longer than valveOpenDuration.
      timeCounter = 0;
  }
  // Turn off `feederState` and `GatePWM`
  if (GatePWM == 255 && timeCounter > valveOpenDuration && feederState == 1){ 
    GatePWM = 0;
    feederState = 0;
    analogWrite(GatePin, GatePWM);
    noTone(BUZZ);
  }
  lastButtonState = buttonState;
  ////
  
  buttonPushDuration = valveOpenSignal(buttonState, buttonPushDuration, pressTimeThresh); // Wash function
  dt = millis()-initialMillis; //Measure time difference between the beginning and the end.
  while(dt < cycleDuration){
    dt = millis()-initialMillis;
  }
  timeCounter += dt;
  Serial.println(licks);
}


long lickCount (int samples, int maximum)
{
  long licksRaw = lickSensor.capacitiveSensor(samples);
  long licks = min(licksRaw, maximum);
  return licks;
}


int valveOpenSignal(int buttonState, int buttonPushDuration, int pressTimeThresh){
  if (buttonState == HIGH)
  {
    buttonPushDuration += 1;
    if (buttonPushDuration >= pressTimeThresh)
    {
      tone(BUZZ, FREQ, 1000);
      analogWrite(GatePin, GatePWM);  //Leave the valve open
      while (1);
    }
    return buttonPushDuration;
  }
  else
    return buttonPushDuration = 0;
}
