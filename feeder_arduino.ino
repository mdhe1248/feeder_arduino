#include <CapacitiveSensor.h>
 
#define  LEVER_PIN 3   // the pin that the pushbutton is attached to
#define LED_PIN 8   // the pin that the LED is attached to
#define GatePin 5 
#define TEST_PIN 2     
#define CAP_SEND 6
#define CAP_REC 7
#define BUZZ 9
#define FREQ 400
#define BUTTCOUNTMAX 100
#define WASHMAX 1000000
//#define BYTES 30
//#define MAX_CAP 60000

int GatePWM = 255;

int washCounter = 0;
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

int bytes = 10;
int caplimit = 6000;

int buttonPushDuration = 0;  // To measure the duration of button press
int pressTimeThresh = 500;      // Open the valve when lever pressed longer than `thresh` 

int test = 0;

unsigned long previousMillis = 0;
unsigned long interval = 1000;

CapacitiveSensor   lickSensor = CapacitiveSensor(CAP_SEND,CAP_REC);

void setup() {
  pinMode(LEVER_PIN, INPUT);
  pinMode(GatePin, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(TEST_PIN, INPUT);

  lickSensor.set_CS_AutocaL_Millis(0xFFFFFFFF);
  lickSensor.set_CS_Timeout_Millis(20); // Timeout
  Serial.begin(9600);
}

void loop() {
  
  unsigned long CurrentMillis = millis();
  
  test = digitalRead(TEST_PIN);
  // read the pushbutton input pin:
  buttonState = digitalRead(LEVER_PIN);
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState && buttonstate == HIGH) 
  {
    licks = ButtonActivate(LEVER_PIN, buttonState);
  }
  else
  {
    licks = lickCount(bytes, caplimit); 
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
  buttonPushDuration = valveOpenSignal(buttonState, buttonPushDuration, pressTimeThresh); //Function to count the duration and leave the valve open.
  analogWrite(GatePin, 0);
  noTone(BUZZ);
  Serial.println(licks); 
  Serial.println("\t");   
  buttonPushCounter = 0; 
}

long lickCount (int samples, int maximum)
{
  long licksRaw = lickSensor.capacitiveSensor(samples);
  long licks = min(licksRaw, maximum);
  return licks;
}

int ButtonActivate(int pin, int state)
{
  if(state == HIGH)
    {
      for(buttonPushCounter = 0; buttonPushCounter <= BUTTCOUNTMAX; buttonPushCounter++)
      {
        analogWrite(GatePin, GatePWM);
        tone(BUZZ, FREQ); 
        licks = lickCount(bytes, caplimit); 
        //Serial.println(lickCount(bytes, caplimit));
      }
      buttonPushDuration = 0;
    }
   else 
    {
      // if the current state is LOW then the button went from on to off:
      analogWrite(GatePin, 0);
      noTone(BUZZ);
      licks = lickCount(bytes, caplimit);
    }
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
      //delay(10000);
    }
    return buttonPushDuration;
  }
}
