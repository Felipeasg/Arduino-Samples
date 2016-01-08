
#include <PID_v1.h>

#define inPinI A0

#define ADC_BITS 10
#define ADC_COUNTS (1<<ADC_BITS)

int Number_of_Samples = 2500;
int sampleI = 0; 
float offsetI = 0.0; //Low-pass filter output 
float filteredI = 0.0; 
float ICAL = 382.62;
float sqI = 0.0, sumI = 0.0;
float Irms = 0.0;

int SupplyVoltage=3300;

// constants won't change. They're used here to
// set pin numbers:
const int buttonPinMore = 10;     // the number of the pushbutton pin
const int buttonPinLess = 11;     // the number of the pushbutton pin
const int buttonOn = 12;     // the number of the pushbutton pin

const int ledPin =  13;      // the number of the LED pin
const int ledPinRed =  7;      // the number of the LED pin
const int ledPinBlue =    6;      // the number of the LED pin
const int ledPinOrange =    5;      // the number of the LED pin

const int analogOutPin = 9; // Analog output pin that the LED is attached to

int outputValue = 255;        // value output to the PWM (analog out)

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=2, Ki=0.8, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void calcVrms()
{    
    
    for (unsigned int n = 0; n < Number_of_Samples; n++)
    {
      sampleI = analogRead(inPinI);
      // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
      // then subtract this - signal is now centered on 0 counts.
      offsetI = (offsetI + (sampleI-offsetI)/1024);
      filteredI = sampleI - offsetI;
      // Root-mean-square method current
      // 1) square current values
      sqI = filteredI * filteredI;
      // 2) sum
      sumI += sqI;
    }

    float I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
    Irms = I_RATIO * sqrt(sumI / Number_of_Samples); 

     //Reset accumulators
    sumI = 0;
}

void setup() {

  Setpoint = 230;
  
  // initialize the LED pin as an output:
  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinBlue, OUTPUT);
  pinMode(ledPinOrange, OUTPUT);
  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPinMore, INPUT);

  analogWrite(analogOutPin, outputValue);
  
  Serial.begin(9600);

  //turn the PID on
  myPID.SetMode(REVERSE);

  while(digitalRead(buttonOn) == HIGH) 
  {
    
  }
}

void loop() {

#if 0
  if (digitalRead(buttonPinMore) == LOW) 
  {
    outputValue += 25;
    if(outputValue >= 250)
    {
      outputValue = 250;
    }

    Serial.print("Out: ");
    Serial.println(outputValue);

    analogWrite(analogOutPin, outputValue);
    
    //digitalWrite(ledPinRed, HIGH);
    while(digitalRead(buttonPinMore) == LOW);
  }
  else
  {
    //digitalWrite(ledPinRed, LOW);
  }
  
  if (digitalRead(buttonPinLess) == LOW) 
  {
    outputValue -= 25;
    if(outputValue <= 0)
      outputValue = 0;

    Serial.print("Out: ");
    Serial.println(outputValue);

    analogWrite(analogOutPin, outputValue);
    
    //digitalWrite(ledPinBlue, HIGH);
    while(digitalRead(buttonPinLess) == LOW);
  } 
  else
  {
    ///digitalWrite(ledPinBlue, LOW);
  }
  
  if (digitalRead(buttonOn) == LOW) 
  {
    outputValue = 255;
    analogWrite(analogOutPin, 255);
    // turn LED on:
    //digitalWrite(ledPinOrange, HIGH);

    Serial.print("Out: ");
    Serial.println(outputValue);
    while(digitalRead(buttonOn) == LOW);
  } 
  else
  {
    //digitalWrite(ledPinOrange, LOW);
  }

  if(outputValue == 255)
  {
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinBlue, LOW);
      digitalWrite(ledPinOrange, LOW);
  }
  if(outputValue < 255)
  {
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinBlue, LOW);
      digitalWrite(ledPinOrange, HIGH);
  }
  if(outputValue < 120)
  {
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinBlue, HIGH);
      digitalWrite(ledPinOrange, HIGH);
  }
  if(outputValue <= 0)
  {
      digitalWrite(ledPinRed, HIGH);
      digitalWrite(ledPinBlue, HIGH);
      digitalWrite(ledPinOrange, HIGH);
  }
#endif

  calcVrms();
  
  Input = 200;
  //input
  myPID.Compute();

  Output = 255 - Output;

  if(Output < 110)
    Output = 110;
  
  analogWrite(analogOutPin, Output);
  
  if(Output == 255)
  {
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinBlue, LOW);
      digitalWrite(ledPinOrange, LOW);
  }
  if(Output < 255)
  {
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinBlue, LOW);
      digitalWrite(ledPinOrange, HIGH);
  }
  if(Output < 120)
  {
      digitalWrite(ledPinRed, LOW);
      digitalWrite(ledPinBlue, HIGH);
      digitalWrite(ledPinOrange, HIGH);
  }
  if(Output <= 0)
  {
      digitalWrite(ledPinRed, HIGH);
      digitalWrite(ledPinBlue, HIGH);
      digitalWrite(ledPinOrange, HIGH);
  }
  
  
  
  //Serial.print("Vrms: ");
  //Serial.println(Input);

  //Serial.print("err: ");
  //Serial.println(Setpoint-Input);

  //Serial.print("out: ");
  //Serial.println(Output);
}
