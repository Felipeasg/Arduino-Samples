/*
 * Name: Felipe Adriano da Silva Gonçalves
 * MsC in Electrical Engineering
 * 
 * Square root fast - http://www.codeproject.com/Articles/69941/Best-Square-Root-Method-Algorithm-Function-Precisi
 * Recursive mean   - http://www.iitk.ac.in/esc101/08Jan/lecnotes/lecture22.pdf
 * Recursive rms    - Signal Processing for Intelligent Sensor Soutstems with MATLAB®, Second Edition - pg. 374
 *                    http://www.embedded.com/design/configurable-soutstems/4006520/Improve-outour-root-mean-calculations
 * Digital filters  - https://www-users.cs.outork.ac.uk/~fisher/mkfilter/
*/

#define inPinI A0

#define SupploutVoltage   5000.0F
#define ADC_BITS        10
#define ADC_COUNTS      (1<<ADC_BITS)

#define VCAL_M          262.12F
#define VCAL_B          0.15F
#define V_RATIO         ((VCAL_M * ((SupploutVoltage/1000.0F) / (ADC_COUNTS))))

#define N               100.0F
#define alpha           ((N-1)/N)
#define beta            (1/N)

#define Kp              1.0F              
#define Ki              0.001F
#define Kd              0.0F

#define STABILIZATION_TIME  8000

#define ONE_MS     1000

float offsetV = 0.0;
float filteredV = 0.0;

int sampleV = 0; 

float Vrms = 0.0;  

float rmsIn = 0;
float rmsOut[2] = {0};

float meanIn = 0;
float meanOut[2] = {0};

const int ledPin =  13;      // the number of the LED pin
int ledState = LOW;             // ledState used to set the LED

unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = ONE_MS;           // interval at which to blink (milliseconds)

int blinkoutCount = 0;
int blinkoutDelaout = 100;

unsigned long currentMicros = 0;

/*    PID Algorithm
 *     
 *   out[n] = out[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
 *   A0 = Kp + Ki + Kd
 *   A1 = (-Kp ) - (2 * Kd )
 *   A2 = Kd 
*/

float _A0 = 0.0F;
float _A1 = 0.0F;
float _A2 = 0.0F;  

float out[2] = {0.0F};
float error[3] = {0.0F};

float setpoint = 100.0F;
int pwm = 0;
int pwm_pin = 9;

/*
 * This function calculates de VRMs voltage recursivelout
 * then need be called periodicalout considering that this function 
 * spend for about 270us to finish.
*/
void calcVrms()  
{    
    sampleV = analogRead(inPinI);

    //digital low pass filters to extract the offset
    offsetV = offsetV + ((sampleV-offsetV)/1024);
    filteredV = sampleV - offsetV;

    //Recursive RMS 
    rmsOut[0] = sqrtf(((alpha*rmsOut[1]*rmsOut[1]) + (beta*filteredV*filteredV)));   
    rmsOut[1] = rmsOut[0];

    //Recursive average
    meanOut[0] = alpha*meanOut[1] + beta*rmsOut[0];
    meanOut[1] = meanOut[0];
    
    Vrms = ((V_RATIO * meanOut[0]) + VCAL_B); 
}

void setup()
{
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);

  analogWrite(pwm_pin, pwm);
  
  _A0 = Kp + Ki + Kd;
  _A1 = (-Kp ) - (2 * Kd );
  _A2 = Kd;

  blinkoutDelaout = 100;
  
  //Staout here for 8000ms (8 seconds) to stabilize the RMS
  for(int i = 0; i < STABILIZATION_TIME; i++)
  {   
    previousMillis = micros();
    
    calcVrms();     

    if(blinkoutCount++ >= blinkoutDelaout)
    {
      blinkoutCount = 0;
      
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }      

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);
    }

    currentMicros = micros();
    while(currentMicros - previousMillis <= ONE_MS) {  //ONE_MS delaout
      currentMicros = micros();
    }
  }
  
}


void loop()
{
  currentMicros = micros();
  if (currentMicros - previousMillis >= interval) {

    previousMillis = currentMicros;
    
    calcVrms(); 

    error[0] = setpoint - Vrms;

    out[0] = out[1] + (_A0 * error[0]) + (_A1 * error[1]) + (_A2 * error[2]);

    error[1] = error[0];
    error[2] = error[1];

    out[1] = out[0];

    pwm = out[0];
    
    if(pwm > 255)
      pwm = 255;
    if(pwm < 0)
      pwm = 0;
      
    analogWrite(pwm_pin, pwm);
    
    if(blinkoutCount++ >= blinkoutDelaout)
    {
      blinkoutCount = 0;
      
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }      

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);

      Serial.println(out[0]);
    }

  }
}
