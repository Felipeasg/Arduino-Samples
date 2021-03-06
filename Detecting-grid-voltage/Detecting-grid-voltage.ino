/*
 * Name: Felipe Adriano da Silva Gonçalves
 * MsC in Electrical Engineering
 * 
 * Square root fast - http://www.codeproject.com/Articles/69941/Best-Square-Root-Method-Algorithm-Function-Precisi
 * Recursive mean   - http://www.iitk.ac.in/esc101/08Jan/lecnotes/lecture22.pdf
 * Recursive rms    - Signal Processing for Intelligent Sensor Systems with MATLAB®, Second Edition - pg. 374
 *                    http://www.embedded.com/design/configurable-systems/4006520/Improve-your-root-mean-calculations
 * Digital filters  - https://www-users.cs.york.ac.uk/~fisher/mkfilter/
*/

#define inPinI A0

#define SupplyVoltage   5000.0F
#define ADC_BITS        10
#define ADC_COUNTS      (1<<ADC_BITS)

#define VCAL_M          262.12F
#define VCAL_B          0.15F
#define V_RATIO         ((VCAL_M * ((SupplyVoltage/1000.0F) / (ADC_COUNTS))))

#define N               100.0F
#define alpha           ((N-1)/N)
#define beta            (1/N)

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

int blinkyCount = 0;
int vrmsDetectedDelayMs = 100;

unsigned long currentMicros = 0;

/*
 * This function calculates de VRMs voltage recursively
 * then need be called periodicaly considering that this function 
 * spend for about 270us to finish.
*/
void calcVrms()  
{    
    digitalWrite(8,HIGH);
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
    digitalWrite(8,LOW);
}

void setup()
{
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  pinMode(8, OUTPUT);
  
  vrmsDetectedDelayMs = 100;
  
  //Stay here for 8000ms (8 seconds) to stabilize the RMS
  for(int i = 0; i < STABILIZATION_TIME; i++)
  {   
    previousMillis = micros();
    
    calcVrms();     

    if(blinkyCount++ >= vrmsDetectedDelayMs)
    {
      blinkyCount = 0;
      
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
    while(currentMicros - previousMillis <= ONE_MS) {  //ONE_MS delay
      currentMicros = micros();
    }
  }
  
  vrmsDetectedDelayMs = 2000;
}


void loop()
{
  currentMicros = micros();
  if (currentMicros - previousMillis >= interval) {

    previousMillis = currentMicros;
    
    calcVrms(); 

    if(blinkyCount++ >= vrmsDetectedDelayMs)
    {
      blinkyCount = 0;
      
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }      

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);
    }

  }
  else
  {
    if(Vrms < 100)
    {
      vrmsDetectedDelayMs = 2000;
    }
    else if(Vrms >= 100 && Vrms < 120)
    {
      vrmsDetectedDelayMs = 1000;
    }
    else if(Vrms >= 120 && Vrms < 140)
    {
      vrmsDetectedDelayMs = 500;
    }
    else if(Vrms >= 140)
    {
      vrmsDetectedDelayMs = 200;
    }
  }

}
