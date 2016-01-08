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

#define inPinI A7

#define SupplyVoltage   3300.0F
#define ADC_BITS        10
#define ADC_COUNTS      (1<<ADC_BITS)
#define VCAL            2277.7F
#define V_RATIO         VCAL * ((SupplyVoltage/1000.0F) / (ADC_COUNTS))

#define N               50.0F
#define alpha           ((N-1)/N)
#define beta            (1/N)

//Parametros filtro passa alta para sinal de tensуo
#define NZEROS_HP_V     1
#define NPOLES_HP_V     1
#define GAIN_HP_V       1.007854143e+00F
#define K_HP_V          0.9844141274F

#define STABILIZATION_TIME  7000

#define BUFFER_SIZE     300

float xv_hp_v[NZEROS_HP_V+1], yv_hp_v[NPOLES_HP_V+1];
int sampleV = 0; 

float Vrms = 0.0;  

float rmsIn = 0;
float rmsOut[2] = {0};

float meanIn = 0;
float meanOut[2] = {0};


const int ledPin =  13;      // the number of the LED pin
int ledState = LOW;             // ledState used to set the LED

unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)

int sendcount = 0;
int waittime = 0;

//To plot data (analising)

float mbuffer[BUFFER_SIZE];
long int samplepos = 0;

float sqrt11(const float number)
{
  const float ACCURACY=0.0001;
  float lower, upper, guess;
  
   if (number < 1)
   {
    lower = number;
    upper = 1;
   }
   else
   {
    lower = 1;
    upper = number;
   }
  
   while ((upper-lower) > ACCURACY)
   {
    guess = (lower + upper)/2;
    if(guess*guess > number)
     upper =guess;
    else
     lower = guess; 
   }
   return (lower + upper)/2;

}  

void calcVrms()
{    
    sampleV = analogRead(inPinI);

    //Filtro passa alta para a amostra de tensуo
    xv_hp_v[0] = xv_hp_v[1];
    xv_hp_v[1] = ((float)sampleV)/GAIN_HP_V;
    yv_hp_v[0] = yv_hp_v[1];
    yv_hp_v[1] =   (xv_hp_v[1] - xv_hp_v[0]) + (  K_HP_V * yv_hp_v[0]);   


    rmsOut[0] = sqrt11(((alpha*rmsOut[1]*rmsOut[1]) + (beta*yv_hp_v[1]*yv_hp_v[1])));
    rmsOut[1] = rmsOut[0];
    
    meanOut[0] = alpha*meanOut[1] + beta*rmsOut[0];
    meanOut[1] = meanOut[0];
    
    Vrms = V_RATIO * meanOut[0]; 
}



void setup()
{
  Serial.begin(9600);

  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  pinMode(8, OUTPUT);

}


void loop()
{
  unsigned long currentMillis = micros();
  if (currentMillis - previousMillis >= interval) {
 
   calcVrms();    
   if(waittime++ > STABILIZATION_TIME)
   {
      
      //mbuffer[samplepos] = sampleV;
      //mbuffer[samplepos] = yv_hp_v[1];
      //mbuffer[samplepos] = rmsOut[1];
      mbuffer[samplepos] = Vrms;
      
      if(samplepos++ > BUFFER_SIZE)
      {
        digitalWrite(ledPin, HIGH);
        for(int i = 0; i < BUFFER_SIZE; i++)
        {
          Serial.println(mbuffer[i]);
          delay(80);
        }
        
        while(1);
      }       

      waittime = STABILIZATION_TIME + 1;
    }

    previousMillis = currentMillis;
  }

}
