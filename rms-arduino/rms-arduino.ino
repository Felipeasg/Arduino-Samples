// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

//#include "EmonLib.h" // Include Emon Library
//EnergyMonitor emon1; // Create an instance

#define inPinI A7

#define ADC_BITS 10
#define ADC_COUNTS (1<<ADC_BITS)

int Number_of_Samples = 1500;
int sampleI = 0; 
float offsetI = 0.0; //Low-pass filter output 
float filteredI = 0.0; 
float ICAL = 376.42;
float sqI = 0.0, sumI = 0.0;
float Irms = 0.0;

int SupplyVoltage=3300;

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

void setup()
{
  Serial.begin(9600);
  
  //emon1.voltage(0, 266.03, 1.7); // Voltage: input pin, calibration, phase_shift
  //emon1.current(1, 111.1); // Current: input pin, calibration.
}

void loop()
{
  calcVrms();
  //Serial.print("Vrms: ");
  Serial.println(Irms);
}
