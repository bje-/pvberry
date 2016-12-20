/* calibrate.ino
 * This sketch is heavily modified from Robin Emley's version. We are
 * only interested in CT1, so we can dramatically simplify the code.
 *
 * Ben Elliston, 2016-12-20
 *
 * THIS sketch provides an easy way of calibrating the current sensors
 * that are connected to the the CT1 and CT2 ports. The measured value
 * is shown at the Serial Monitor.
 * 
 * CT1 is for 'grid' current, to be measured at the grid supply point.
 * CT2 is for the load current, so that diverted energy can be recorded
 *
 * The selected CT should be clipped around a lead through which a known
 * amount of power is flowing.  This can be compared against the measured value 
 * which is proportional to the powerCal setting.  Once the optimal powerCal values 
 * have been obtained for each channel, these values can be transferred into the 
 * main Mk2 PV Router sketch.  
 * 
 * Depending on which way around the CT is connected, the measured value may be
 * positive or negative.
 *
 *      Robin Emley
 *      www.Mk2PVrouter.co.uk
 *      March 2014
 */

#include <Arduino.h> 

#include <TimerOne.h>
#define ADC_TIMER_PERIOD 125 // uS (determines the sampling rate / amount of idle time)

// Physical constants, please do not change!
#define SECONDS_PER_MINUTE 60
#define MINUTES_PER_HOUR 60
#define JOULES_PER_WATT_HOUR 3600 //  (0.001 kWh = 3600 Joules)

// Change these values to suit the local mains frequency and supply meter
#define CYCLES_PER_SECOND 50 

// definition of enumerated types
enum polarities {NEGATIVE, POSITIVE};

// allocation of digital pins for prototype PCB-based rig (with simple display adapter)
// ******************************************************
// D0 & D1 are reserved for the Serial i/f
// D2 is a driver line for the 4-digit display (segment D, via series resistor)
// D3 is not used
// D4 is not used   
// D5 is a driver line for the 4-digit display (segment B, via series resistor)
// D6 is a driver line for the 4-digit display (digit 3, via wire link)
// D7 is a driver line for the 4-digit display (digit 2, via wire link)
// D8 is a driver line for the 4-digit display (segment F, via series resistor)
// D9 is a driver line for the 4-digit display (segment A, via series resistor)
// D10 is a driver line for the 4-digit display (segment DP, via series resistor)
// D11 is a driver line for the 4-digit display (segment C, via series resistor)
// D12 is a driver line for the 4-digit display (segment G, via series resistor)
// D13 is a driver line for the 4-digit display (digit 4, via wire link)

// allocation of analogue pins
// ***************************
// A0 (D14) is a driver line for the 4-digit display (digit 1, via wire link)
// A1 (D15) is a driver line for the 4-digit display (segment E, via series resistor)
// A2 (D16) is not used 
const byte voltageSensor = 0;          // A3 is for the voltage sensor
const byte currentSensor = 1;     // A5 is for CT1 which measures grid current


const int DCoffset_I = 512;    // nominal mid-point value of ADC @ x1 scale

// General global variables that are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be 'long'
//
boolean beyondStartUpPhase = false;     // start-up delay, allows things to settle
long cycleCount = 0;                    // counts mains cycles from start-up 
long energyThisSecond;
long DCoffset_V_long;              // <--- for LPF
long DCoffset_V_min;               // <--- for LPF
long DCoffset_V_max;               // <--- for LPF
long IEU_per_Joule;

// for interaction between the main processor and the ISRs 
volatile boolean dataReady = false;
int sampleI;
int sampleV;


// Calibration 
//------------
//   
// powerCal is a floating point variable which is used for converting the 
// product of voltage and current samples into Watts.
//
// The correct value of powerCal is dependent on the hardware that is 
// in use.  For best resolution, the hardware should be configured so that the 
// voltage and current waveforms each span most of the ADC's usable range.  For 
// many systems, the maximum power that will need to be measured is around 3kW. 
//
// With my PCB-based hardware, the ADC has an input range of 0 to 3.3V and an 
// output range of 0 to 1023.  The purpose of each input sensor is to 
// convert the measured parameter into a low-voltage signal which fits nicely 
// within the ADC's input range. 
//
// In the case of 240V mains voltage, the numerical value of the input signal 
// in Volts is likely to be fairly similar to the output signal in ADC levels.  
// 240V AC has a peak-to-peak amplitude of 679V, which is not far from the ideal 
// output range.  Stated more formally, the conversion rate of the overall system 
// for measuring VOLTAGE is likely to be around 1 ADC-step per Volt (RMS).
//
// In the case of AC current, however, the situation is very different.  At
// mains voltage, a power of 3kW corresponds to an RMS current of 12.5A which 
// has a peak-to-peak range of 35A.  This is smaller than the output signal of
// the ADC by around a factor of twenty.  The conversion rate of the overall system 
// for measuring CURRENT is therefore likely to be around 20 ADC-steps per Amp.
//
// When calculating "real power", which is what this code does, the individual 
// conversion rates for voltage and current are not of importance.  It is 
// only the conversion rate for POWER which is important.  This is the 
// product of the individual conversion rates for voltage and current.  It 
// therefore has the units of ADC-steps squared per Watt.  Most systems will
// have a power conversion rate of around 20 (ADC-steps squared per Watt).
// 
// powerCal is the RECIPR0CAL of the power conversion rate.  A good value 
// to start with is therefore 1/20 = 0.05 (Watts per ADC-step squared)
//
const float powerCal = 0.05;  // for CT1
 

void setup()
{  
  delay(5000); // allow time to open Serial monitor     
 
  Serial.begin(9600);
  Serial.println();
  Serial.println("calibrate.ino (calibrating CT1)");
  Serial.println();
 
  // For converting Integer Energy Units into Joules
  IEU_per_Joule = (long)CYCLES_PER_SECOND * (1/powerCal); 
 
      
  // Define operating limits for the LP filter which identifies DC offset in the voltage 
  // sample stream.  By limiting the output range, the filter always should start up 
  // correctly.
  DCoffset_V_long = 512L * 256; // nominal mid-point value of ADC @ x256 scale  
  DCoffset_V_min = (long)(512L - 100) * 256; // mid-point of ADC minus a working margin
  DCoffset_V_max = (long)(512L + 100) * 256; // mid-point of ADC plus a working margin

  Serial.print ("ADC mode:       ");
  Serial.print (ADC_TIMER_PERIOD);
  Serial.println ( " uS fixed timer");

  // Set up the ADC to be triggered by a hardware timer of fixed duration  
  ADCSRA  = (1<<ADPS0)+(1<<ADPS1)+(1<<ADPS2);  // Set the ADC's clock to system clock / 128
  ADCSRA |= (1 << ADEN);                 // Enable ADC

  Timer1.initialize(ADC_TIMER_PERIOD);   // set Timer1 interval
  Timer1.attachInterrupt( timerIsr );    // declare timerIsr() as interrupt service routine
        
  Serial.print ( "powerCal =      "); Serial.println (powerCal,4);
  
  Serial.print(">>free RAM = ");
  Serial.println(freeRam());  // a useful value to keep an eye on
  Serial.println ("----");    
}

// An Interrupt Service Routine is now defined in which the ADC is
// instructed to perform the two analogue measurements in sequence:
// Voltage and I.  A "data ready" flag is set after each voltage
// conversion has been completed.  This flag is cleared within loop().
// This Interrupt Service Routine is executed whenever the ADC timer
// expires.  The next ADC conversion is initiated from within this
// ISR.

void timerIsr(void)
{                                         
  static unsigned char sample_index = 0;

  switch(sample_index)
  {
    case 0:
      sampleV = ADC;                    // store the ADC value (this one is for Voltage)
      ADMUX = 0x40 + currentSensor; // set up the next conversion, which is for Diverted Current
      ADCSRA |= (1<<ADSC);              // start the ADC
      sample_index = 2;                 // increment the control flag
      break;
    case 2:
      sampleI = ADC;               // store the ADC value (this one is for Grid Current)
      ADMUX = 0x40 + voltageSensor;  // set up the next conversion, which is for Voltage
      ADCSRA |= (1<<ADSC);              // start the ADC
      sample_index = 0;                 // reset the control flag
      dataReady = true;                 // both ADC values can now be processed
      break;
    default:
      sample_index = 0;                 // to prevent lockup (should never get here)      
  }
}


// When using interrupt-based logic, the main processor waits in loop() until the 
// dataReady flag has been set by the ADC.  Once this flag has been set, the main
// processor clears the flag and proceeds with all the processing for one set of 
// ADC samples.  It then returns to loop() to wait for the next set to become 
// available.

void loop()             
{   
  if (dataReady)   // flag is set after every set of ADC conversions
  {
    dataReady = false; // reset the flag
    allGeneralProcessing(); // executed once for each set of V&I samples
  }     
}


// This routine is called to process each set of V & I samples. The main processor and 
// the ADC work autonomously, their operation being only linked via the dataReady flag.  
// As soon as a new set of data is made available by the ADC, the main processor can 
// start to work on it immediately.  
//
void allGeneralProcessing()
{
  static int samplesDuringThisCycle;             // for normalising the power in each mains cycle
  static long sumP;                              // for per-cycle summation of 'real power'
  static enum polarities polarityOfLastSampleV;  // for zero-crossing detection
  static long cumVdeltasThisCycle_long;          // for the LPF which determines DC offset (voltage)
  static byte perSecondCounter = 0;

  // remove DC offset from the raw voltage sample by subtracting the accurate value 
  // as determined by a LP filter.
  long sampleVminusDC_long = ((long)sampleV<<8) - DCoffset_V_long; 

  // determine the polarity of the latest voltage sample
  enum polarities polarityNow = (sampleVminusDC_long > 0) ? POSITIVE : NEGATIVE;

  Serial.println("Here! " + String(sampleV));

  if (polarityNow == POSITIVE) 
  { 
    if (beyondStartUpPhase)
    {     
      if (polarityOfLastSampleV != POSITIVE)
      {
        // This is the start of a new +ve half cycle (just after the zero-crossing point)
        cycleCount++;  
            
        // Calculate the real power and energy during the last whole mains cycle.
        //
        // sumP contains the sum of many individual calculations of instantaneous power.  In  
        // order to obtain the average power during the relevant period, sumP must first be 
        // divided by the number of samples that have contributed to its value.
        //
        // The next stage would normally be to apply a calibration factor so that real power 
        // can be expressed in Watts.  That's fine for floating point maths, but it's not such
        // a good idea when integer maths is being used.  To keep the numbers large, and also 
        // to save time, calibration of power is omitted at this stage.  Real Power (stored as 
        // a 'long') is therefore (1/powerCal) times larger than the actual power in Watts.
        //
        long realPower = sumP / samplesDuringThisCycle; // proportional to Watts
   
        // Next, the energy content of this power rating needs to be determined.  Energy is 
        // power multiplied by time, so the next step is normally to multiply the measured
        // value of power by the time over which it was measured.
        //   Instanstaneous power is calculated once every mains cycle. When integer maths is 
        // being used, a repetitive power-to-energy conversion seems an unnecessary workload.  
        // As all sampling intervals are of equal duration, it is more efficient simply to 
        // add all of the power samples together, and note that their sum is actually 
        // CYCLES_PER_SECOND greater than it would otherwise be.
        //   Although the numerical value itself does not change, I thought that a new name 
        // may be helpful so as to minimise confusion.  
        //   The 'energy' variable below is CYCLES_PER_SECOND * (1/powerCal) times larger than 
        // the actual energy in Joules.

        // Add these energy contributions to the relevant accumulator. 
        energyThisSecond += realPower;
        
        // Once per second, the contents of the selected accumulator is 
        // converted to Joules and displayed as Watts.  Both accumulators
        // are then reset.

        if(perSecondCounter >= CYCLES_PER_SECOND) 
        { 
          perSecondCounter = 0;   
          int powerInWatts = energyThisSecond / IEU_per_Joule;
	  Serial.println(powerInWatts);
          energyThisSecond = 0;
        }
        else
          perSecondCounter++;

        // clear the per-cycle accumulators for use in this new mains cycle.  
        samplesDuringThisCycle = 0;
        sumP = 0;
      } // end of processing that is specific to the first Vsample in each +ve half cycle 
    }
    else
    {  
      // wait until the DC-blocking filters have had time to settle
      if (millis() > 3000)
      {
        beyondStartUpPhase = true;
        sumP = 0;
        samplesDuringThisCycle = 0;
        Serial.println ("Go!");
      }
    }
  } // end of processing that is specific to samples where the voltage is positive
  
  else // the polatity of this sample is negative
  {     
    if (polarityOfLastSampleV != NEGATIVE)
    {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)      
      // which is a convenient point to update the Low Pass Filter for DC-offset removal
      //
      long previousOffset = DCoffset_V_long;
      DCoffset_V_long = previousOffset + (cumVdeltasThisCycle_long>>6); // faster than * 0.01
      cumVdeltasThisCycle_long = 0;
      
      // To ensure that the LPF will always start up correctly when 240V AC is available, its
      // output value needs to be prevented from drifting beyond the likely range of the 
      // voltage signal.  This avoids the need to use a HPF as was done for initial Mk2 builds.
      //
      if (DCoffset_V_long < DCoffset_V_min)
        DCoffset_V_long = DCoffset_V_min;
      else if (DCoffset_V_long > DCoffset_V_max)
        DCoffset_V_long = DCoffset_V_max;

    } // end of processing that is specific to the first Vsample in each -ve half cycle
  } // end of processing that is specific to samples where the voltage is negative
  
  // processing for EVERY pair of samples
  //
  // First, deal with the power at the grid connection point (as measured via CT1)
  // remove most of the DC offset from the current sample (the precise value does not matter)

  Serial.println("I: " + String(sampleI) + ", V: " + String(sampleV));

  long sampleIminusDC = ((long)(sampleI-DCoffset_I))<<8;
   
  // phase-shift the voltage waveform so that it aligns with the grid current waveform
  long  phaseShiftedSampleVminusDC = sampleVminusDC_long; // <- simple version for when
                                                               // phaseCal is not in use
                                                               
  // calculate the "real power" in this sample pair and add to the accumulated sum
  long filtV_div4 = phaseShiftedSampleVminusDC>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4 = sampleIminusDC>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP = filtV_div4 * filtI_div4;  // 32-bits (now x4096, or 2^12)
  instP = instP>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  Serial.println("P: " + String(instP));
  sumP +=instP; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  samplesDuringThisCycle++;
  
  // store items for use during next loop
  cumVdeltasThisCycle_long += sampleVminusDC_long; // for use with LP filter
  polarityOfLastSampleV = polarityNow;  // for identification of half cycle boundaries
}


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
