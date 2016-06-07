#include <Arduino.h>
#include <TimerOne.h>
#include "myassert.h"

void timerIsr(void);
void allGeneralProcessing();
void confirmPolarity();

#define ADC_TIMER_PERIOD 125 // uS (determines the sampling rate / amount of idle time)

// Physical constants, please do not change!
#define SECONDS_PER_MINUTE 60
#define MINUTES_PER_HOUR 60
#define JOULES_PER_WATT_HOUR 3600

// Change these values to suit the local mains frequency and supply meter
#define CYCLES_PER_SECOND 50 
#define WORKING_RANGE_IN_JOULES 3600 
#define REQUIRED_EXPORT_IN_WATTS 0 // when set to a negative value, this acts as a PV generator 


enum polarities {NEGATIVE, POSITIVE};
enum loadStates {LOAD_ON, LOAD_OFF}; // the external trigger device is active low

const byte outputForTrigger = 4; // <-- an output which is active-low
const byte voltageSensor = 0;          // A0 is for the voltage sensor
const byte currentSensor_grid = 1;     // A1 is for the CT which measures grid current

const byte delayBeforeSerialStarts = 2;  // in seconds, to allow Serial window to be opened
const byte startUpPeriod = 3;  // in seconds, to allow LP filter to settle
const int DCoffset_I = 512;    // nominal mid-point value of ADC @ x1 scale

// General global variables that are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be 'long'

boolean beyondStartUpPhase = false;     // start-up delay, allows things to settle
long triggerThreshold_long;        // for determining when the trigger may be safely armed
long energyInBucket_long;          // in Integer Energy Units
long capacityOfEnergyBucket_long;  // depends on powerCal, frequency & the 'sweetzone' size.
long lowerEnergyThreshold_long;    // for turning load off
long upperEnergyThreshold_long;    // for turning load on
long DCoffset_V_long;              // <--- for LPF
long DCoffset_V_min;               // <--- for LPF
long DCoffset_V_max;               // <--- for LPF
long IEU_per_Wh; // depends on powerCal, frequency & the 'sweetzone' size.

const long mainsCyclesPerHour = (long) CYCLES_PER_SECOND * SECONDS_PER_MINUTE * MINUTES_PER_HOUR;

// for interaction between the main processor and the ISRs 
volatile boolean dataReady = false;
volatile int sampleI_grid;
volatile int sampleV;

// For an enhanced polarity detection mechanism, which includes a persistence check
#define PERSISTENCE_FOR_POLARITY_CHANGE 1 // sample sets
enum polarities polarityOfMostRecentVsample;   
enum polarities polarityConfirmed;  
enum polarities polarityConfirmedOfLastSampleV;  

// For a mechanism to check the continuity of the sampling sequence
#define CONTINUITY_CHECK_MAXCOUNT 250 // mains cycles
int sampleCount_forContinuityChecker;
int sampleSetsDuringThisMainsCycle;    
int lowestNoOfSampleSetsPerMainsCycle;

// Calibration values
//-------------------
// Two calibration values are used: powerCal and phaseCal. 
// With most hardware, the default values are likely to work fine without 
// need for change.  A full explanation of each of these values now follows:
//   
// See original source code for more explanation.
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

const float powerCal_grid = 0.0435;  // for CT1
 
long requiredExportPerMainsCycle_inIEU;

void setup()
{
  // used for diagnostics LED
  pinMode(13, OUTPUT);
  pinMode(outputForTrigger, OUTPUT);  
  digitalWrite (outputForTrigger, LOAD_OFF); // the external trigger is active low
  
  delay(delayBeforeSerialStarts * 1000); // allow time to open Serial monitor      
  Serial.begin(9600);
  Serial.println();
  Serial.println("RetroFiT 1.0");
  Serial.println();

  // When using integer maths, calibration values that have supplied in floating point 
  // form need to be rescaled.  
  
  // When using integer maths, the SIZE of the ENERGY BUCKET is altered to match the
  // scaling of the energy detection mechanism that is in use.  This avoids the need 
  // to re-scale every energy contribution, thus saving processing time.  This process 
  // is described in more detail in the function, allGeneralProcessing(), just before 
  // the energy bucket is updated at the start of each new cycle of the mains.
  //
  // An electricity meter has a small range over which energy can ebb and flow without 
  // penalty.  This has been termed its "sweet-zone".  For optimal performance, the energy
  // bucket of a PV Router should match this value.  The sweet-zone value is therefore 
  // included in the calculation below.
  //
  // For the flow of energy at the 'grid' connection point (CT1) 
  capacityOfEnergyBucket_long = 
     (long)WORKING_RANGE_IN_JOULES * CYCLES_PER_SECOND * (1/powerCal_grid);
  energyInBucket_long = 0;
  
  requiredExportPerMainsCycle_inIEU = (long)REQUIRED_EXPORT_IN_WATTS * (1/powerCal_grid); 


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

  Serial.print ( "powerCal_grid =      "); Serial.println (powerCal_grid,4);
  
  Serial.print ("Export rate (Watts) = ");
  Serial.println (REQUIRED_EXPORT_IN_WATTS);
  
  Serial.print ("zero-crossing persistence (sample sets) = ");
  Serial.println (PERSISTENCE_FOR_POLARITY_CHANGE);
  Serial.print ("continuity sampling display rate (mains cycles) = ");
  Serial.println (CONTINUITY_CHECK_MAXCOUNT);  
}

// An Interrupt Service Routine is now defined in which the ADC is
// instructed to measure each analogue input in sequence.  A dataReady
// flag is set after each voltage conversion has been completed.

// For each pair of samples, the current sample is taken before the
// one for voltage.  This is appropriate because each waveform current
// is generally slightly advanced relative to the waveform for
// voltage.  The dataReady flag is cleared within loop().

// This Interrupt Service Routine is for use when the ADC is fixed
// timer mode.  It is executed whenever the ADC timer expires.  In
// this mode, the next ADC conversion is initiated from within this
// ISR.

void timerIsr(void)
{
  static enum adc_source { A0_VOLTAGE, A1_CURRENT } sample_source = A0_VOLTAGE;
  static int sampleI_grid_raw;
  
  switch(sample_source)
    {
      // voltage sensor is on A0
    case A0_VOLTAGE:
      sampleV = ADC;                    // store the ADC value (this one is for Voltage)
      ADMUX = 0x40 + currentSensor_grid;  // set up the next conversion, which is for grid Current
      ADCSRA |= (1<<ADSC);              // start the ADC
      sample_source = A1_CURRENT;
      sampleI_grid = sampleI_grid_raw;
      dataReady = true;                 // all three ADC values can now be processed
      break;
      // current sensor is on A1
    case A1_CURRENT:
      sampleI_grid_raw = ADC;               // store the ADC value (this one is for Grid Current)
      ADMUX = 0x40 + voltageSensor;  // set up the next conversion, which is for Voltage
      ADCSRA |= (1<<ADSC);              // start the ADC
      sample_source = A0_VOLTAGE;
      break;
    default:
      unreachable();
    }
}


// When using interrupt-based logic, the main processor spins in
// loop() until the dataReady flag has been set by the ADC.  Once this
// flag has been set, the main processor clears the flag and proceeds
// with all the processing for one pair of V & I samples.  It then
// returns to loop() to wait for the next set to become available.

// If the next set of samples become available before the processing
// of the previous set has been completed, data could be lost.  This
// situation can be avoided by prior use of the WORKLOAD_CHECK mode.
// Using this facility, the amount of spare processing capacity per
// loop can be determined.  If there is insufficient processing
// capacity to do all that is required, the base workload can be
// reduced by increasing the duration of ADC_TIMER_PERIOD.

void loop()             
{ 
  if (dataReady)
    {
      // clear the flag
      dataReady = false; 
      // executed once for each set of V&I samples
      allGeneralProcessing();
    }
}


/* This routine is called to process each pair of V & I samples. The
   main processor and the ADC work autonomously, their operation being
   only linked via the dataReady flag.  As soon as a new set of data
   is made available by the ADC, the main processor can start to work
   on it immediately.  */

void allGeneralProcessing()
{
  static long sumP_grid;                              // for per-cycle summation of 'real power' 
  static long cumVdeltasThisCycle_long;    // for the LPF which determines DC offset (voltage)
  static long lastSampleVminusDC_long;     //    for the phaseCal algorithm
  static enum loadStates nextStateOfLoad = LOAD_OFF;  

  // remove DC offset from the raw voltage sample by subtracting the accurate value 
  // as determined by a LP filter.
  long sampleVminusDC_long = ((long)sampleV<<8) - DCoffset_V_long; 

  // determine the polarity of the latest voltage sample
  polarityOfMostRecentVsample = (sampleVminusDC_long > 0) ? POSITIVE : NEGATIVE;
  confirmPolarity();
  
  if (polarityConfirmed == POSITIVE) 
  { 
    if (polarityConfirmedOfLastSampleV != POSITIVE)
    {
      // This is the start of a new +ve half cycle (just after the zero-crossing point)
      if (beyondStartUpPhase)
      {     
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
        long realPower_grid = sumP_grid / sampleSetsDuringThisMainsCycle; // proportional to Watts
   
        realPower_grid -= requiredExportPerMainsCycle_inIEU; // <- useful for PV simulation
 
        // Next, the energy content of this power rating needs to be determined.  Energy is 
        // power multiplied by time, so the next step is normally to multiply the measured
        // value of power by the time over which it was measured.
        //   Instanstaneous power is calculated once every mains cycle. When integer maths is 
        // being used, a repetitive power-to-energy conversion seems an unnecessary workload.  
        // As all sampling periods are of similar duration, it is more efficient simply to 
        // add all of the power samples together, and note that their sum is actually 
        // CYCLES_PER_SECOND greater than it would otherwise be.
        //   Although the numerical value itself does not change, I thought that a new name 
        // may be helpful so as to minimise confusion.  
        //   The 'energy' variable below is CYCLES_PER_SECOND * (1/powerCal) times larger than 
        // the actual energy in Joules.

        long realEnergy_grid = realPower_grid; 
   
        // Energy contributions from the grid connection point (CT1) are summed in an 
        // accumulator which is known as the energy bucket.  The purpose of the energy bucket 
        // is to mimic the operation of the supply meter.  The range over which energy can 
        // pass to and fro without loss or charge to the user is known as its 'sweet-zone'.
        // The capacity of the energy bucket is set to this same value within setup().

        // The latest contribution can now be added to this energy bucket
        energyInBucket_long += realEnergy_grid;   
         
        // Apply max and min limits to bucket's level.  This is to ensure correct operation
        // when conditions change, i.e. when import changes to export, and vici versa.

        if (energyInBucket_long > capacityOfEnergyBucket_long)
          energyInBucket_long = capacityOfEnergyBucket_long;
        else if (energyInBucket_long < 0)
	    energyInBucket_long = 0;  
  
        // continuity checker
        sampleCount_forContinuityChecker++;
        if (sampleCount_forContinuityChecker >= CONTINUITY_CHECK_MAXCOUNT)
        {
          sampleCount_forContinuityChecker = 0;
          Serial.println(lowestNoOfSampleSetsPerMainsCycle);
          lowestNoOfSampleSetsPerMainsCycle = 999;
        }  

        // clear the per-cycle accumulators for use in this new mains cycle.  
        sampleSetsDuringThisMainsCycle = 0;
        sumP_grid = 0;
      }
      else
      {  
        // wait until the DC-blocking filters have had time to settle
        if (millis() > (delayBeforeSerialStarts + startUpPeriod) * 1000) 
        {
          beyondStartUpPhase = true;
          sumP_grid = 0;
          sampleSetsDuringThisMainsCycle = 0; // not yet dealt with for this cycle
          sampleCount_forContinuityChecker = 1; // opportunity has been missed for this cycle
          lowestNoOfSampleSetsPerMainsCycle = 999;
          Serial.println ("Go!");
        }
      }
    } // end of processing that is specific to the first Vsample in each +ve half cycle 
  
    // still processing samples where the voltage is POSITIVE ...    
    // check to see whether the trigger device can now be reliably armed
    if (sampleSetsDuringThisMainsCycle == 3) // much easier than checking the voltage level
    {
      if (beyondStartUpPhase)
      {           
        if (energyInBucket_long < lowerEnergyThreshold_long)
          // when below the lower threshold, always set the load to "off" 
          nextStateOfLoad = LOAD_OFF;
        else if (energyInBucket_long > upperEnergyThreshold_long)
          // when above the upper threshold, always set the load to "off"
          nextStateOfLoad = LOAD_ON;
        else
          // otherwise, leave the load's state unchanged (hysteresis)
	  ;
                  
        // set the Arduino's output pin accordingly, and clear the flag
        digitalWrite(outputForTrigger, nextStateOfLoad);   
      }
    }    
  } // end of processing that is specific to samples where the voltage is positive
  else
  {
    // the polatity of this sample is negative  
    if (polarityConfirmedOfLastSampleV != NEGATIVE)
    {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)      
      // which is a convenient point to update the Low Pass Filter for DC-offset removal
      //  The portion which is fed back into the integrator is approximately one percent
      // of the average offset of all the Vsamples in the previous mains cycle.
      //
      long previousOffset = DCoffset_V_long;
      DCoffset_V_long = previousOffset + (cumVdeltasThisCycle_long>>12); 
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
  
  // processing for EVERY set of samples
  //
  // First, deal with the power at the grid connection point (as measured via CT1)
  // remove most of the DC offset from the current sample (the precise value does not matter)
  long sampleIminusDC_grid = ((long)(sampleI_grid-DCoffset_I))<<8;
   
  // phase-shift the voltage waveform so that it aligns with the grid current waveform
  long  phaseShiftedSampleVminusDC_grid = sampleVminusDC_long; // <- simple version for when
                                                               // phaseCal is not in use
  
  // calculate the "real power" in this sample pair and add to the accumulated sum
  long filtV_div4 = phaseShiftedSampleVminusDC_grid>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4 = sampleIminusDC_grid>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP = filtV_div4 * filtI_div4;  // 32-bits (now x4096, or 2^12)
  instP = instP>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)       
  sumP_grid +=instP; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  sampleSetsDuringThisMainsCycle++;
  
  // store items for use during next loop
  cumVdeltasThisCycle_long += sampleVminusDC_long; // for use with LP filter
  lastSampleVminusDC_long = sampleVminusDC_long;  // required for phaseCal algorithm
  polarityConfirmedOfLastSampleV = polarityConfirmed;  // for identification of half cycle boundaries
}


/* This routine prevents a zero-crossing point from being declared
   until a certain number of consecutive samples in the 'other' half
   of the waveform are observed.  */

void confirmPolarity()
{
  static byte count = 0;
  if (polarityOfMostRecentVsample != polarityConfirmedOfLastSampleV)
    count++;
  else
    count = 0;

  if (count > PERSISTENCE_FOR_POLARITY_CHANGE)
    {
      count = 0;
      polarityConfirmed = polarityOfMostRecentVsample;
    }
}

// settings for normal mode
//  lowerEnergyThreshold_long = capacityOfEnergyBucket_long * 0.5; 
//  upperEnergyThreshold_long = capacityOfEnergyBucket_long * 0.5;   
