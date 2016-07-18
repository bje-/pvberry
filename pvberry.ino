/*
 * Divert suplus PV power to a dump load using a triac.
 * Based on code by Robin Emley.
 */

#include <avr/wdt.h>
#include <Arduino.h>
#include <TimerOne.h>
#include <myassert.h>

// uS (determines the sampling rate / amount of idle time)
const int ADC_TIMER_PERIOD = 125;

// Physical constants, please do not change!
const int SECONDS_PER_HOUR = 3600;
const int JOULES_PER_WATT_HOUR = 3600;   //  (0.001 kWh = 3600 Joules)

// Change these values to suit the local mains frequency and supply meter
const int CYCLES_PER_SECOND = 50;
const int WORKING_RANGE_IN_JOULES = 3600;

enum polarities {NEGATIVE, POSITIVE};
enum loadStates {LOAD_ON, LOAD_OFF}; // the external trigger device is active low

void configureParamsForSelectedOutputMode();
void timerIsr(void);
void allGeneralProcessing();
void confirmPolarity();

const byte outputForTrigger = 4;       // <-- an output which is active-low
const byte voltageSensor = 3;          // A3 is for the voltage sensor
const byte currentSensor = 4;          // A4 is for CT measuring grid current

const byte delayBeforeSerialStarts = 3;  // in seconds, to allow Serial window to be opened
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

long mainsCyclesPerHour = (long) CYCLES_PER_SECOND * SECONDS_PER_HOUR;

// this setting is only used if anti-flicker mode is enabled
float offsetOfEnergyThresholdsInAFmode = 0.1; // <-- must not exceeed 0.5

// for interaction between the main processor and the ISRs
volatile boolean dataReady = false;
volatile int sampleI;
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

const float powerCal = 0.0435;

void setup()
{
#if 0
    wdt_enable(WDTO_8S);
#endif

    pinMode(outputForTrigger, OUTPUT);
    digitalWrite(outputForTrigger, LOAD_OFF); // the external trigger is active low

    delay(delayBeforeSerialStarts * 1000); // allow time to open Serial monitor
    Serial.begin(9600);
    Serial.println();
    Serial.println("-------------------------------------");
    Serial.println("Sketch ID:      pvberry.ino");
    Serial.println();

    // When using integer maths, calibration values that have been
    // supplied in floating point form need to be rescaled.

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
        (long)WORKING_RANGE_IN_JOULES * CYCLES_PER_SECOND * (1/powerCal);
    energyInBucket_long = 0;

    // Define operating limits for the LP filter which identifies DC offset in the voltage
    // sample stream.  By limiting the output range, the filter always should start up
    // correctly.
    DCoffset_V_long = 512L * 256; // nominal mid-point value of ADC @ x256 scale
    DCoffset_V_min = (long)(512L - 100) * 256; // mid-point of ADC minus a working margin
    DCoffset_V_max = (long)(512L + 100) * 256; // mid-point of ADC plus a working margin

    Serial.print("ADC mode:       ");
    Serial.print(ADC_TIMER_PERIOD);
    Serial.println( " uS fixed timer");

    // Set up the ADC to be triggered by a hardware timer of fixed duration
    ADCSRA  = (1<<ADPS0)+(1<<ADPS1)+(1<<ADPS2);  // Set the ADC's clock to system clock / 128
    ADCSRA |= (1 << ADEN);                 // Enable ADC

    Timer1.initialize(ADC_TIMER_PERIOD);   // set Timer1 interval
    Timer1.attachInterrupt( timerIsr );    // declare timerIsr() as interrupt service routine

    Serial.println();
    Serial.print("powerCal =      ");
    Serial.println(powerCal,4);

    Serial.print("zero-crossing persistence (sample sets) = ");
    Serial.println(PERSISTENCE_FOR_POLARITY_CHANGE);
    Serial.print("continuity sampling display rate (mains cycles) = ");
    Serial.println(CONTINUITY_CHECK_MAXCOUNT);

    lowerEnergyThreshold_long =
	capacityOfEnergyBucket_long * (0.5 - offsetOfEnergyThresholdsInAFmode);
    upperEnergyThreshold_long =
	capacityOfEnergyBucket_long * (0.5 + offsetOfEnergyThresholdsInAFmode);
}


void timerIsr(void)
{
    static unsigned char state = 0;
    static int sampleI_raw;

    switch (state) {
    case 0:
        sampleV = ADC;                    // store the voltage
        ADMUX = 0x40 + currentSensor;     // set up the next conversion (current)
        ADCSRA |= (1<<ADSC);              // start the ADC
        state = 1;                        // jump to state 1
        sampleI = sampleI_raw;
        dataReady = true;                 // all three ADC values can now be processed
        break;
    case 1:
        sampleI_raw = ADC;                // store the current
        ADMUX = 0x40 + voltageSensor;	  // set up the next conversion (voltage)
        ADCSRA |= (1<<ADSC);              // start the ADC
        state = 0;                        // back to state 0
        break;
    default:
        unreachable();
    }
}


void loop()
{
#if 0
    wdt_reset();  // poke the watchdog
#endif
    if (dataReady) {
        // flag is set after every set of ADC conversions
        dataReady = false;
        allGeneralProcessing(); // executed once for each set of V&I samples
    }
}


void allGeneralProcessing()
{
    static long sumP;                   // for per-cycle summation of 'real power'
    static long cumVdeltasThisCycle_long;    // for the LPF which determines DC offset (voltage)
    static enum loadStates nextStateOfLoad = LOAD_OFF;

    // remove DC offset from the raw voltage sample by subtracting the accurate value
    // as determined by a LP filter.
    long sampleVminusDC_long = ((long)sampleV<<8) - DCoffset_V_long;

    // determine the polarity of the latest voltage sample
    polarityOfMostRecentVsample = (sampleVminusDC_long > 0) ? POSITIVE : NEGATIVE;
    confirmPolarity();

    if (polarityConfirmed == POSITIVE) {
        if (polarityConfirmedOfLastSampleV != POSITIVE) {
            // This is the start of a new +ve half cycle (just after the zero-crossing point)
            if (beyondStartUpPhase) {
                // a simple routine for checking the performance of this new ISR structure
                if (sampleSetsDuringThisMainsCycle < lowestNoOfSampleSetsPerMainsCycle)
                    lowestNoOfSampleSetsPerMainsCycle = sampleSetsDuringThisMainsCycle;

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

                long realPower = sumP / sampleSetsDuringThisMainsCycle; // proportional to Watts

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

                long realEnergy = realPower;

                // Energy contributions from the grid connection point (CT1) are summed in an
                // accumulator which is known as the energy bucket.  The purpose of the energy bucket
                // is to mimic the operation of the supply meter.  The range over which energy can
                // pass to and fro without loss or charge to the user is known as its 'sweet-zone'.
                // The capacity of the energy bucket is set to this same value within setup().
                //
                // The latest contribution can now be added to this energy bucket
                energyInBucket_long += realEnergy;

                // Apply max and min limits to bucket's level.  This is to ensure correct operation
                // when conditions change, i.e. when import changes to export, and vice-versa.
                //
                if (energyInBucket_long > capacityOfEnergyBucket_long)
                    energyInBucket_long = capacityOfEnergyBucket_long;
                else if (energyInBucket_long < 0)
                    energyInBucket_long = 0;

                // continuity checker
                sampleCount_forContinuityChecker++;
                if (sampleCount_forContinuityChecker >= CONTINUITY_CHECK_MAXCOUNT) {
                    sampleCount_forContinuityChecker = 0;
                    Serial.println(lowestNoOfSampleSetsPerMainsCycle);
                    lowestNoOfSampleSetsPerMainsCycle = 999;
                }

                // clear the per-cycle accumulators for use in this new mains cycle.
                sampleSetsDuringThisMainsCycle = 0;
                sumP = 0;
            } else {
                // wait until the DC-blocking filters have had time to settle
                if (millis() > (delayBeforeSerialStarts + startUpPeriod) * 1000) {
                    beyondStartUpPhase = true;
                    sumP = 0;
                    sampleSetsDuringThisMainsCycle = 0; // not yet dealt with for this cycle
                    sampleCount_forContinuityChecker = 1; // opportunity has been missed for this cycle
                    lowestNoOfSampleSetsPerMainsCycle = 999;
                    Serial.println ("Go!");
                }
            }
        } // end of processing that is specific to the first Vsample in each +ve half cycle

        // still processing samples where the voltage is POSITIVE ...
        // check to see whether the trigger device can now be reliably armed
        if (sampleSetsDuringThisMainsCycle == 3) { // much easier than checking the voltage level
            if (beyondStartUpPhase) {
                if (energyInBucket_long < lowerEnergyThreshold_long) {
                    // when below the lower threshold, always set the load to "off"
                    nextStateOfLoad = LOAD_OFF;
                } else if (energyInBucket_long > upperEnergyThreshold_long) {
                    // when above the upper threshold, always set the load to "off"
                    nextStateOfLoad = LOAD_ON;
                }
                // (else) otherwise, leave the load's state unchanged (hysteresis)

                // set the Arduino's output pin accordingly, and clear the flag
                digitalWrite(outputForTrigger, nextStateOfLoad);
            }
        }
    } // end of processing that is specific to samples where the voltage is positive

    else { // the polatity of this sample is negative
        if (polarityConfirmedOfLastSampleV != NEGATIVE) {
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
    long sampleIminusDC = ((long) (sampleI - DCoffset_I)) << 8;

    // phase-shift the voltage waveform so that it aligns with the grid current waveform
    long phaseShiftedSampleVminusDC = sampleVminusDC_long; // <- simple version for when

    // calculate the "real power" in this sample pair and add to the accumulated sum
    long filtV_div4 = phaseShiftedSampleVminusDC >> 2;  // reduce to 16-bits (now x64, or 2^6)
    long filtI_div4 = sampleIminusDC >> 2; // reduce to 16-bits (now x64, or 2^6)
    long instP = filtV_div4 * filtI_div4;  // 32-bits (now x4096, or 2^12)
    instP = instP>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
    sumP += instP; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)

    sampleSetsDuringThisMainsCycle++;

    // store items for use during next loop
    cumVdeltasThisCycle_long += sampleVminusDC_long; // for use with LP filter
    polarityConfirmedOfLastSampleV = polarityConfirmed;  // for identification of half cycle boundaries
}



/* This routine prevents a zero-crossing point from being declared
   until a certain number of consecutive samples in the 'other' half
   of the waveform have been encountered.  */
void confirmPolarity()
{
    static byte count = 0;
    if (polarityOfMostRecentVsample != polarityConfirmedOfLastSampleV)
        count++;
    else
        count = 0;

    if (count > PERSISTENCE_FOR_POLARITY_CHANGE) {
        count = 0;
        polarityConfirmed = polarityOfMostRecentVsample;
    }
}
