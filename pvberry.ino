/* Mk2_bothDisplays_4.ino
 *
 *  (initially released as Mk2_bothDisplays_1 in March 2014)
 * This sketch is for diverting suplus PV power to a dump load using a triac or
 * Solid State Relay. It is based on the Mk2i PV Router code that I have posted in on
 * the OpenEnergyMonitor forum.  The original version, and other related material,
 * can be found on my Summary Page at www.openenergymonitor.org/emon/node/1757
 *
 * In this latest version, the pin-allocations have been changed to suit my
 * PCB-based hardware for the Mk2 PV Router.  The integral voltage sensor is
 * fed from one of the secondary coils of the transformer.  Current is measured
 * via Current Transformers at the CT1 and CT2 ports.
 *
 * CT1 is for 'grid' current, to be measured at the grid supply point.
 *
 * September 2014: renamed as Mk2_bothDisplays_2, with these changes:
 * - cycleCount removed (was not actually used in this sketch, but could have overflowed);
 * - removal of unhelpful comments in the IO pin section;
 * - tidier initialisation of display logic in setup();
 * - addition of REQUIRED_EXPORT_IN_WATTS logic (useful as a built-in PV simulation facility);
 *
 * December 2014: renamed as Mk2_bothDisplays_3, with these changes:
 * - persistence check added for zero-crossing detection (polarityConfirmed)
 * - lowestNoOfSampleSetsPerMainsCycle added, to check for any disturbances
 *
 * December 2014: renamed as Mk2_bothDisplays_3a, with some typographical errors fixed.
 *
 * January 2016: renamed as Mk2_bothDisplays_3b, with a minor change in the ISR to
 *   remove a timing uncertainty.
 *
 * January 2016: updated to Mk2_bothDisplays_3c:
 *   The variables to store the ADC results are now declared as "volatile" to remove
 *   any possibility of incorrect operation due to optimisation by the compiler.
 *
 * February 2016: updated to Mk2_bothDisplays_4, with these changes:
 * - improvements to the start-up logic.  The start of normal operation is now
 *    synchronised with the start of a new mains cycle.
 * - reduce the amount of feedback in the Low Pass Filter for removing the DC content
 *     from the Vsample stream. This resolves an anomaly which has been present since
 *     the start of this project.  Although the amount of feedback has previously been
 *     excessive, this anomaly has had minimal effect on the system's overall behaviour.
 * - removal of the unhelpful "triggerNeedsToBeArmed" mechanism
 * - tidying of the "confirmPolarity" logic to make its behaviour more clear
 * - SWEETZONE_IN_JOULES changed to WORKING_RANGE_IN_JOULES
 * - change "triac" to "load" wherever appropriate
 *
 *      Robin Emley
 *      www.Mk2PVrouter.co.uk
 */

#include <Arduino.h>
#include <TimerOne.h>

#include <myassert.h>

// uS (determines the sampling rate / amount of idle time)
const int ADC_TIMER_PERIOD = 125;

// Physical constants, please do not change!
const int SECONDS_PER_MINUTE = 60;
const int MINUTES_PER_HOUR = 60;
const int JOULES_PER_WATT_HOUR = 3600;   //  (0.001 kWh = 3600 Joules)

// Change these values to suit the local mains frequency and supply meter
const int CYCLES_PER_SECOND = 50;
const int WORKING_RANGE_IN_JOULES = 3600;

enum polarities {NEGATIVE, POSITIVE};
enum loadStates {LOAD_ON, LOAD_OFF}; // the external trigger device is active low
enum outputModes {ANTI_FLICKER, NORMAL};

void configureParamsForSelectedOutputMode();
void timerIsr(void);
void allGeneralProcessing();
void confirmPolarity();

// The power-diversion logic can operate in either of two modes:
//
// - NORMAL, where the load switches rapidly on/off to maintain a constant energy level.
// - ANTI_FLICKER, whereby the repetition rate is reduced to avoid rapid fluctuations
//    of the local mains voltage.
//
// The output mode is determined in realtime via a selector switch
enum outputModes outputMode;

// allocation of digital pins which are not dependent on the display type that is in use
// *************************************************************************************
const byte outputForTrigger = 4; // <-- an output which is active-low

// allocation of analogue pins which are not dependent on the display type that is in use
// **************************************************************************************
const byte voltageSensor = 3;          // A3 is for the voltage sensor
const byte currentSensor = 5;     // A5 is for CT1 which measures grid current

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

long mainsCyclesPerHour = (long) CYCLES_PER_SECOND * SECONDS_PER_MINUTE * MINUTES_PER_HOUR;

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
    pinMode(outputForTrigger, OUTPUT);
    digitalWrite(outputForTrigger, LOAD_OFF); // the external trigger is active low

    // use NORMAL for now
    outputMode = NORMAL;

    delay(delayBeforeSerialStarts * 1000); // allow time to open Serial monitor

    Serial.begin(9600);
    Serial.println();
    Serial.println("-------------------------------------");
    Serial.println("Sketch ID:      pvberry.ino");
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

    Serial.print("Output mode:    ");
    if (outputMode == NORMAL)
        Serial.println( "normal");
    else {
        Serial.println( "anti-flicker");
        Serial.print( "  offsetOfEnergyThresholds  = ");
        Serial.println( offsetOfEnergyThresholdsInAFmode);
    }

    Serial.println();
    Serial.print("powerCal =      ");
    Serial.println(powerCal,4);

    Serial.print("zero-crossing persistence (sample sets) = ");
    Serial.println(PERSISTENCE_FOR_POLARITY_CHANGE);
    Serial.print("continuity sampling display rate (mains cycles) = ");
    Serial.println(CONTINUITY_CHECK_MAXCOUNT);

    configureParamsForSelectedOutputMode();

    Serial.println ("----");
}


void timerIsr(void)
{
    static unsigned char sample_index = 0;
    static int sampleI_raw;

    switch (sample_index) {
    case 0:
        sampleV = ADC;                    // store the ADC value (Voltage)
        ADMUX = 0x40 + currentSensor; // set up the next conversion, which is for Grid Current
        ADCSRA |= (1<<ADSC);              // start the ADC
        sample_index++;                   // jump to state 1
        sampleI = sampleI_raw;
        dataReady = true;                 // all three ADC values can now be processed
        break;
    case 1:
        sampleI_raw = ADC;           // store the ADC value (Grid Current)
        ADMUX = 0x40 + voltageSensor;	  // set up the next conversion, which is for Voltage
        ADCSRA |= (1<<ADSC);              // start the ADC
        sample_index = 0;                 // back to state 0
        break;
    default:
        // should never get here
        unreachable();
    }
}


void loop()
{
    if (dataReady) { // flag is set after every set of ADC conversions
        dataReady = false; // reset the flag
        allGeneralProcessing(); // executed once for each set of V&I samples
    }
}


void allGeneralProcessing()
{
    static long sumP_grid;                   // for per-cycle summation of 'real power'
    static long cumVdeltasThisCycle_long;    // for the LPF which determines DC offset (voltage)
    static long lastSampleVminusDC_long;     //    for the phaseCal algorithm
    static enum loadStates nextStateOfLoad = LOAD_OFF;

    // remove DC offset from the raw voltage sample by subtracting the accurate value
    // as determined by a LP filter.
    long sampleVminusDC_long = ((long)sampleV<<8) - DCoffset_V_long;

    // determine the polarity of the latest voltage sample
    if (sampleVminusDC_long > 0) {
        polarityOfMostRecentVsample = POSITIVE;
    } else {
        polarityOfMostRecentVsample = NEGATIVE;
    }
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

                long realPower_grid = sumP_grid / sampleSetsDuringThisMainsCycle; // proportional to Watts

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
                //
                // The latest contribution can now be added to this energy bucket
                energyInBucket_long += realEnergy_grid;

                // Apply max and min limits to bucket's level.  This is to ensure correct operation
                // when conditions change, i.e. when import changes to export, and vici versa.
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
                sumP_grid = 0;
            } else {
                // wait until the DC-blocking filters have had time to settle
                if (millis() > (delayBeforeSerialStarts + startUpPeriod) * 1000) {
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
    long sampleIminusDC_grid = ((long)(sampleI-DCoffset_I))<<8;

    // phase-shift the voltage waveform so that it aligns with the grid current waveform
    long  phaseShiftedSampleVminusDC_grid = sampleVminusDC_long; // <- simple version for when
    // phaseCal is not in use


    // calculate the "real power" in this sample pair and add to the accumulated sum
    long filtV_div4 = phaseShiftedSampleVminusDC_grid>>2;  // reduce to 16-bits (now x64, or 2^6)
    long filtI_div4 = sampleIminusDC_grid>>2; // reduce to 16-bits (now x64, or 2^6)
    long instP = filtV_div4 * filtI_div4;  // 32-bits (now x4096, or 2^12)
    instP = instP>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
    sumP_grid += instP; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)

    sampleSetsDuringThisMainsCycle++;

    // store items for use during next loop
    cumVdeltasThisCycle_long += sampleVminusDC_long; // for use with LP filter
    lastSampleVminusDC_long = sampleVminusDC_long;  // required for phaseCal algorithm
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


void configureParamsForSelectedOutputMode()
{
    if (outputMode == ANTI_FLICKER) {
        lowerEnergyThreshold_long =
            capacityOfEnergyBucket_long * (0.5 - offsetOfEnergyThresholdsInAFmode);
        upperEnergyThreshold_long =
            capacityOfEnergyBucket_long * (0.5 + offsetOfEnergyThresholdsInAFmode);
    } else {
        lowerEnergyThreshold_long = capacityOfEnergyBucket_long * 0.5;
        upperEnergyThreshold_long = capacityOfEnergyBucket_long * 0.5;
    }

    // display relevant settings for selected output mode
    Serial.print("  capacityOfEnergyBucket_long = ");
    Serial.println(capacityOfEnergyBucket_long);
    Serial.print("  lowerEnergyThreshold_long   = ");
    Serial.println(lowerEnergyThreshold_long);
    Serial.print("  upperEnergyThreshold_long   = ");
    Serial.println(upperEnergyThreshold_long);
}
