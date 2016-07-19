// pvberry.ino: Divert suplus PV power to a dump load.
// Based on Mk2 PV Router code by Robin Emley.

// PVBerry is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.

// PVBerry is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

#include <avr/wdt.h>
#include <Arduino.h>
#include <TimerOne.h>

#include "myassert.h"

// uS (determines the sampling rate / amount of idle time)
const int ADC_TIMER_PERIOD = 125;

// Physical constants, please do not change!
const int SECONDS_PER_HOUR = 3600;
const int JOULES_PER_WATT_HOUR = 3600;

// Change these values to suit the local mains frequency and supply meter
const int CYCLES_PER_SECOND = 50;
const int WORKING_RANGE_IN_JOULES = 3600;

enum polarities {NEGATIVE, POSITIVE};
enum loadStates {LOAD_ON, LOAD_OFF}; // the triac is active low

void configureParamsForSelectedOutputMode();
void timerIsr(void);
void allGeneralProcessing();
void confirmPolarity();

const byte outputForTrigger = 4;
const byte voltageSensor = 3;          // A3 is for the voltage sensor
const byte currentSensor = 4;          // A4 is for CT measuring grid current

const byte delayBeforeSerialStarts = 3;  // in seconds, to allow Serial window to be opened
const byte startUpPeriod = 3;  // in seconds, to allow LP filter to settle
const int DCoffset_I = 512;    // nominal mid-point value of ADC @ x1 scale

// General global variables that are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be longs.

boolean beyondStartUpPhase = false; // start-up delay, allows things to settle
long energyInBucket_long;           // in Integer Energy Units (IEU)
long capacityOfEnergyBucket_long;   // depends on powerCal, frequency & the 'sweetzone' size.

long lowerEnergyThreshold_long;     // for turning load off
long upperEnergyThreshold_long;     // for turning load on

// for low-pass filter
long DCoffset_V_long;
long DCoffset_V_min;
long DCoffset_V_max;

long mainsCyclesPerHour = (long) CYCLES_PER_SECOND * SECONDS_PER_HOUR;

// this setting is only used if anti-flicker mode is enabled
float offsetOfEnergyThresholdsInAFmode = 0.1; // <-- must not exceeed 0.5

// for interaction between the main processor and the ISRs
volatile boolean dataReady = false;
volatile int sampleI;
volatile int sampleV;

// For an enhanced polarity detection mechanism, which includes a persistence check
const int PERSISTENCE_FOR_POLARITY_CHANGE = 1; // sample sets
enum polarities polarityOfMostRecentVsample;
enum polarities polarityConfirmed;
enum polarities polarityConfirmedOfLastSampleV;

// For a mechanism to check the continuity of the sampling sequence
// (maxcount is in mains cycles)
const int CONTINUITY_CHECK_MAXCOUNT = 250;
int sampleCount_forContinuityChecker;
int sampleSetsDuringThisMainsCycle;
int lowestNoOfSampleSetsPerMainsCycle;

// calibration factor
const float powerCal = 0.091;

// A data type that holds 100 bools in a circular fashion.

class BoolBuffer {
public:
    BoolBuffer(void)
    {
        index = 0;
    }

    void insert(bool item)
    {
        if (index >= 100)
            index = 0;
        buf[index] = item;
        index++;
    }

    int count()
    {
        int result = 0;
        for (int i = 0; i < 100; i++)
            if (buf[i])
                result++;
        return result;
    }

private:
    bool buf[100];
    unsigned int index;
} boolbuf;

void setup()
{
#if 0
    wdt_enable(WDTO_8S);
#endif

    for (int pinNo = 8; pinNo < 12; pinNo++) {
        pinMode(pinNo, OUTPUT);
        digitalWrite(pinNo, HIGH);
        delay(250);
    }
    for (int i = 8; i < 12; i++)
        digitalWrite(i, LOW);

    pinMode(outputForTrigger, OUTPUT);
    digitalWrite(outputForTrigger, LOAD_OFF);

    // allow time to open Serial monitor
    delay(delayBeforeSerialStarts * 1000);
    Serial.begin(9600);
    Serial.println("Welcome to the PVBerry!");
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

    // Set up the ADC to be triggered by a hardware timer of fixed duration
    // Set the ADC's clock to system clock / 128
    ADCSRA  = (1 << ADPS0) + (1 << ADPS1) + (1 << ADPS2);
    // Enable ADC
    ADCSRA |= (1 << ADEN);

    Timer1.initialize(ADC_TIMER_PERIOD);
    Timer1.attachInterrupt(timerIsr);

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
        ADMUX = 0x40 + currentSensor;     // switch ADC channel
        ADCSRA |= (1 << ADSC);            // start a conversion
        state = 1;
        sampleI = sampleI_raw;
        dataReady = true;                 // both ADC values can now be processed
        break;
    case 1:
        sampleI_raw = ADC;                // store the current
        ADMUX = 0x40 + voltageSensor;	  // switch ADC channel
        ADCSRA |= (1 << ADSC);            // start a conversion
        state = 0;
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
    long sampleVminusDC_long = ((long) sampleV << 8) - DCoffset_V_long;

    // determine the polarity of the latest voltage sample
    polarityOfMostRecentVsample = (sampleVminusDC_long > 0) ? POSITIVE : NEGATIVE;
    confirmPolarity();

    if (polarityConfirmed == POSITIVE) {
        if (polarityConfirmedOfLastSampleV == NEGATIVE) {
            // This is the start of a new +ve half cycle (just after the zero-crossing point)
            if (beyondStartUpPhase) {
                // a simple routine for checking the performance of this new ISR structure
                if (sampleSetsDuringThisMainsCycle < lowestNoOfSampleSetsPerMainsCycle)
                    lowestNoOfSampleSetsPerMainsCycle = sampleSetsDuringThisMainsCycle;

                long realPower = sumP / sampleSetsDuringThisMainsCycle; // proportional to Watts

                // realEnergy is CYCLES_PER_SECOND * (1/powerCal) times larger than
                // the actual energy in J.
                long realEnergy = realPower;

                energyInBucket_long += realEnergy;

                // Apply max and min limits to bucket's level.  This is to ensure correct operation
                // when conditions change, i.e. when import changes to export, and vice-versa.
                if (energyInBucket_long > capacityOfEnergyBucket_long)
                    energyInBucket_long = capacityOfEnergyBucket_long;
                else if (energyInBucket_long < 0)
                    energyInBucket_long = 0;

                // continuity checker
                sampleCount_forContinuityChecker++;
                if (sampleCount_forContinuityChecker >= CONTINUITY_CHECK_MAXCOUNT) {
                    sampleCount_forContinuityChecker = 0;
                    // Serial.println(lowestNoOfSampleSetsPerMainsCycle);
                    lowestNoOfSampleSetsPerMainsCycle = 999;
                }

                // clear the per-cycle accumulators for use in this new mains cycle.
                sampleSetsDuringThisMainsCycle = 0;
                sumP = 0;
            } else {
                // wait until the DC-blocking filters have had time to settle
	        // nb. This call to millis() can only occur only during
  	        // the start-up phase, so will never rollover.
                if (millis() > (delayBeforeSerialStarts + startUpPeriod) * 1000) {
                    beyondStartUpPhase = true;
                    sumP = 0;
                    sampleSetsDuringThisMainsCycle = 0; // not yet dealt with for this cycle
                    sampleCount_forContinuityChecker = 1; // opportunity has been missed for this cycle
                    lowestNoOfSampleSetsPerMainsCycle = 999;
                    Serial.println ("Ready.");
                }
            }
        } // end of processing that is specific to the first Vsample in each +ve half cycle

        // still processing samples where the voltage is POSITIVE ...
        // check to see whether the trigger device can now be reliably armed
        if (sampleSetsDuringThisMainsCycle == 3) { // much easier than checking the voltage level
            if (beyondStartUpPhase) {
                if (energyInBucket_long < lowerEnergyThreshold_long) {
                    // when below the lower threshold, turn the load OFF
                    nextStateOfLoad = LOAD_OFF;
                } else if (energyInBucket_long > upperEnergyThreshold_long) {
                    // when above the upper threshold, turn the load ON
                    nextStateOfLoad = LOAD_ON;
                }
                // (else) otherwise, leave the load's state unchanged (hysteresis)

                // set the Arduino's output pin accordingly
                digitalWrite(outputForTrigger, nextStateOfLoad);
            }
        }
    } else {
        // processing that is specific to the first Vsample in each -ve half cycle
        // the polarity of this sample is negative
        if (polarityConfirmedOfLastSampleV == POSITIVE) {
            // This is the start of a new -ve half cycle (just after the zero-crossing point)
            // which is a convenient point to update the Low Pass Filter for DC-offset removal.
            // The portion which is fed back into the integrator is approximately one percent
            // of the average offset of all the Vsamples in the previous mains cycle.
            long previousOffset = DCoffset_V_long;
            DCoffset_V_long = previousOffset + (cumVdeltasThisCycle_long >> 12);
            cumVdeltasThisCycle_long = 0;

            // To ensure that the LPF will always start up correctly
            // when 240V AC is available, its output value needs to be
            // prevented from drifting beyond the likely range of the
            // voltage signal.  This avoids the need to use a HPF as
            // was done for initial Mk2 builds.
            if (DCoffset_V_long < DCoffset_V_min)
                DCoffset_V_long = DCoffset_V_min;
            else if (DCoffset_V_long > DCoffset_V_max)
                DCoffset_V_long = DCoffset_V_max;
        }
        // note the current state of the load for this cycle
        boolbuf.insert(nextStateOfLoad == LOAD_ON);

        // count # of cycles in the last 100 that the load was on
        int oncycles = boolbuf.count();
        for (int i = 8; i < 12; i++)
            digitalWrite(i, LOW);

        if (oncycles > 75)
            digitalWrite(8, HIGH);
        if (oncycles > 50)
            digitalWrite(9, HIGH);
        if (oncycles > 25)
            digitalWrite(10, HIGH);
        if (oncycles > 0)
            digitalWrite(11, HIGH);


        // processing for EVERY set of I, V samples

        // First, deal with the power at the grid connection point (as measured via the CT)
        // remove most of the DC offset from the current sample (the precise value does not matter)
        long sampleIminusDC = ((long) (sampleI - DCoffset_I)) << 8;

        // phase-shift the voltage waveform so that it aligns with the grid current waveform
        long phaseShiftedSampleVminusDC = sampleVminusDC_long;

        // calculate the "real power" in this sample pair and add to the accumulated sum
        long filtV_div4 = phaseShiftedSampleVminusDC >> 2;  // reduce to 16-bits (now x64, or 2^6)
        long filtI_div4 = sampleIminusDC >> 2; // reduce to 16-bits (now x64, or 2^6)
        long instP = filtV_div4 * filtI_div4;  // 32-bits (now x4096, or 2^12)
        instP = instP >> 12;                   // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
        sumP += instP;                         // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)

        sampleSetsDuringThisMainsCycle++;

        // store items for use during next loop
        cumVdeltasThisCycle_long += sampleVminusDC_long; // for use with LP filter
        polarityConfirmedOfLastSampleV = polarityConfirmed;  // for identification of half cycle boundaries
    }
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
