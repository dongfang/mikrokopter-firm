#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>

#include "analog.h"
#include "sensors.h"
// for Delay functions used in calibration.
#include "timer0.h"
// For reading and writing acc. meter offsets.
#include "eeprom.h"
#include "debug.h"

// set ADC enable & ADC Start Conversion & ADC Interrupt Enable bit
#define startADC() (ADCSRA |= (1<<ADEN)|(1<<ADSC)|(1<<ADIE))

// TODO: Off to PROGMEM .
const char* recal = ", recalibration needed.";

/*
 * Gyro and accelerometer values for attitude computation.
 * Unfiltered (this is unnecessary as noise should get absorbed in DCM).
 * Normalized to rad/s.
 * Data flow: ADCs (1 iteration) --> samplingADCData --offsetting-->  gyro_attitude_tmp
 * --rotation-->
 * [filtering] --> gyro_attitude.
 * Altimeter is also considered part of the "long" attitude loop.
 */
Vector3f gyro_attitude;
Vector3f accel;

/*
 * This stuff is for the aircraft control thread. It runs in unprocessed integers.
 * (well some sort of scaling will be required).
 * Data flow: ADCs (1 iteration) -> samplingADCData -> [offsetting and rotation] ->
 * [filtering] --> gyro_control
 */
int16_t gyro_control[3];
int16_t gyroD[2];
int16_t gyroDWindow[2][GYRO_D_WINDOW_LENGTH];
uint8_t gyroDWindowIdx;

/*
 * Air pressure. TODO: Might as well convert to floats / well known units.
 */
int32_t groundPressure;
int16_t dHeight;

/*
 * Offset values. These are the raw gyro and acc. meter sums when the copter is
 * standing still. They are used for adjusting the gyro and acc. meter values
 * to be centered on zero.
 */
sensorOffset_t gyroOffset;
sensorOffset_t accelOffset;
sensorOffset_t gyroAmplifierOffset;

/*
 * Redo this to that quadrant 0 is normal with an FC2.x.
 */
void rotate(int16_t* result, uint8_t quadrant, uint8_t reverse) {
    static const int8_t rotationTab[] = { 1, 1, 0, -1, -1, -1, 0, 1 };
    // Pitch to Pitch part
    int8_t xx = reverse ? rotationTab[(quadrant + 4) & 7] : rotationTab[quadrant]; // 1
    // Roll to Pitch part
    int8_t xy = rotationTab[(quadrant + 2) & 7]; // -1
    // Pitch to Roll part
    int8_t yx = reverse ? rotationTab[(quadrant + 2) & 7] : rotationTab[(quadrant + 6) & 7]; // -1  
    // Roll to Roll part
    int8_t yy = rotationTab[quadrant]; // -1

    int16_t xIn = result[0];
    int32_t tmp0, tmp1;

    tmp0 = xx * xIn + xy * result[1];
    tmp1 = yx * xIn + yy * result[1];

    if (quadrant & 1) {
        tmp0 = (tmp0 * 181L) >> 8;
        tmp1 = (tmp1 * 181L) >> 8;
    }

    result[0] = (int16_t) tmp0;
    result[1] = (int16_t) tmp1;
}

/*
 * Air pressure
 */
volatile uint8_t rangewidth = 105;

// Direct from sensor, irrespective of range.

// Value of 2 samples, with range.
uint16_t simpleAirPressure;

// Value of AIRPRESSURE_OVERSAMPLING samples, with range, filtered.
int32_t filteredAirPressure;

#define MAX_D_AIRPRESSURE_WINDOW_LENGTH 32
//int32_t lastFilteredAirPressure;
int16_t dAirPressureWindow[MAX_D_AIRPRESSURE_WINDOW_LENGTH];
uint8_t dWindowPtr = 0;

#define MAX_AIRPRESSURE_WINDOW_LENGTH 32
int16_t airPressureWindow[MAX_AIRPRESSURE_WINDOW_LENGTH];
int32_t windowedAirPressure;
uint8_t windowPtr = 0;

// Partial sum of AIRPRESSURE_SUMMATION_FACTOR samples.
int32_t airPressureSum;

// The number of samples summed into airPressureSum so far.
uint8_t pressureSumCount;

/*
 * Battery voltage, in units of: 1k/11k / 3V * 1024 = 31.03 per volt.
 * That is divided by 3 below, for a final 10.34 per volt.
 * So the initial value of 100 is for 9.7 volts.
 */
int16_t UBat = 100;

/*
 * Control and status.
 */
volatile uint16_t samplingADCData[8];
volatile uint16_t attitudeADCData[8];

volatile uint8_t analog_controlDataStatus = CONTROL_SENSOR_DATA_READY;
volatile uint8_t analog_attitudeDataStatus = ATTITUDE_SENSOR_NO_DATA;
// Number of ADC iterations done for current attitude loop.
volatile uint8_t attitudeSumCount;

volatile uint8_t ADCSampleCount;
volatile uint8_t adChannel;


const uint8_t channelsForStates[] PROGMEM = {
    AD_GYRO_X,
    AD_GYRO_Y,
    AD_GYRO_Z,

    AD_ACCEL_X,
    AD_ACCEL_Y,

    AD_GYRO_X,
    AD_GYRO_Y,
    //AD_GYRO_Z,

    AD_ACCEL_Z,
    AD_AIRPRESSURE,

    AD_GYRO_X,
    AD_GYRO_Y,
    AD_GYRO_Z,

    AD_ACCEL_X,
    AD_ACCEL_Y,

    AD_GYRO_X,
    AD_GYRO_Y,
    //AD_GYRO_Z,

    //AD_ACCEL_Z,
    //AD_AIRPRESSURE,

    AD_GYRO_X,
    AD_GYRO_Y,
    AD_GYRO_Z,

    AD_ACCEL_X,
    AD_ACCEL_Y,

    AD_GYRO_X,
    AD_GYRO_Y,
    //AD_GYRO_Z,

    AD_ACCEL_Z,
    AD_AIRPRESSURE,

    AD_GYRO_Y,
    AD_GYRO_X,
    AD_GYRO_Z,

    AD_ACCEL_X,
    AD_ACCEL_Y,

    AD_GYRO_X,
    AD_GYRO_Y,
    // AD_GYRO_Z,

    //AD_ACCEL_Z,
    //AD_AIRPRESSURE,
    AD_UBAT
};

// Feature removed. Could be reintroduced later - but should work for all gyro types then.
// uint8_t GyroDefectPitch = 0, GyroDefectRoll = 0, GyroDefectYaw = 0;

void analog_init(void) {
    uint8_t sreg = SREG;
    // disable all interrupts before reconfiguration
    cli();

    //ADC0 ... ADC7 is connected to PortA pin 0 ... 7
    DDRA = 0x00;
    PORTA = 0x00;
    // Digital Input Disable Register 0
    // Disable digital input buffer for analog adc_channel pins
    DIDR0 = 0xFF;
    // external reference, adjust data to the right
    ADMUX &= ~((1 << REFS1) | (1 << REFS0) | (1 << ADLAR));
    // set muxer to ADC adc_channel 0 (0 to 7 is a valid choice)
    ADMUX = (ADMUX & 0xE0);
    //Set ADC Control and Status Register A
    //Auto Trigger Enable, Prescaler Select Bits to Division Factor 128, i.e. ADC clock = SYSCKL/128 = 156.25 kHz
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    //Set ADC Control and Status Register B
    //Trigger Source to Free Running Mode
    ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

    for (uint8_t i = 0; i < MAX_AIRPRESSURE_WINDOW_LENGTH; i++) {
        airPressureWindow[i] = 0;
    }

    windowedAirPressure = 0;

    startADCCycle();

    // restore global interrupt flags
    SREG = sreg;
}

// Convert axis number (X, Y, Z to ADC channel mapping (1, 2, 0)
uint16_t gyroValue(uint8_t axis, volatile uint16_t dataArray[]) {
    switch (axis) {
    case X:
        return dataArray[AD_GYRO_X];
    case Y:
        return dataArray[AD_GYRO_Y];
    case Z:
        return dataArray[AD_GYRO_Z];
    default:
        return 0; // should never happen.
    }
}

uint16_t gyroValueForFC13DACCalibration(uint8_t axis) {
    return gyroValue(axis, samplingADCData);
}

// Convert axis number (X, Y, Z to ADC channel mapping (6, 7, 5)
uint16_t accValue(uint8_t axis, volatile uint16_t dataArray[]) {
    switch (axis) {
    case X:
        return dataArray[AD_ACCEL_X];
    case Y:
        return dataArray[AD_ACCEL_Y];
    case Z:
        return dataArray[AD_ACCEL_Z];
    default:
        return 0; // should never happen.
    }
}

/*
 * Min.: 0
 * Max: About 106 * 240 + 2047 = 27487; it is OK with just a 16 bit type.
 */
uint16_t getSimplePressure(int advalue) {
    uint16_t result = (uint16_t) OCR0A * /*(uint16_t)*/ rangewidth + advalue;
    result += (/*accel.z*/0 * (staticParams.airpressureAccZCorrection - 128)) >> 10;
    return result;
}

void startADCCycle(void) {
    for (uint8_t i=0; i<8; i++) {
        samplingADCData[i] = 0;
    }
    ADCSampleCount = 0;
    adChannel = AD_GYRO_X;
    ADMUX = (ADMUX & 0xE0) | adChannel;
    analog_controlDataStatus = CONTROL_SENSOR_SAMPLING_DATA;
    J4HIGH;
    startADC();
}

/*****************************************************
 * Interrupt Service Routine for ADC
 * Runs at 12 kHz. When all states are processed
 * further conversions are stopped.
 *****************************************************/
ISR( ADC_vect) {
    samplingADCData[adChannel] += ADC;
    // set up for next state.
    ADCSampleCount++;
    if (ADCSampleCount < sizeof(channelsForStates)) {
        adChannel = pgm_read_byte(&channelsForStates[ADCSampleCount]);
        // set adc muxer to next adChannel
        ADMUX = (ADMUX & 0xE0) | adChannel;
        // after full cycle stop further interrupts
        startADC();
    } else {
        J4LOW;
        analog_controlDataStatus = CONTROL_SENSOR_DATA_READY;
        // do not restart ADC converter.
    }
}

/*
 * Used in calibration only!
 * Wait the specified number of millis, and then run a full sensor ADC cycle.
 */
void waitADCCycle(uint16_t delay) {
    delay_ms(delay);
    startADCCycle();
    while(analog_controlDataStatus != CONTROL_SENSOR_DATA_READY)
        ;
}

void analog_updateControlData(void) {
    /*
     * 1) Near-saturation boost (dont bother with Z)
     * 2) Offset
     * 3) Rotation
     * 4) Filter
     * 5) Extract gyroD (should this be without near-saturation boost really? Ignore issue)
     */

    int16_t tempOffsetGyro[2];
    int16_t tempGyro;
    
    for (uint8_t axis=X; axis<=Y; axis++) {
        tempGyro = gyroValue(axis, samplingADCData);
        //debugOut.analog[3 + axis] = tempGyro;
        //debugOut.analog[3 + 2] = gyroValue(Z, samplingADCData);

        /*
         * Process the gyro data for the PID controller.
         */
        // 1) Extrapolate: Near the ends of the range, we boost the input significantly. This simulates a
        //    gyro with a wider range, and helps counter saturation at full control.
        //    There is hardly any reason to bother extrapolating yaw.

        if (staticParams.bitConfig & CFG_GYRO_SATURATION_PREVENTION) {
            if (tempGyro < SENSOR_MIN_XY) {
                debugOut.digital[0] |= DEBUG_SENSORLIMIT;
                tempGyro = tempGyro * EXTRAPOLATION_SLOPE - EXTRAPOLATION_LIMIT;
            } else if (tempGyro > SENSOR_MAX_XY) {
                debugOut.digital[0] |= DEBUG_SENSORLIMIT;
                tempGyro = (tempGyro - SENSOR_MAX_XY) * EXTRAPOLATION_SLOPE + SENSOR_MAX_XY;
            }
        }

        // 2) Apply offset (rotation will take care of signs).
        tempOffsetGyro[axis] = tempGyro - gyroOffset.offsets[axis];
    }

    // 2.1: Transform axes.
    rotate(tempOffsetGyro, IMUConfig.gyroQuadrant, IMUConfig.imuReversedFlags & IMU_REVERSE_GYRO_XY);

    for (uint8_t axis=X; axis<=Y; axis++) {
        // Filter. There is no filter for Z and no need for one.
      
      tempGyro = (gyro_control[axis] * (IMUConfig.gyroPIDFilterConstant - 1) + tempOffsetGyro[axis]) / IMUConfig.gyroPIDFilterConstant;
        // 5) Differential measurement.
        int16_t diff = tempGyro - gyro_control[axis];
        gyroD[axis] -= gyroDWindow[axis][gyroDWindowIdx];
        gyroD[axis] += diff;
        gyroDWindow[axis][gyroDWindowIdx] = diff;

        // 6) Done.
        gyro_control[axis] = tempGyro;
    }

    if (++gyroDWindowIdx >= IMUConfig.gyroDWindowLength) {
        gyroDWindowIdx = 0;
    }
    
    if (IMUConfig.imuReversedFlags & IMU_REVERSE_GYRO_Z)
      tempGyro = gyroOffset.offsets[Z] - gyroValue(Z, samplingADCData);
    else
      tempGyro = gyroValue(Z, samplingADCData) - gyroOffset.offsets[Z];

    gyro_control[Z] = tempGyro;
    
    startADCCycle();
}

/*
 * The uint16s can take a max. of 1<<16-10) = 64 samples summed.
 * Assuming a max oversampling count of 8 for the control loop, this is 8 control loop iterations
 * summed. After 8 are reached, we just throw away all further data. This (that the attitude loop
 * is more than 8 times slower than the control loop) should not happen anyway so there is no waste.
 */
#define MAX_OVEROVERSAMPLING_COUNT 8

void analog_sumAttitudeData(void) {
    // From when this procedure completes, there is attitude data available.
    if (analog_attitudeDataStatus == ATTITUDE_SENSOR_NO_DATA)
        analog_attitudeDataStatus = ATTITUDE_SENSOR_DATA_READY;


    if (analog_attitudeDataStatus == ATTITUDE_SENSOR_DATA_READY && attitudeSumCount < MAX_OVEROVERSAMPLING_COUNT) {
        for (uint8_t i = 0; i < 8; i++) {
            attitudeADCData[i] += samplingADCData[i];
        }
        attitudeSumCount++;
        debugOut.analog[24] = attitudeSumCount;
    }
}

void clearAttitudeData(void) {
    for (uint8_t i = 0; i < 8; i++) {
        attitudeADCData[i] = 0;
    }
    attitudeSumCount = 0;
    analog_attitudeDataStatus = ATTITUDE_SENSOR_NO_DATA;
}

void updateAttitudeVectors(void) {
    /*
     int16_t gyro_attitude_tmp[3];
     Vector3f gyro_attitude;
     Vector3f accel;
     */

    int16_t tmpSensor[3];

    // prevent gyro_attitude_tmp and attitudeSumCount from being updated.
    // TODO: This only prevents interrupts from starting. Well its good enough really?
    analog_attitudeDataStatus = ATTITUDE_SENSOR_READING_DATA;

    tmpSensor[X] = gyroValue(X, attitudeADCData) - gyroOffset.offsets[X] * attitudeSumCount;
    tmpSensor[Y] = gyroValue(Y, attitudeADCData) - gyroOffset.offsets[Y] * attitudeSumCount;

    rotate(tmpSensor, IMUConfig.gyroQuadrant, IMUConfig.imuReversedFlags & IMU_REVERSE_GYRO_XY);

    if (IMUConfig.imuReversedFlags & IMU_REVERSE_GYRO_Z)
        tmpSensor[Z] = gyroOffset.offsets[Z] * attitudeSumCount - gyroValue(Z, attitudeADCData);
    else
        tmpSensor[Z] = gyroValue(Z, attitudeADCData) - gyroOffset.offsets[Z] * attitudeSumCount;

    gyro_attitude.x = ((float) tmpSensor[X]) / (GYRO_RATE_FACTOR_XY * attitudeSumCount);
    gyro_attitude.y = ((float) tmpSensor[Y]) / (GYRO_RATE_FACTOR_XY * attitudeSumCount);
    gyro_attitude.z = ((float) tmpSensor[Z]) / (GYRO_RATE_FACTOR_Z  * attitudeSumCount);

    // Done with gyros. Now accelerometer:
    tmpSensor[X] = accValue(X, attitudeADCData) - accelOffset.offsets[X] * attitudeSumCount;
    tmpSensor[Y] = accValue(Y, attitudeADCData) - accelOffset.offsets[Y] * attitudeSumCount;

    rotate(tmpSensor, IMUConfig.accQuadrant, IMUConfig.imuReversedFlags & IMU_REVERSE_ACCEL_XY);

    // Z acc.
    if (IMUConfig.imuReversedFlags & IMU_REVERSE_ACCEL_Z)
        tmpSensor[Z] = accelOffset.offsets[Z] * attitudeSumCount - accValue(Z, attitudeADCData);
    else
        tmpSensor[Z] = accValue(Z, attitudeADCData) - accelOffset.offsets[Z] * attitudeSumCount;

    // Blarh!!! We just skip acc filtering. There are trillions of samples already.
    accel.x = (float)tmpSensor[X] / (ACCEL_FACTOR_XY * attitudeSumCount); // (accel.x + (float)tmpSensor[X] / (ACCEL_FACTOR_XY * attitudeSumCount)) / 2.0;
    accel.y = (float)tmpSensor[Y] / (ACCEL_FACTOR_XY * attitudeSumCount); // (accel.y + (float)tmpSensor[Y] / (ACCEL_FACTOR_XY * attitudeSumCount)) / 2.0;
    accel.z = (float)tmpSensor[Z] / (ACCEL_FACTOR_Z  * attitudeSumCount); // (accel.z + (float)tmpSensor[Z] / (ACCEL_FACTOR_Z  * attitudeSumCount)) / 2.0;

    for (uint8_t i=0; i<3; i++) {
      debugOut.analog[3 + i] = (int16_t)(gyro_attitude[i] * 100);
      debugOut.analog[6 + i] = (int16_t)(accel[i] * 100);
    }
}

void analog_updateAirPressure(void) {
    static uint16_t pressureAutorangingWait = 25;
    uint16_t rawAirPressure;
    int16_t newrange;
    // air pressure
    if (pressureAutorangingWait) {
        //A range switch was done recently. Wait for steadying.
        pressureAutorangingWait--;
    } else {
        rawAirPressure = attitudeADCData[AD_AIRPRESSURE] / attitudeSumCount;
        if (rawAirPressure < MIN_RAWPRESSURE) {
            // value is too low, so decrease voltage on the op amp minus input, making the value higher.
            newrange = OCR0A - (MAX_RAWPRESSURE - MIN_RAWPRESSURE) / (rangewidth * 4);
            if (newrange > MIN_RANGES_EXTRAPOLATION) {
                pressureAutorangingWait = (OCR0A - newrange) * AUTORANGE_WAIT_FACTOR; // = OCRA0 - OCRA0 +
                OCR0A = newrange;
            } else {
                if (OCR0A) {
                    OCR0A--;
                    pressureAutorangingWait = AUTORANGE_WAIT_FACTOR;
                }
            }
        } else if (rawAirPressure > MAX_RAWPRESSURE) {
            // value is too high, so increase voltage on the op amp minus input, making the value lower.
            // If near the end, make a limited increase
            newrange = OCR0A + (MAX_RAWPRESSURE - MIN_RAWPRESSURE) / (rangewidth * 4);
            if (newrange < MAX_RANGES_EXTRAPOLATION) {
                pressureAutorangingWait = (newrange - OCR0A) * AUTORANGE_WAIT_FACTOR;
                OCR0A = newrange;
            } else {
                if (OCR0A < 254) {
                    OCR0A++;
                    pressureAutorangingWait = AUTORANGE_WAIT_FACTOR;
                }
            }
        }

        // Even if the sample is off-range, use it.
        simpleAirPressure = getSimplePressure(rawAirPressure);
        
        if (simpleAirPressure < (uint16_t)(MIN_RANGES_EXTRAPOLATION * rangewidth)) {
            // Danger: pressure near lower end of range. If the measurement saturates, the
            // copter may climb uncontrolledly... Simulate a drastic reduction in pressure.
            debugOut.digital[1] |= DEBUG_SENSORLIMIT;
            airPressureSum += (int16_t) MIN_RANGES_EXTRAPOLATION * rangewidth
                    + (simpleAirPressure - (int16_t) MIN_RANGES_EXTRAPOLATION
                            * rangewidth) * PRESSURE_EXTRAPOLATION_COEFF;
        } else if (simpleAirPressure > (uint16_t)(MAX_RANGES_EXTRAPOLATION * rangewidth)) {
            // Danger: pressure near upper end of range. If the measurement saturates, the
            // copter may descend uncontrolledly... Simulate a drastic increase in pressure.
            debugOut.digital[1] |= DEBUG_SENSORLIMIT;
            airPressureSum += (int16_t) MAX_RANGES_EXTRAPOLATION * rangewidth
                    + (simpleAirPressure - (int16_t) MAX_RANGES_EXTRAPOLATION
                            * rangewidth) * PRESSURE_EXTRAPOLATION_COEFF;
        } else {
            // normal case.
            // If AIRPRESSURE_OVERSAMPLING is an odd number we only want to add half the double sample.
            // The 2 cases above (end of range) are ignored for this.
            debugOut.digital[1] &= ~DEBUG_SENSORLIMIT;
            airPressureSum += simpleAirPressure;
        }

        // 2 samples were added.
        pressureSumCount += 2;
        // Assumption here: AIRPRESSURE_OVERSAMPLING is even (well we all know it's 28...)
        if (pressureSumCount == AIRPRESSURE_OVERSAMPLING) {

            // The best oversampling count is 14.5. We add a quarter of the double ADC value to get the final half.
            airPressureSum += simpleAirPressure >> 2;

            uint32_t lastFilteredAirPressure = filteredAirPressure;

            if (!staticParams.airpressureWindowLength) {
                filteredAirPressure = (filteredAirPressure
                        * (staticParams.airpressureFilterConstant - 1)
                        + airPressureSum
                        + staticParams.airpressureFilterConstant / 2)
                        / staticParams.airpressureFilterConstant;
            } else {
                // use windowed.
                windowedAirPressure += simpleAirPressure;
                windowedAirPressure -= airPressureWindow[windowPtr];
                airPressureWindow[windowPtr++] = simpleAirPressure;
                if (windowPtr >= staticParams.airpressureWindowLength)
                    windowPtr = 0;
                filteredAirPressure = windowedAirPressure / staticParams.airpressureWindowLength;
            }

            // positive diff of pressure
            int16_t diff = filteredAirPressure - lastFilteredAirPressure;
            // is a negative diff of height.
            dHeight -= diff;
            // remove old sample (fifo) from window.
            dHeight += dAirPressureWindow[dWindowPtr];
            dAirPressureWindow[dWindowPtr++] = diff;
            if (dWindowPtr >= staticParams.airpressureDWindowLength)
                dWindowPtr = 0;
            pressureSumCount = airPressureSum = 0;
        }
    }
    
    debugOut.analog[25] = simpleAirPressure;
    debugOut.analog[26] = OCR0A;
    debugOut.analog[27] = filteredAirPressure;
}

void analog_updateBatteryVoltage(void) {
    // Battery. The measured value is: (V * 1k/11k)/3v * 1024 = 31.03 counts per volt (max. measurable is 33v).
    // This is divided by 3 --> 10.34 counts per volt.
    UBat = (3 * UBat + attitudeADCData[AD_UBAT] / 3) / 4;
}

void analog_updateAttitudeData(void) {
    updateAttitudeVectors();

    // TODO: These are waaaay off by now.
    analog_updateAirPressure();
    analog_updateBatteryVoltage();

    clearAttitudeData();
}

void analog_setNeutral() {
    gyro_init();

    if (gyroOffset_readFromEEProm()) {
        printf("gyro offsets invalid%s", recal);
        gyroOffset.offsets[X] = gyroOffset.offsets[Y] = 512 * GYRO_OVERSAMPLING_XY;
        gyroOffset.offsets[Z] = 512 * GYRO_OVERSAMPLING_Z;
        // This will get the DACs for FC1.3 to offset to a reasonable value.
        gyro_calibrate();
    }

    if (accelOffset_readFromEEProm()) {
        printf("acc. meter offsets invalid%s", recal);
        accelOffset.offsets[X] = accelOffset.offsets[Y] = 512 * ACCEL_OVERSAMPLING_XY;
        accelOffset.offsets[Z] = 512 * ACCEL_OVERSAMPLING_Z;
        if (IMUConfig.imuReversedFlags & IMU_REVERSE_ACCEL_Z) {
            accelOffset.offsets[Z] -= ACCEL_G_FACTOR_Z;
        } else {
            accelOffset.offsets[Z] += ACCEL_G_FACTOR_Z;
        }
    }

    // Noise is relative to offset. So, reset noise measurements when changing offsets.
    for (uint8_t i=X; i<=Y; i++) {
        // gyroNoisePeak[i] = 0;
        gyroD[i] = 0;
        for (uint8_t j=0; j<GYRO_D_WINDOW_LENGTH; j++) {
            gyroDWindow[i][j] = 0;
        }
    }
    // Setting offset values has an influence in the analog.c ISR
    // Therefore run measurement for 100ms to achive stable readings
    waitADCCycle(100);
}

void analog_calibrateGyros(void) {
#define GYRO_OFFSET_CYCLES 100
    uint8_t i, axis;
    int32_t offsets[3] = { 0, 0, 0 };

    flightControlStatus = BLOCKED_FOR_CALIBRATION;
    delay_ms(10);

    gyro_calibrate();

    // determine gyro bias by averaging (requires that the copter does not rotate around any axis!)
    for (i = 0; i < GYRO_OFFSET_CYCLES; i++) {
        waitADCCycle(5);
        for (axis=X; axis<=Z; axis++) {
            offsets[axis] += gyroValue(axis, samplingADCData);
        }
    }

    for (axis=X; axis<=Z; axis++) {
        gyroOffset.offsets[axis] = (offsets[axis] + GYRO_OFFSET_CYCLES/2) / GYRO_OFFSET_CYCLES;
        int16_t min = (512 - 200) * (axis==Z) ? GYRO_OVERSAMPLING_Z : GYRO_OVERSAMPLING_XY;
        int16_t max = (512 + 200) * (axis==Z) ? GYRO_OVERSAMPLING_Z : GYRO_OVERSAMPLING_XY;
        if (gyroOffset.offsets[axis] < min || gyroOffset.offsets[axis] > max)
            versionInfo.hardwareErrors[0] |= FC_ERROR0_GYRO_X << axis;
    }

    gyroOffset_writeToEEProm();
    //startADCCycle();
}

/*
 * Find acc. offsets for a neutral reading, and write them to EEPROM.
 * Does not (!} update the local variables. This must be done with a
 * call to analog_calibrate() - this always (?) is done by the caller
 * anyway. There would be nothing wrong with updating the variables
 * directly from here, though.
 */
void analog_calibrateAcc(void) {
#define ACCEL_OFFSET_CYCLES 100
    uint8_t i, axis;
    int32_t offsets[3] = { 0, 0, 0 };

    flightControlStatus = BLOCKED_FOR_CALIBRATION;
    delay_ms(10);

    for (i = 0; i < ACCEL_OFFSET_CYCLES; i++) {
        waitADCCycle(5);
        for (axis=X; axis<=Z; axis++) {
            offsets[axis] += accValue(axis, samplingADCData);
        }
    }

    for (axis=X; axis<=Z; axis++) {
        accelOffset.offsets[axis] = (offsets[axis] + ACCEL_OFFSET_CYCLES / 2) / ACCEL_OFFSET_CYCLES;
        int16_t min, max;
        if (axis == Z) {
            if (IMUConfig.imuReversedFlags & IMU_REVERSE_ACCEL_Z) {
                // TODO: This assumes a sensitivity of +/- 2g.
                min = (256 - 200) * ACCEL_OVERSAMPLING_Z;
                max = (256 + 200) * ACCEL_OVERSAMPLING_Z;
            } else {
                // TODO: This assumes a sensitivity of +/- 2g.
                min = (768 - 200) * ACCEL_OVERSAMPLING_Z;
                max = (768 + 200) * ACCEL_OVERSAMPLING_Z;
            }
        } else {
            min = (512 - 200) * ACCEL_OVERSAMPLING_XY;
            max = (512 + 200) * ACCEL_OVERSAMPLING_XY;
        }
        if (gyroOffset.offsets[axis] < min || gyroOffset.offsets[axis] > max) {
            versionInfo.hardwareErrors[0] |= FC_ERROR0_ACCEL_X << axis;
        }
    }

    if (IMUConfig.imuReversedFlags & IMU_REVERSE_ACCEL_Z) {
        accelOffset.offsets[Z] -= ACCEL_G_FACTOR_Z;
    } else {
        accelOffset.offsets[Z] += ACCEL_G_FACTOR_Z;
    }

    accelOffset_writeToEEProm();
    // startADCCycle();
}

void analog_setGround() {
    groundPressure = filteredAirPressure;
}

int32_t analog_getHeight(void) {
    int32_t height = groundPressure - filteredAirPressure;
    debugOut.analog[28] = height;
    return height;
}

int16_t analog_getDHeight(void) {
    return dHeight;
}
