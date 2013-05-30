#ifndef _DEFINITIONS_H
#define _DEFINITIONS_H

/*
 * Signal qualities, used to determine the availability of a control.
 * NO_SIGNAL means there was never a signal. SIGNAL_LOST that there was a signal, but it was lost.
 * SIGNAL_BAD is too bad for flight. This is the hysteresis range for deciding whether to engage
 * or disengage emergency landing.
 * SIGNAL_OK means the signal is usable for flight.
 * SIGNAL_GOOD means the signal can also be used for setting parameters.
 */
#define NO_SIGNAL   0
#define SIGNAL_LOST 1
#define SIGNAL_BAD  2
#define SIGNAL_OK   3
#define SIGNAL_GOOD 4

/*
 * The RPTY arrays
 */
#define CONTROL_ROLL     0
#define CONTROL_PITCH    1
#define CONTROL_THROTTLE 2
#define CONTROL_YAW      3


/*
 * The controls operate in [-1024, 1024] just about.
 * We compute control input and output in terms of 11 bit ranges then.
 */

#define LOG_CONTROL_RANGE 11
#define CONTROL_RANGE (1<<LOG_CONTROL_RANGE)

// For scaling control input quantities to bytes (configuration parameters).
#define LOG_CONTROL_BYTE_SCALING (LOG_CONTROL_RANGE - 8)

// MK BLC 1.x use 8 bit values. MK BLC 2.x use 10 bits. 10 bits is not implemented and probably doesnt 
// make a practical difference anyway.
#define LOG_I2C_CONTROL_RANGE 8

// Scale throttle levels to MK BLCs 8 bit range:
#define LOG_I2C_CONTROL_SCALING (LOG_CONTROL_RANGE - LOG_I2C_CONTROL_RANGE)

#endif
