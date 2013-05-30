#ifndef _DEBUG_H
#define _DEBUG_H

/*
 * Some digital output useful for timing and tracing things with a scope.
 */
#define J3HIGH    PORTD |= (1<<PORTD5)
#define J3LOW     PORTD &= ~(1<<PORTD5)
#define J3TOGGLE  PORTD ^= (1<<PORTD5)

#define J4HIGH    PORTD |= (1<<PORTD4)
#define J4LOW     PORTD &= ~(1<<PORTD4)
#define J4TOGGLE  PORTD ^= (1<<PORTD4)

#define J5HIGH    PORTD |= (1<<PORTD3)
#define J5LOW     PORTD &= ~(1<<PORTD3)
#define J5TOGGLE  PORTD ^= (1<<PORTD3)

// invert means: An "1" bit in digital debug data make a LOW on the output.
#define DIGITAL_DEBUG_INVERT 0

/*
 * Some digital debugs. A digital debug is 2 signals on the 2 LED outputs,
 * turned on and off depending on some condtions given in the code.
 * Only one can be selected, by defining DIGITAL_DEBUG_MASK to the value
 * of the debug.
 * In the code one can do like:
 * if (whatever_condition) {
 *  DebugOut.Digital[0] |= DEBUG_MYOWNDEBUGGER;
 * } else {
 *  DebugOut.Digital[0] &= ~DEBUG_MYOWNDEBUGGER;
 * }
 * ...
 * if (whatever_other_condition) {
 *  DebugOut.Digital[1] |= DEBUG_MYOWNDEBUGGER;
 * } else {
 *  DebugOut.Digital[1] &= ~DEBUG_MYOWNDEBUGGER;
 * }
 *
 * Digital debugs may be added as desired, and removed when the mystery
 * at hand is resolved.
 */

#define DEBUG_MAINLOOP_TIMER 1
#define DEBUG_HEIGHT_DIFF    2
#define DEBUG_INVERTED       4
#define DEBUG_COMMAND        8
#define DEBUG_COMPASS       16
#define DEBUG_PRESSURERANGE 32
#define DEBUG_CLIP          64
#define DEBUG_SENSORLIMIT  128

#define OUTPUTFLAGS_INVERT_0         1 // Inverted: 1 means low output on atmega. Does not affect on-board LED (if used with the OUTPUTOPTIONS_USE_ONBOARD_LEDS option)
#define OUTPUTFLAGS_INVERT_1         2 // Inverted: 1 means low output on atmega. Does not affect on-board LED (if used with the OUTPUTOPTIONS_USE_ONBOARD_LEDS option)
#define OUTPUTFLAGS_FLASH_0_AT_BEEP  4 // Flash LED when beeper beeps
#define OUTPUTFLAGS_FLASH_1_AT_BEEP  8 // Flash LED when beeper beeps
#define OUTPUTFLAGS_USE_ONBOARD_LEDS 16 // Control on-board LEDs in addition to outputs
#define OUTPUTFLAGS_TEST_OFF         32 // For testing: Turn off both outputs
#define OUTPUTFLAGS_TEST_ON          64 // For testing: Turn on both outputs

// For practical reasons put here instead of in uart0.h
typedef struct {
    uint8_t digital[2];
    int16_t analog[32]; // debug values
}__attribute__((packed)) DebugOut_t;

extern DebugOut_t debugOut;

#endif