#include <inttypes.h>
#include <string.h>
#include "profiler.h"

uint8_t currentProfiledActivity;
volatile uint32_t totalProfilerHits;
volatile uint16_t activitiesTimerHits[16];

char profilerLabel0[]  PROGMEM = "Unaccounted";
char profilerLabel1[]  PROGMEM = "AnalogUpd";
char profilerLabel2[]  PROGMEM = "MatrixUpd1";
char profilerLabel3[]  PROGMEM = "MatrixUpd2";
char profilerLabel4[]  PROGMEM = "Normalize1";
char profilerLabel5[]  PROGMEM = "Normalize2";
char profilerLabel6[]  PROGMEM = "DriftCorr";
char profilerLabel7[]  PROGMEM = "CheckMatrix";
char profilerLabel8[]  PROGMEM = "EulerAngles";
char profilerLabel9[]  PROGMEM = "AnglesOutput";
char profilerLabel10[] PROGMEM = "ControlMixer";
char profilerLabel11[] PROGMEM = "Commands";
char profilerLabel12[] PROGMEM = "FlightControl";
char profilerLabel13[] PROGMEM = "UART";
char profilerLabel14[] PROGMEM = "Outputs";
char profilerLabel15[] PROGMEM = "";

PGM_P PROFILER_LABELS[] PROGMEM = {
    profilerLabel0,
    profilerLabel1,
    profilerLabel2,
    profilerLabel3,
    profilerLabel4,
    profilerLabel5,
    profilerLabel6,
    profilerLabel7,
    profilerLabel8,
    profilerLabel9,
    profilerLabel10,
    profilerLabel11,
    profilerLabel12,
    profilerLabel13,
    profilerLabel14,
    profilerLabel15
};

void setCurrentProfiledActivity(uint8_t what) {
  currentProfiledActivity = what;
}

void reset(void) {
  memset((uint8_t*)&activitiesTimerHits, 0, sizeof(activitiesTimerHits));
  totalProfilerHits = 0;
}

void profiler_scoreTimerHit(void) {
  activitiesTimerHits[currentProfiledActivity]++;
  totalProfilerHits++;
}
