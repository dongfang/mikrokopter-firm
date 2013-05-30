#ifndef _NAVICONTROL_H
#define _NAVICONTROL_H

#include<inttypes.h>

void navi_setNeutral(void);
void navigation_periodicTaskAndRPTY(int16_t* RPTY);

#endif
