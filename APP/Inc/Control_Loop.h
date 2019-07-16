#ifndef _CONTROL_LOOP_
#define _CONTROL_LOOP_

#include "kalman_filter.h"

#define CH0Radio	((float)-0.0005)
#define CH1Radio	((float)-0.0003)
#define CH2Radio	((float)1)
#define CH3Radio	((float)10)  //ËÙ¶È 4000

extern void sysControl(void);
extern kalman_filter_t yaw_kalman_filter, pitch_kalman_filter;
extern kalman_filter_t yaw_velo_kf;
extern float kf0, kf1;
extern float sum;

#endif
