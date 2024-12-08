/*
 * software_timer.h
 *
 *  Created on: 29 thg 10, 2024
 *      Author: ADMIN
 */

#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_

#include "main.h"
void setTimer(int index, int value);
int isTimerExpired(int index);
void timerRun();
void clear_timer_flag(int index);

#endif /* INC_SOFTWARE_TIMER_H_ */
