/*
 * software_timer.c
 *
 *  Created on: 29 thg 10, 2024
 *      Author: ADMIN
 */
#include "software_timer.h"
#define MAX_COUNTER 10
#define TIMER_TICK 10

int timer_counter[MAX_COUNTER];
int timer_flag[MAX_COUNTER];

void setTimer(int index, int value){
	timer_counter[index]=value/TIMER_TICK;
	timer_flag[index]=0;
}

int isTimerExpired(int index){
	if(timer_flag[index]==1){
		timer_flag[index]=0;
		return 1;
	}
	return 0;
}
void clear_timer_flag(int index){
	timer_flag[index]=0;
}
void timerRun(){
	for(int i=0;i<MAX_COUNTER;i++){
		if(timer_counter[i]>0){
			timer_counter[i]--;
			if(timer_counter[i]<=0) timer_flag[i]=1;
		}
	}
}
