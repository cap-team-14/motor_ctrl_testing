#include "Timer.h"

// constructor
Timer::Timer(uint16_t period, unsigned long *c_time){
    this->period = period;
    this->c_time = c_time;
    this->l_time = *c_time;

    this->update();
}


// change the period for this timer
void Timer::set_period(uint16_t period){
    this->period = period;
}


// update the status of this timer
void Timer::update(){
    if(l_time + period <= *c_time){
        this->triggered = true;
        this->l_time = *c_time;
    }
    //Serial.println(c_time);
}


// return the status of the timer
bool Timer::query(){
    update();
    bool temp = triggered;
    triggered = false;
    return temp;
}