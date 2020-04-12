#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>
#include <inttypes.h>

class Timer{
public:
    Timer(uint16_t period, unsigned long *c_time);

    void set_period(uint16_t period);
    void update();
    bool query();



private:
    unsigned long *c_time;
    unsigned long l_time;
    uint16_t period;
    bool triggered;
};

#endif