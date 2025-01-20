#ifndef SOFTWARE_TIMER_H
#define SOFTWARE_TIMER_H

#include <Arduino.h>

class _TimerLib {
public:
    _TimerLib();
    void setInterval_us(void (*isr)(), unsigned long interval);
    void clearTimer();
    void timerLoop();

private:
    void (*isr)();
    unsigned long interval;
    unsigned long lastMillis;
    bool timerActive;
};


_TimerLib::_TimerLib() : isr(nullptr), interval(0), lastMillis(0), timerActive(false) {}

void _TimerLib::setInterval_us(void (*isr)(), unsigned long interval) {
    this->isr = isr;
    this->interval = interval / 1000; // 将微秒转换为毫秒
    this->lastMillis = millis();
    this->timerActive = true;
}

void _TimerLib::clearTimer() {
    this->timerActive = false;
    this->isr = nullptr;
}

void _TimerLib::timerLoop() {
    if (timerActive && isr != nullptr) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastMillis >= interval) {
            lastMillis = currentMillis;
            isr();
        }
    }
}

// 用软件实现的计时器，timerLoop 成员函数需要在 loop 函数中调用，以检查定时器是否到期并调用中断服务程序（ISR）
_TimerLib TimerLib;

#endif // SOFTWARE_TIMER_H