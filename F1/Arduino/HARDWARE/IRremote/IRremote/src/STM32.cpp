#ifdef STM32

// This file contains functions specific to the STM32.

#include "IRremote.h"

// "Idiot check"
//#ifdef USE_DEFAULT_ENABLE_IR_IN
//#error Must undef USE_DEFAULT_ENABLE_IR_IN
//#endif
HardwareTimer *EthTim = new HardwareTimer(TIM3);
HardwareTimer *EthTim_0 = new HardwareTimer(TIM4);

//hw_timer_t *timerIR;
void IRTimer(); // defined in IRremote.cpp, masqueraded as ISR(TIMER_INTR_NAME)
void IR_IT_callback(HardwareTimer *htim)
{
  togglePin(PB12);
	IRTimer();
}
//+=============================================================================
// initialization
//
void IRrecv::enableIRIn() {
// Interrupt Service Routine - Fires every 50uS
    // ESP32 has a proper API to setup timers, no weird chip macros needed
    // simply call the readable API versions :)
    // 3 timers, choose #1, 80 divider nanosecond precision, 1 to count up

		EthTim->setMode(1, TIMER_OUTPUT_COMPARE,0);

  /* Timer set to 1ms */
		EthTim->setOverflow(50, MICROSEC_FORMAT);
		EthTim->attachInterrupt(IR_IT_callback);
		EthTim->resume();

    // Initialize state machine variables
    irparams.rcvstate = IR_REC_STATE_IDLE;
    irparams.rawlen = 0;

    // Set pin modes
    pinMode(irparams.recvpin, INPUT);
}

void IRrecv::disableIRIn() {
    EthTim->pause();
    EthTim->detachInterrupt();
}

void IRsend::enableIROut(int khz) {
//    ledcSetup(LEDCHANNEL, khz * 1000, 8);  // 8 bit PWM resolution
//    ledcAttachPin(IR_SEND_PIN, LEDCHANNEL); // bind pin to channel
	EthTim_0->setPWM(1,PB6,khz*1000,50);
	EthTim_0->pause();
}
void IRsend::mark(unsigned int time) {
#ifdef USE_SOFT_CARRIER
    unsigned long start = micros();
    unsigned long stop = start + time;
    if (stop + periodTime < start) {
        // Counter wrap-around, happens very seldomly, but CAN happen.
        // Just give up instead of possibly damaging the hardware.
        return;
    }
    unsigned long nextPeriodEnding = start;
    unsigned long now = micros();
    while (now < stop) {
        SENDPIN_ON(sendPin);
        sleepMicros (periodOnTime);
        SENDPIN_OFF(sendPin);
        nextPeriodEnding += periodTime;
        sleepUntilMicros(nextPeriodEnding);
        now = micros();
    }
#elif defined(USE_NO_CARRIER)
    digitalWrite(sendPin, LOW); // Set output to active low.
#else
    EthTim_0->resume();//TIMER_ENABLE_PWM; // Enable pin 3 PWM output
#endif
    // ! This is a bug, no need to delay here
    // if (time > 0) {
    //     custom_delay_usec(time);
    // }
}

//+=============================================================================
// Leave pin off for time (given in microseconds)
// Sends an IR space for the specified number of microseconds.
// A space is no output, so the PWM output is disabled.
//
void IRsend::space(unsigned int time) {
#if defined(USE_NO_CARRIER)
    digitalWrite(sendPin, HIGH); // Set output to inactive high.
#else
   EthTim_0->pause();// TIMER_DISABLE_PWM; // Disable pin 3 PWM output
#endif
    if (time > 0) {
        IRsend::custom_delay_usec(time);
    }
}

#endif // STM32
