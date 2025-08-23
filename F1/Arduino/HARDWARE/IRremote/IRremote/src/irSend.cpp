#include "IRremote.h"

#ifdef SENDING_SUPPORTED
//+=============================================================================
void IRsend::sendRaw(const unsigned int buf[], unsigned int len, unsigned int hz) {
    // Set IR carrier frequency
    enableIROut(hz);

    for (unsigned int i = 0; i < len; i++) {
        if (i & 1) {
            space(buf[i]);
        } else {
            mark(buf[i]);
        }
    }

    space(0);  // Always end with the LED off
}

void IRsend::sendRaw_P(const unsigned int buf[], unsigned int len, unsigned int hz) {
#if !defined(__AVR__)
    sendRaw(buf,len,hz); // Let the function work for non AVR platforms
#else
    // Set IR carrier frequency
    enableIROut(hz);

    for (unsigned int i = 0; i < len; i++) {
        uint16_t duration = pgm_read_word_near(buf + sizeof(uint16_t) * i);
        if (i & 1) {
            space(duration);
        } else {
            mark(duration);
        }
    }
    space(0);  // Always end with the LED off
#endif

}

#ifdef USE_SOFT_CARRIER
void inline IRsend::sleepMicros(unsigned long us) {
#ifdef USE_SPIN_WAIT
    sleepUntilMicros(micros() + us);
#else
    if (us > 0U) { // Is this necessary? (Official docu https://www.arduino.cc/en/Reference/DelayMicroseconds does not tell.)
        delayMicroseconds((unsigned int) us);
    }
#endif
}

void inline IRsend::sleepUntilMicros(unsigned long targetTime) {
#ifdef USE_SPIN_WAIT
    while (micros() < targetTime)
    ;
#else
    unsigned long now = micros();
    if (now < targetTime) {
        sleepMicros(targetTime - now);
    }
#endif
}
#endif // USE_SOFT_CARRIER

//+=============================================================================
// Sends an IR mark for the specified number of microseconds.
// The mark output is modulated at the PWM frequency.
//





//+=============================================================================
// Custom delay function that circumvents Arduino's delayMicroseconds limit

void IRsend::custom_delay_usec(unsigned long uSecs) {
    if (uSecs > 4) {
        unsigned long start = micros();
        unsigned long endMicros = start + uSecs - 4;
        if (endMicros < start) { // Check if overflow
            while (micros() > start) {
            } // wait until overflow
        }
        while (micros() < endMicros) {
        } // normal wait
    }
    //else {
    //  __asm__("nop\n\t"); // must have or compiler optimizes out
    //}
}

#endif // SENDING_SUPPORTED
