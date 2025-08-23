
#ifndef HardwareSerial_h
#define HardwareSerial_h

#include "USARTClass.h"

extern void serialEventRun(void) __attribute__((weak));

extern USARTClass Serial;
extern USARTClass Serial1;
extern USARTClass Serial2;
extern USARTClass Serial3;
extern USARTClass Serial4;
extern USARTClass Serial5;
extern USARTClass Serial6;

//#include <inttypes.h>

//#include "Stream.h"


//class HardwareSerial : public Stream
//{
//  public:
//    void begin(unsigned long);
//    void end();
//    virtual int available(void) = 0;
//    virtual int peek(void) = 0;
//    virtual int read(void) = 0;
//    virtual void flush(void) = 0;
//    virtual size_t write(uint8_t) = 0;
//    using Print::write; // pull in write(str) and write(buf, size) from Print
//    virtual operator bool() = 0;
//};

//extern void serialEventRun(void) __attribute__((weak));

#endif

