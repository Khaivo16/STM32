#ifndef __SOFTWIRE_H
#define __SOFTWIRE_H 	
#ifdef __cplusplus
 extern "C" {
#endif 
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 

typedef struct {
  uint8_t scl;
  uint8_t sda;
} Port;

/* You must update the online docs if you change this value. */
#define WIRE_BUFSIZ 101

/* return codes from endTransmission() */
#define SUCCESS   0        /* transmission was successful */
#define EDATA     1        /* too much data */
#define ENACKADDR 2        /* received nack on transmit of address */
#define ENACKTRNS 3        /* received nack on transmit of data */
#define EOTHER    4        /* other error */

#define SDA PB9
#define SCL PB8

#define I2C_WRITE 0
#define I2C_READ  1

#define I2C_DELAY do{for(int i=0;i<15 ;i++) {asm volatile("nop");}}while(0)//delay_us(1)

class SoftWire {
 private:
    uint8_t rx_buf[WIRE_BUFSIZ];      /* receive buffer */
    uint8_t rx_buf_idx;               /* first unread idx in rx_buf */
    uint8_t rx_buf_len;               /* number of bytes read */

    uint8_t tx_addr;                  /* address transmitting to */
    uint8_t tx_buf[WIRE_BUFSIZ];      /* transmit buffer */
    uint8_t tx_buf_idx;  /* next idx available in tx_buf, -1 overflow */
    boolean tx_buf_overflow;
    Port port;
    uint8_t writeOneByte(uint8_t);
    uint8_t readOneByte(uint8_t, uint8_t*);
 public:
    SoftWire();
    void begin();
    void begin(uint8_t, uint8_t);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t requestFrom(uint8_t, int);
    uint8_t requestFrom(int, int);
    void write(uint8_t);
    void write(uint8_t*, int);
    void write(int);
    void write(int*, int);
    void write(char*);
    uint8_t available();
    uint8_t read();
};

void    i2c_start(Port port);
void    i2c_stop(Port port);
boolean i2c_get_ack(Port port);
void    i2c_send_ack(Port port);
void    i2c_send_nack(Port port);
uint8_t   i2c_shift_in(Port port);
void    i2c_shift_out(Port port, uint8_t val);

extern SoftWire sWire;

#ifdef __cplusplus
}
#endif


#endif // _WIRE_H_
//////////////////////////////////////////////////////////////////////////////////

