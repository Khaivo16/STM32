#include <modbusRTU.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
modbusDevice regBank;
modbusSlave slave;

#define RS485TxEnablePin 2
#define RS485Baud 9600
#define RS485Format SERIAL_8E1


#define LED1 11
#define LED2 12

void setup()
{   
//Assign the modbus device ID.  
  regBank.setId(1);

/*
modbus registers follow the following format
00001-09999  Digital Outputs, A master device can read and write to these registers
10001-19999  Digital Inputs, A master device can only read the values from these registers
30001-39999  Analog Inputs, A master device can only read the values from these registers
40001-49999  Analog Outputs, A master device can read and write to these registers 

Analog values are 16 bit unsigned words stored with a range of 0-32767
Digital values are stored as bytes, a zero value is OFF and any nonzer value is ON

*/

//Add Analog Input registers to the register bank
  regBank.add(30001);  
  regBank.add(30002);  
 

//Add Analog Output registers to the register bank
  regBank.add(40001);  
  regBank.add(40002);


  slave._device = &regBank;  

  slave.setBaud(&Serial,RS485Baud,RS485Format,RS485TxEnablePin);   
pinMode(LED1, OUTPUT);
pinMode(LED2, OUTPUT); 
}

void loop()
{
  digitalWrite(LED1, regBank.get(40001));
  digitalWrite(LED2, regBank.get(40002));
  regBank.set(30001, (word) analogRead(A0)); //from 0 - 1023
  regBank.set(30002, (word) analogRead(A1)); //from 0 - 1023
  slave.run();  
}
