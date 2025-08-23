   #include <modbusRTU.h>
  #include <modbusDevice.h>
  #include <modbusRegBank.h>
  #include <modbusSlave.h>
  modbusDevice regBank;
  modbusSlave slave;
  float out1 = 0.0; float out2 = 0.0; float out3 = 0.0; float out4 = 0.0; float AIO; float AI1; float AI2; int AI3; float vpv = 0.0; float vbatt = 0.0; float eff = 0.0; float p1 = 0.0; float p2 = 0.0; int mVperAmp = 100; int ACSoffset = 2500; double Voltage2 = 0; double Voltage3 = 0; double Ampspv = 0; double Ampsbatt = 0; float R1 = 30000.0; float R2 = 7500.0; void setup()
  Serial.begin(9600);
  regBank.setId(10); ///Set Slave ID
  regBank.add(30001); regBank.add(30002); regBank.add(30003); regBank.ad
  d(3 0004); regBank.add(30005);
  slave._device = &regBank;
  slave.setBaud(9600);
  pinMode(0,INPUT); pinMode(1,INPUT); pinMode(2,INPUT); pinMode(3,I
  NP UT); }
  void loop()
  while(1)
  int AIO = analogRead(0); int AI1 = analogRead(1); int AI2 = analogRead(2); int AI3 = analogRead(3); vpv = (AIO  5.0)/1024.0;
  out1 = vpv/(R2/(R1+R2));
  vbatt = (AI2  5.0)/1024.0;
  out2 = vbatt/(R2/(R1+R2));
  Voltage2 = (AI1/1024.0)  5000;
  Ampspv = ((Voltage2 - ACSoffset)/mVperAmp); Voltage3 = (AI3/1024.0)  5000;
  Ampsbatt = ((Voltage3 - ACSoffset)/mVperAmp); p1 = vpv  Ampspv;
  p2 = vbatt  Ampsbatt;
  eff = p2/p1;
  Ampspv=abs(Ampspv);
  Ampsbatt=abs(Ampsbatt);
  out1=out1100;
  Ampspv= Ampspv(100);
  out2=out2100;
  Ampsbatt=Ampsbatt(100);
  eff=eff100;
  regBank.set(30001, (word) out1);
  regBank.set(30002, (word) Ampspv);
  regBank.set(30003, (word) out2);
  regBank.set(30004, (word) Ampsbatt);
  regBank.set(30005, (word) eff);
  delay(60000)
  slave.run();}}
