
#define TIME_SEND 100
#define pin_LED 15
#define I2C_SLAVE_ADDR 8

typedef struct __attribute((__packed__)) Master2Slave 
{
  byte iValue = 0;
  float fValue = 3.14;
} ;

Master2Slave oMaster2Slave;
Master2Slave oSlave2Master;



//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

template <typename O,typename T> unsigned int SerialWrite (O& oSerial, const T& o)
{
  oSerial.write((byte *) &o, sizeof (o));
  byte iCRC8 = CRC8((byte *) &o,sizeof (o));
  oSerial.write(iCRC8);
  return sizeof (o);
}


template <typename C,typename S, typename I> bool SerialRead(C& oSerial,S& o, I iAvailable)
{
  if (iAvailable < sizeof(o)+1 ) 
    return false;
  
  byte buff[sizeof(o)];
  byte* p = buff;
  for (unsigned int i=0; i < sizeof o; i++)
      *p++ = oSerial.read();
  byte iMust = CRC8(buff,sizeof(o) );
  byte iCRC8 = oSerial.read();
  if (iMust != iCRC8) 
    return false;
  memcpy(&o,buff,sizeof(o));
  return true;
}

#ifdef DEBUG
  #define DEBUG(code) {code}
  #define OUT(s)  {Serial.print(s);}
  #define OUT2(s,i) {Serial.print(s);Serial.print(": ");Serial.print(i);}
  #define OUT2T(s,i)  {Serial.print(s);Serial.print(": ");Serial.print(i);Serial.print("\t ");}
  #define OUT2N(s,i)  {Serial.print(s);Serial.print(": ");Serial.print(i);Serial.print("\n");}
  #define OUTN(s)  {Serial.println(s);}
#else
  #define DEBUG(code)
  #define OUT(s)
  #define OUT2(s)
  #define OUT2T(s)
  #define OUT2N(s)
  #define OUTN(s)
#endif
