#include <Arduino.h>
#include <Config.h>
#include <Defines.h>
#include <Wire.h>

/*
#ifdef DEBUG_UART // use Serial1 with differen rx/tx pins as DEBUG_UART
  HardwareSerial oSerialSteer(PB7, PB6,0);  // 1 = uart_index = Serial2 ; 0 = uart_index = Serial1
  #undef DEBUG_UART
  #define DEBUG_UART oSerialSteer
#endif
*/


class CIO
{	
private:
	int m_iPin;
	int m_iType;  // OUTPUT, INPUT, INPUT_PULLUP, INPUT_PULLDOWN
public:
	CIO(int iPin, int iType);
	void Init();
	void Set(bool bOn = true);
	bool Get(void);
};

CIO::CIO(int iPin, int iType=INPUT){	m_iPin = iPin;  m_iType = iType;}
void CIO::Init(){  pinMode(m_iPin, m_iType);  }	
void CIO::Set(bool bOn){ digitalWrite(m_iPin,bOn); }	
bool CIO::Get(void){  return digitalRead(m_iPin); }


CIO oKeepOn = CIO(SELF_HOLD_PIN,OUTPUT);
CIO oOnOff = CIO(BUTTON_PIN);
CIO oLedRed     = CIO(LED_RED,OUTPUT);


unsigned int iRequests = 0;   // log how often requestEvent() is called
unsigned int iReceived = 0;   // log how often receiveEvent() is called

struct __attribute((__packed__)) Master2Slave 
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

template <typename O,typename T> unsigned int SerialWrite (O& oSerial, const T& o)  // for uart and i2c :-)
{
  oSerial.write((byte *) &o, sizeof (o));
  byte iCRC8 = CRC8((byte *) &o,sizeof (o));
  oSerial.write(iCRC8);
  return sizeof (o);
}

template <typename C,typename S, typename I> bool SerialRead(C& oSerial,S& o, I iAvailable) // for uart and i2c :-)
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


// function that executes whenever data is requested by master
void requestEvent() 
{
  iRequests++;
  SerialWrite(Wire,oSlave2Master);     // to send struct with crc: from openPanasonicBike
}

void receiveEvent(int iAvail) 
{
  iReceived++;
  if (SerialRead(Wire,oMaster2Slave,iAvail))
  {
    oSlave2Master.iValue = oMaster2Slave.iValue;
    oSlave2Master.fValue += 0.01;
  }
  else  if (iAvail)
  {
    OUT2("\nCRC fail. avail: ",iAvail);
  }

  DEBUG( 
    OUT2T("slave",millis()) 
    OUT2T("iRequests",iRequests);
    OUT2(" iReceived",iReceived);
    OUTLN()
  )
}

void setup()
{
  oKeepOn.Init();     // hoverboard needs this to keep power on
  oKeepOn.Set(true);  // now we can release the oOnOff button :-)
  oOnOff.Init();

  #ifdef DEBUG_UART
    DEBUG_UART.begin(DEBUG_UART_BAUD);
  #endif

  OUTLN("GD32 I2C Slave example :-)")

  oLedRed.Init();
  oLedRed.Set(true);
  
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // receiveEvent called when data from master received
  Wire.onRequest(requestEvent); // called when master requests data from this slave

  oLedRed.Set(false);

}

unsigned long iTimeSend = 0;

void loop()
{
  unsigned long iNow = millis();

  oLedRed.Set((iNow%1000) < 200);


  if (iTimeSend > iNow) 
    return;
  iTimeSend = iNow + TIME_SEND;

  if (oOnOff.Get()) oKeepOn.Set(false);   // turn off hoverboard 

  #ifdef SERIALDEBUG  // with UART_SEND_TEST on I2C_Master, rx gets sent back
    if (SERIALDEBUG.available()) 
    {
      while (SERIALDEBUG.available() )
      {
        SERIALDEBUG.write(SERIALDEBUG.read());
      }
      SERIALDEBUG.println();
    }
  #endif
}