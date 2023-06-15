
#define DEBUG

// On GD32 you can use custome uart pins only for Serial1 with: HardwareSerial oSerialSteer(rx_pin, Tx_pin,0);  // 1 = uart_index = Serial2 ; 0 = uart_index = Serial1
// HardwareSerial oSerialSteer(rx_pin, Tx_pin,1);  to reasign Serial2 does not seem to work.
// default pins can be found in VisualCode by selection Serial1 or Serial2 and click F12 a few times to find the place where SERIAL1_RX etc. is defined
#define UART_PASSTHROUGH  // also receive GD32_i2c_slave debug info via a second uart connection
//#define UART_SEND_TEST

#include <Wire.h>
#include "include.h"

void setup() 
{
  pinMode(pin_LED, OUTPUT);
  Serial.begin(115200);
  Serial.println("ESP32_S2_Mini I2c Master :-)");
  
  #if defined(UART_PASSTHROUGH) || defined(UART_SEND_TEST)
    Serial1.begin(115200, SERIAL_8N1, 16, 18);  //, pin_RX, pin_TX
  #endif
  
  Wire.begin(37,39);  // SDIO,CLK ESP32 S2 Mini
}

unsigned long iNow = 0;
unsigned long iTimeSend = 0;
void loop() 
{
  iNow = millis();
  digitalWrite(pin_LED, (iNow%500) < 200 ? HIGH : LOW);   // turn the LED on (HIGH is the voltage level)

    
  if (iTimeSend > iNow) 
    return;
  iTimeSend = iNow + TIME_SEND;


  #ifdef UART_PASSTHROUGH
    while (Serial.available())  // If anything comes in Serial (USB),
    {        
      Serial1.write(Serial.read());  // read it and send it out Serial1
    }
  
    while (Serial1.available()) // If anything comes in Serial1
    { 
      Serial.write(Serial1.read());  // read it and send it out Serial (USB)
    }
  #endif

 #ifdef UART_SEND_TEST
    Serial1.print("hello from ESP32 S2 Mini");  // uart: send data to slave
  #endif


  // i2c: send data to slave
  oMaster2Slave.iValue++;   // to see something change :-)
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  SerialWrite(Wire,oMaster2Slave);     // to send struct with crc: from openPanasonicBike
  byte error = Wire.endTransmission();
  if (error != 0) 
    OUT2N("master: i2c request failed, timeout error code",error)


  // i2c request data from slave
  Wire.requestFrom(I2C_SLAVE_ADDR, sizeof(oSlave2Master)+1);  // +1 because of one additional byte crc
  delay(1);
  unsigned int iAvail = Wire.available();
  if (SerialRead(Wire,oSlave2Master,iAvail))
  {
    OUT2T("slave.iValue",oSlave2Master.iValue);
    OUT2T("slave.fValue",oSlave2Master.fValue);
  }
  else  if (iAvail)
  {
    OUT2("\nmaster: CRC fail or too less avail: ",iAvail);
  }

 
}
