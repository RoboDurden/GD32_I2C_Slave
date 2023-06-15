# GD32 I2C Slave Test :-)

GD32F130C6 and GD32F130C8 i2c slave test code.
GD32: PlaformIO/VisualCode , ESP32: Arduino IDE

EPS32 log output:
```
slave.iValue: 95	 slave.fValue: 10.47	 slave: 100967	iRequests: 732	 iReceived: 733
slave.iValue: 96	 slave.fValue: 10.48	 slave: 101068	iRequests: 733	 iReceived: 734
slave.iValue: 97	 slave.fValue: 10.49	 slave: 101169	iRequests: 734	 iReceived: 735
slave.iValue: 98	 slave.fValue: 10.50	 slave: 101270	iRequests: 735	 iReceived: 736
slave.iValue: 99	 slave.fValue: 10.51	 slave: 101370	iRequests: 736	 iReceived: 737
slave.iValue: 100	 slave.fValue: 10.52	 slave: 101472	iRequests: 737	 iReceived: 738
slave.iValue: 101	 slave.fValue: 10.53	 slave: 101573	iRequests: 738	 iReceived: 739
slave.iValue: 102	 slave.fValue: 10.54	 slave: 101674	iRequests: 739	 iReceived: 740
```

`slave.iValue: 102	 slave.fValue: 10.54` are the GD32 i2c received, incremented and i2c returned values. ESP32 code:

```
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
```



`slave: 101674	iRequests: 739	 iReceived: 740` is the uart GD32 debug output starting with it's `millis()`. GD32 code:

```
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
```


<img src="https://raw.githubusercontent.com/RoboDurden/GD32_I2C_Slave/main/img/Gen2.0%20test%20setup.jpg?raw=true" width="80%"/>

<img src="https://github.com/RoboDurden/GD32_I2C_Slave/blob/main/img/Esp32%20S2%20Mini%20i2+uart.jpg?raw=true" width="80%"/>

<img src="https://github.com/RoboDurden/GD32_I2C_Slave/blob/main/img/ESP32%20S2%20Mini%20pinout.jpg?raw=true" width="50%"/>

best chances to find a gen2.0 board is inside old 25V (7s liIon) hoverboards with these bldc motos:
<img src="https://github.com/RoboDurden/GD32_I2C_Slave/blob/main/img/Gen2.0%20motors.jpg?raw=true" width="30%"/>

For the different Gen2 layoutouts look here: https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x

Thanks to [Candas1](https://github.com/Candas1) for the [pin definitions spreadsheet](https://docs.google.com/spreadsheets/d/15msbDAIMxC2rIkq8Au8vf82ub1qEaS67Lc1cFb86Jpc/edit#gid=0) 
