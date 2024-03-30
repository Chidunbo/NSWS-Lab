#include "OPC.h"
// SsetSSpin = high -> turn off device
// pull down = trigger

OPC::OPC(uint8_t ssPin, uint8_t chipSelect)
  : ssPin_OPC(ssPin),ssPin_SD(chipSelect), cloopTime(0), currentTime(0), SPI_in_index(0){}

void OPC::begin() {
  wdt_reset(); //Reset watchdog timer
  wdt_enable(WDTO_8S); //Enable watchdog timer, countdown 8s (max)

  Serial.begin(9600);
  
  pinMode(ssPin_OPC, OUTPUT); // OPC cs config
  pinMode(ssPin_SD, OUTPUT);// OPC cs config
  
  pinMode(pin_output_control, OUTPUT); // mutiplux config
  digitalWrite(pin_output_control, LOW); // ALWAYS LOW
  
  pinMode(pin_select, OUTPUT); // LOW for SD, HIGH for sensor


  delay(1000); //delay in case of noise on power connection. Also allows OPC to boot up.
  
  PrintFirmwareVer(Serial);

  // start the SPI library:
  SPI.begin(); //Enable SPI for OPC comms(and SD)

  wdt_reset(); //Reset watchdog timer
  InitDevice();
  wdt_reset(); //Reset watchdog timer

  PrintDataLabels(Serial); //Print labels to serial port

  currentTime = millis();
  cloopTime = currentTime;
}

void OPC::InitDevice(void)
{
  wdt_reset(); //Reset watchdog timer

  digitalWrite(pin_select, HIGH); // high for sensor, low for SD
  
  ReadOPCstring(0x10); //Get serialstr from OPC device
  ReadOPCstring(0x3F); //Get infostr from OPC device

  StartOPC(); //Switch on power to fan and laser
  wdt_reset(); //Reset watchdog timer
  ReadOPCconfig(Serial); //Get Config data (bin boundaries etc.) from OPC device
}

void OPC::loop() {

  // stage 1: read OPC
  wdt_reset(); // Reset watchdog timer

  currentTime = millis(); // millis count will reset on sketch restart
  if (currentTime >= cloopTime) {
    cloopTime += 5000; // Updates cloopTime

    wdt_reset(); // Reset watchdog timer

    // Switch power ON to fan and laser
    StartOPC();

    wdt_reset(); // Reset watchdog timer

    // Get 10 histogram data sets (don't record the first one as it will contain invalid data)
    unsigned long GetHistTime = millis(); // Set initial GetHistTime
    for (byte i = 0; i < 10; i++) {
      delay(1000);
      ReadOPChist(); // Read OPC histogram data
      if (i != 0) {
        // Print time since start (millis() returns an unsigned long of number of ms since program started. It wraps around in ~50 days.)
        Serial.print(millis());
        PrintData(Serial); // Print data to serial
      }
      wdt_reset(); // Reset watchdog timer
    }

    // Switch power OFF to fan and laser
    StopOPC();
    // END Device #1

    // Serial.println("Waiting until next cycle");
    SDdata();
  }
}

void OPC::SDdata(){
  //SPI.setDataMode(SPI_MODE0); // set SPI mode 0
  
  digitalWrite(pin_select, LOW); // low for SD, high for sensor
  digitalWrite(ssPin_SD, LOW); // low for SD, high for sensor
  
  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(ssPin_SD)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  // make a string for assembling the data to log:
  /*String dataString = "";

  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 68; analogPin++) {
    //int sensor = analogRead(analogPin);
    if (analogPin < 67) {
      dataString += SPI_in[analogPin];
      dataString += ",";
    }
*/
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("test329.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("Data test");
    
    PrintData(dataFile);
    wdt_reset();
    
    dataFile.close();
    Serial.println("SD over");
    
    delay(5000);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening test314.txt");
  }
  
  digitalWrite(pin_select, HIGH); 
  digitalWrite(ssPin_SD, HIGH); // low for SD, high for sensor (turn off SD)
}

void OPC::StartOPC() {
  //Turn ON fan and peripheral power
  GetReadyResponse(0x03);
  SPI.transfer(0x03); //Turn ON fan and peripheral power
  SetSSpin(HIGH);
  SPI.endTransaction();
  delay(10);

  //Wait for fan to reach full speed (and for multiple attempts by OPC firmware to turn on fan)
  for (byte i=0; i<5; i++)
  {
    wdt_reset(); //Reset watchdog timer
    delay(1000);
  }
}

void OPC::StopOPC() {
  //Turn OFF fan and peripheral power
  GetReadyResponse(0x03);
  SPI.transfer(0x00); //Turn OFF fan and peripheral power
  SetSSpin(HIGH); 
  SPI.endTransaction();
  SPI.end();
  delay(1000);
}

void OPC::ReadOPChist() {
  // Read OPC histogram data
  GetReadyResponse(0x30);
  for (SPI_in_index=0; SPI_in_index<64; SPI_in_index++)
  {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = SPI.transfer(0x01); //Value of outgoing byte doesn't matter
  }
  SetSSpin(HIGH);
  SPI.endTransaction();
  delay(10);
}


void OPC::AddDelimiter(Stream& port) {
  port.print(F(",")); // delimiter
}

void OPC::SetSSpin(bool pinState) {
  digitalWrite(ssPin_OPC, pinState); // Set output to pinState for OPC
  digitalWrite(ssPin_SD, ~pinState); //
}

float OPC::ConvSTtoTemperature(uint16_t ST) {
  // Convert ST to temperature
  return -45 + 175*(float)ST/65535;
}

float OPC::ConvSRHtoRelativeHumidity(uint16_t SRH) {
  // Convert SRH to relative humidity
  return 100*(float)SRH/65535;
}

uint16_t OPC::MODBUS_CalcCRC(unsigned char data[], unsigned char nbrOfBytes){
  // Calculate MODBUS CRC
  #define POLYNOMIAL_MODBUS 0xA001 //Generator polynomial for MODBUS crc
  #define InitCRCval_MODBUS 0xFFFF //Initial CRC value

  unsigned char _bit; // bit mask
  uint16_t crc = InitCRCval_MODBUS; // initialise calculated checksum
  unsigned char byteCtr; // byte counter

  // calculates 16-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (uint16_t)data[byteCtr];
    for(_bit = 0; _bit < 8; _bit++)
    {
      if (crc & 1) //if bit0 of crc is 1
      {
        crc >>= 1;
        crc ^= POLYNOMIAL_MODBUS;
      }
      else
        crc >>= 1;
    }
  }
  return crc;
}

void OPC::GetReadyResponse(unsigned char SPIcommand)
{
  unsigned char Response;
  digitalWrite(pin_select, HIGH);
  SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE1));

  //Try reading a byte here to clear out anything remnant of SD card SPI activity
  //No need for this in this example of code
  //Response = SPI.transfer(SPIcommand);
  //delay(1);  //wait 1ms

  do
  {
    SetSSpin(LOW);
    unsigned char Tries = 0;
    do
    {
      Response = SPI.transfer(SPIcommand);
      if (Response != SPI_OPC_ready) delay(1); //wait 1ms
    }
    while ((Tries++ < 20) && (Response != SPI_OPC_ready));

    if (Response != SPI_OPC_ready)
    {
      if (Response == SPI_OPC_busy)
      {
        SetSSpin(HIGH);
        Serial.println(F("ERROR Waiting 2s (for OPC comms timeout)")); //signal user
        Serial.flush();
        wdt_reset();
        delay(2000); //wait 2s
      }
      else
      {
        /*
        //Can just wait for WDT to timeout and SPI to be restablished on reset
        SetSSpin(HIGH);
        Serial.println(F("ERROR Waiting for UNO WDT timeout")); //signal user
        Serial.flush();
        while(1);
        */

        //End SPI and wait a few seconds for it to be cleared
        SetSSpin(HIGH);
        Serial.println(F("ERROR Resetting SPI")); //signal user
        Serial.flush();
        SPI.endTransaction();
        //Wait 6s here for buffer to be cleared
        wdt_reset();
        delay(6000);
        wdt_reset();
        SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE1));
      }
    }
  }
  while ((Response != SPI_OPC_ready) && (Serial.available()==0)); //don't hang on this if data is coming in on serial interface
  delay(10);

  wdt_reset();
}

void OPC::ReadOPCstring(unsigned char SPIcommand)
{
  GetReadyResponse(SPIcommand);
  for (SPI_in_index=0; SPI_in_index<60; SPI_in_index++)
  {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = SPI.transfer(0x01); //Value of outgoing byte doesn't matter
  }

  SetSSpin(HIGH);
  SPI.endTransaction();

  PrintOPCstring(Serial);
}

void OPC::PrintOPCstring(Stream &port)
{
  port.write(SPI_in, 60); //print 60 characters from SPI_in[] array
  port.println("");
  port.flush();
}

void OPC::DiscardSPIbytes(byte NumToDiscard)
{
  for (SPI_in_index=0; SPI_in_index<NumToDiscard; SPI_in_index++)
  {
    delayMicroseconds(10);
    SPI.transfer(0x01); //Value of outgoing byte doesn't matter
  }
}

void OPC::ReadOPCconfig(Stream &port)
{
  //unsigned int *pUInt16;
  uint16_t *pUInt16; //'unsigned int' can be interpreted as a 4-byte variable instead of 2-byte for some MCUs. Safer to use 'uint16_t' type.
  float *pFloat;

  //Have to read config from OPC device in this 'chunks' manner as Arduino buffer isn't big enough to hold all config data at once and OPC could timeout if Arduino took time to print/save data from the buffer during the SPI transfer sequence.
  //Instead, config data is read several times, and each time a different chunk is saved to the Arduino buffer and printed. This way there is no delay during each individual SPI transfer sequence.

  //Get config from OPC device (Bin Boundaries ADC)
    GetReadyResponse(0x3C);
    for (SPI_in_index=0; SPI_in_index<34; SPI_in_index++)
    {
      delayMicroseconds(10);
      SPI_in[SPI_in_index] = SPI.transfer(0x01); //Value of outgoing byte doesn't matter
    }
    //Throw away any remaining bytes OPC expects to transfer. Although not putting it in buffer yet, must complete SPI transfer of all config bytes as OPC is expecting this
    DiscardSPIbytes(159);
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);

    port.print(F("BinBoundaries(ADC)"));
    for (SPI_in_index=0; SPI_in_index<34; SPI_in_index+=2)
    {
      AddDelimiter(port);
      pUInt16 = (uint16_t *)&SPI_in[SPI_in_index];
      port.print(*pUInt16, DEC);
    }
    port.println("");
    port.flush();
  //END Get config from OPC device (Bin Boundaries ADC)

  //Get config from OPC device (Bin Boundaries um)
    GetReadyResponse(0x3C);
    //Throw away bytes until reaching desired bytes in config
    DiscardSPIbytes(34);
    //Put required config bytes in buffer
    for (SPI_in_index=0; SPI_in_index<68; SPI_in_index++)
    {
      delayMicroseconds(10);
      SPI_in[SPI_in_index] = SPI.transfer(0x01); //Value of outgoing byte doesn't matter
    }
    //Throw away any remaining bytes OPC expects to transfer
    DiscardSPIbytes(91);
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);

    port.print(F("BinBoundaries(um)"));
    for (SPI_in_index=0; SPI_in_index<68; SPI_in_index+=4)
    {
      AddDelimiter(port);
      pFloat = (float *)&SPI_in[SPI_in_index];
      port.print(*pFloat, 2); //print to 2dp
    }
    port.println("");
    port.flush();
  //END Get config from OPC device (Bin Boundaries um)

  //Get config from OPC device (Bin Weightings)
    GetReadyResponse(0x3C);
    //Throw away bytes until reaching desired bytes in config
    DiscardSPIbytes(102);
    //Put required config bytes in buffer
    for (SPI_in_index=0; SPI_in_index<64; SPI_in_index++)
    {
      delayMicroseconds(10);
      SPI_in[SPI_in_index] = SPI.transfer(0x01); //Value of outgoing byte doesn't matter
    }
    //Throw away any remaining bytes OPC expects to transfer
    DiscardSPIbytes(27);
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);

    port.print(F("BinWeightings"));
    for (SPI_in_index=0; SPI_in_index<64; SPI_in_index+=4)
    {
      AddDelimiter(port);
      pFloat = (float *)&SPI_in[SPI_in_index];
      port.print(*pFloat, 2); //print to 2dp
    }
    port.println("");
    port.flush();
  //END Get config from OPC device (Bin Weightings)

  //Get config from OPC device (Misc)
    GetReadyResponse(0x3C);
    //Throw away bytes until reaching desired bytes in config
    DiscardSPIbytes(166);
    //Put required config bytes in buffer
    for (SPI_in_index=0; SPI_in_index<27; SPI_in_index++)
    {
      delayMicroseconds(10);
      SPI_in[SPI_in_index] = SPI.transfer(0x01); //Value of outgoing byte doesn't matter
    }
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);

    port.print(F("GSC,"));
    pFloat = (float *)&SPI_in[0];
    port.println(*pFloat, 2); //print to 2dp

    //Don't bother printing this as is shows in histogram data anyway
    //port.print(F("SFR(ml/s),"));
    //pFloat = (float *)&SPI_in[4];
    //port.println(*pFloat, 2); //print to 2dp

    port.print(F("TOFtoSFRfactor,"));
    port.println(SPI_in[8], DEC);

    port.print(F("M_A(um),"));
    pFloat = (float *)&SPI_in[9];
    port.println(*pFloat, 2); //print to 2dp
    port.print(F("M_B(um),"));
    pFloat = (float *)&SPI_in[13];
    port.println(*pFloat, 2); //print to 2dp
    port.print(F("M_C(um),"));
    pFloat = (float *)&SPI_in[17];
    port.println(*pFloat, 2); //print to 2dp

    port.print(F("PVP(us),"));
    port.println((float)SPI_in[21]/48, 2); //print to 2dp

    port.print(F("PowerStatus,"));
    port.println(SPI_in[22], DEC);

    port.print(F("MaxTOF(us),"));
    pUInt16 = (uint16_t *)&SPI_in[23];
    port.println((float)*pUInt16/48, 2); //print to 2dp

    port.print(F("LaserDAC,"));
    port.println(SPI_in[25], DEC);

    port.print(F("BinWeightingIndex,"));
    port.println(SPI_in[26], DEC);
    port.flush();
  //END Get config from OPC device (Misc)
}

void OPC::PrintData(Stream &port)
{
  unsigned char i;
  uint16_t *pUInt16;
  float *pFloat;
  float Afloat;

  //Histogram bins (UInt16) x16
  for (i=0; i<32; i+=2)
  {
    AddDelimiter(port);
    pUInt16 = (uint16_t *)&SPI_in[i];
    port.print(*pUInt16, DEC);
  }

  //MToF bytes (UInt8) x4
  for (i=32; i<36; i++)
  {
    AddDelimiter(port);
    Afloat = (float)SPI_in[i];
    Afloat /= 3; //convert to us
    port.print(Afloat, 2);
  }

  //SFR (4-byte float) x1
  AddDelimiter(port);
  pFloat = (float *)&SPI_in[36];
  port.print(*pFloat, 3); //print to 3dp

  //Temperature (UInt16) x1
  AddDelimiter(port);
  pUInt16 = (uint16_t *)&SPI_in[40];
  port.print(ConvSTtoTemperature(*pUInt16), 1); //print to 1dp

  //Relative humidity (UInt16) x1
  AddDelimiter(port);
  pUInt16 = (uint16_t *)&SPI_in[42];
  port.print(ConvSRHtoRelativeHumidity(*pUInt16), 1); //print to 1dp

  //Sampling period(s) (4-byte float) x1
  AddDelimiter(port);
  pFloat = (float *)&SPI_in[44];
  port.print(*pFloat, 3); //print to 3dp

  //Reject count Glitch (UInt8) x1
  AddDelimiter(port);
  port.print(SPI_in[48], DEC);

  //Reject count LongTOF (UInt8) x1
  AddDelimiter(port);
  port.print(SPI_in[49], DEC);

  //PM values(ug/m^3) (4-byte float) x3
  for (i=50; i<62; i+=4)
  {
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[i];
    port.print(*pFloat, 3); //print to 3dp
  }

  //Checksum (UInt16) x1
  AddDelimiter(port);
  pUInt16 = (uint16_t *)&SPI_in[62];
  port.println(*pUInt16, DEC);

  //Compare recalculated Checksum with one sent
  if (*pUInt16 != MODBUS_CalcCRC(SPI_in, 62)) //if checksums aren't equal
    port.println(F("Checksum error in line above!"));

  port.flush();
}

void OPC::PrintDataLabels(Stream &port)
{
  unsigned char i;

  port.print(F("Time(ms)"));

  for (i=0; i<16; i++)
  {
    port.print(F(",Bin"));
    if (i < 10) port.print(F("0")); //leading 0 for single digit bin numbers
    port.print(i, DEC);
  }

  for (i=1; i<9; i+=2)
  {
    port.print(F(",MToFBin"));
    port.print(i, DEC);
    if (i == 1) port.print(F("(us)")); //print units for first value of this type
  }

  port.println(F(",SFR(ml/s),T(C),RH(%),SampPrd(s),#RejectGlitch,#RejectLong,PM_A(ug/m^3),PM_B,PM_C,Checksum"));

  port.flush();
}

void OPC::PrintFirmwareVer (Stream &port)
{
  port.print(F("Datalogger firmware ver "));
  port.println(FirmwareVer);
}
