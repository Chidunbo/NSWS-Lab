#ifndef OPC_H
#define OPC_H
#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3
#define FirmwareVer "OPC-R2-02(UNOr3)(res divider on SS)"

#include <Arduino.h>
#include <SPI.h>
#include <avr/wdt.h>
#include <SD.h>

class OPC {
public:
  OPC(uint8_t ssPin, uint8_t chipSelect);

  void begin();
  void loop();

private:
  uint8_t ssPin_OPC;
  uint8_t ssPin_SD;
  //Stream& opSerial;
  unsigned long cloopTime;
  unsigned long currentTime;
  unsigned char SPI_in[68];
  unsigned char SPI_in_index;
  const int pin_output_control = 6; // to control digital switch, to low
  const int pin_select = 7; // to control digital switch, low for SD, high for sensor


  void InitDevice();
  void SDdata();
  void StartOPC();
  void StopOPC();
  void ReadOPChist();
  void PrintData(Stream& port);
  void PrintDataLabels(Stream& port);
  void AddDelimiter(Stream& port);
  void SetSSpin(bool pinState);
  float ConvSTtoTemperature(uint16_t ST);
  float ConvSRHtoRelativeHumidity(uint16_t SRH);
  uint16_t MODBUS_CalcCRC(unsigned char* buf, unsigned char len);

  void GetReadyResponse(unsigned char SPIcommand);
  void ReadOPCstring(unsigned char SPIcommand);
  void PrintOPCstring(Stream &port);
  void DiscardSPIbytes (byte NumToDiscard);
  void ReadOPCconfig (Stream &port);
  void PrintFirmwareVer(Stream &port);



};

#endif
