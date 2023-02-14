#include <Arduino.h>
#include "OPT3101/OPT3101.h"
#include <Wire.h>

OPT3101 sensor;

#define IRQ_PIN PA4
#define LED PA2


HardwareSerial Serial1(PA10, PA9);  // custom serial port
TwoWire i2c_port(PB11, PB10);       // custom I2C pins

uint16_t amplitudes[3];
int16_t distances[3];

bool obstruction = false; 

void setup()
{
  Serial1.begin(115200);
  Serial1.println("Begin");

  i2c_port.begin();
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  pinMode(IRQ_PIN, OUTPUT);
  digitalWrite(IRQ_PIN, LOW);


  // Wait for the Serial1 port to be opened before printing
  // messages (only applies to boards with native USB).
  while (!Serial1) {}

  sensor.init(i2c_port); // Init the sensor with the new I2C line

  if (sensor.getLastError())
  {
    Serial1.print(F("Failed to initialize OPT3101: error "));
    Serial1.println(sensor.getLastError());
    while (1) {}
  }

  sensor.setFrameTiming(256);
  sensor.setChannel(0);
  sensor.setBrightness(OPT3101Brightness::Adaptive);

  sensor.startSample();
  Serial1.println("Started");
}
void loop()
{
  if (sensor.isSampleDone())
  {
    sensor.readOutputRegs();

    amplitudes[sensor.channelUsed] = sensor.amplitude;
    distances[sensor.channelUsed] = sensor.distanceMillimeters;

    if (sensor.channelUsed == 2)
    {
      for (uint8_t i = 0; i < 3; i++)
      {
        Serial1.print(distances[i]);
        Serial1.print(", ");
      }
      Serial1.print(" || ");
      for (uint8_t i = 0; i < 3; i++)
      {
        Serial1.print(amplitudes[i]);
        Serial1.print(", ");
      }
      Serial1.println();
    }
    sensor.nextChannel();
    sensor.startSample();
    
    obstruction = (distances[0]<300 || distances[1]<350 || distances[2]<300);

    digitalWrite(LED, !obstruction);
    digitalWrite(IRQ_PIN, !obstruction);
    
  }
}
